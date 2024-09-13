#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
//#include "kaanh.h"
#include "robot.h"
#include<aris.hpp>

#include"plan.h"
#include"planRT.h"

using namespace aris::dynamic;
using namespace aris::plan;
//------------------
#include<iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include"serial.h"
//------------------
namespace robot {


        /*-----------FitTog------------*/
        FitTog::FitTog(const std::string &name){
            aris::core::fromXmlString(command(),
                                      "<Command name=\"fit\">"
                                      "	<GroupParam>"
                                      "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                      "		<Param name=\"tool\" default=\"tool0\"/>"
                                      "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                      "     <Param name=\"Mass\" default=\"{0.3,0.3,0.3,1.0,1.0,1.0}\" abbreviation=\"m\"/>"
                                      "     <Param name=\"Damp\" default=\"{0.9,0.9,0.9,0.0,0.0,0.0}\" abbreviation=\"b\"/>"
                                      "	</GroupParam>"
                                      "</Command>"
                                      );
        }
        struct FitTogParam{
            aris::dynamic::Marker * tool, * wobj;
            //parameters in prepareNT
            double x_e; //environment position
            double vel_limit[6];//the velocity limitation of motors
            //parameters in excuteRT
            double pm_init[16];//the position matrix of the tool center in world frame
            double start_Pos[6];//ee eu_type postion and angle
            double theta_setup = -90;// the install angle of force sensor
            double pos_setup = 0.061;// the install position of force sensor
            double fs2tpm[16]; //
            float init_force[6]; // compensate the gravity of tool
            double ke = 220000;
//            double B = 0.7;//the damping factor of end effector0.7 has a better performance
//            double M = 0.1;//0.1 has a better performance
            double K = 3;//
            double loop_period = 1e-3;
            double B[6]{0.25,0.25,0.7,0.0,0.0,0.0};//the damping factor of end effector0.7 has a better performance
            double M[6]{0.1,0.1,0.1,0.1,0.1,0.1};//0.1 has a better performance
            double last_v[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double maxTrans =0.00010;
            int HOME = 0;
            double angle_offset = 1.5 * 3.1415926535 / 180;
            bool contacted = false;// flag to show if the end effector contact with the surface
            double desired_force = 0.8;
            double j6_start_angle;//when connect, remember j6 start angle
            long contacted_start_count;//before contact count()
            long get_init_fs_count;//when HOME = 1, remember fs data
            double contacted_start_x;//the pos when contact
            double reset_center_count;
        };
        struct FitTog::Imp : public FitTogParam{};
        auto FitTog::prepareNrt() -> void
        {
            std::cout<<"prepare begin"<<std::endl;
            imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
            imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
            imp_-> x_e = 0;
            for (auto cmd_param : cmdParams()) {
                if (cmd_param.first == "vellimit") {
                    auto a = matrixParam(cmd_param.first);
                    if (a.size() == 6) {
                        std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                    } else {
                        THROW_FILE_LINE("");
                    }
                }else if (cmd_param.first == "Mass"){
                    auto m = matrixParam(cmd_param.first);
                    if (m.size() == 6) {
                        std::copy(m.data(), m.data() + 6, imp_->M);
                    }else if (m.size() == 1) {
                        std::copy(m.data(), m.data() + 1, imp_->M);
                        std::copy(m.data(), m.data() + 1, imp_->M+1);
                        std::copy(m.data(), m.data() + 1, imp_->M+2);
                    }else{
                        THROW_FILE_LINE("");
                    }
                }else if(cmd_param.first == "Damp"){
                    auto m = matrixParam(cmd_param.first);
                    if (m.size() == 6) {
                        std::copy(m.data(), m.data() + 6, imp_->B);
                    }else if (m.size() == 1) {
                        std::copy(m.data(), m.data() + 1, imp_->B);
                        std::copy(m.data(), m.data() + 1, imp_->B+1);
                        std::copy(m.data(), m.data() + 1, imp_->B+2);
                    }else{
                        THROW_FILE_LINE("");
                    }
                }
            }
            for (auto &option : motorOptions())//???
            {
                option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
            }
            std::vector <std::pair<std::string, std::any>> ret_value;
            ret() = ret_value;
            std::cout<<"prepare finished"<<std::endl;
        }
        auto FitTog::executeRT() ->int
        {
            const int FS_NUM = 7;
            static std::size_t motionNum = controller()->motorPool().size();
            // end-effector //
            //auto &ee = model()->generalmotorPool()[0];
            auto &ee = model()->generalMotionPool()[0];
            // Function Pointer
    //        auto &cout = controller()->mout();
    //        auto &lout = controller()->lout();
            char eu_type[4]{'1', '2', '3', '\0'};
            // the lambda function to get the force
            auto get_force_data = [&](float *data){
                for (std::size_t i =0; i< motionNum;++i)
                {
                    //this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
                    ecMaster()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
                }
            };
            //the function to update model according to real motor and excecute one forward kinematic
            auto forwardPos = [&](){
                for(std::size_t i =0; i<motionNum; ++i)
                {
                   // model()->motorPool()[i].setMp(controller()->motorPool()[i].targetPos());
                    model()->motionPool()[i].setMp(controller()->motorPool()[i].targetPos());
                }
                if(model()->solverPool()[1].kinPos())
                {
                    std::cout<<"forward kinematic failed, exit"<<std::endl;
                }
                //ee.updMpm();
                ee.updP();
            };
            //safty check the position change
            auto checkPos = [&](double * data)
            {
                for(std::size_t i = 0; i< motionNum; i++)
                {
                    if(abs(data[i] - controller()->motorPool()[i].targetPos()) > imp_->vel_limit[i]){
                        std::cout<<"joint "<<i<<" move too fast"<<std::endl;
                        std::cout<<"pi-1 = "<<controller()->motorPool()[i].targetPos()<<std::endl;
                        std::cout<<"pi = "<<data[i]<<std::endl;
                        std::cout<<"vellimit = "<<imp_->vel_limit[i]<<std::endl;
                        return false;
                    }
                }
                return true;
            };
            for(std::size_t i =0; i<motionNum; ++i)
            {
                model()->motionPool()[i].setMp(controller()->motorPool()[i].targetPos());
            }
            if (model()->solverPool().at(1).kinPos()){
                std::cout<<"forward kinematic failed";
                return -1;
            }else{
                ee.updP();
            }
            //first loop record
            if(count() ==1)
            {
                //set the log file
               // controller()->logFileRawName("motion_replay");
                ecMaster()->logFileRawName("motion_replay");
                //get the force transformation matrix in tool frame
                double theta = (-imp_->theta_setup) * PI / 180;
                double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
                s_pq2pm(pq_setup, imp_->fs2tpm);
//                std::cout << "mpstart:" << controller()->motorPool()[5].actualPos() << std::endl;
            }

            //HOME


            double mp[6]{0,0,0,0,0,0};
            if(imp_->HOME == 0)
            {
                double s_tcp[6];
                //ee.getMpe(s_tcp, eu_type);
                ee.getP(s_tcp);
                if (count() % 10 == 0) {
                std::cout <<"  Zpos:" << s_tcp[2] ;
                std::cout<<std::endl;
                }
                if(s_tcp[2]<=0.5){
                    s_tcp[2] += 0.0001;
                    //ee.setMpe(s_tcp, eu_type);
                    ee.setP(s_tcp);

                    model()->solverPool()[0].kinPos();
                    double x_joint[6];
                    for(std::size_t i = 0; i<motionNum; ++i)
                    {
                        model()->motionPool()[i].updP();
                        x_joint[i] = model()->motionPool()[i].mp();
                    }
                    if(checkPos(x_joint)){
                        for(std::size_t i = 0; i<motionNum; ++i)
                        {
                            controller()->motorPool()[i].setTargetPos(x_joint[i]);
                        }
                    }
                }
                else{
                //z-axis move back

                    for (std::size_t i =0; i< motionNum;++i)
                    {
                        if(i != 4){
    //                        std::cout << "Mpos:" << controller()->motorPool()[i].targetPos() << std::endl;
                            if(controller()->motorPool()[i].actualPos() >= 0.0001)
                            {

                                mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
                            }
                            else if(controller()->motorPool()[i].targetPos() <= -0.0001){

                                mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
                            }
                            else{
                                mp[i] = 0;

                            }
                        }
                        else{
    //                        std::cout << "Mpos:" << controller()->motorPool()[i].targetPos() << std::endl;
                            if(controller()->motorPool()[i].actualPos() > 1.57087 - imp_->angle_offset)
                            {

                                mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
                            }
                            else if(controller()->motorPool()[i].targetPos() <= 1.57075 - imp_->angle_offset){

                                mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
                            }
                            else{
                                mp[i] = 1.57080 - imp_->angle_offset;

                            }
                        }

                        controller()->motorPool()[i].setTargetPos(mp[i]);
                    }

                }

                if(imp_->HOME == 0){
                    if(mp[0] == 0.0){
                        if (mp[1] == 0.0){
                            if(mp[2] == 0.0){
                                if(mp[3] == 0.0){
                                    if(mp[4] == 1.57080 - imp_->angle_offset){
                                        if(mp[5] == 0.0){
                                            imp_->HOME = 1;
                                            imp_->get_init_fs_count = count();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(imp_->HOME == 1)
            {
                if(count() == imp_->get_init_fs_count)get_force_data(imp_->init_force);
                //减去力传感器初始偏置
                float force_data[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                double force_data_double[6];
                get_force_data(force_data);
                for (int i = 0; i < 6; i++)
                    force_data[i] -= imp_->init_force[i];
                for (int i = 0; i < 6; ++i)force_data_double[i] = static_cast<double>(force_data[i]);
                //获取每个周期末端所受的力
                double xyz_temp[3]{force_data_double[0], force_data_double[1], force_data_double[2]}, abc_temp[3]{
                        force_data_double[3], force_data_double[4], force_data_double[5]};
                //get the position of tcp
                double pm_begin[16];
                //transform the force to world frame, store in imp_->force_target
                ee.updP();
                imp_->tool->getPm(*imp_->wobj, pm_begin);
                double t2bpm[16];
                s_pm_dot_pm(pm_begin, imp_->fs2tpm, t2bpm);
                double net_force[6];
                s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, net_force, 1);
                s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, net_force + 3, 1);
                //print the force and pos of z
                double s_tcp[6];
                //ee.getMpe(s_tcp, eu_type);
                ee.getP(s_tcp);
                if (count() % 10 == 0) {
                    //std::cout << count() <<"f:" << net_force[0]<<" "<< net_force[1]<< " "<< net_force[2]<< "  pos:" << s_tcp[0] <<" "<< s_tcp[1] <<" "<< s_tcp[2] <<" "<<count()- imp_->contacted_start_count;
                    std::cout << "f:" << net_force[0]<<" "<< net_force[1]<< " "<< net_force[2]<<" "<< net_force[3]<<" "<< net_force[4]<<" "<< net_force[5] <<std::endl;
                    std::cout << "pos:" << s_tcp[0] <<" "<< s_tcp[1] <<" "<< s_tcp[2] << " " << s_tcp[3] <<" "<< s_tcp[4] <<" "<< s_tcp[5] ;
                    std::cout<<std::endl;
                }



                if(abs(net_force[2]) > 0.5 || imp_->contacted)
                {                    
                    if (count() % 1000 == 0)
                        {

                            std::cout << " force control start! " ;
                            std::cout << std::endl;
                        }
                    /*
                    * if判断语句的作用：
                    * abs(imp_->force_target[2])>0.1 表示已经与目标物体接触；
                    * else表示还没接触的时候；
                    * 如果没有接触就以指定速度下降，如果产生接触则执行阻抗控制命令；
                    */

                    double s_tcp[6];
                    //get the velocity of tool center
                    double v_tcp[6];
                    model()->generalMotionPool().at(0).getV(v_tcp);
        //            model()->generalPool().at(0).getMve(v_tcp,eu_type);
                    //get the position of tool center
                    model()->generalMotionPool().at(0).getP(s_tcp);
        //            model()->generalmotorPool().at(0).getMpe(s_tcp,eu_type);
                    if(!imp_->contacted)
                        {                                               
                            imp_->contacted = true;
                            imp_->x_e = s_tcp[2];
                            std::cout << "x_e: " << imp_->x_e << std::endl;
                        }
                    //get the x_reference of
                    double x_r = imp_->x_e - (imp_->desired_force / imp_->ke );
                    //calculate the desired acceleration and velocity
                    double a = (net_force[2] - imp_->desired_force- imp_->B[2] * v_tcp[2] - imp_->K * (s_tcp[2] - x_r))/imp_->M[2];//- imp_->K * (s_tcp[2] - x_r))/imp_->M;
                    double x = s_tcp[2] + 0.5 * a * imp_->loop_period* imp_->loop_period + v_tcp[2] * imp_->loop_period;
                    double v = (x - s_tcp[2]);
                    double ee_v [6]{0.0,0.0,v,0.0,0.0,0.0};
                    s_tcp[2] = x;
                    //ee.setMpe(s_tcp, eu_type);
                    ee.setP(s_tcp);
                    model()->solverPool()[0].kinPos();
                    double x_joint[6];
                    for(std::size_t i = 0; i<motionNum; ++i)
                        {
                            model()->motionPool()[i].updP();
                            x_joint[i] = model()->motionPool()[i].mp();
                        }
                    if(checkPos(x_joint))
                        {
                            for(std::size_t i = 0; i<motionNum; ++i)
                            {
                                controller()->motorPool()[i].setTargetPos(x_joint[i]);
                            }
                            //ee.setMve(ee_v,eu_type);
                            ee.setV(ee_v);
                        }

                    //controller()->motorPool()[5].setTargetPos(imp_->j6_start_angle + 1 * sin((double((count() - imp_->contacted_start_count -1) % 15000))/15000 * PI * 2));


                    double s_tcp1[6];
                    //ee.getMpe(s_tcp1, eu_type);
                    ee.getP(s_tcp1);
//                    if (count() % 10 == 0) {
//                        std::cout << "Zp:" << imp_->contacted_start_x - s_tcp1[2] ;
//                        std::cout<<std::endl;
//                    }
                    //stop rotating end effector when inserting
                    ee.updP();


                    if(count()- imp_->contacted_start_count<=6000&&s_tcp1[2]>=0.1065){//move j6
                        s_tcp[5] -= 0.001 * sin((double((count() - imp_->contacted_start_count -1) % 3000))/3000 * PI * 2);
                        //ee.setMpe(s_tcp, eu_type);
                        ee.getP(s_tcp);
                        model()->solverPool()[0].kinPos();
                        double x_joint[6];
                        for(std::size_t i = 0; i<motionNum; ++i)
                        {
                            model()->motionPool()[i].updP();
                            x_joint[i] = model()->motionPool()[i].mp();
                        }
                        if(checkPos(x_joint)){
                            for(std::size_t i = 0; i<motionNum; ++i)
                            {
                                controller()->motorPool()[i].setTargetPos(x_joint[i]);
                            }

                        }
                        //controller()->motorPool()[5].setTargetPos(imp_->j6_start_angle + 1 * sin((double((count() - imp_->contacted_start_count -1) % 10000))/10000 * PI * 2));

                    }else if(count()- imp_->contacted_start_count>6000&&count()- imp_->contacted_start_count<12000){//&&count()- imp_->contacted_start_count<=15000
                        double fedge = 0.015;
                        double movestep = 0.000002;
                        double zback = movestep/4;
                        double kx = (abs(net_force[0])-fedge)*90;
                        double ky = (abs(net_force[1])-fedge)*90;
                        if(kx>3){
                            kx=3;
                        }
                        if(ky>3){
                            ky=3;
                        }
                            if(net_force[0]>=fedge){
                                s_tcp1[2] += zback;
                                s_tcp1[0] += kx*movestep;

                            }else if(net_force[0]<=-fedge){
                                s_tcp1[2] += zback;
                                s_tcp1[0] -= kx*movestep;

                            }
                                if(net_force[1]>=fedge){
                                    s_tcp1[2] += zback;
                                    s_tcp1[1] += ky*movestep;

                                }else if(net_force[1]<=-fedge){
                                    s_tcp1[2] += zback;
                                    s_tcp1[1] -= ky*movestep;

                                }


                            //ee.setMpe(s_tcp1, eu_type);
                            ee.getP(s_tcp1);
                            model()->solverPool()[0].kinPos();
                            double x_joint[6];
                            for(std::size_t i = 0; i<motionNum; ++i)
                            {
                                model()->motionPool()[i].updP();
                                x_joint[i] = model()->motionPool()[i].mp();
                            }
                            if(checkPos(x_joint)){
                                for(std::size_t i = 0; i<motionNum; ++i)
                                {
                                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                                }
                            }
                            //imp_->reset_center_count = count()- imp_->contacted_start_count;

                    }else if(count()- imp_->contacted_start_count > 12000 && s_tcp1[2]>=0.1065){//move j6
                        s_tcp[5] -= 0.001 * sin((double((count() - imp_->contacted_start_count -1) % 3000))/3000 * PI * 2);
                        //ee.setMpe(s_tcp, eu_type);
                        ee.getP(s_tcp);
                        model()->solverPool()[0].kinPos();
                        double x_joint[6];
                        for(std::size_t i = 0; i<motionNum; ++i)
                        {
                            model()->motionPool()[i].updP();
                            x_joint[i] = model()->motionPool()[i].mp();
                        }
                        if(checkPos(x_joint)){
                            for(std::size_t i = 0; i<motionNum; ++i)
                            {
                                controller()->motorPool()[i].setTargetPos(x_joint[i]);
                            }

                        }
                    }
                    //stop when finished
                    if(s_tcp1[2]<=0.10325){
                        std::cout << "count costs:" << count()-imp_->contacted_start_count ;
                        std::cout<<std::endl;
                        return 0;
                    }

                    //std::cout << imp_->j6_start_angle << "j6pos:" << imp_->j6_start_angle + 0.1 * sin((double((count() - imp_->contacted_start_count - 1) % 15000))/15000 * PI * 2) << std::endl;
                }
                    else
                    {
                        forwardPos();
                        double s_tcp[6];
                        //ee.getMpe(s_tcp, eu_type);
                        ee.getP(s_tcp);
                        s_tcp[2] -= 0.00002;
                        //s_tcp[2] -= 0.00002*sin(count()%10000/10000.0*2*PI);
                        //s_tcp[5] -= 0.001 * sin((double((count() ) % 3000))/3000 * PI * 2);
                        //s_tcp[4] -= 0.001 * sin((double((count() ) % 3000))/3000 * PI * 2);
                        //s_tcp[3] -= 0.001 * sin((double((count() ) % 3000))/3000 * PI * 2);

                        //std::cout  << "count" << count()%1000/1000.0 << ":" << 0.01*sin(count()%1000/1000.0*2*PI) << ":"  << std::endl;

                        //s_tcp[0] += 0.0001*cos(((count()-imp_->contacted_start_count)%10000)/10000*PI*2);
                        //ee.setMpe(s_tcp, eu_type);
                        ee.setP(s_tcp);
                        //double ee_v[6]{0.0, 0.0, -0.00001, 0.0, 0.0, 0.0};
                        model()->solverPool()[0].kinPos();
                        double x_joint[6];
                        for(std::size_t i = 0; i<motionNum; ++i)
                        {
                        //    model()->motorPool()[i].updMp();
                            model()->motionPool()[i].updP();
                         //   x_joint[i] = model()->motorPool()[i].mp();
                            x_joint[i] = model()->motionPool()[i].mp();
                        }
                        if(checkPos(x_joint)){
                            for(std::size_t i = 0; i<motionNum; ++i)
                            {
                                controller()->motorPool()[i].setTargetPos(x_joint[i]);
                            }
                            //model()->generalmotorPool()[0].setMve(ee_v, eu_type);
                        }
                        imp_->contacted_start_count = count();
                        imp_->contacted_start_x = s_tcp[2];
                        imp_->j6_start_angle = controller()->motorPool()[5].actualPos();
                        //std::cout << "f:" << net_force[2];//<< " v_tcp[2] : "<<v_tcp[2];
                        //std::cout<<std::endl;
                        //std::cout  << "j6"  << ":" << imp_->j6_start_angle << std::endl;
                        if(count()>=2000){
                            return 0;
                        }

                    }
            }
            double stcp[6]{0,0,0,0,0,0};
            ee.updP();
            //ee.getMpe(stcp, eu_type);
            ee.getP(stcp);
            if(count() % 5000 == 0)
            {
                for (std::size_t i =0; i< motionNum;++i)
                {
                    std::cout  << "stcp" << i  << ":" << stcp[i] << std::endl;
                }
            }

            return 1;
        }
        auto FitTog::collectNrt() -> void{}
        FitTog::~FitTog() = default;
        FitTog::FitTog(const FitTog &other) = default;

    auto createPlanRoot() -> std::unique_ptr <aris::plan::PlanRoot> {
        std::unique_ptr <aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
        //用户自己开发指令集
//        plan_root->planPool().add<robot::MoveS>();
        plan_root->planPool().add<robot::MoveTest>();
        plan_root->planPool().add<robot::MoveJoint>();
        plan_root->planPool().add<robot::ImpedPos>();
        plan_root->planPool().add<robot::Drag>();
        plan_root->planPool().add<robot::DragTrans>();
        plan_root->planPool().add<robot::DragRot>();
        plan_root->planPool().add<robot::Moveto>();
//        plan_root->planPool().add<robot::Imu>();
        plan_root->planPool().add<robot::FitTog>();
        plan_root->planPool().add<robot::read_force>();

        //aris库提供指令集
        plan_root->planPool().add<aris::plan::Enable>();//
        plan_root->planPool().add<aris::plan::Disable>();
        plan_root->planPool().add<aris::plan::Start>();
        plan_root->planPool().add<aris::plan::Stop>();
        plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Clear>();
//        plan_root->planPool().add<aris::server::GetInfo>();
        //kaanh库提供指令集
        plan_root->planPool().add<kaanh::Home>();
        plan_root->planPool().add<kaanh::Sleep>();
        plan_root->planPool().add<kaanh::Recover>();
        plan_root->planPool().add<kaanh::Reset>();
        plan_root->planPool().add<kaanh::MoveAbsJ>();
        plan_root->planPool().add<kaanh::MoveL>();
        plan_root->planPool().add<kaanh::MoveJ>();
        plan_root->planPool().add<kaanh::MoveC>();
        plan_root->planPool().add<kaanh::Get>();
        plan_root->planPool().add<kaanh::Var>();
        plan_root->planPool().add<kaanh::Evaluate>();
        plan_root->planPool().add<kaanh::JogJ1>();
        plan_root->planPool().add<kaanh::JogJ2>();
        plan_root->planPool().add<kaanh::JogJ3>();
        plan_root->planPool().add<kaanh::JogJ4>();
        plan_root->planPool().add<kaanh::JogJ5>();
        plan_root->planPool().add<kaanh::JogJ6>();
        plan_root->planPool().add<kaanh::JogJ7>();
        plan_root->planPool().add<kaanh::JX>();
        plan_root->planPool().add<kaanh::JY>();
        plan_root->planPool().add<kaanh::JZ>();
        plan_root->planPool().add<kaanh::JRX>();
        plan_root->planPool().add<kaanh::JRY>();
        plan_root->planPool().add<kaanh::JRZ>();
        plan_root->planPool().add<kaanh::SetDH>();
        plan_root->planPool().add<kaanh::SetPG>();
        plan_root->planPool().add<kaanh::SetPPath>();
        plan_root->planPool().add<kaanh::SetUI>();
        plan_root->planPool().add<kaanh::SetDriver>();
        plan_root->planPool().add<kaanh::SaveXml>();
        plan_root->planPool().add<kaanh::ScanSlave>();
        plan_root->planPool().add<kaanh::GetEsiPdoList>();
        plan_root->planPool().add<kaanh::SetEsiPath>();
        plan_root->planPool().add<kaanh::GetXml>();
        plan_root->planPool().add<kaanh::SetXml>();
        plan_root->planPool().add<kaanh::SetCT>();
        plan_root->planPool().add<kaanh::SetVel>();
        plan_root->planPool().add<kaanh::Run>();
        plan_root->planPool().add<kaanh::MoveF>();
        plan_root->planPool().add<kaanh::Switch>();
        plan_root->planPool().add<kaanh::CalibFZero>();
        plan_root->planPool().add<CalibT4P>();
        plan_root->planPool().add<CalibT5P>();
        plan_root->planPool().add<CalibT6P>();
        plan_root->planPool().add<CalibW3P>();
        plan_root->planPool().add<kaanh::FCStop>();
        plan_root->planPool().add<kaanh::MoveDJ>();
        plan_root->planPool().add<kaanh::AdmitInit>();
        plan_root->planPool().add<kaanh::Kunwei>();

        //plan_root->planPool().add<kaanh::MoveJoint>();
        return plan_root;
    }
}
