#include "aris.hpp"
#include "kaanh/log/log.hpp"
#include "kaanh/robot/robot.hpp"
#include "kaanhbot/robot/robot.hpp"
#include "kaanhbot/config/kaanhconfig.hpp"
#include "aris_version.h"
#include "kaanh_version.h"
#include "kaanhbot_version.h"
#include "kaanhbin_version.h"
#include "aris/core/version.hpp"
#include "kaanh/general/version.hpp"
#include "kaanhbot/system/version.hpp"
#include "kaanhbot/system/about.hpp"

#define __S(x) #x
#define _S(x) __S(x)

auto xmlpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
auto logpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string logfolder = "log";

int main(int argc, char *argv[]){
	xmlpath = xmlpath / xmlfile;
	logpath = logpath / logfolder;

	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);
	auto path = argc < 2 ? xmlpath : argv[2];
	auto logp = argc < 2 ? logpath : argv[3];

	std::cout << "port:" << port << std::endl;
	std::cout << "xmlpath:" << xmlpath << std::endl;
	std::cout << "path:" << path << std::endl;
	std::cout << "logfolder:" << logp << std::endl;

	// 重载Aris log接口
	aris::core::setLogMethod([](aris::core::LogData data)->void {
		// switch (data.level) {
		// case aris::core::LogLvl::kError :
		// case aris::core::LogLvl::kFatal :
		// 	ERROR_SYS_NC("Aris -- "+data.msg, data.code);
		// 	break;
		// case aris::core::LogLvl::kDebug :
		// case aris::core::LogLvl::kInfo :
		// default :
		// 	break;
		// } 
		if ((int)(data.level)>(int)(aris::core::LogLvl::kInfo))
			std::cout << "aris-log -- " << data.msg << std::endl;
	});

	aris::core::fromXmlFile(cs, path);
    //cs.resetModel(kaanhbot::config::createModelLansi().release());
    //cs.resetModel(kaanhbot::config::createModelScaraLansi().release());
    //cs.resetModel(kaanhbot::config::createModelFullDeltaLansi().release());
    cs.init();
    //cs.resetTransferModelController(new aris::server::ScaraTransferModelController(0.016));
    //aris::core::toXmlFile(cs, path);

	// 重载Aris log接口
	aris::core::setLogMethod([](aris::core::LogData data)->void {
		// switch (data.level) {
		// case aris::core::LogLvl::kError :
		// case aris::core::LogLvl::kFatal :
		// 	ERROR_SYS_NC("Aris -- "+data.msg, data.code);
		// 	break;
		// case aris::core::LogLvl::kDebug :
		// case aris::core::LogLvl::kInfo :
		// default :
		// 	break;
		// } 
		std::cout << "aris-log -- " << data.msg << std::endl;
	});

    std::cout <<"motor size: "<< cs.controller().motorPool().size() << std::endl;
    std::cout <<"io size: "<< cs.controller().digitalIoPool().size() << std::endl;
    std::cout <<"fs size: "<< cs.controller().ftSensorPool().size() << std::endl;

	// 设置版本回调
	kaanhbot::system::About::registerCbk("aris", []()->std::pair<std::string,std::string>{
		return {std::string("v")+_S(ARIS_VERSION_MAJOR)+"."+_S(ARIS_VERSION_MINOR)+"."+_S(ARIS_VERSION_PATCH)+"."+_S(ARIS_VERSION_TIME), _S(ARIS_DESCRIPTION)};
	}, aris::core::version);
	kaanhbot::system::About::registerCbk("kaanh", []()->std::pair<std::string,std::string>{
		return {std::string("v")+_S(KAANH_VERSION_MAJOR)+"."+_S(KAANH_VERSION_MINOR)+"."+_S(KAANH_VERSION_PATCH)+"."+_S(KAANH_VERSION_TIME), _S(KAANH_DESCRIPTION)};
	}, kaanh::general::version);
	kaanhbot::system::About::registerCbk("kaanhbot", []()->std::pair<std::string,std::string>{
		return {std::string("v")+_S(KAANHBOT_VERSION_MAJOR)+"."+_S(KAANHBOT_VERSION_MINOR)+"."+_S(KAANHBOT_VERSION_PATCH)+"."+_S(KAANHBOT_VERSION_TIME), _S(KAANHBOT_DESCRIPTION)};
	}, kaanhbot::system::version);
	kaanhbot::system::About::registerCbk("kaanhbin", []()->std::pair<std::string,std::string>{
		return {std::string("v")+_S(KAANHBIN_VERSION_MAJOR)+"."+_S(KAANHBIN_VERSION_MINOR)+"."+_S(KAANHBIN_VERSION_PATCH)+"."+_S(KAANHBIN_VERSION_TIME), _S(KAANHBIN_DESCRIPTION)};
	}, []()->std::pair<std::string,std::string>{
		return {std::string("v")+_S(KAANHBIN_VERSION_MAJOR)+"."+_S(KAANHBIN_VERSION_MINOR)+"."+_S(KAANHBIN_VERSION_PATCH)+"."+_S(KAANHBIN_VERSION_TIME), _S(KAANHBIN_DESCRIPTION)};
	});

	// 防止kaanhbot库被编译器优化
	kaanhbot::robot::Robot::instanceInCs();

	// 重载Aris Rt-log接口
    auto func = [](aris::plan::Plan *p, int error_num, const char *error_msg) {
        RT_ERROR_SYS_NC(error_msg, error_num);
    };
    cs.setRtErrorCallback(func);

    //实时回调函数，每个实时周期调用一次
    cs.setRtPlanPreCallback([](aris::server::ControlServer&cs){
        static int32_t update_counter = 0;
        if(update_counter < 2000){
            update_counter++;
            return;
        }
        kaanh::robot::Robot::instanceInCs().rtUpdate(cs);
    });

    for(aris::Size i = 0; i < cs.controller().motorPool().size(); i++){
        cs.idleMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        cs.globalMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }

    //开启控制器服务
    try {
        cs.start();
		INFO_SYS(LOCALE_SELECT("system starts", "系统启动"), 0)
        //电机抱闸设置恢复
        int driver_num=cs.controller().motorPool().size();
        for(int i= 0; i < driver_num; i++){
            auto &motor=cs.controller().motorPool().at(i);
            motor.setOutputIoNrt(1,0x00);
            motor.setOutputIoNrt(2,0x00);
        }

        cs.executeCmd("md");

        cs.executeCmd("rc");
    }
    catch (const std::exception& err){
        kaanh::log::Sqlite3Log::instance().triggerState(-9);
    }


	//Start Web Socket
	cs.open();

	//Receive Command
	cs.runCmdLine();

	return 0;
}
