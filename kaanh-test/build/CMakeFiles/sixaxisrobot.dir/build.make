# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/cmake-3.12.3-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /usr/cmake-3.12.3-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kaanh/Desktop/xyn/kaanh-test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaanh/Desktop/xyn/kaanh-test/build

# Include any dependencies generated for this target.
include CMakeFiles/sixaxisrobot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sixaxisrobot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sixaxisrobot.dir/flags.make

CMakeFiles/sixaxisrobot.dir/src/main.cpp.o: CMakeFiles/sixaxisrobot.dir/flags.make
CMakeFiles/sixaxisrobot.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/xyn/kaanh-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sixaxisrobot.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sixaxisrobot.dir/src/main.cpp.o -c /home/kaanh/Desktop/xyn/kaanh-test/src/main.cpp

CMakeFiles/sixaxisrobot.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sixaxisrobot.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/xyn/kaanh-test/src/main.cpp > CMakeFiles/sixaxisrobot.dir/src/main.cpp.i

CMakeFiles/sixaxisrobot.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sixaxisrobot.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/xyn/kaanh-test/src/main.cpp -o CMakeFiles/sixaxisrobot.dir/src/main.cpp.s

CMakeFiles/sixaxisrobot.dir/src/robot.cpp.o: CMakeFiles/sixaxisrobot.dir/flags.make
CMakeFiles/sixaxisrobot.dir/src/robot.cpp.o: ../src/robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/xyn/kaanh-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sixaxisrobot.dir/src/robot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sixaxisrobot.dir/src/robot.cpp.o -c /home/kaanh/Desktop/xyn/kaanh-test/src/robot.cpp

CMakeFiles/sixaxisrobot.dir/src/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sixaxisrobot.dir/src/robot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/xyn/kaanh-test/src/robot.cpp > CMakeFiles/sixaxisrobot.dir/src/robot.cpp.i

CMakeFiles/sixaxisrobot.dir/src/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sixaxisrobot.dir/src/robot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/xyn/kaanh-test/src/robot.cpp -o CMakeFiles/sixaxisrobot.dir/src/robot.cpp.s

CMakeFiles/sixaxisrobot.dir/src/plan.cpp.o: CMakeFiles/sixaxisrobot.dir/flags.make
CMakeFiles/sixaxisrobot.dir/src/plan.cpp.o: ../src/plan.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/xyn/kaanh-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/sixaxisrobot.dir/src/plan.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sixaxisrobot.dir/src/plan.cpp.o -c /home/kaanh/Desktop/xyn/kaanh-test/src/plan.cpp

CMakeFiles/sixaxisrobot.dir/src/plan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sixaxisrobot.dir/src/plan.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/xyn/kaanh-test/src/plan.cpp > CMakeFiles/sixaxisrobot.dir/src/plan.cpp.i

CMakeFiles/sixaxisrobot.dir/src/plan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sixaxisrobot.dir/src/plan.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/xyn/kaanh-test/src/plan.cpp -o CMakeFiles/sixaxisrobot.dir/src/plan.cpp.s

CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.o: CMakeFiles/sixaxisrobot.dir/flags.make
CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.o: ../src/planRT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/xyn/kaanh-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.o -c /home/kaanh/Desktop/xyn/kaanh-test/src/planRT.cpp

CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/xyn/kaanh-test/src/planRT.cpp > CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.i

CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/xyn/kaanh-test/src/planRT.cpp -o CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.s

CMakeFiles/sixaxisrobot.dir/src/serial.cpp.o: CMakeFiles/sixaxisrobot.dir/flags.make
CMakeFiles/sixaxisrobot.dir/src/serial.cpp.o: ../src/serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/xyn/kaanh-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/sixaxisrobot.dir/src/serial.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sixaxisrobot.dir/src/serial.cpp.o -c /home/kaanh/Desktop/xyn/kaanh-test/src/serial.cpp

CMakeFiles/sixaxisrobot.dir/src/serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sixaxisrobot.dir/src/serial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/xyn/kaanh-test/src/serial.cpp > CMakeFiles/sixaxisrobot.dir/src/serial.cpp.i

CMakeFiles/sixaxisrobot.dir/src/serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sixaxisrobot.dir/src/serial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/xyn/kaanh-test/src/serial.cpp -o CMakeFiles/sixaxisrobot.dir/src/serial.cpp.s

# Object files for target sixaxisrobot
sixaxisrobot_OBJECTS = \
"CMakeFiles/sixaxisrobot.dir/src/main.cpp.o" \
"CMakeFiles/sixaxisrobot.dir/src/robot.cpp.o" \
"CMakeFiles/sixaxisrobot.dir/src/plan.cpp.o" \
"CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.o" \
"CMakeFiles/sixaxisrobot.dir/src/serial.cpp.o"

# External object files for target sixaxisrobot
sixaxisrobot_EXTERNAL_OBJECTS =

sixaxisrobot: CMakeFiles/sixaxisrobot.dir/src/main.cpp.o
sixaxisrobot: CMakeFiles/sixaxisrobot.dir/src/robot.cpp.o
sixaxisrobot: CMakeFiles/sixaxisrobot.dir/src/plan.cpp.o
sixaxisrobot: CMakeFiles/sixaxisrobot.dir/src/planRT.cpp.o
sixaxisrobot: CMakeFiles/sixaxisrobot.dir/src/serial.cpp.o
sixaxisrobot: CMakeFiles/sixaxisrobot.dir/build.make
sixaxisrobot: /usr/kaanh/kaanh-2.4.4.221209/lib/release/libkaanh_lib.so
sixaxisrobot: /usr/aris/aris-2.3.5.221111/lib/release/libaris_lib.so
sixaxisrobot: CMakeFiles/sixaxisrobot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaanh/Desktop/xyn/kaanh-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable sixaxisrobot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sixaxisrobot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sixaxisrobot.dir/build: sixaxisrobot

.PHONY : CMakeFiles/sixaxisrobot.dir/build

CMakeFiles/sixaxisrobot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sixaxisrobot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sixaxisrobot.dir/clean

CMakeFiles/sixaxisrobot.dir/depend:
	cd /home/kaanh/Desktop/xyn/kaanh-test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaanh/Desktop/xyn/kaanh-test /home/kaanh/Desktop/xyn/kaanh-test /home/kaanh/Desktop/xyn/kaanh-test/build /home/kaanh/Desktop/xyn/kaanh-test/build /home/kaanh/Desktop/xyn/kaanh-test/build/CMakeFiles/sixaxisrobot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sixaxisrobot.dir/depend

