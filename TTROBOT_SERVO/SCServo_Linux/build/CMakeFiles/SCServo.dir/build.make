# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nvidia/TTROBOT_SERVO/SCServo_Linux

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/TTROBOT_SERVO/SCServo_Linux/build

# Include any dependencies generated for this target.
include CMakeFiles/SCServo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/SCServo.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SCServo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SCServo.dir/flags.make

CMakeFiles/SCServo.dir/SCS.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/SCS.cpp.o: ../SCS.cpp
CMakeFiles/SCServo.dir/SCS.cpp.o: CMakeFiles/SCServo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SCServo.dir/SCS.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SCServo.dir/SCS.cpp.o -MF CMakeFiles/SCServo.dir/SCS.cpp.o.d -o CMakeFiles/SCServo.dir/SCS.cpp.o -c /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCS.cpp

CMakeFiles/SCServo.dir/SCS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/SCS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCS.cpp > CMakeFiles/SCServo.dir/SCS.cpp.i

CMakeFiles/SCServo.dir/SCS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/SCS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCS.cpp -o CMakeFiles/SCServo.dir/SCS.cpp.s

CMakeFiles/SCServo.dir/SCSCL.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/SCSCL.cpp.o: ../SCSCL.cpp
CMakeFiles/SCServo.dir/SCSCL.cpp.o: CMakeFiles/SCServo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/SCServo.dir/SCSCL.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SCServo.dir/SCSCL.cpp.o -MF CMakeFiles/SCServo.dir/SCSCL.cpp.o.d -o CMakeFiles/SCServo.dir/SCSCL.cpp.o -c /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCSCL.cpp

CMakeFiles/SCServo.dir/SCSCL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/SCSCL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCSCL.cpp > CMakeFiles/SCServo.dir/SCSCL.cpp.i

CMakeFiles/SCServo.dir/SCSCL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/SCSCL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCSCL.cpp -o CMakeFiles/SCServo.dir/SCSCL.cpp.s

CMakeFiles/SCServo.dir/SCSerial.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/SCSerial.cpp.o: ../SCSerial.cpp
CMakeFiles/SCServo.dir/SCSerial.cpp.o: CMakeFiles/SCServo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/SCServo.dir/SCSerial.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SCServo.dir/SCSerial.cpp.o -MF CMakeFiles/SCServo.dir/SCSerial.cpp.o.d -o CMakeFiles/SCServo.dir/SCSerial.cpp.o -c /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCSerial.cpp

CMakeFiles/SCServo.dir/SCSerial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/SCSerial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCSerial.cpp > CMakeFiles/SCServo.dir/SCSerial.cpp.i

CMakeFiles/SCServo.dir/SCSerial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/SCSerial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SCSerial.cpp -o CMakeFiles/SCServo.dir/SCSerial.cpp.s

CMakeFiles/SCServo.dir/SMSBL.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/SMSBL.cpp.o: ../SMSBL.cpp
CMakeFiles/SCServo.dir/SMSBL.cpp.o: CMakeFiles/SCServo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/SCServo.dir/SMSBL.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SCServo.dir/SMSBL.cpp.o -MF CMakeFiles/SCServo.dir/SMSBL.cpp.o.d -o CMakeFiles/SCServo.dir/SMSBL.cpp.o -c /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMSBL.cpp

CMakeFiles/SCServo.dir/SMSBL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/SMSBL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMSBL.cpp > CMakeFiles/SCServo.dir/SMSBL.cpp.i

CMakeFiles/SCServo.dir/SMSBL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/SMSBL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMSBL.cpp -o CMakeFiles/SCServo.dir/SMSBL.cpp.s

CMakeFiles/SCServo.dir/SMSCL.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/SMSCL.cpp.o: ../SMSCL.cpp
CMakeFiles/SCServo.dir/SMSCL.cpp.o: CMakeFiles/SCServo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/SCServo.dir/SMSCL.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SCServo.dir/SMSCL.cpp.o -MF CMakeFiles/SCServo.dir/SMSCL.cpp.o.d -o CMakeFiles/SCServo.dir/SMSCL.cpp.o -c /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMSCL.cpp

CMakeFiles/SCServo.dir/SMSCL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/SMSCL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMSCL.cpp > CMakeFiles/SCServo.dir/SMSCL.cpp.i

CMakeFiles/SCServo.dir/SMSCL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/SMSCL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMSCL.cpp -o CMakeFiles/SCServo.dir/SMSCL.cpp.s

CMakeFiles/SCServo.dir/SMS_STS.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/SMS_STS.cpp.o: ../SMS_STS.cpp
CMakeFiles/SCServo.dir/SMS_STS.cpp.o: CMakeFiles/SCServo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/SCServo.dir/SMS_STS.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SCServo.dir/SMS_STS.cpp.o -MF CMakeFiles/SCServo.dir/SMS_STS.cpp.o.d -o CMakeFiles/SCServo.dir/SMS_STS.cpp.o -c /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMS_STS.cpp

CMakeFiles/SCServo.dir/SMS_STS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/SMS_STS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMS_STS.cpp > CMakeFiles/SCServo.dir/SMS_STS.cpp.i

CMakeFiles/SCServo.dir/SMS_STS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/SMS_STS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/TTROBOT_SERVO/SCServo_Linux/SMS_STS.cpp -o CMakeFiles/SCServo.dir/SMS_STS.cpp.s

# Object files for target SCServo
SCServo_OBJECTS = \
"CMakeFiles/SCServo.dir/SCS.cpp.o" \
"CMakeFiles/SCServo.dir/SCSCL.cpp.o" \
"CMakeFiles/SCServo.dir/SCSerial.cpp.o" \
"CMakeFiles/SCServo.dir/SMSBL.cpp.o" \
"CMakeFiles/SCServo.dir/SMSCL.cpp.o" \
"CMakeFiles/SCServo.dir/SMS_STS.cpp.o"

# External object files for target SCServo
SCServo_EXTERNAL_OBJECTS =

libSCServo.so: CMakeFiles/SCServo.dir/SCS.cpp.o
libSCServo.so: CMakeFiles/SCServo.dir/SCSCL.cpp.o
libSCServo.so: CMakeFiles/SCServo.dir/SCSerial.cpp.o
libSCServo.so: CMakeFiles/SCServo.dir/SMSBL.cpp.o
libSCServo.so: CMakeFiles/SCServo.dir/SMSCL.cpp.o
libSCServo.so: CMakeFiles/SCServo.dir/SMS_STS.cpp.o
libSCServo.so: CMakeFiles/SCServo.dir/build.make
libSCServo.so: CMakeFiles/SCServo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library libSCServo.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SCServo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SCServo.dir/build: libSCServo.so
.PHONY : CMakeFiles/SCServo.dir/build

CMakeFiles/SCServo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SCServo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SCServo.dir/clean

CMakeFiles/SCServo.dir/depend:
	cd /home/nvidia/TTROBOT_SERVO/SCServo_Linux/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/TTROBOT_SERVO/SCServo_Linux /home/nvidia/TTROBOT_SERVO/SCServo_Linux /home/nvidia/TTROBOT_SERVO/SCServo_Linux/build /home/nvidia/TTROBOT_SERVO/SCServo_Linux/build /home/nvidia/TTROBOT_SERVO/SCServo_Linux/build/CMakeFiles/SCServo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SCServo.dir/depend

