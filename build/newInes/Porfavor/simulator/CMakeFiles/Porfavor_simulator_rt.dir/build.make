# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_SOURCE_DIR = /home/nessy/Documents/NewFlairSim/flair-src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build

# Include any dependencies generated for this target.
include newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/compiler_depend.make

# Include the progress variables for this target.
include newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/progress.make

# Include the compile flags for this target's objects.
include newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/flags.make

newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o: newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/flags.make
newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o: /home/nessy/Documents/NewFlairSim/flair-src/newInes/Porfavor/simulator/src/main.cpp
newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o: newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o"
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/simulator && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o -MF CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o.d -o CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o -c /home/nessy/Documents/NewFlairSim/flair-src/newInes/Porfavor/simulator/src/main.cpp

newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.i"
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/simulator && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nessy/Documents/NewFlairSim/flair-src/newInes/Porfavor/simulator/src/main.cpp > CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.i

newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.s"
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/simulator && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nessy/Documents/NewFlairSim/flair-src/newInes/Porfavor/simulator/src/main.cpp -o CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.s

# Object files for target Porfavor_simulator_rt
Porfavor_simulator_rt_OBJECTS = \
"CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o"

# External object files for target Porfavor_simulator_rt
Porfavor_simulator_rt_EXTERNAL_OBJECTS =

newInes/Porfavor/simulator/Porfavor_simulator_rt: newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/src/main.cpp.o
newInes/Porfavor/simulator/Porfavor_simulator_rt: newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/build.make
newInes/Porfavor/simulator/Porfavor_simulator_rt: /home/nessy/flair/flair-install/lib/x86_64/libFlairSimulator_gl.a
newInes/Porfavor/simulator/Porfavor_simulator_rt: /home/nessy/flair/flair-install/lib/x86_64/libFlairCore_rt.a
newInes/Porfavor/simulator/Porfavor_simulator_rt: /usr/lib/libxml2.so
newInes/Porfavor/simulator/Porfavor_simulator_rt: newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Porfavor_simulator_rt"
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/simulator && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Porfavor_simulator_rt.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "Porfavor_simulator_rt built for  architecture"

# Rule to build all files generated by this target.
newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/build: newInes/Porfavor/simulator/Porfavor_simulator_rt
.PHONY : newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/build

newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/clean:
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/simulator && $(CMAKE_COMMAND) -P CMakeFiles/Porfavor_simulator_rt.dir/cmake_clean.cmake
.PHONY : newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/clean

newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/depend:
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nessy/Documents/NewFlairSim/flair-src /home/nessy/Documents/NewFlairSim/flair-src/newInes/Porfavor/simulator /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/simulator /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : newInes/Porfavor/simulator/CMakeFiles/Porfavor_simulator_rt.dir/depend

