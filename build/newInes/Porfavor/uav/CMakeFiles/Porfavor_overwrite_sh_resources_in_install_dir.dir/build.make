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

# Utility rule file for Porfavor_overwrite_sh_resources_in_install_dir.

# Include any custom commands dependencies for this target.
include newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/compiler_depend.make

# Include the progress variables for this target.
include newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/progress.make

newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir:
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/uav && cp -a /home/nessy/Documents/NewFlairSim/flair-src/newInes/Porfavor/uav/resources/x86_64/*.sh /home/nessy/flair/flair-install/bin/demos/x86_64/Porfavor/

Porfavor_overwrite_sh_resources_in_install_dir: newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir
Porfavor_overwrite_sh_resources_in_install_dir: newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/build.make
.PHONY : Porfavor_overwrite_sh_resources_in_install_dir

# Rule to build all files generated by this target.
newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/build: Porfavor_overwrite_sh_resources_in_install_dir
.PHONY : newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/build

newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/clean:
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/uav && $(CMAKE_COMMAND) -P CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/cmake_clean.cmake
.PHONY : newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/clean

newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/depend:
	cd /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nessy/Documents/NewFlairSim/flair-src /home/nessy/Documents/NewFlairSim/flair-src/newInes/Porfavor/uav /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/uav /home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : newInes/Porfavor/uav/CMakeFiles/Porfavor_overwrite_sh_resources_in_install_dir.dir/depend

