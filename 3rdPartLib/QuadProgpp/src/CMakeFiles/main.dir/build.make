# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp

# Include any dependencies generated for this target.
include src/CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/main.dir/flags.make

src/CMakeFiles/main.dir/main.cc.o: src/CMakeFiles/main.dir/flags.make
src/CMakeFiles/main.dir/main.cc.o: src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/main.dir/main.cc.o"
	cd /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cc.o -c /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src/main.cc

src/CMakeFiles/main.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cc.i"
	cd /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src/main.cc > CMakeFiles/main.dir/main.cc.i

src/CMakeFiles/main.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cc.s"
	cd /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src/main.cc -o CMakeFiles/main.dir/main.cc.s

src/CMakeFiles/main.dir/main.cc.o.requires:

.PHONY : src/CMakeFiles/main.dir/main.cc.o.requires

src/CMakeFiles/main.dir/main.cc.o.provides: src/CMakeFiles/main.dir/main.cc.o.requires
	$(MAKE) -f src/CMakeFiles/main.dir/build.make src/CMakeFiles/main.dir/main.cc.o.provides.build
.PHONY : src/CMakeFiles/main.dir/main.cc.o.provides

src/CMakeFiles/main.dir/main.cc.o.provides.build: src/CMakeFiles/main.dir/main.cc.o


# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cc.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

src/main: src/CMakeFiles/main.dir/main.cc.o
src/main: src/CMakeFiles/main.dir/build.make
src/main: src/libquadprog.a
src/main: src/CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable main"
	cd /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/main.dir/build: src/main

.PHONY : src/CMakeFiles/main.dir/build

src/CMakeFiles/main.dir/requires: src/CMakeFiles/main.dir/main.cc.o.requires

.PHONY : src/CMakeFiles/main.dir/requires

src/CMakeFiles/main.dir/clean:
	cd /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src && $(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/main.dir/clean

src/CMakeFiles/main.dir/depend:
	cd /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src /home/jeremy/camel_ws/src/uavcamel/3rdPartLib/QuadProgpp/src/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/main.dir/depend

