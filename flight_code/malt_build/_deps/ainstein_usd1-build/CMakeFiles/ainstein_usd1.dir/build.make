# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tuan/Projects/ua_spaaro/flight_code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tuan/Projects/ua_spaaro/flight_code/malt_build

# Include any dependencies generated for this target.
include _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/flags.make

_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj: _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/flags.make
_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj: _deps/ainstein_usd1-src/src/ainstein_usd1.cpp
_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj: _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tuan/Projects/ua_spaaro/flight_code/malt_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj -MF CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj.d -o CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj -c /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-src/src/ainstein_usd1.cpp

_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.i"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-src/src/ainstein_usd1.cpp > CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.i

_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.s"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-src/src/ainstein_usd1.cpp -o CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.s

# Object files for target ainstein_usd1
ainstein_usd1_OBJECTS = \
"CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj"

# External object files for target ainstein_usd1
ainstein_usd1_EXTERNAL_OBJECTS =

_deps/ainstein_usd1-build/libainstein_usd1.a: _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/src/ainstein_usd1.cpp.obj
_deps/ainstein_usd1-build/libainstein_usd1.a: _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/build.make
_deps/ainstein_usd1-build/libainstein_usd1.a: _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tuan/Projects/ua_spaaro/flight_code/malt_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libainstein_usd1.a"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build && $(CMAKE_COMMAND) -P CMakeFiles/ainstein_usd1.dir/cmake_clean_target.cmake
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ainstein_usd1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/build: _deps/ainstein_usd1-build/libainstein_usd1.a
.PHONY : _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/build

_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/clean:
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build && $(CMAKE_COMMAND) -P CMakeFiles/ainstein_usd1.dir/cmake_clean.cmake
.PHONY : _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/clean

_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/depend:
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tuan/Projects/ua_spaaro/flight_code /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-src /home/tuan/Projects/ua_spaaro/flight_code/malt_build /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/ainstein_usd1-build/CMakeFiles/ainstein_usd1.dir/depend

