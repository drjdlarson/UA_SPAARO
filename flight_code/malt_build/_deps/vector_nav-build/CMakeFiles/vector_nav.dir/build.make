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
include _deps/vector_nav-build/CMakeFiles/vector_nav.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include _deps/vector_nav-build/CMakeFiles/vector_nav.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/vector_nav-build/CMakeFiles/vector_nav.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/vector_nav-build/CMakeFiles/vector_nav.dir/flags.make

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.obj: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/flags.make
_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.obj: _deps/vector_nav-src/src/vn100.cpp
_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.obj: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tuan/Projects/ua_spaaro/flight_code/malt_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.obj"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.obj -MF CMakeFiles/vector_nav.dir/src/vn100.cpp.obj.d -o CMakeFiles/vector_nav.dir/src/vn100.cpp.obj -c /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn100.cpp

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector_nav.dir/src/vn100.cpp.i"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn100.cpp > CMakeFiles/vector_nav.dir/src/vn100.cpp.i

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector_nav.dir/src/vn100.cpp.s"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn100.cpp -o CMakeFiles/vector_nav.dir/src/vn100.cpp.s

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.obj: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/flags.make
_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.obj: _deps/vector_nav-src/src/vn200.cpp
_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.obj: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tuan/Projects/ua_spaaro/flight_code/malt_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.obj"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.obj -MF CMakeFiles/vector_nav.dir/src/vn200.cpp.obj.d -o CMakeFiles/vector_nav.dir/src/vn200.cpp.obj -c /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn200.cpp

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector_nav.dir/src/vn200.cpp.i"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn200.cpp > CMakeFiles/vector_nav.dir/src/vn200.cpp.i

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector_nav.dir/src/vn200.cpp.s"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn200.cpp -o CMakeFiles/vector_nav.dir/src/vn200.cpp.s

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.obj: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/flags.make
_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.obj: _deps/vector_nav-src/src/vn300.cpp
_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.obj: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tuan/Projects/ua_spaaro/flight_code/malt_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.obj"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.obj -MF CMakeFiles/vector_nav.dir/src/vn300.cpp.obj.d -o CMakeFiles/vector_nav.dir/src/vn300.cpp.obj -c /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn300.cpp

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector_nav.dir/src/vn300.cpp.i"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn300.cpp > CMakeFiles/vector_nav.dir/src/vn300.cpp.i

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector_nav.dir/src/vn300.cpp.s"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && /usr/local/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src/src/vn300.cpp -o CMakeFiles/vector_nav.dir/src/vn300.cpp.s

# Object files for target vector_nav
vector_nav_OBJECTS = \
"CMakeFiles/vector_nav.dir/src/vn100.cpp.obj" \
"CMakeFiles/vector_nav.dir/src/vn200.cpp.obj" \
"CMakeFiles/vector_nav.dir/src/vn300.cpp.obj"

# External object files for target vector_nav
vector_nav_EXTERNAL_OBJECTS =

_deps/vector_nav-build/libvector_nav.a: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn100.cpp.obj
_deps/vector_nav-build/libvector_nav.a: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn200.cpp.obj
_deps/vector_nav-build/libvector_nav.a: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/src/vn300.cpp.obj
_deps/vector_nav-build/libvector_nav.a: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/build.make
_deps/vector_nav-build/libvector_nav.a: _deps/vector_nav-build/CMakeFiles/vector_nav.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tuan/Projects/ua_spaaro/flight_code/malt_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libvector_nav.a"
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && $(CMAKE_COMMAND) -P CMakeFiles/vector_nav.dir/cmake_clean_target.cmake
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vector_nav.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/vector_nav-build/CMakeFiles/vector_nav.dir/build: _deps/vector_nav-build/libvector_nav.a
.PHONY : _deps/vector_nav-build/CMakeFiles/vector_nav.dir/build

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/clean:
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build && $(CMAKE_COMMAND) -P CMakeFiles/vector_nav.dir/cmake_clean.cmake
.PHONY : _deps/vector_nav-build/CMakeFiles/vector_nav.dir/clean

_deps/vector_nav-build/CMakeFiles/vector_nav.dir/depend:
	cd /home/tuan/Projects/ua_spaaro/flight_code/malt_build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tuan/Projects/ua_spaaro/flight_code /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-src /home/tuan/Projects/ua_spaaro/flight_code/malt_build /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build /home/tuan/Projects/ua_spaaro/flight_code/malt_build/_deps/vector_nav-build/CMakeFiles/vector_nav.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/vector_nav-build/CMakeFiles/vector_nav.dir/depend

