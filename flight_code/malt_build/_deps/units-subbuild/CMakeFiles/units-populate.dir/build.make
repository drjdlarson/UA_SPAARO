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
CMAKE_SOURCE_DIR = /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild

# Utility rule file for units-populate.

# Include any custom commands dependencies for this target.
include CMakeFiles/units-populate.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/units-populate.dir/progress.make

CMakeFiles/units-populate: CMakeFiles/units-populate-complete

CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-install
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-mkdir
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-download
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-update
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-patch
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-configure
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-build
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-install
CMakeFiles/units-populate-complete: units-populate-prefix/src/units-populate-stamp/units-populate-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'units-populate'"
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles
	/usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles/units-populate-complete
	/usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-done

units-populate-prefix/src/units-populate-stamp/units-populate-update:
.PHONY : units-populate-prefix/src/units-populate-stamp/units-populate-update

units-populate-prefix/src/units-populate-stamp/units-populate-build: units-populate-prefix/src/units-populate-stamp/units-populate-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No build step for 'units-populate'"
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E echo_append
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-build

units-populate-prefix/src/units-populate-stamp/units-populate-configure: units-populate-prefix/tmp/units-populate-cfgcmd.txt
units-populate-prefix/src/units-populate-stamp/units-populate-configure: units-populate-prefix/src/units-populate-stamp/units-populate-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "No configure step for 'units-populate'"
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E echo_append
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-configure

units-populate-prefix/src/units-populate-stamp/units-populate-download: units-populate-prefix/src/units-populate-stamp/units-populate-gitinfo.txt
units-populate-prefix/src/units-populate-stamp/units-populate-download: units-populate-prefix/src/units-populate-stamp/units-populate-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'units-populate'"
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps && /usr/local/bin/cmake -P /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/tmp/units-populate-gitclone.cmake
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps && /usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-download

units-populate-prefix/src/units-populate-stamp/units-populate-install: units-populate-prefix/src/units-populate-stamp/units-populate-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'units-populate'"
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E echo_append
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-install

units-populate-prefix/src/units-populate-stamp/units-populate-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'units-populate'"
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-src
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/tmp
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src
	/usr/local/bin/cmake -E make_directory /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp
	/usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-mkdir

units-populate-prefix/src/units-populate-stamp/units-populate-patch: units-populate-prefix/src/units-populate-stamp/units-populate-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'units-populate'"
	/usr/local/bin/cmake -E echo_append
	/usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-patch

units-populate-prefix/src/units-populate-stamp/units-populate-update:
.PHONY : units-populate-prefix/src/units-populate-stamp/units-populate-update

units-populate-prefix/src/units-populate-stamp/units-populate-test: units-populate-prefix/src/units-populate-stamp/units-populate-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No test step for 'units-populate'"
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E echo_append
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-build && /usr/local/bin/cmake -E touch /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/src/units-populate-stamp/units-populate-test

units-populate-prefix/src/units-populate-stamp/units-populate-update: units-populate-prefix/src/units-populate-stamp/units-populate-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Performing update step for 'units-populate'"
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-src && /usr/local/bin/cmake -P /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/units-populate-prefix/tmp/units-populate-gitupdate.cmake

units-populate: CMakeFiles/units-populate
units-populate: CMakeFiles/units-populate-complete
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-build
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-configure
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-download
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-install
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-mkdir
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-patch
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-test
units-populate: units-populate-prefix/src/units-populate-stamp/units-populate-update
units-populate: CMakeFiles/units-populate.dir/build.make
.PHONY : units-populate

# Rule to build all files generated by this target.
CMakeFiles/units-populate.dir/build: units-populate
.PHONY : CMakeFiles/units-populate.dir/build

CMakeFiles/units-populate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/units-populate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/units-populate.dir/clean

CMakeFiles/units-populate.dir/depend:
	cd /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild /home/tuan/Projects/super/ua_spaaro/flight_code/malt_build/_deps/units-subbuild/CMakeFiles/units-populate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/units-populate.dir/depend

