# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/erik/lander/Vertical-Lander-Bang-Bang

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/erik/lander/Vertical-Lander-Bang-Bang/build

# Include any dependencies generated for this target.
include CMakeFiles/hop_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hop_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hop_test.dir/flags.make

CMakeFiles/hop_test.dir/src/hop_test.cpp.o: CMakeFiles/hop_test.dir/flags.make
CMakeFiles/hop_test.dir/src/hop_test.cpp.o: ../src/hop_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/erik/lander/Vertical-Lander-Bang-Bang/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hop_test.dir/src/hop_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hop_test.dir/src/hop_test.cpp.o -c /home/erik/lander/Vertical-Lander-Bang-Bang/src/hop_test.cpp

CMakeFiles/hop_test.dir/src/hop_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hop_test.dir/src/hop_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/erik/lander/Vertical-Lander-Bang-Bang/src/hop_test.cpp > CMakeFiles/hop_test.dir/src/hop_test.cpp.i

CMakeFiles/hop_test.dir/src/hop_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hop_test.dir/src/hop_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/erik/lander/Vertical-Lander-Bang-Bang/src/hop_test.cpp -o CMakeFiles/hop_test.dir/src/hop_test.cpp.s

# Object files for target hop_test
hop_test_OBJECTS = \
"CMakeFiles/hop_test.dir/src/hop_test.cpp.o"

# External object files for target hop_test
hop_test_EXTERNAL_OBJECTS =

hop_test: CMakeFiles/hop_test.dir/src/hop_test.cpp.o
hop_test: CMakeFiles/hop_test.dir/build.make
hop_test: librobotcontrol_build/library/librobotics_cape.a
hop_test: CMakeFiles/hop_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/erik/lander/Vertical-Lander-Bang-Bang/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable hop_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hop_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hop_test.dir/build: hop_test

.PHONY : CMakeFiles/hop_test.dir/build

CMakeFiles/hop_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hop_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hop_test.dir/clean

CMakeFiles/hop_test.dir/depend:
	cd /home/erik/lander/Vertical-Lander-Bang-Bang/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/erik/lander/Vertical-Lander-Bang-Bang /home/erik/lander/Vertical-Lander-Bang-Bang /home/erik/lander/Vertical-Lander-Bang-Bang/build /home/erik/lander/Vertical-Lander-Bang-Bang/build /home/erik/lander/Vertical-Lander-Bang-Bang/build/CMakeFiles/hop_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hop_test.dir/depend

