# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/lihao/software/clion-2018.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/lihao/software/clion-2018.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/lihao/workspace/opencv/ChromaTag(video)"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug"

# Include any dependencies generated for this target.
include Utilities/Math/CMakeFiles/Math.dir/depend.make

# Include the progress variables for this target.
include Utilities/Math/CMakeFiles/Math.dir/progress.make

# Include the compile flags for this target's objects.
include Utilities/Math/CMakeFiles/Math.dir/flags.make

Utilities/Math/CMakeFiles/Math.dir/JMD_Math_Common.cpp.o: Utilities/Math/CMakeFiles/Math.dir/flags.make
Utilities/Math/CMakeFiles/Math.dir/JMD_Math_Common.cpp.o: ../Utilities/Math/JMD_Math_Common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Utilities/Math/CMakeFiles/Math.dir/JMD_Math_Common.cpp.o"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Math.dir/JMD_Math_Common.cpp.o -c "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Math/JMD_Math_Common.cpp"

Utilities/Math/CMakeFiles/Math.dir/JMD_Math_Common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Math.dir/JMD_Math_Common.cpp.i"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Math/JMD_Math_Common.cpp" > CMakeFiles/Math.dir/JMD_Math_Common.cpp.i

Utilities/Math/CMakeFiles/Math.dir/JMD_Math_Common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Math.dir/JMD_Math_Common.cpp.s"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Math/JMD_Math_Common.cpp" -o CMakeFiles/Math.dir/JMD_Math_Common.cpp.s

# Object files for target Math
Math_OBJECTS = \
"CMakeFiles/Math.dir/JMD_Math_Common.cpp.o"

# External object files for target Math
Math_EXTERNAL_OBJECTS =

Utilities/Math/libMath.a: Utilities/Math/CMakeFiles/Math.dir/JMD_Math_Common.cpp.o
Utilities/Math/libMath.a: Utilities/Math/CMakeFiles/Math.dir/build.make
Utilities/Math/libMath.a: Utilities/Math/CMakeFiles/Math.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libMath.a"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math" && $(CMAKE_COMMAND) -P CMakeFiles/Math.dir/cmake_clean_target.cmake
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Math.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Utilities/Math/CMakeFiles/Math.dir/build: Utilities/Math/libMath.a

.PHONY : Utilities/Math/CMakeFiles/Math.dir/build

Utilities/Math/CMakeFiles/Math.dir/clean:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math" && $(CMAKE_COMMAND) -P CMakeFiles/Math.dir/cmake_clean.cmake
.PHONY : Utilities/Math/CMakeFiles/Math.dir/clean

Utilities/Math/CMakeFiles/Math.dir/depend:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/lihao/workspace/opencv/ChromaTag(video)" "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Math" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Math/CMakeFiles/Math.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Utilities/Math/CMakeFiles/Math.dir/depend

