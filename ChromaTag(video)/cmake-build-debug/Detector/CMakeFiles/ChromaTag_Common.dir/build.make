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
include Detector/CMakeFiles/ChromaTag_Common.dir/depend.make

# Include the progress variables for this target.
include Detector/CMakeFiles/ChromaTag_Common.dir/progress.make

# Include the compile flags for this target's objects.
include Detector/CMakeFiles/ChromaTag_Common.dir/flags.make

Detector/CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.o: Detector/CMakeFiles/ChromaTag_Common.dir/flags.make
Detector/CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.o: ../Detector/JMD_ChromaTag_Common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Detector/CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.o"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.o -c "/home/lihao/workspace/opencv/ChromaTag(video)/Detector/JMD_ChromaTag_Common.cpp"

Detector/CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.i"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lihao/workspace/opencv/ChromaTag(video)/Detector/JMD_ChromaTag_Common.cpp" > CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.i

Detector/CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.s"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lihao/workspace/opencv/ChromaTag(video)/Detector/JMD_ChromaTag_Common.cpp" -o CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.s

# Object files for target ChromaTag_Common
ChromaTag_Common_OBJECTS = \
"CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.o"

# External object files for target ChromaTag_Common
ChromaTag_Common_EXTERNAL_OBJECTS =

Detector/libChromaTag_Common.a: Detector/CMakeFiles/ChromaTag_Common.dir/JMD_ChromaTag_Common.cpp.o
Detector/libChromaTag_Common.a: Detector/CMakeFiles/ChromaTag_Common.dir/build.make
Detector/libChromaTag_Common.a: Detector/CMakeFiles/ChromaTag_Common.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libChromaTag_Common.a"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && $(CMAKE_COMMAND) -P CMakeFiles/ChromaTag_Common.dir/cmake_clean_target.cmake
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ChromaTag_Common.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Detector/CMakeFiles/ChromaTag_Common.dir/build: Detector/libChromaTag_Common.a

.PHONY : Detector/CMakeFiles/ChromaTag_Common.dir/build

Detector/CMakeFiles/ChromaTag_Common.dir/clean:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && $(CMAKE_COMMAND) -P CMakeFiles/ChromaTag_Common.dir/cmake_clean.cmake
.PHONY : Detector/CMakeFiles/ChromaTag_Common.dir/clean

Detector/CMakeFiles/ChromaTag_Common.dir/depend:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/lihao/workspace/opencv/ChromaTag(video)" "/home/lihao/workspace/opencv/ChromaTag(video)/Detector" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector/CMakeFiles/ChromaTag_Common.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Detector/CMakeFiles/ChromaTag_Common.dir/depend

