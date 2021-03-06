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
CMAKE_SOURCE_DIR = /home/lihao/workspace/opencv/ChromaTag_location

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug

# Include any dependencies generated for this target.
include Detector/CMakeFiles/ChromaTag_Decode.dir/depend.make

# Include the progress variables for this target.
include Detector/CMakeFiles/ChromaTag_Decode.dir/progress.make

# Include the compile flags for this target's objects.
include Detector/CMakeFiles/ChromaTag_Decode.dir/flags.make

Detector/CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.o: Detector/CMakeFiles/ChromaTag_Decode.dir/flags.make
Detector/CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.o: ../Detector/JMD_ChromaTag_Decode.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Detector/CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.o"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.o -c /home/lihao/workspace/opencv/ChromaTag_location/Detector/JMD_ChromaTag_Decode.cpp

Detector/CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.i"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihao/workspace/opencv/ChromaTag_location/Detector/JMD_ChromaTag_Decode.cpp > CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.i

Detector/CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.s"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihao/workspace/opencv/ChromaTag_location/Detector/JMD_ChromaTag_Decode.cpp -o CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.s

# Object files for target ChromaTag_Decode
ChromaTag_Decode_OBJECTS = \
"CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.o"

# External object files for target ChromaTag_Decode
ChromaTag_Decode_EXTERNAL_OBJECTS =

Detector/libChromaTag_Decode.a: Detector/CMakeFiles/ChromaTag_Decode.dir/JMD_ChromaTag_Decode.cpp.o
Detector/libChromaTag_Decode.a: Detector/CMakeFiles/ChromaTag_Decode.dir/build.make
Detector/libChromaTag_Decode.a: Detector/CMakeFiles/ChromaTag_Decode.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libChromaTag_Decode.a"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector && $(CMAKE_COMMAND) -P CMakeFiles/ChromaTag_Decode.dir/cmake_clean_target.cmake
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ChromaTag_Decode.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Detector/CMakeFiles/ChromaTag_Decode.dir/build: Detector/libChromaTag_Decode.a

.PHONY : Detector/CMakeFiles/ChromaTag_Decode.dir/build

Detector/CMakeFiles/ChromaTag_Decode.dir/clean:
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector && $(CMAKE_COMMAND) -P CMakeFiles/ChromaTag_Decode.dir/cmake_clean.cmake
.PHONY : Detector/CMakeFiles/ChromaTag_Decode.dir/clean

Detector/CMakeFiles/ChromaTag_Decode.dir/depend:
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihao/workspace/opencv/ChromaTag_location /home/lihao/workspace/opencv/ChromaTag_location/Detector /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Detector/CMakeFiles/ChromaTag_Decode.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Detector/CMakeFiles/ChromaTag_Decode.dir/depend

