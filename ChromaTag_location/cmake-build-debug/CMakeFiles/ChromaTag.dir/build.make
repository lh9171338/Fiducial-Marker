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
include CMakeFiles/ChromaTag.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ChromaTag.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ChromaTag.dir/flags.make

CMakeFiles/ChromaTag.dir/main.cpp.o: CMakeFiles/ChromaTag.dir/flags.make
CMakeFiles/ChromaTag.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ChromaTag.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ChromaTag.dir/main.cpp.o -c /home/lihao/workspace/opencv/ChromaTag_location/main.cpp

CMakeFiles/ChromaTag.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ChromaTag.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihao/workspace/opencv/ChromaTag_location/main.cpp > CMakeFiles/ChromaTag.dir/main.cpp.i

CMakeFiles/ChromaTag.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ChromaTag.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihao/workspace/opencv/ChromaTag_location/main.cpp -o CMakeFiles/ChromaTag.dir/main.cpp.s

# Object files for target ChromaTag
ChromaTag_OBJECTS = \
"CMakeFiles/ChromaTag.dir/main.cpp.o"

# External object files for target ChromaTag
ChromaTag_EXTERNAL_OBJECTS =

ChromaTag: CMakeFiles/ChromaTag.dir/main.cpp.o
ChromaTag: CMakeFiles/ChromaTag.dir/build.make
ChromaTag: Detector/libJMD_ChromaTag.a
ChromaTag: Utilities/Common/libWriter.a
ChromaTag: Utilities/Common/libOptions.a
ChromaTag: Utilities/Common/libTimer.a
ChromaTag: Detector/libChromaTag_Detect.a
ChromaTag: Utilities/Math/libMath.a
ChromaTag: Utilities/Vision/libVision_Process.a
ChromaTag: Utilities/Vision/libVision_Line.a
ChromaTag: Utilities/Vision/libVision_Point.a
ChromaTag: Detector/libChromaTag_Decode.a
ChromaTag: Detector/libChromaTag_Pose.a
ChromaTag: Detector/libChromaTag_Common.a
ChromaTag: /usr/local/lib/libopencv_objdetect.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_dnn.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_superres.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_shape.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_stitching.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_ml.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_videostab.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_video.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_photo.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_calib3d.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_features2d.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_flann.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_highgui.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_videoio.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_imgcodecs.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_imgproc.so.3.4.4
ChromaTag: /usr/local/lib/libopencv_core.so.3.4.4
ChromaTag: Utilities/Common/libWriter.a
ChromaTag: Utilities/Common/libObject.a
ChromaTag: CMakeFiles/ChromaTag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ChromaTag"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ChromaTag.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ChromaTag.dir/build: ChromaTag

.PHONY : CMakeFiles/ChromaTag.dir/build

CMakeFiles/ChromaTag.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ChromaTag.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ChromaTag.dir/clean

CMakeFiles/ChromaTag.dir/depend:
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihao/workspace/opencv/ChromaTag_location /home/lihao/workspace/opencv/ChromaTag_location /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/CMakeFiles/ChromaTag.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ChromaTag.dir/depend

