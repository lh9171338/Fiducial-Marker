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
include Utilities/Common/CMakeFiles/Options.dir/depend.make

# Include the progress variables for this target.
include Utilities/Common/CMakeFiles/Options.dir/progress.make

# Include the compile flags for this target's objects.
include Utilities/Common/CMakeFiles/Options.dir/flags.make

Utilities/Common/CMakeFiles/Options.dir/JMD_Utils_Options.cpp.o: Utilities/Common/CMakeFiles/Options.dir/flags.make
Utilities/Common/CMakeFiles/Options.dir/JMD_Utils_Options.cpp.o: ../Utilities/Common/JMD_Utils_Options.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Utilities/Common/CMakeFiles/Options.dir/JMD_Utils_Options.cpp.o"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Options.dir/JMD_Utils_Options.cpp.o -c "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Common/JMD_Utils_Options.cpp"

Utilities/Common/CMakeFiles/Options.dir/JMD_Utils_Options.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Options.dir/JMD_Utils_Options.cpp.i"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Common/JMD_Utils_Options.cpp" > CMakeFiles/Options.dir/JMD_Utils_Options.cpp.i

Utilities/Common/CMakeFiles/Options.dir/JMD_Utils_Options.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Options.dir/JMD_Utils_Options.cpp.s"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Common/JMD_Utils_Options.cpp" -o CMakeFiles/Options.dir/JMD_Utils_Options.cpp.s

# Object files for target Options
Options_OBJECTS = \
"CMakeFiles/Options.dir/JMD_Utils_Options.cpp.o"

# External object files for target Options
Options_EXTERNAL_OBJECTS =

Utilities/Common/libOptions.a: Utilities/Common/CMakeFiles/Options.dir/JMD_Utils_Options.cpp.o
Utilities/Common/libOptions.a: Utilities/Common/CMakeFiles/Options.dir/build.make
Utilities/Common/libOptions.a: Utilities/Common/CMakeFiles/Options.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libOptions.a"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common" && $(CMAKE_COMMAND) -P CMakeFiles/Options.dir/cmake_clean_target.cmake
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Options.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Utilities/Common/CMakeFiles/Options.dir/build: Utilities/Common/libOptions.a

.PHONY : Utilities/Common/CMakeFiles/Options.dir/build

Utilities/Common/CMakeFiles/Options.dir/clean:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common" && $(CMAKE_COMMAND) -P CMakeFiles/Options.dir/cmake_clean.cmake
.PHONY : Utilities/Common/CMakeFiles/Options.dir/clean

Utilities/Common/CMakeFiles/Options.dir/depend:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/lihao/workspace/opencv/ChromaTag(video)" "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Common" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Common/CMakeFiles/Options.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Utilities/Common/CMakeFiles/Options.dir/depend

