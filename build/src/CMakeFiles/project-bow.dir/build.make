# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/huo/Documents/virtual-slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/huo/Documents/virtual-slam/build

# Include any dependencies generated for this target.
include src/CMakeFiles/project-bow.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/project-bow.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/project-bow.dir/flags.make

src/CMakeFiles/project-bow.dir/bow.cpp.o: src/CMakeFiles/project-bow.dir/flags.make
src/CMakeFiles/project-bow.dir/bow.cpp.o: ../src/bow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/huo/Documents/virtual-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/project-bow.dir/bow.cpp.o"
	cd /home/huo/Documents/virtual-slam/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/project-bow.dir/bow.cpp.o -c /home/huo/Documents/virtual-slam/src/bow.cpp

src/CMakeFiles/project-bow.dir/bow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/project-bow.dir/bow.cpp.i"
	cd /home/huo/Documents/virtual-slam/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/huo/Documents/virtual-slam/src/bow.cpp > CMakeFiles/project-bow.dir/bow.cpp.i

src/CMakeFiles/project-bow.dir/bow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/project-bow.dir/bow.cpp.s"
	cd /home/huo/Documents/virtual-slam/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/huo/Documents/virtual-slam/src/bow.cpp -o CMakeFiles/project-bow.dir/bow.cpp.s

src/CMakeFiles/project-bow.dir/bow.cpp.o.requires:

.PHONY : src/CMakeFiles/project-bow.dir/bow.cpp.o.requires

src/CMakeFiles/project-bow.dir/bow.cpp.o.provides: src/CMakeFiles/project-bow.dir/bow.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/project-bow.dir/build.make src/CMakeFiles/project-bow.dir/bow.cpp.o.provides.build
.PHONY : src/CMakeFiles/project-bow.dir/bow.cpp.o.provides

src/CMakeFiles/project-bow.dir/bow.cpp.o.provides.build: src/CMakeFiles/project-bow.dir/bow.cpp.o


src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o: src/CMakeFiles/project-bow.dir/flags.make
src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o: ../src/BagOfWords.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/huo/Documents/virtual-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o"
	cd /home/huo/Documents/virtual-slam/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/project-bow.dir/BagOfWords.cpp.o -c /home/huo/Documents/virtual-slam/src/BagOfWords.cpp

src/CMakeFiles/project-bow.dir/BagOfWords.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/project-bow.dir/BagOfWords.cpp.i"
	cd /home/huo/Documents/virtual-slam/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/huo/Documents/virtual-slam/src/BagOfWords.cpp > CMakeFiles/project-bow.dir/BagOfWords.cpp.i

src/CMakeFiles/project-bow.dir/BagOfWords.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/project-bow.dir/BagOfWords.cpp.s"
	cd /home/huo/Documents/virtual-slam/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/huo/Documents/virtual-slam/src/BagOfWords.cpp -o CMakeFiles/project-bow.dir/BagOfWords.cpp.s

src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.requires:

.PHONY : src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.requires

src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.provides: src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/project-bow.dir/build.make src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.provides.build
.PHONY : src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.provides

src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.provides.build: src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o


# Object files for target project-bow
project__bow_OBJECTS = \
"CMakeFiles/project-bow.dir/bow.cpp.o" \
"CMakeFiles/project-bow.dir/BagOfWords.cpp.o"

# External object files for target project-bow
project__bow_EXTERNAL_OBJECTS =

../bin/project-bow: src/CMakeFiles/project-bow.dir/bow.cpp.o
../bin/project-bow: src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o
../bin/project-bow: src/CMakeFiles/project-bow.dir/build.make
../bin/project-bow: /usr/local/lib/libopencv_viz.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_videostab.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_superres.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_stitching.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_shape.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_photo.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_objdetect.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_calib3d.so.3.1.0
../bin/project-bow: /usr/local/lib/libDBoW3.so
../bin/project-bow: /usr/local/lib/libopencv_features2d.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_ml.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_highgui.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_videoio.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_flann.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_video.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_imgproc.so.3.1.0
../bin/project-bow: /usr/local/lib/libopencv_core.so.3.1.0
../bin/project-bow: src/CMakeFiles/project-bow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/huo/Documents/virtual-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/project-bow"
	cd /home/huo/Documents/virtual-slam/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/project-bow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/project-bow.dir/build: ../bin/project-bow

.PHONY : src/CMakeFiles/project-bow.dir/build

src/CMakeFiles/project-bow.dir/requires: src/CMakeFiles/project-bow.dir/bow.cpp.o.requires
src/CMakeFiles/project-bow.dir/requires: src/CMakeFiles/project-bow.dir/BagOfWords.cpp.o.requires

.PHONY : src/CMakeFiles/project-bow.dir/requires

src/CMakeFiles/project-bow.dir/clean:
	cd /home/huo/Documents/virtual-slam/build/src && $(CMAKE_COMMAND) -P CMakeFiles/project-bow.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/project-bow.dir/clean

src/CMakeFiles/project-bow.dir/depend:
	cd /home/huo/Documents/virtual-slam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/huo/Documents/virtual-slam /home/huo/Documents/virtual-slam/src /home/huo/Documents/virtual-slam/build /home/huo/Documents/virtual-slam/build/src /home/huo/Documents/virtual-slam/build/src/CMakeFiles/project-bow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/project-bow.dir/depend
