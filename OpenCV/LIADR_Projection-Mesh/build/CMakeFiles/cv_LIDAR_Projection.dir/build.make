# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/build

# Include any dependencies generated for this target.
include CMakeFiles/cv_LIDAR_Projection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cv_LIDAR_Projection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cv_LIDAR_Projection.dir/flags.make

CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o: CMakeFiles/cv_LIDAR_Projection.dir/flags.make
CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o: ../LIDAR_Projection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o -c /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/LIDAR_Projection.cpp

CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/LIDAR_Projection.cpp > CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.i

CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/LIDAR_Projection.cpp -o CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.s

CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.requires:
.PHONY : CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.requires

CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.provides: CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.requires
	$(MAKE) -f CMakeFiles/cv_LIDAR_Projection.dir/build.make CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.provides.build
.PHONY : CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.provides

CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.provides.build: CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o

# Object files for target cv_LIDAR_Projection
cv_LIDAR_Projection_OBJECTS = \
"CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o"

# External object files for target cv_LIDAR_Projection
cv_LIDAR_Projection_EXTERNAL_OBJECTS =

cv_LIDAR_Projection: CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o
cv_LIDAR_Projection: CMakeFiles/cv_LIDAR_Projection.dir/build.make
cv_LIDAR_Projection: /usr/local/lib/libopencv_videostab.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_videoio.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_video.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_superres.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_stitching.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_shape.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_photo.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_objdetect.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_ml.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_imgproc.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_highgui.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_flann.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_features2d.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudev.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudawarping.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudastereo.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudaoptflow.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudaobjdetect.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudalegacy.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudaimgproc.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudafilters.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudafeatures2d.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudacodec.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudabgsegm.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudaarithm.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_core.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_calib3d.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudawarping.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_objdetect.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudafilters.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudaarithm.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_features2d.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_ml.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_highgui.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_videoio.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_flann.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_video.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_imgproc.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_core.so.3.1.0
cv_LIDAR_Projection: /usr/local/lib/libopencv_cudev.so.3.1.0
cv_LIDAR_Projection: CMakeFiles/cv_LIDAR_Projection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable cv_LIDAR_Projection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cv_LIDAR_Projection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cv_LIDAR_Projection.dir/build: cv_LIDAR_Projection
.PHONY : CMakeFiles/cv_LIDAR_Projection.dir/build

CMakeFiles/cv_LIDAR_Projection.dir/requires: CMakeFiles/cv_LIDAR_Projection.dir/LIDAR_Projection.cpp.o.requires
.PHONY : CMakeFiles/cv_LIDAR_Projection.dir/requires

CMakeFiles/cv_LIDAR_Projection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cv_LIDAR_Projection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cv_LIDAR_Projection.dir/clean

CMakeFiles/cv_LIDAR_Projection.dir/depend:
	cd /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/build /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/build /home/ubuntu/GPU_LIDAR_3DSensing/OpenCV/LIADR_Projection-Mesh/build/CMakeFiles/cv_LIDAR_Projection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cv_LIDAR_Projection.dir/depend

