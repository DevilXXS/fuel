# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/xinhr/ORB_cloud/src/ORB-SLAM2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xinhr/ORB_cloud/src/ORB-SLAM2/build

# Include any dependencies generated for this target.
include CMakeFiles/mono_tum.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mono_tum.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono_tum.dir/flags.make

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o: CMakeFiles/mono_tum.dir/flags.make
CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o: ../Examples/Monocular/mono_tum.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xinhr/ORB_cloud/src/ORB-SLAM2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o -c /home/xinhr/ORB_cloud/src/ORB-SLAM2/Examples/Monocular/mono_tum.cc

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xinhr/ORB_cloud/src/ORB-SLAM2/Examples/Monocular/mono_tum.cc > CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.i

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xinhr/ORB_cloud/src/ORB-SLAM2/Examples/Monocular/mono_tum.cc -o CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.s

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires:

.PHONY : CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires
	$(MAKE) -f CMakeFiles/mono_tum.dir/build.make CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides.build
.PHONY : CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides.build: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o


# Object files for target mono_tum
mono_tum_OBJECTS = \
"CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o"

# External object files for target mono_tum
mono_tum_EXTERNAL_OBJECTS =

../bin/mono_tum: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o
../bin/mono_tum: CMakeFiles/mono_tum.dir/build.make
../bin/mono_tum: ../lib/libORB_SLAM2.so
../bin/mono_tum: /usr/local/lib/libopencv_dnn.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_highgui.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_ml.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_objdetect.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_shape.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_stitching.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_superres.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_videostab.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_calib3d.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_features2d.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_flann.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_photo.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_video.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_videoio.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_imgcodecs.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_imgproc.so.3.4.16
../bin/mono_tum: /usr/local/lib/libopencv_core.so.3.4.16
../bin/mono_tum: /usr/local/lib/libpangolin.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libGLEW.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../bin/mono_tum: ../Thirdparty/DBoW2/lib/libDBoW2.so
../bin/mono_tum: ../Thirdparty/g2o/lib/libg2o.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/libvtkWrappingTools-6.3.a
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkverdict-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOExport-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOImport-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkVPIC-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingExternal-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeAMR-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libXt.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOXML-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtksys-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_common.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
../bin/mono_tum: /usr/lib/libOpenNI.so
../bin/mono_tum: /usr/lib/libOpenNI2.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libexpat.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libgl2ps.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_io.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_search.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_features.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_people.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_common.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
../bin/mono_tum: /usr/lib/libOpenNI.so
../bin/mono_tum: /usr/lib/libOpenNI2.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libexpat.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libgl2ps.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_io.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_search.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_features.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpcl_people.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libfreetype.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpython2.7.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libpng.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libjpeg.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libtiff.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libproj.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libnetcdf.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libtheoradec.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libogg.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libxml2.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libsz.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libz.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libdl.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/libm.so
../bin/mono_tum: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
../bin/mono_tum: CMakeFiles/mono_tum.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xinhr/ORB_cloud/src/ORB-SLAM2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/mono_tum"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono_tum.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono_tum.dir/build: ../bin/mono_tum

.PHONY : CMakeFiles/mono_tum.dir/build

CMakeFiles/mono_tum.dir/requires: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires

.PHONY : CMakeFiles/mono_tum.dir/requires

CMakeFiles/mono_tum.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono_tum.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono_tum.dir/clean

CMakeFiles/mono_tum.dir/depend:
	cd /home/xinhr/ORB_cloud/src/ORB-SLAM2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xinhr/ORB_cloud/src/ORB-SLAM2 /home/xinhr/ORB_cloud/src/ORB-SLAM2 /home/xinhr/ORB_cloud/src/ORB-SLAM2/build /home/xinhr/ORB_cloud/src/ORB-SLAM2/build /home/xinhr/ORB_cloud/src/ORB-SLAM2/build/CMakeFiles/mono_tum.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono_tum.dir/depend

