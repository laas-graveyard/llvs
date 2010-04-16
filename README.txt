Read me 
date 04/16/2010

------------------------------------------------------------
ABOUT

This library is a vision tools server. It works with the middleware Corba.


-------------------------------------------------------------
HOW TO BUILD ? 

This project uses cmake to generate the makefile. 
Please, install cmake before starting.

In order to build the project out of the source, make a build dir and move into it
> mkdir build
> cd build

Then launch the cmake command
> ccmake ..

In the ccmake menu
First you have to set to ON the option associated to the lib you want to use.
Then type 'c' to configure. It may happen that some path are not found. Please, fill the corresponding field.Once everything is set up. Type 'c' then 'g' to generate the make files.

Then 
> make 

And if you want to install your lib
> make install


-------------------------------------------------------------------
HOW TO RUN ?

Basically this server detects the camera that are connected to your system. Then it matches the current detected camera with some previously defined vision profiles. Before to start, you need to copy the vision profiles in the same folder as the executables. Existing profiles are stored in LLVS/server/VisionProfiles.

Go to the build folder then
> cp ../server/VisionProfiles/* .
> cp ../scripts/run.sh

Set up the varibles in the run.sh if needed.
Then run the script.


You will read : 

SWidth 640  SHeight 480
libdc1394 error: Invalid feature: in dc1394_feature_get_value (control.c, line 1230): You should use the specific functions to read from multiple-value features
libdc1394 error: Invalid feature: in dc1394_feature_set_value (control.c, line 1256): You should use the specific functions to write from multiple-value features
libdc1394 error: Invalid feature: in dc1394_feature_get_value (control.c, line 1230): You should use the specific functions to read from multiple-value features
libdc1394 error: Invalid feature: in dc1394_feature_get_value (control.c, line 1230): You should use the specific functions to read from multiple-value features
libdc1394 error: Invalid feature: in dc1394_feature_set_value (control.c, line 1256): You should use the specific functions to write from multiple-value features
libdc1394 error: Invalid feature: in dc1394_feature_get_value (control.c, line 1230): You should use the specific functions to read from multiple-value features
libdc1394 error: Error: Failed to allocate iso bandwidth
libdc1394 error: Error: Failed to setup DMA capture
libdc1394 error: Generic failure: in InitializeCameras (/home/dune/devel-src/LLVS/src/dc1394/IEEE1394DCImagesInputMethod.cpp, line 1166): Could not setup camera-
make sure                           that the video mode and framerate                           are
supported by your camera
ConnectionToSot:Mem alloc done.
LowLevelVisionServer:m_NumberOfImagesToStack2
LowLevelVisionServer:COULD ALLOCATE ENOUGH MEMORY for stack
LowLevelVisionServer:COULD ALLOCATE ENOUGH MEMORY for Gyro
LowLevelVisionServer:COULD ALLOCATE ENOUGH MEMORY for Waist Velocity
LowLevelVisionServer:COULD ALLOCATE ENOUGH MEMORY for Waist Orientation
LowLevelVisionServer:Finished

Then ^C will start the Server.


