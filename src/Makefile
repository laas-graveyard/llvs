TARGET = ../server/LowLevelVisionServer
ROBOT=HRP2JRL
all: $(TARGET)

MODULE = LowLevelVisionServer


# Source files needed for the LowLevelVisionServer
SRCS= 	ColorDetection.cpp \
	ImagesInputMethod.cpp \
	SimulatorInputMethod.cpp \
	FileImagesInputMethod.cpp \
	VisionBasicProcess.cpp \
	DisparityProcess.cpp \
	OpticalFlowProcess.cpp \
	MotionEvaluationProcess.cpp \
	Camera_impl.cpp \
	MireDetectionProcess.cpp \
	RectificationProcess.cpp \
	EdgeDetectionProcess.cpp \
	BRepDetectionProcess.cpp \
	SingleCameraSLAMProcess.cpp \
	IEEE1394ImagesInputMethod.cpp \
	FindFeaturesInImage.cpp \
	LowLevelVisionServer.cpp \
	hrp_model_creators.cpp \
	monoslamhrp.cpp \
	models_camera_height.cpp \
	models_waist_velocity.cpp \
	models_threed_gyro.cpp \
	models_orientation.cpp \
	ImageDifference.cpp \
	StereoVision_impl.cpp 


# Header files needed for the LowLevelVisionServer
HRDSSPECS= ColorDetection.h \
	ImagesInputMethod.h \
	FileImagesInputMethod.h \
	VisionBasicProcess.h \
	DisparityProcess.h \
	OpticalFlowProcess.h \
	MotionEvaluationProcess.h \
	RectificationProcess.h \
	LowLevelVisionServer.h \
	MireDetectionProcess.h \
	EdgeDetectionProcess.h \
	BRepDetectionProcess.h \
	SingleCameraSLAMProcess.h \
	FindFeaturesInImage.h\
	Camera_impl.h \
	hrp_model_creators.h \
	monoslamhrp.h \
	models_camera_height.h \
	models_waist_velocity.h \
	models_threed_gyro.h \
	models_orientation.h \
	ImageDifference.h \
	StereoVision_impl.h


SRCDIR=$(PWD)

HRDS= $(HRDSSPECS) \
	visionsensor.h \
	visionsensor_skel.h \
	integrator.h 

TOP =../../../
include $(TOP)Make.rules

# LIBRARIES FLAGS 

#VVVDIR=/mnt/usr_old/local/VVV/
VVVDIR=/usr/local/VVV
#VVVDIR=/home/stasse/src/OpenHRP/HRP2VisionSystem/src/libraries/VVV_bin2/

# VVV libraries
LIBS= -L../../lib -L$(VVVDIR)/lib 

LIBS+= -lGeometry -lImageProcessing

# Andy libraries
LIBS+= -lMonoSLAM  -lSceneImproc -lScene  -lVW -lVNL

# JRL libraries
LIBS+=    -ldisparitymap
#-lcstereo

# VVV libraries
LIBS+=    -lpebutil  -lisoluminance   -lvfgb  -lrange \
	 -lTUIeee1394++ -lTUTools++ -lraw1394  -lepbmtobrep -lbrep -lvvvstereo -lstepone -lcalib -lscm -llcm  -lepbm\
	-lbstereo -lstepfifteen -lvvvstd -lmatutil 

# JRL libraries
LIBS+=  -lmmx 

# Benoit and OpenCV libraries
LIBS+= -lcv -lintervals 

# OpenGL libraries
LIBS+=  -lGL

# COMPILATION FLAGS
#CXX_FLAGS = -c -Wall -DOMNIORB4 -I$(OMNIORB4_DIR)/include -I$(COMMON) -I. -I/usr/local/include -DVERSION=$(VERSION) $(EXTRA_FLAGS)
CXX_FLAGS += -O3 -c  -Wall -Wno-unused -fexceptions -DSTDC_HEADERS

# Flags needed for OMNIORB
ifeq ($(ORB),OMNIORB4)
CXX_FLAGS += -D__OMNIORB4__ -I../stub -D_REENTRANT  -I. -I. -I../include 
CXX_FLAGS += -I/home/stasse/omni_inst/include  -I../../include
CXX_FLAGS += -D__OSVERSION__=2 -D__linux__ -D__x86__  -D__OPENCV__
endif

ifeq ($(ORB),ORBIXE)
CXX_FLAGS += -D__ORBIX__ -I/usr/local/IONA/OBE/2.2/acpp/include/ -I../corba/ORBIX/cpp -D__OPENCV__
endif

# Flags specific to vision processes.
CXX_FLAGS += -I../../include -I$(VVVDIR)/include -I. -I../../include/nScene
CXX_FLAGS += -I/usr/local/include/opencv -DWITH_IEEE1394 
CXX_FLAGS += -I$(OPENHRPHOME)/HRP2VisionSystem/plugin/GetGyroAndAccelerometer/corba/cpp/
CXX_FLAGS += -I$(OPENHRPHOME)/VisualServoing/plugin/corba/ORBIX/cpp/
CXX_FLAGS += -D_ISOC99_SOURCE -D_BSD_SOURCE -D_SVID_SOURCE

ifeq ($(ORB), ORBIXE)
OBJS = ./LowLevelVisionSystem.o  ./MainEntryPoint.o\
	./common.o  ./LowLevelVisionSystem_skel.o \
	./BRep.o ./BRep_skel.o \
	./Scene.o ./Scene_skel.o \
	./StereoVision.o ./StereoVision_skel.o \
	./VisualServoingServer.o \
	visionsensor.o \

else
OBJS = ./LowLevelVisionSytem.o \
	../corba/common.o .

endif

#OBJS_LLVS= ImagesInputMethod.o    \
#	FileImagesInputMethod.o   \
#	IEEE1394ImagesInputMethod.o \
#	DisparityProcess.o        \
#	VisionBasicProcess.o      \
#	OpticalFlowProcess.o      \
#	SimulatorInputMethod.o    \
#	RectificationProcess.o    \
#	MireDetectionProcess.o \
#	EdgeDetectionProcess.o \
#	BRepDetectionProcess.o \
#	SingleCameraSLAMProcess.cpp \
#	Camera_impl.o \
#	MotionEvaluationProcess.o
OBJS_LLVS = ${SRCS:.cpp=.o}

ifeq ($(ORB),OMNIORB4)

common.cpp common.h : $(COMMON_DIR)/common.idl
	$(OB_OMNIORB4_IDLC) $(OMNIORB4_OB_IDL_FLAGS) $<
endif

ifeq ($(ORB),OMNIORB4)
.cpp.o:
	$(CXX) $(CXX_FLAGS) -o $@ $<
endif

testFileImagesInputMethod: testFileImagesInputMethod.o FileImagesInputMethod.o
	$(LINK) -o $@ $< ImagesInputMethod.o FileImagesInputMethod.o

ifeq ($(ORB),OMNIORB4)
LowLevelVisionServer: LowLevelVisionServer.o LowLevelVisionSystem.o common.o $(OBJS_LLVS)
	$(LINK) -L. -L$(OB_LIB_DIR) -L/usr/local/lib -ldl -rdynamic    \
	 -o $@ $< LowLevelVisionSystem.o common.o GetGyroAndAccelerometer.o $(OBJS_LLVS) $(OB_LIBS) $(LIBS) -lpebutil
endif

ifeq ($(ORB),ORBIXE)
LINK=$(CXX) -L../../lib -ldl
$(TARGET): $(OBJS) $(OBJS_LLVS) $(SERVER_OBJS)
	if [ ! -e ../server ]; then mkdir ../server; fi
	$(LINK)  $(LINK_FLAGS) -Wall -o $(TARGET) $(OBJS) GetGyroAndAccelerometer.o  $(OBJS_LLVS)\
	-L../include -I../include -L$(OB_LIB_DIR) $(OB_LIBS)   $(SERVER_OBJS)  $(LINK_LIBS)  $(LIBS) 

#  ../obj/kcpit.o ../obj/corbaClient.o \

test.o: test.cpp
	$(CXX) $(CXX_FLAGS) $<

test: test.o
	$(LINK)  $(LINK_FLAGS) -Wall -o test -L$(IOSRVBIN) -L$(DYNAMICS)/server \
	-L../include -I../include -L$(OB_LIB_DIR) $(OB_LIBS) test.o GetGyroAndAccelerometer.o -lbodyinfo -lhrpModel -lplugin
#../obj/kcpit.o ../obj/corbaClient.o

LowLevelVisionSystem.o: ../corba/ORBIX/cpp/LowLevelVisionSystem.cpp common.h visionsensor.h GetGyroAndAccelerometer.o
	$(CXX) $(CXX_FLAGS) $<

LowLevelVisionSystem_skel.o: ../corba/ORBIX/cpp/LowLevelVisionSystem_skel.cpp common_skel.h
	$(CXX) $(CXX_FLAGS) $<

BRep.o: ../corba/ORBIX/cpp/BRep.cpp
	$(CXX) $(CXX_FLAGS) $<

BRep_skel.o: ../corba/ORBIX/cpp/BRep_skel.cpp common_skel.h
	$(CXX) $(CXX_FLAGS) $<

Scene.o: ../corba/ORBIX/cpp/Scene.cpp
	$(CXX) $(CXX_FLAGS) $<

Scene_skel.o: ../corba/ORBIX/cpp/Scene_skel.cpp common_skel.h
	$(CXX) $(CXX_FLAGS) $<

StereoVision.o: ../corba/ORBIX/cpp/StereoVision.cpp
	rm $(SRCDIR)/StereoVision.h
	$(CXX) $(CXX_FLAGS) $<

StereoVision_skel.o: ../corba/ORBIX/cpp/StereoVision_skel.cpp common_skel.h
	$(CXX) $(CXX_FLAGS) $<

#visionsensor.o: ../corba/ORBIX/visionsensor.cpp
#	$(CXX) $(CXX_FLAGS) $<

#visionsensor_skel.o: ../corba/ORBIX/visionsensor_skel.cpp
#	$(CXX) $(CXX_FLAGS) $<

GetGyroAndAccelerometer.o: $(OPENHRPHOME)/HRP2VisionSystem/plugin/GetGyroAndAccelerometer/corba/cpp/GetGyroAndAccelerometer.cpp
	$(CXX) $(CXX_FLAGS) $<

VisualServoingServer.o: $(OPENHRPHOME)/VisualServoing/plugin/corba/ORBIX/cpp/VisualServoingServer.cpp
	$(CXX) $(CXX_FLAGS) $<

#LowLevelVisionServer: LowLevelVisionServer.o LowLevelVisionSystem.o common.o $(OBJS_LLVS)
#	$(LINK) $(LINK_FLAGS) -rdynamic -o $@ $^ $(OB_LIBS)
endif

#LowLevelVisionSystem.o: LowLevelVisionSystem.cpp common.h  visionsensor.h integrator.h LowLevelVisionSystem.h
#	$(CXX) $(CXX_FLAGS) -I../corba $<

#LowLevelVisionServer.o : LowLevelVisionServer.cpp common.h LowLevelVisionSystem.h
#	$(CXX) $(CXX_FLAGS) -I../corba $<	

clean: del_generic
	-rm -f $(TARGET) 

clean-all:
	rm -f *.o
	rm -f *~
#common.h visionsensor.h visionsensor.cpp common_skel.h LowLevelVisionSystem_skel.h LowLevelVisionSystem.h

server: $(OBJS)

doc:	$(SRCS) $(HDRSSPECS)
	doxygen Doxyfile


