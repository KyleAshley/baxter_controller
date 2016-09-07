#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_DLIB_EXT=so
CND_CONF=Release
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/src/InertiaCalculations.o \
	${OBJECTDIR}/src/Minimization.o \
	${OBJECTDIR}/src/OPEMain.o \
	${OBJECTDIR}/src/OPESettings.o \
	${OBJECTDIR}/src/OPEUtils.o \
	${OBJECTDIR}/src/ObjectPoseEstimator.o \
	${OBJECTDIR}/src/Plane.o \
	${OBJECTDIR}/src/PointCloudCapture.o \
	${OBJECTDIR}/src/SQFitting.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lboost_system -lboost_thread -lOpenNI -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_visualization -lvtkCommon -lvtkFiltering -lvtkHybrid -lvtkRendering

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk bin/${CND_CONF}/${CND_PLATFORM}/ope-new

bin/${CND_CONF}/${CND_PLATFORM}/ope-new: ${OBJECTFILES}
	${MKDIR} -p bin/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o bin/${CND_CONF}/${CND_PLATFORM}/ope-new ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/src/InertiaCalculations.o: src/InertiaCalculations.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/InertiaCalculations.o src/InertiaCalculations.cpp

${OBJECTDIR}/src/Minimization.o: src/Minimization.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Minimization.o src/Minimization.cpp

${OBJECTDIR}/src/OPEMain.o: src/OPEMain.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/OPEMain.o src/OPEMain.cpp

${OBJECTDIR}/src/OPESettings.o: src/OPESettings.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/OPESettings.o src/OPESettings.cpp

${OBJECTDIR}/src/OPEUtils.o: src/OPEUtils.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/OPEUtils.o src/OPEUtils.cpp

${OBJECTDIR}/src/ObjectPoseEstimator.o: src/ObjectPoseEstimator.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/ObjectPoseEstimator.o src/ObjectPoseEstimator.cpp

${OBJECTDIR}/src/Plane.o: src/Plane.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Plane.o src/Plane.cpp

${OBJECTDIR}/src/PointCloudCapture.o: src/PointCloudCapture.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/PointCloudCapture.o src/PointCloudCapture.cpp

${OBJECTDIR}/src/SQFitting.o: src/SQFitting.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -Iinclude -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/SQFitting.o src/SQFitting.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} bin/${CND_CONF}/${CND_PLATFORM}/ope-new

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
