# Contents of Makefile
#
# This Makefile is intended to allow one to develop ROS workspace C++ applications
# from the NetBeans Integrated Development Environment (IDE).
#
#
# Andrew Willis
# Senior NRC Research Associate
# July 1, 2015
#
# Some work to try and satisfy NetBeans LD_PRELOAD needs for 'libBuildTrace.so'
# "ERROR: ld.so: object 'libBuildTrace.so' from LD_PRELOAD cannot 
# be preloaded (cannot open shared object file): ignored."
# LD_PRELOAD=""
# LD_LIBRARY_PATH += ~/bin/netbeans-8.0.2/cnd/bin/Linux-x86_64/:~/bin/netbeans-8.0.2/cnd/bin/Linux-x86/
# The IDE use "interception" technology to configure code assistance. 
# You can turn off it in Project Properties->Code Assistance->Use Build 
# Analyzer (but it turns off a lot of "configuring code assistance" 
# functionality). 
# Another way to turn off interceptor (right unset) is unset 
# LD_LIBRARY_PATH and LD_PRELOAD on Linux and Solaris, DYLD_LIBRARY_PATH 
# and DYLD_INSERT_LIBRARIES on Mac in your script (compare with half unset 
# in your script). 

# path to the ROOT folder of the ROS install
ROS_SETUP_ROOT=/opt/ros/indigo

# catkin_make/cmake ENVIRONMENT VARIABLE PARAMETERS (prefix command line with -D${ENVVAR})
SOURCE_DIRECTORY=src
CATKIN_DEVEL_PREFIX=devel
ROS_BUILD_PREFIX=build
ROS_WORKSPACE=ros_ws
CMAKE_INSTALL_PREFIX=install
#CMAKE_INSTALL_PREFIX=${ROS_SETUP_ROOT}

ifeq ($(wildcard nbproject/project.properties),)
    IGNORE := $(shell bash -c "cat ../nbproject/project.properties | \
	grep -E '(ros.root|ros.ws|ros.ws.src|ros.ws.build|ros.ws.devel|ros.ws.install)' | \
	sed 's/ros.root=/ROS_SETUP_ROOT=/' |           \
	sed 's/ros.ws=/ROS_WORKSPACE=/' |              \
	sed 's/ros.ws.src=/SOURCE_DIRECTORY=/' |       \
	sed 's/ros.ws.build=/ROS_BUILD_PREFIX=/' |     \
	sed 's/ros.ws.devel=/CATKIN_DEVEL_PREFIX=/' |  \
	sed 's/ros.ws.install=/CMAKE_INSTALL_PREFIX=/' \
	> Makefile.ros_env") 
    include Makefile.ros_env
endif

# catkin_make command line options
CATKIN_MAKE_FLAGS=--source ./${SOURCE_DIRECTORY}

# to detect newly added packages in previously compiled workspace
#CATKIN_MAKE_FLAGS +=--force-cmake

# if set to "ON" makes Makefile execution verbose in shell
#CMAKE_VERBOSE_MAKEFILE=ON
ifeq ($(CMAKE_VERBOSE_MAKEFILE),ON)
    CATKIN_MAKE_FLAGS += -DCMAKE_VERBOSE_MAKEFILE=${CMAKE_VERBOSE_MAKEFILE}
endif

CMAKE_BUILD_TYPE=Release
#CMAKE_BUILD_TYPE=Development
#CMAKE_BUILD_TYPE=Debug
ifdef CMAKE_BUILD_TYPE
    CATKIN_MAKE_FLAGS += -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
endif


CMAKE_BUILD_FLAGS=../${SOURCE_DIRECTORY} 

# used to install a package into the ros system packages
ifdef CMAKE_INSTALL_PREFIX
    CMAKE_BUILD_FLAGS += -DCMAKE_INSTALL_PREFIX=../${CMAKE_INSTALL_PREFIX}
endif

ifdef CATKIN_DEVEL_PREFIX
    CMAKE_BUILD_FLAGS += -DCATKIN_DEVEL_PREFIX=../${CATKIN_DEVEL_PREFIX}
endif

#ifeq ($(wildcard $(ROS_SETUP_ROOT_BASH)),)
# setup the ROS environment variables -- equivalent to running 'source ${ROS_ROOT}/setup.sh'
IGNORE := $(shell bash -c "source ${ROS_SETUP_ROOT}/setup.sh; env | sed 's/=/:=/' | sed 's/^/export /' > Makefile.ros_env")
include Makefile.ros_env
#endif
#.PHONY : ros_setup ws_setup ws_overlay build devel install all clean

all : build devel
#all: build install
#.PHONY : all
		
build :
	echo ${ROS_SETUP_ROOT}
	echo ${CATKIN_DEVEL_PREFIX}
	#/usr/bin/env
#	${ROS_SETUP_ROOT}/bin/catkin_make -j1 ${CATKIN_MAKE_FLAGS}
	${ROS_SETUP_ROOT}/bin/catkin_make ${CATKIN_MAKE_FLAGS}
	$(MAKE) -C ${ROS_BUILD_PREFIX}
.PHONY : build	
	
build-single :
#	/opt/ros/indigo/bin/catkin_make clean
	catkin_make --pkg ${ROS_PACKAGE_NAME}	

devel : build
	cd ${ROS_BUILD_PREFIX}; \
	cmake ${CMAKE_BUILD_FLAGS}

install : build 
	$(MAKE) -C ${ROS_BUILD_PREFIX} ${CMAKE_INSTALL_PREFIX}
	# equivalent to catkin_make install from workspace directory
	
clean :
#	/opt/ros/indigo/bin/catkin_make clean
	rm -rf ${ROS_BUILD_PREFIX} ${CATKIN_DEVEL_PREFIX} ${CMAKE_INSTALL_PREFIX} Makefile.ros_env Makefile.build_env

clean-single :
#	/opt/ros/indigo/bin/catkin_make clean
	catkin_make --pkg ${ROS_PACKAGE_NAME} --make-args clean


#ws_setup: 
# if src directory does not exist -> mkdir -p src
# if .catkin_workspace does not exist -> cd src; wstool init AND/OR catkin_init_workspace; cd ..
# Toplevel CMakeLists.txt is a link to /opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake
# invoke this to overlay the ROS packages of your workspace onto those from ROS_ROOT
#
#ws_overlay :
#	echo "Running target ws_overlay"
#	IGNORE := $(shell bash -c "source build/setup.sh; env | sed 's/=/:=/' | sed 's/^/export /' > Makefile.build_env")
#        include Makefile.build_env
