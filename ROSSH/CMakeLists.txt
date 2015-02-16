cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
find_package(cmake_modules REQUIRED)
find_package(cmake_modules REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(dccmsg src/cpp/dccmsg.cpp)
#add_library(dccmsg src/cpp/dccmsg.cpp)
#target_link_libraries(${PROJECT_NAME} serialcoms)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#add_classpath(/usr/share/java/RXTXcomm.jar)
#add_runtime_classpath(/usr/share/java/RXTXcomm.jar)
rosbuild_add_executable(dcc_tcp src/cpp/dcc_tcp.cpp)
target_link_libraries(dcc_tcp network)

rosbuild_add_executable(DCCSH src/cpp/sigdog.cpp)
target_link_libraries(DCCSH dccmsg)

####################  NEW EXECUTABLES  ######################################
########rosbuild_add_executable(ScreenPrinter src/cpp/receiver.cpp)
########target_link_libraries(ScreenPrinter dccmsg)
#rosbuild_add_executable(RoboDCC src/cpp/split.cpp)
#target_link_libraries(RoboDCC dccmsg)

####################  OLD, USEFUL EXECUTABLES  ##############################
########rosbuild_add_executable(History src/cpp/sampler.cpp)
########target_link_libraries(DCC dccmsg)
########rosbuild_add_executable(FilteredHistory src/cpp/filteredsampler.cpp)
########target_link_libraries(DCC dccmsg)
#rosbuild_add_executable(setfreq src/cpp/setfreq.cpp src/cpp/dccmsg.cpp)
#rosbuild_add_executable(setgain src/cpp/setgain.cpp src/cpp/dccmsg.cpp)

####################  DEPRICATED ############################################
#rosbuild_add_executable(DCC src/cpp/input.cpp)
#target_link_libraries(DCC dccmsg)

#rosbuild_add_executable(BackupRosDCC src/cpp/roshost.cpp src/cpp/dccmsg.cpp)
#rosbuild_add_executable(RosDCC src/cpp/roshostsingle.cpp src/cpp/dccmsg.cpp)

###################   TESTS   ###############################################
#rosbuild_add_executable(test_stress src/cpp/strtest.cpp)
rosbuild_add_executable(fake src/cpp/faker.cpp)
#rosbuild_add_executable(zero src/cpp/zero.cpp)

#target_link_libraries(RosDCC serialcoms)
#add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
#rospack_add_java_executable(RosDCC rsn.dcc.DccNode)
##add_classpath(${PREFIX}/msg_gen/java)
