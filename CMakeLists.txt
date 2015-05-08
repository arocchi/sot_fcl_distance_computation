cmake_minimum_required(VERSION 2.8)
PROJECT(sot_fcl_distance_computation)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#common commands for building c++ executables and libraries
find_package(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE(catkin REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(fcl REQUIRED)
FIND_PACKAGE(geometric_shapes REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(urdf REQUIRED)

INCLUDE_DIRECTORIES(include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                            ${fcl_INCLUDE_DIRS} ${geometric_shapes_INCLUDED_DIRS}
                            ${kdl_parser_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}
                            ${rosconsole_INCLUDE_DIRS} ${tf_INCLUDE_DIRS}
                            ${urdf_INCLUDE_DIRS})
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

add_library(sot_fcl_distance_computation src/DistanceComputation.cpp
                                                  src/conversions.cpp)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES sot_fcl_distance_computation
 CATKIN_DEPENDS geometric_shapes kdl_parser roscpp tf urdf
 DEPENDS Boost Eigen3 fcl orocos_kdl
)

add_executable(distance_query src/distance_query.cpp ${sot_INCLUDES})
target_link_libraries(distance_query sot_fcl_distance_computation
                                    ${Boost_LIBRARIES} ${fcl_LIBRARIES}
                                    ${geometric_shapes_LIBRARIES} ${kdl_parser_LIBRARIES}
                                    ${orocos_kdl_LIBRARIES} ${ros_console_LIBRARIES}
                                    ${tf_LIBRARIES} ${urdf_LIBRARIES})

install(TARGETS sot_fcl_distance_computation distance_query
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
