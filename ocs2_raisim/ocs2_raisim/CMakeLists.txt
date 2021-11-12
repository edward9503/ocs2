cmake_minimum_required(VERSION 3.0.2)
# set(CMAKE_CXX_STANDARD 17)
# message(${CMAKE_CXX_FLAGS})
project(ocs2_raisim)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_oc
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(raisim REQUIRED)
find_package(OGRE REQUIRED)
find_package(raisimOgre REQUIRED)
message(${raisim_INCLUDE_DIRS})
message(${OGRE_INCLUDE_DIRS})
message(${raisimOgre_INCLUDE_DIRS})
message(${CMAKE_CURRENT_SOURCE_DIR})
message(${raisimOgre_LIBRARIES})
message(${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/imgui.cpp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${raisim_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${raisimOgre_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    raisim
    OGRE
    raisim::raisimOgre
)


###########
## Build ##
###########
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${raisim_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Bites
  ${OGRE_INCLUDE_DIRS}/HLMS
  ${OGRE_INCLUDE_DIRS}/MeshLodGenerator
  ${OGRE_INCLUDE_DIRS}/Overlay
  ${OGRE_INCLUDE_DIRS}/Paging
  ${OGRE_INCLUDE_DIRS}/Plugins/BSPSceneManager
  ${OGRE_INCLUDE_DIRS}/Plugins/DotScene
  ${OGRE_INCLUDE_DIRS}/Plugins/EXRCodec
  ${OGRE_INCLUDE_DIRS}/Plugins/FreeImageCodec
  ${OGRE_INCLUDE_DIRS}/Plugins/OctreeSceneManager
  ${OGRE_INCLUDE_DIRS}/Plugins/OctreeZone
  ${OGRE_INCLUDE_DIRS}/Plugins/ParticleFX
  ${OGRE_INCLUDE_DIRS}/Plugins/PCZSceneManager
  ${OGRE_INCLUDE_DIRS}/Plugins/STBICodec
  ${OGRE_INCLUDE_DIRS}/Property
  ${OGRE_INCLUDE_DIRS}/RenderSystems/GL
  ${OGRE_INCLUDE_DIRS}/RenderSystems/GL/GL
  ${OGRE_INCLUDE_DIRS}/RenderSystems/GL3Plus
  ${OGRE_INCLUDE_DIRS}/RenderSystems/GL3Plus/GL
  ${OGRE_INCLUDE_DIRS}/RTShaderSystem
  ${OGRE_INCLUDE_DIRS}/Terrain
  ${OGRE_INCLUDE_DIRS}/Threading
  ${OGRE_INCLUDE_DIRS}/Volume
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/examples/include
  ${raisimOgre_INCLUDE_DIRS}
)

link_directories(${RAISIM_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS} ${RAISIMOGRE_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/RaisimRollout.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/imgui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/imgui_plot.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/imgui_draw.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/imgui_widgets.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/ImguiManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/ImguiRenderable.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/AssimpLoader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/OgreVis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../raisimOgre/src/CameraMan.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${RAISIM_LIBRARIES} ${OGRE_LIBRARIES} ${RAISIMOGRE_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_link_libraries(${PROJECT_NAME}_lintTarget
    ${catkin_LIBRARIES}
    raisim::raisim
)

add_executable(${PROJECT_NAME}_raisim_test
  src/raisim_test.cpp
)
target_link_libraries(${PROJECT_NAME}_raisim_test
    ${catkin_LIBRARIES}
    raisim::raisim
)

add_executable(${PROJECT_NAME}_raisim_PDOnly
  src/raisim_PDOnly.cpp
)
target_link_libraries(${PROJECT_NAME}_raisim_PDOnly PUBLIC
    ${catkin_LIBRARIES}
    ${RAISIM_LIBRARIES} ${OGRE_LIBRARIES} ${RAISIMOGRE_LIBRARIES}
    raisim::raisim
    raisim::raisimOgre

)
target_compile_definitions(${PROJECT_NAME}_raisim_PDOnly PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=/home/bmtsz/ocs2_ws/src/raisimOgre/examples/rsc/")

add_executable(${PROJECT_NAME}_raisim_PDOnly_tele
  src/raisim_PDOnly_tele.cpp
)
target_link_libraries(${PROJECT_NAME}_raisim_PDOnly_tele PUBLIC
    ${catkin_LIBRARIES}
    ${RAISIM_LIBRARIES} ${OGRE_LIBRARIES} ${RAISIMOGRE_LIBRARIES}
    raisim::raisim
    raisim::raisimOgre

)
target_compile_definitions(${PROJECT_NAME}_raisim_PDOnly_tele PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=/home/bmtsz/ocs2_ws/src/raisimOgre/examples/rsc/")

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget ${PROJECT_NAME}_raisim_test ${PROJECT_NAME}_raisim_PDOnly ${PROJECT_NAME}_raisim_PDOnly_tele
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
    CT_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
