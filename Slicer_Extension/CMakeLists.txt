cmake_minimum_required(VERSION 2.8.9)

project(SimpleFrame)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://github.com/joshicola/SimpleStereotactic")
set(EXTENSION_CATEGORY "IGT")
set(EXTENSION_CONTRIBUTORS "Joshua Jacobs")
set(EXTENSION_DESCRIPTION "This Slicer Extension models the targeting and trajectory for a simple frame in Compass coordinates.")
# ToDo: These below need to be more fully specified.
set(EXTENSION_ICONURL "https://raw.githubusercontent.com/Slicer/Extensions/SimpleFrame.png")
set(EXTENSION_SCREENSHOTURLS "https://github.com/flywheel-apps/SlicerFlywheelConnect/blob/master/Images/TreeD_Slicer.png"
set(EXTENSION_DEPENDS SlicerIGT) 

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})
##mark_as_superbuild(Slicer_DIR)

##find_package(Git REQUIRED)
##mark_as_superbuild(GIT_EXECUTABLE)

#-----------------------------------------------------------------------------
# SuperBuild setup
##option(${EXTENSION_NAME}_SUPERBUILD "Build ${EXTENSION_NAME} and the projects it depends on." ON)
##mark_as_advanced(${EXTENSION_NAME}_SUPERBUILD)
##if(${EXTENSION_NAME}_SUPERBUILD)
##  include("${CMAKE_CURRENT_SOURCE_DIR}/SuperBuild.cmake")
##  return()
##endif()

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(simple_frame)

#NEXT_MODULE

#-----------------------------------------------------------------------------
##set(CPACK_INSTALL_CMAKE_PROJECTS "${CPACK_INSTALL_CMAKE_PROJECTS};${CMAKE_BINARY_DIR};${EXTENSION_NAME};ALL;/")

include(${Slicer_EXTENSION_CPACK})
