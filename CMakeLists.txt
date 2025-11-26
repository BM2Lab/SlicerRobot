cmake_minimum_required(VERSION 3.16.3...3.19.7 FATAL_ERROR)

project(SlicerCR)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://github.com/BM2Lab/SlicerCR")
set(EXTENSION_CATEGORY "SlicerCR")
set(EXTENSION_CONTRIBUTORS "Letian Ai (BM2)")
set(EXTENSION_DESCRIPTION "This is extension for continuum robot operation and visualization in Slicer.")
set(EXTENSION_ICONURL "https://www.example.com/Slicer/Extensions/SlicerRobot.png")
set(EXTENSION_SCREENSHOTURLS "https://www.example.com/Slicer/Extensions/SlicerRobot/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(SlicerComm)
add_subdirectory(SlicerRobotViz)
add_subdirectory(UsageTemplate)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
