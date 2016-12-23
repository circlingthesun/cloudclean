include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FIND_PACKAGE(PCL 1.7 REQUIRED COMPONENTS
#    common
#    io
#    octree
    kdtree
#    search
#    features
#    filters
#    segmentation
)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Bsymbolic")

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
QT5_ADD_RESOURCES(graphcut_RESOURCES_RCC graphcut_resources.qrc)
add_library(graphcut SHARED ${plugin_SOURCE} ${graphcut_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(graphcut
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)

if(WIN32)
    target_link_libraries(graphcut ${GLEW_LIBRARIES})
endif()

QT5_USE_MODULES(graphcut Core Gui OpenGL)

INSTALL(TARGETS graphcut DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
