
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
QT5_ADD_RESOURCES(VRESOURCES_RCC vresources.qrc)
add_library(visualisenormals SHARED ${plugin_SOURCE} ${VRESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(visualisenormals
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    normalestimator
)

if(WIN32)
    target_link_libraries(visualisenormals ${GLEW_LIBRARIES})
endif()

QT5_USE_MODULES(visualisenormals Core OpenGL)

INSTALL(TARGETS visualisenormals DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
