include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
QT5_ADD_RESOURCES(snake_RESOURCES_RCC snake_resources.qrc)
add_library(snake SHARED ${plugin_SOURCE} ${snake_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(snake
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
)

if(WIN32)
    target_link_libraries(snake ${GLEW_LIBRARIES})
endif()

QT5_USE_MODULES(snake Core Gui OpenGL)

INSTALL(TARGETS snake DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
