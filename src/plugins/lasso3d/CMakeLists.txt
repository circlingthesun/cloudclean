include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
QT5_ADD_RESOURCES(lasso3d_RESOURCES_RCC lasso3d_resources.qrc)
add_library(lasso3d SHARED ${plugin_SOURCE} ${lasso3d_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(lasso3d
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
)

if(WIN32)
    target_link_libraries(lasso3d ${GLEW_LIBRARIES})
endif()

QT5_USE_MODULES(lasso3d Core Gui OpenGL)

INSTALL(TARGETS lasso3d DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
