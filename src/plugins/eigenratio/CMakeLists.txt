include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(eigenratio_RESOURCES_RCC eigenratio.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(eigenratio SHARED ${plugin_SOURCE} ${eigenratio_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    eigenratio
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(eigenratio Core Gui OpenGL)

INSTALL(TARGETS eigenratio DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
