include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(flood_RESOURCES_RCC flood_resources.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(flood SHARED ${plugin_SOURCE} ${flood_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(flood
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    normalestimator
)

QT5_USE_MODULES(flood Core Gui OpenGL)

INSTALL(TARGETS flood DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
