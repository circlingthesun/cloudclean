include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(evaluate_RESOURCES_RCC evaluate.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(evaluate SHARED ${plugin_SOURCE} ${evaluate_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    evaluate
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(evaluate Core Gui OpenGL)

INSTALL(TARGETS evaluate DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
