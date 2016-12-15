include_directories(
    ${'$'}{PCL_INCLUDE_DIRS}
    ${'$'}{FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(${lower_name}_RESOURCES_RCC ${lower_name}.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(${lower_name} SHARED ${'$'}{plugin_SOURCE} ${'$'}{${lower_name}_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    ${lower_name}
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${'$'}{PCL_LIBRARIES}
)


QT5_USE_MODULES(${lower_name} Core Gui OpenGL)

INSTALL(TARGETS ${lower_name} DESTINATION ${'$'}{PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
