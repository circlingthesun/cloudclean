add_definitions(-Dpluginsystem_EXPORTS)

FILE (GLOB pluginsystem_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(
    cloudclean_pluginsystem SHARED
    ${pluginsystem_SRCS}
)

target_link_libraries(
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)

QT5_USE_MODULES(cloudclean_pluginsystem Core Widgets)

INSTALL(TARGETS cloudclean_pluginsystem DESTINATION ${LIB_INSTALL} COMPONENT CloudClean)
