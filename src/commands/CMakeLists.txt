add_definitions(-Dcommands_EXPORTS)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB commands_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(cloudclean_commands SHARED ${commands_SOURCE})
target_link_libraries(cloudclean_commands
    cloudclean_model
)

QT5_USE_MODULES(cloudclean_commands Widgets)

INSTALL(TARGETS cloudclean_commands DESTINATION ${LIB_INSTALL} COMPONENT CloudClean)
