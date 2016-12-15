SET(CMAKE_AUTOMOC ON)

INCLUDE_DIRECTORIES(
    ${PCL_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

IF(CMAKE_BUILD_TYPE MATCHES "Debug")
    ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)

ENDIF()

ADD_DEFINITIONS(-DCC_GL_DEBUG_)

#
# Build sub projects
#
add_subdirectory(utilities)
add_subdirectory(gui)
add_subdirectory(model)
add_subdirectory(plugins)
add_subdirectory(pluginsystem)
add_subdirectory(commands)

#
# Build main binary
#
FILE (GLOB project_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp *.rc)
SET(project_BIN ${PROJECT_NAME})

IF(WIN32)
    ADD_EXECUTABLE(${project_BIN} WIN32 ${project_SRCS})
ELSE()
    ADD_EXECUTABLE(${project_BIN} ${project_SRCS})
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Bsymbolic" )
ENDIF()

SET_TARGET_PROPERTIES(${project_BIN}
    PROPERTIES VERSION
    "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}"
    OUTPUT_NAME ${project_BIN}
    CLEAN_DIRECT_OUTPUT 1
)

TARGET_LINK_LIBRARIES(${project_BIN}
    ${Qt5Core_QTMAIN_LIBRARIES}
    ${Boost_LIBRARIES}
    cloudclean_gui
    cloudclean_model
    cloudclean_util
    cloudclean_commands
    cloudclean_pluginsystem
)

QT5_USE_MODULES(${project_BIN} Core Widgets Gui OpenGL)

INSTALL(TARGETS ${project_BIN} DESTINATION ${BIN_INSTALL})
