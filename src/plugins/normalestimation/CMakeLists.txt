add_definitions(-Dnormalestimator_EXPORTS)
#FIND_PACKAGE (OpenCL REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    #${OPENCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(normalestimator SHARED ${plugin_SOURCE} ../../pluginsystem/iplugin.h)
target_link_libraries(normalestimator
    #${OPENCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    cloudclean_pluginsystem
    cloudclean_model
)

QT5_USE_MODULES(normalestimator Core Gui OpenGL)

INSTALL(TARGETS normalestimator DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
