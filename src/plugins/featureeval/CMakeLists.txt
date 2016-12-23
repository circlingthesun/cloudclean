add_definitions(-Dfeature_eval_EXPORTS)
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(featureeval_RESOURCES_RCC featureeval.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(featureeval SHARED ${plugin_SOURCE} ${featureeval_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(featureeval
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARY}
    ${Boost_LIBRARIES}
    normalestimator
)

QT5_USE_MODULES(featureeval Core Gui OpenGL)

INSTALL(TARGETS featureeval DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
