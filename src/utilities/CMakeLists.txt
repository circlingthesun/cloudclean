add_definitions(-Dutilities_EXPORTS)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

FILE (GLOB util_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)



add_library(cloudclean_util SHARED ${util_SOURCE})
target_link_libraries(
    cloudclean_util
    ${OPENGL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    cloudclean_model
    cloudclean_gui
)

#QT5_ADD_RESOURCES(cloudclean_util util_resources.qrc)
QT5_USE_MODULES(cloudclean_util Core Gui OpenGL)

INSTALL(TARGETS cloudclean_util DESTINATION ${LIB_INSTALL} COMPONENT CloudClean)
