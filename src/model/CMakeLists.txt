add_definitions(-Dmodel_EXPORTS)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB model_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(cloudclean_model SHARED ${model_SOURCE})
target_link_libraries(
    cloudclean_model
    ${PCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY}
)

QT5_USE_MODULES(cloudclean_model Core Gui Widgets)

INSTALL(TARGETS cloudclean_model DESTINATION ${LIB_INSTALL} COMPONENT CloudClean)
