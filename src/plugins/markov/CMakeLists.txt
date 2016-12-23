include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${OPENCL_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/thirdparty/omclboost/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/thirdparty/EigenLibSVM/include)

QT5_ADD_RESOURCES(markov_RESOURCES_RCC markov.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(markov SHARED ${plugin_SOURCE} ${markov_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
ADD_DEPENDENCIES(markov onlineforest)

IF(WIN32)
    set(LIBSVM_ROOT_DIR "c:/libsvm")
ENDIF()
find_package(LibSVM REQUIRED)
include_directories(${LIBSVM_INCLUDE_DIR})

target_link_libraries(
    markov
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    normalestimator
    omclboost
    svmlib
    ${PCL_LIBRARIES}
    ${LIBSVM_LIBRARY}
)


QT5_USE_MODULES(markov Core Gui OpenGL)

include(installtools)

# Installation
IF(WIN32)
    FIND_DLLS("${LIBSVM_LIBRARY}" "" LIBSVM_LIBRARY)
    INSTALL(FILES ${LIBSVM_LIBRARY} DESTINATION ${LIB_INSTALL} COMPONENT LIBSVM)
ELSE()
    RESOLVE_SYMLINKS("${LIBSVM_LIBRARY}" LIBSVM_LIBRARY)
    INSTALL(FILES ${LIBSVM_LIBRARY} DESTINATION ${LIB_INSTALL} COMPONENT LIBSVM)
ENDIF()

INSTALL(TARGETS markov DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
