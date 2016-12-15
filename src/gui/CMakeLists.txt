add_definitions(-DQT_SHARED)
add_definitions(-Dgui_EXPORTS)

include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${QT_QTGUI_INCLUDE_DIR}
    ${QT_QTOPENGL_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

if(Leap_FOUND)
    include_directories(${Leap_INCLUDE_DIR})
endif()

FILE (GLOB app_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
FILE (GLOB app_FORMS *.ui)

if(NOT Leap_FOUND)
    foreach(file ${app_SRCS})
        if(file MATCHES ".*leaplistener.cpp|.*leaplistener.h")
            list(REMOVE_ITEM app_SRCS ${file})
        endif()
    endforeach()
    MESSAGE("NO LEAP SUPPORT!")
else()
    add_definitions("-DLEAP_SUPPORT")
    MESSAGE("YAY FOR LEAP SUPPORT!")
endif()

QT5_WRAP_UI(app_FORMS_HEADERS ${app_FORMS})
QT5_ADD_RESOURCES(app_RESOURCES_RCC resources/resources.qrc)

add_library(
    cloudclean_gui SHARED
    ${app_FORMS_HEADERS}
    ${app_SRCS}
    ${app_RESOURCES_RCC}
)

if(WIN32)
    target_link_libraries(cloudclean_gui ${GLEW_LIBRARIES})
endif()

target_link_libraries(
    cloudclean_gui
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARY}
    #${OPENCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    cloudclean_model
    cloudclean_commands
)

if(Leap_FOUND)
    TARGET_LINK_LIBRARIES(cloudclean_gui ${Leap_LIBRARIES})
endif()

QT5_USE_MODULES(cloudclean_gui Widgets Gui OpenGL)

INSTALL(TARGETS cloudclean_gui DESTINATION ${LIB_INSTALL} COMPONENT CloudClean)
