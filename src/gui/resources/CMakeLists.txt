qt4_add_resources(RESOURCES_RCC resources.qrc)
add_library(cloudclean_resources STATIC ${RESOURCES_RCC})
target_link_libraries(cloudclean_resources ${QT_LIBRARIES})
#qt5_use_modules(cloudclean_resources Core)