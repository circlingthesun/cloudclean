FILE (GLOB_RECURSE test_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
SET (test_LIBS gtest)

SET (test_BIN ${PROJECT_NAME}-unittests)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/thirdparty/gtest/include
    ${PCL_INCLUDE_DIRS}
)


ADD_EXECUTABLE(${test_BIN} ${test_SRCS})
ADD_DEPENDENCIES(${test_BIN} googletest)
TARGET_LINK_LIBRARIES(${test_BIN} ${test_LIBS} ${Boost_LIBRARIES} cloudclean_model ${QT_LIBRARIES})
QT5_USE_MODULES(${test_BIN} Core) # Widgets Gui OpenGL

ADD_CUSTOM_TARGET(check ALL "${MAINFOLDER}/bin/${test_BIN}" DEPENDS ${test_BIN} COMMENT "Executing unit tests..." VERBATIM SOURCES ${test_SRCS})
ADD_CUSTOM_TARGET(test "${MAINFOLDER}/bin/${test_BIN}" DEPENDS ${test_BIN} COMMENT "Executing unit tests..." VERBATIM SOURCES ${test_SRCS})
