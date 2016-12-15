cmake_minimum_required (VERSION 2.6)
project (eigenlibsvm)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
add_library(svmlib src/svm_utils.cpp)
include_directories("${PROJECT_SOURCE_DIR}/include")

find_package(Eigen REQUIRED)
include_directories("${EIGEN_INCLUDE_DIRS}")

IF(WIN32)
    set(LIBSVM_ROOT_DIR "c:/libsvm")
ENDIF()
find_package(LibSVM REQUIRED)
include_directories(${LIBSVM_INCLUDE_DIR})

add_executable (svm_test test/svm_test.cpp)
target_link_libraries (svm_test svmlib ${LIBSVM_LIBRARY})

