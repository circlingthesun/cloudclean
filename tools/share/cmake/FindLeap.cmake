# - Try to find the Leap library
# Once done this will define
#
#  Leap_FOUND - system has Leap
#  Leap_INCLUDE_DIR - the Leap include directory
#  Leap_LIBRARIES - The libraries needed to use Leap

if(Leap_INCLUDE_DIR AND Leap_LIBRARIES)
   set(Leap_FOUND TRUE)
else(Leap_INCLUDE_DIR AND Leap_LIBRARIES)

FIND_PATH(Leap_INCLUDE_DIR NAMES Leap.h
   PATHS
   /usr/include
   /usr/local/include
   $ENV{INCLUDE}
   $ENV{LeapROOT}/include
   $ENV{Leap_ROOT}/include
   $ENV{Leap_DIR}/include
   $ENV{Leap_DIR}/inc
)

FIND_LIBRARY(Leap_LIBRARIES NAMES Leap
   PATHS
   /usr/lib
   /usr/lib64
   /usr/local/lib
   /usr/local/lib64
   $ENV{LIBPATH}
   $ENV{LeapROOT}/lib
   $ENV{Leap_ROOT}/lib
   $ENV{Leap_DIR}/lib
   PATH_SUFFIXES
   .
   Leap
   DOC "Leap library name"
)

MESSAGE("LEAP include: " ${Leap_INCLUDE_DIR})
MESSAGE("LEAP lib: " ${Leap_LIBRARIES})

if(Leap_INCLUDE_DIR AND Leap_LIBRARIES)
   set(Leap_FOUND TRUE)
endif(Leap_INCLUDE_DIR AND Leap_LIBRARIES)


if(Leap_FOUND)
   if(NOT Leap_FIND_QUIETLY)
      message(STATUS "Found Leap: ${Leap_LIBRARIES}")
   endif(NOT Leap_FIND_QUIETLY)
else(Leap_FOUND)
   if(Leap_FIND_REQUIRED)
      message(FATAL_ERROR "could NOT find Leap")
   endif(Leap_FIND_REQUIRED)
endif(Leap_FOUND)

MARK_AS_ADVANCED(Leap_INCLUDE_DIR Leap_LIBRARIES)

endif(Leap_INCLUDE_DIR AND Leap_LIBRARIES)
