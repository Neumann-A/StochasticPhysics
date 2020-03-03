message(STATUS "Reported configuration types:  ${CMAKE_CONFIGURATION_TYPES}")

if(General_FAST_MATH)
  add_definitions(-DEIGEN_FAST_MATH)
  add_definitions(-D__FAST_MATH__)
else()
  add_definitions(-UEIGEN_FAST_MATH)
endif()
