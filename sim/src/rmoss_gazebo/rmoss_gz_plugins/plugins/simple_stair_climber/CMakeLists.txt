# SimpleStairClimber plugin
add_library(SimpleStairClimber SHARED
  SimpleStairClimber.cc
)
target_link_libraries(SimpleStairClimber
  ${IGNITION-GAZEBO_LIBRARIES}
)
target_include_directories(SimpleStairClimber PUBLIC
  ${IGNITION-GAZEBO_INCLUDE_DIRS}
)

install(TARGETS SimpleStairClimber
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
