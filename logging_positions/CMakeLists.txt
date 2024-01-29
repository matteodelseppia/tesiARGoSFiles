include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(logging_positions MODULE 
  logging_positions.h
  logging_positions.cpp)

target_link_libraries(logging_positions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})