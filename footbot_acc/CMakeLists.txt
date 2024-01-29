add_library(footbot_acc MODULE footbot_acc.h footbot_acc.cpp)
target_link_libraries(footbot_acc
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
