#include "simulation/simulation.h"
#include "startup.h"

void wml::RobotStartup::Start(std::function<int()> robotFunc) {
  static simulation::harness sim_harness;
  sim_harness.run(robotFunc);
}
