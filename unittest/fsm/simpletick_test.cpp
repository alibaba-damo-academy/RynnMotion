#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "test_fsm.hpp"

int main() {
  MotionFsm::start();

  constexpr float dt = 0.001f;
  float sim_time = 0.f;
  float last_output_time = 0.f;

  std::cout << "Starting FSM test..." << std::endl;
  std::cout << "Time: " << std::fixed << std::setprecision(1) << sim_time << "s" << std::endl;

  while (sim_time < 20.f) {
    MotionFsm::dispatch(EventTick{dt});

    if (sim_time - last_output_time >= 1.0f) {
      std::cout << "Time: " << std::fixed << std::setprecision(1) << sim_time << "s" << std::endl;
      last_output_time = sim_time;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    sim_time += dt;
  }

  std::cout << "FSM test completed after " << std::fixed << std::setprecision(1) << sim_time << "s" << std::endl;
  return 0;
}
