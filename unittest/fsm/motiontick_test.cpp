#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "runtime_data.hpp"
#include "fsm_manager.hpp"

int main() {
  // Initialize RuntimeData for basic FSM test
  std::shared_ptr<data::RuntimeData> rdata = std::make_shared<data::RuntimeData>(0, 0);

  rynn::FsmManager fsmManager;

  constexpr float dt = 0.001f;
  float sim_time = 0.f;
  float last_output_time = 0.f;

  std::cout << "Starting FSM test with FsmManager..." << std::endl;
  std::cout << "Time: " << std::fixed << std::setprecision(1) << sim_time << "s" << std::endl;

  while (sim_time < 20.f) {
    // Update timing directly in RuntimeData
    rdata->simTime = static_cast<double>(sim_time);
    rdata->wallTime = static_cast<double>(sim_time);
    rdata->duration = static_cast<double>(dt);

    fsmManager.tick(dt);

    if (sim_time - last_output_time >= 1.0f) {
      std::cout << "Time: " << std::fixed << std::setprecision(1) << sim_time << "s"
                << " | FSM State: " << static_cast<int>(fsmManager.getCurrentState())
                << " | State Time: " << std::setprecision(2) << fsmManager.getStateTime() << "s"
                << " | Total Time: " << std::setprecision(2) << fsmManager.getTotalTime() << "s"
                << " | State Changed: " << (fsmManager.isOnEntry() ? "Yes" : "No")
                << std::endl;
      last_output_time = sim_time;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    sim_time += dt;
  }

  std::cout << "\nFSM test completed after " << std::fixed << std::setprecision(1) << sim_time << "s" << std::endl;
  std::cout << "Final FSM State: " << static_cast<int>(fsmManager.getCurrentState()) << std::endl;
  std::cout << "Final State Time: " << std::setprecision(2) << fsmManager.getStateTime() << "s" << std::endl;
  std::cout << "Final Total Time: " << std::setprecision(2) << fsmManager.getTotalTime() << "s" << std::endl;

  return 0;
}
