#include <iostream>
#include <memory>
#include <mujoco/mujoco.h>
#include "mjpc/tasks/simple_car/simple_car.h"
#include "mjpc/task.h"

int main() {
  // Create the task
  auto task = std::make_unique<mjpc::SimpleCar>();
  
  std::cout << "Task name: " << task->Name() << std::endl;
  std::cout << "XML path: " << task->XmlPath() << std::endl;
  
  // Try to load the model
  char error[1000] = "";
  std::string xml_path = "build/mjpc/tasks/" + task->XmlPath();
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, 1000);
  
  if (!model) {
    std::cerr << "Error loading model: " << error << std::endl;
    return 1;
  }
  
  std::cout << "Model loaded successfully!" << std::endl;
  std::cout << "Number of actuators: " << model->nu << std::endl;
  std::cout << "Number of DOFs: " << model->nv << std::endl;
  std::cout << "Number of positions: " << model->nq << std::endl;
  
  // Create data
  mjData* data = mj_makeData(model);
  
  // Test residual function
  auto residual_fn = task->InternalResidual();
  double residual[4] = {0};
  
  mj_resetData(model, data);
  residual_fn->Residual(model, data, residual);
  
  std::cout << "\nInitial residuals:" << std::endl;
  for (int i = 0; i < 4; i++) {
    std::cout << "  residual[" << i << "] = " << residual[i] << std::endl;
  }
  
  // Clean up
  mj_deleteData(data);
  mj_deleteModel(model);
  
  std::cout << "\nTest passed!" << std::endl;
  return 0;
}
