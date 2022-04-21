#include "controller_plugin_base/controller_base.hpp"
#include <vector>

int main(int argc, char** argv)
{
  std::vector<controller_plugin_base::ControllerBase*> controllers;
  std::cout << "Hello World" << std::endl;
  // print array length
  std::cout << "Array length: " << controllers.size() << std::endl;
  return 0;
}
