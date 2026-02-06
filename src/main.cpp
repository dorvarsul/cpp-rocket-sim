#include "rendering/RocketSimApp.h"

int main() {
  RocketSimApp app;
  if (!app.init()) {
    return 1;
  }
  app.run();
  return 0;
}
