#include <hal/HAL.h>
#include <iostream>
#include "gtest/gtest.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  std::cout << "Tests run!" << std::endl;
  return ret;
}

