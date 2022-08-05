#pragma once

#include "Strategy/Strategy.h"
#include "RobotMap.h"

class Vision : public wml::StrategySystem, public wml::loops::LoopSystem {
 public: 
  Vision();

  void updateVision(double dt);

  void Update(double dt);
 private:
};