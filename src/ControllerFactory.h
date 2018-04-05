#pragma once

#include "QuadControl.h"

inline ControllerHandle CreateController(string name, string controllerType, string config)
{
  ControllerHandle ret;

  if (controllerType == "QuadControl")
  {
    ret.reset(new QuadControl(name, config));
  }
  
  return ret;
}