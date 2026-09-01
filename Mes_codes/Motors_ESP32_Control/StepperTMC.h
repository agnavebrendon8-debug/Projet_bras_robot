#ifndef __STEPPER__TMC__
#define __STEPPER__TMC__

#include <Arduino.h>
#include "config_stepper_TMC.h"

class StepperTMC {

  public :
      StepperTMC(ConfigStepperTMC& tmc);
      void begin();

      void moveTo(long position, float velocity, float acceleration);
      void moveRelative(long step, float velocity, float acceleration);

      void setVelocity(float velocity);
      void setAcceleration(float accelaration);

      void Stop();
      void hardStop();

      long getPosition() ;
      long getTargetPosition() ;

      bool isMoving(); 

  private :

      ConfigStepperTMC& _tmc ;
      long _targetPosition ;
      float _velocity;
      float _acceleration ;

};

#endif