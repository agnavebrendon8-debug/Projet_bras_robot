#ifndef __MOTOR_C0NTROLLER__
#define __MOTOR_CONTROLLER__

#include <Arduino.h>
#include "StepperTMC.h"


class MotorController {
  
  public :
      // Nombre de motor a changer ici 
      MotorController(StepperTMC& m1, StepperTMC& m2, StepperTMC& m3 , StepperTMC& m4, StepperTMC& m5, StepperTMC& m6);
      void begin();

      void moveTo(int Idx_motor ,long position, float velocity, float acceleration);
      void moveRelative(int Idx_motor,  long step, float velocity, float acceleration);

      void stopAll();
      bool allOnTarget();

    //   long getPosition() ;
    //   long getTargetPostion() ; peut etre une cinématique inverse

  private :

      StepperTMC* _axes[6];
};

#endif