#include "MotorController.h"



MotorController::MotorController(StepperTMC& m1, StepperTMC& m2, StepperTMC& m3, StepperTMC& m4, StepperTMC& m5, StepperTMC& m6){
 
  _axes[0] = &m1 ;
  _axes[1] = &m2 ;
  _axes[2] = &m3 ;
  _axes[3] = &m4 ;
  _axes[4] = &m5 ;
  _axes[5] = &m6 ;

}

void MotorController::begin(){
  for(int i=0 ; i < 6 ; i++){
    _axes[i]->begin();
  }
}

void MotorController::moveTo(int Idx_motor , long position, float velocity, float acceleration)
{  if ( Idx_motor >= 0 && Idx_motor < 6 ){
      _axes[Idx_motor]->moveTo(position, velocity , acceleration);
    }
}

void MotorController::moveRelative(int Idx_motor , long step, float velocity, float acceleration ){
  
if ( Idx_motor >= 0 && Idx_motor < 6 ){
  _axes[Idx_motor]->moveRelative(step , velocity , acceleration );
  }
}


void MotorController::stopAll(){
    for(int i = 0; i < 6; i++){
      _axes[i]->Stop();
    }
}

bool MotorController::allOnTarget(){
  for(int i = 0; i < 6; i++){
    if(_axes[i]->isMoving()){
      return false ;
    }
  }
  return true ;
}



