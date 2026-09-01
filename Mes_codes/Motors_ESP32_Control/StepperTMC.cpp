#include "StepperTMC.h"



StepperTMC::StepperTMC(ConfigStepperTMC& tmc): _tmc(tmc), _targetPosition(0), _velocity(0), _acceleration(0) {

}

void StepperTMC::begin(){

  _tmc.enable();

  _tmc.driver().RAMPMODE(0);
  _tmc.driver().XACTUAL(0);
  _tmc.driver().XTARGET(0);

}

void StepperTMC::moveTo(long position, float velocity, float acceleration)
{
    //Application de la configuration 
  _tmc.begin(1200, 16);

}

void StepperTMC::moveRelative(long step, float velocity, float acceleration ){
  long current = getPosition();

  moveTo(current + step, velocity, acceleration);

}

void StepperTMC::setVelocity(float velocity){
  _velocity = velocity ;
  _tmc.driver().VMAX(static_cast<uint32_t>(_velocity));

}

void StepperTMC::setAcceleration(float acceleration ){
  _acceleration = acceleration ;
  
  _tmc.driver().AMAX(static_cast<uint32_t>(_acceleration));
  _tmc.driver().DMAX(static_cast<uint32_t>(_acceleration));

}

void StepperTMC::Stop(){
    /*
     * On demande au TMC5160
     * de revenir à sa position actuelle.
     */
  long current = getPosition() ;
  _tmc.driver().XTARGET(current);

}

void StepperTMC::hardStop(){
      /*
     * Arrêt rapide.
     */
  _tmc.driver().RAMPMODE(0);
  _tmc.driver().XTARGET( _tmc.driver().XACTUAL());

}

long StepperTMC::getPosition(){

  return _tmc.driver().XACTUAL();

}

long StepperTMC::getTargetPosition(){
  
  return _tmc.driver().XTARGET();

}

bool StepperTMC::isMoving(){

  return getPosition() != getTargetPosition();

}
