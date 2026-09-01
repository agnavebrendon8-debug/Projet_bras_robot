#include "config_stepper_TMC.h"

ConfigStepperTMC::ConfigStepperTMC( float rSense, uint8_t cs_Pin, uint8_t enPin, uint8_t dirPin ,
                  uint8_t stepPin ): _csPin(cs_Pin), _enPin(enPin), _dirPin(dirPin),_stepPin(stepPin), _rSense(rSense)
                  , _driver(_csPin, _rSense) 
{

}

void ConfigStepperTMC::begin(uint16_t current_mA, uint16_t microStep)
{
 if(_stepPin != Default_pin && _dirPin != Default_pin && _enPin != Default_pin){

    pinMode(_stepPin , OUTPUT); 
    pinMode(_dirPin , OUTPUT); 
    pinMode(_enPin , OUTPUT);

    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin,LOW);

 }

 disable(); // Désactivation pendant la congiguration 

 SPI.begin();
 
 _driver.begin();

  // Configuration de base 
  // Configuration des registres de hachage (Chopper configuration) de base

  _driver.toff(4);
  _driver.blank_time(24);

  setCurrent(current_mA);  // Courant moteur
  setMicroStep(microStep); // micro step
  setStealthChop(true);    // Activation du mode Silencieux
  setPwmAutoscale(true);   // Ajustement automatique de la tension PWM 

// Mode mouvement :
  // 0 = position mode 
  _driver.RAMPMODE(0); // 0=Position Mode (driver gere la rampe automatiquement ) 
  _driver.XACTUAL(0);  // Remise a zero de la position physique réelle 
  _driver.XTARGET(0); // Remise a zero de La cible 

}


void ConfigStepperTMC::enable(){
  digitalWrite(_enPin, LOW);
}


void ConfigStepperTMC::disable(){
  digitalWrite(_enPin, HIGH);
}

void ConfigStepperTMC::setCurrent(uint8_t current_mA){
  _driver.rms_current(current_mA);
}

void ConfigStepperTMC::setMicroStep(uint8_t microStep){
  _driver.microsteps(microStep);
}

void ConfigStepperTMC::setStealthChop(bool enable){
  _driver.en_pwm_mode(enable);
}

void ConfigStepperTMC::setPwmAutoscale(bool enable){
  _driver.pwm_autoscale(enable);
}

bool ConfigStepperTMC::testConnection(){
  return _driver.test_connection();
}


uint32_t ConfigStepperTMC::getDriverStatus(){
  return _driver.DRV_STATUS();
}

uint32_t ConfigStepperTMC::getGlobalStatus(){
  return _driver.GSTAT();
}

TMC5160Stepper& ConfigStepperTMC::driver(){
  return _driver;
}

uint8_t ConfigStepperTMC::getStepPin() const {
  return _stepPin ;
}

uint8_t ConfigStepperTMC::getDirPin() const {
  return _dirPin ;
}

uint8_t ConfigStepperTMC::getEnPin() const {
  return _enPin ;
}
