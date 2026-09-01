#ifndef __CONFIG_STEPPER_TMC__
#define __CONFIG_STEPPER_TMC__

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>

#define Default_pin 100

class ConfigStepperTMC {
  public :
      ConfigStepperTMC(float rSense , uint8_t cs_Pin , uint8_t enPin = Default_pin, uint8_t dirPin = Default_pin , uint8_t stepPin = Default_pin );
      
      void begin(uint16_t current_mA=1200 , uint16_t microstep=16);
      void enable();
      void disable();

      void setCurrent(uint8_t current_mA);
      void setMicroStep(uint8_t microStep);

      void setStealthChop(bool enable);
      void setPwmAutoscale(bool enable);

      bool testConnection();
      
      uint32_t getDriverStatus();
      uint32_t getGlobalStatus();

      TMC5160Stepper& driver() ;

      uint8_t getStepPin() const ;
      uint8_t getDirPin() const ;
      uint8_t getEnPin() const ;

  private :
      uint8_t _csPin;
      uint8_t _enPin;
      uint8_t _dirPin;
      uint8_t _stepPin;

      float _rSense ;

      TMC5160Stepper _driver;

};




#endif
