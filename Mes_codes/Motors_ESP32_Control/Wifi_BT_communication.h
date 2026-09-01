#ifndef __WIFI_BT_COMMUNICATION_H__
#define __WIFI_BT_COMMUNICATION_H__

class MotorController;

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "StepperTMC.h"
#include "BluetoothSerial.h"
#include "Broches.h"
#include "CommunicationInterface.h"


class WifiController : public CommunicationInterface {
  public :
      WifiController(MotorController& motors , String Wifi_SSID = WIFI_SSID, String Wifi_PASSWORD = WIFI_PASSWORD, uint16_t Udp_PORT = UDP_PORT);
      void init_wifi();
      void onReceiveMessage(); // Recevoir en temps réel les message wifi et faire envoyer les commmande aux drivers
      void coverMessage(String& msg );   // Traiter le Message reçu

      void sendACK(const String& msg) override;

  private :
      WiFiUDP _udp ;
      String _Wifi_SSID ;
      String _Wifi_PASSWORD ;
      uint16_t _Udp_PORT ;

      MotorController& _motors ;
      
      char _tamponUdp[512]; // UN tampon de 512 octet 

};


class BTController : public CommunicationInterface{
  public :
      BTController(MotorController& motors, String BT_Device_name=BT_DEVICE_NAME);
      void init_BT();
      void onReceiveMessage();
      void coverMessage(String& msg);

     //Implémentation de la fonction virtuel    
      void sendACK(const String& msg) override ; // override : pour dire a la machine que l'on remplace la fonction virtuel on ne creer pas 

  private :
      MotorController& _motors ;
      String _BT_Device_name ;
      BluetoothSerial _SerialBT ;

};


#endif