#include "Wifi_BT_communication.h"
#include "MotorController.h"
#include <ArduinoJson.h>

extern CommunicationInterface* canalActif ;

WifiController::WifiController(MotorController& motors , String Wifi_SSID , String Wifi_PASSWORD, uint16_t Udp_PORT) : _Wifi_SSID(Wifi_SSID) , _Wifi_PASSWORD(Wifi_PASSWORD) ,_Udp_PORT(Udp_PORT) , _motors(motors)
{
    memset(_tamponUdp, 0, sizeof(_tamponUdp));
}

void WifiController::init_wifi(){

  WiFi.softAP(_Wifi_SSID, _Wifi_PASSWORD);
  _udp.begin(_Udp_PORT);
  Serial.println("WiFi UDP pret !");
  
}



void WifiController::onReceiveMessage(){
  
  int taillePaquet = _udp.parsePacket();

  if(taillePaquet) {
    int len = _udp.read(_tamponUdp, sizeof(_tamponUdp) - 1);
    if(len  > 0){
      _tamponUdp[len]=0; //Terminer la chaine de caractere
      String msg = String(_tamponUdp);
      coverMessage(msg);
    }
  }
}



void WifiController::sendACK(const String& msg) {
  //Envoi du paquet UDP a l'adresse IP et au Port du PC qui nous a parlé en dernier 
  _udp.beginPacket(_udp.remoteIP(), _udp.remotePort());
  _udp.print(msg);
  _udp.endPacket();

}


void WifiController::coverMessage(String& msg){
  msg.trim();
  if(msg.length()==0) return;
  
  // Allocation de lla mémoire pour document JSON (Taille estimé pour notre structure)
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc , msg);
  
  if (error){
    Serial.print("Erreur de lecture JSON : ");
    Serial.println(error.f_str());
    return;
  }

  if (doc.containsKey("COM") && doc.containsKey("p") && doc.containsKey("v") && doc.containsKey("a")){
      String command = doc["COM"];

      long positions[5];
      float velocity[5];
      float acceleration[5];
      
      if(command == "CR"){
        // commande relative 
        for(int i=0; i < 5 ; i++) {          
          positions[i]    = doc["p"][i];
          velocity[i]     = doc["v"][i];
          acceleration[i] = doc["a"][i];
        }  
        for(int i=0; i < 5 ; i++){
          _motors.moveRelative(i ,positions[i], velocity[i], acceleration[i]);
        }
      }
      
      else if(command == "CA"){
        int i ;
        for(i=0; i < 5 ; i++) { 
                    
          positions[i]    = doc["p"][i];
          velocity[i]     = doc["v"][i];
          acceleration[i] = doc["a"][i];
      }  
      for(i=0; i < 5 ; i++){
          _motors.moveTo( i ,positions[i], velocity[i], acceleration[i]);
      }
    }
  }
}

BTController::BTController(MotorController& motors , String BT_Device_name): _BT_Device_name(BT_Device_name), _motors(motors){

}

void BTController::init_BT(){ // A changer Plus tard en Booleen pour verifaication  

  if(_SerialBT.begin(_BT_Device_name)) {
    Serial.println("Bluetooth Serial activé ! ");
  }
  else {
    Serial.println("Erreur lors l'initialisatiion du Bluetooth");
  }
}

void BTController::onReceiveMessage(){
  if(_SerialBT.available()){
    String msg = _SerialBT.readStringUntil('\n');
    coverMessage(msg);
  }
}

void BTController::sendACK(const String& msg){
  if(_SerialBT.connected()){
    _SerialBT.print(msg); //Envoi physuque en Bluetooth
  }
}

void BTController::coverMessage(String& msg ){
  msg.trim();
  if(msg.length() == 0 ) return ;

  JsonDocument doc ;
  
  DeserializationError error = deserializeJson(doc, msg) ;
  if (error){
    Serial.println("Erreur Json");
    Serial.println(error.f_str());
    return ;
  }

    if (doc.containsKey("COM") && doc.containsKey("p") && doc.containsKey("v") && doc.containsKey("a")){

      canalActif = this ;

      String command = doc["COM"];

      long positions[5];
      float velocity[5];
      float acceleration[5];
      int i ;
        for(i=0; i < 5 ; i++) { 
                    
          positions[i]    = doc["p"][i];
          velocity[i]     = doc["v"][i];
          acceleration[i] = doc["a"][i];
      }  
      // On mémorise que c'est ce canal qui doit répondre quand aura fini ;


      if(command == "CR"){
        // commande relative 
        for(int i=0; i < 5 ; i++){
          _motors.moveRelative(i , positions[i], velocity[i], acceleration[i]);
        }
      }
      
      else if(command == "CA"){
        
      for(i=0; i < 5 ; i++){
          _motors.moveTo( i ,positions[i], velocity[i], acceleration[i]);
      }
    }
  }


}