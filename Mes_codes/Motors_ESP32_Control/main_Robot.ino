#include"StepperTMC.h"
#include"MotorController.h"
#include"Broches.h"
#include"config_stepper_TMC.h"
#include"Wifi_BT_communication.h"
#include"CommunicationInterface.h"


// Instanciation des Configurations matérielles ;
ConfigStepperTMC cfgm1(R_SENSE , CS_AXE1_PIN);
ConfigStepperTMC cfgm2(R_SENSE , CS_AXE2_PIN);
ConfigStepperTMC cfgm3(R_SENSE , CS_AXE3_PIN);
ConfigStepperTMC cfgm4(R_SENSE , CS_AXE4_PIN);
ConfigStepperTMC cfgm5(R_SENSE , CS_AXE5_PIN);
ConfigStepperTMC cfgm6(R_SENSE , CS_AXE6_PIN);

// Instanciations des 6 axes (motors) individuels avec les Configurations 
StepperTMC moteurBase(cfgm1);
StepperTMC moteurJoint2(cfgm2); // Epaule
StepperTMC moteurJoint3(cfgm3); // Coude 
StepperTMC moteurJoint4(cfgm4); // Poignet Rotation
StepperTMC moteurJoint5(cfgm5); // Poignet Inliné
StepperTMC moteurJoint6(cfgm6); // Pince Rot

// Regroupement des axes dans le Controleur Global du bras robot
MotorController BrasRobot(moteurBase , moteurJoint2 , moteurJoint3, moteurJoint4 , moteurJoint5, moteurJoint6 );

// Instanciation du module de communication
CommunicationInterface *canalActif = nullptr ;
// WIFI 
WifiController serveurWifi(BrasRobot);
//Bluetooth 
BTController serveurBT(BrasRobot);

bool wasMoving = false ;

void setup() {
  Serial.begin(115200);
  Serial.println("---DEMARRAGE DU BRAS OBOTIQUE 6 AXES--- ");

  BrasRobot.begin();
  Serial.println("Drivers TMC5160 initialisés via SPI ");

  serveurWifi.init_wifi();
  serveurBT.init_BT();

}

void loop() {
  serveurWifi.onReceiveMessage();
  serveurBT.onReceiveMessage();

  bool isMoving = !BrasRobot.allOnTarget();

  if(isMoving){
    wasMoving = true ;
  }
  else if(wasMoving){
    
    String ackMessage = "{\"statut\":\"READY\"}\n";
    Serial.print(ackMessage);

    if (canalActif != nullptr){
      canalActif->sendACK(ackMessage) ;
    
    }
    wasMoving = false ;//Reinitialisation 
  }
}
