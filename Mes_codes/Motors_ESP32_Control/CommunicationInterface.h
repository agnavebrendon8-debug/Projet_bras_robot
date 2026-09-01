#ifndef __COMMUNICATION_INTERFACE__
#define __COMMUNICATION_INTERFACE__

#include <Arduino.h>

class CommunicationInterface {
  public :
    // Destructeur virtuel pour la propreté de la memoire
    virtual ~CommunicationInterface() {}; // methode de définition des methodes virtuales 

    //methode virtuel pure : 
    virtual void sendACK(const String& msg);     // a définir dans les classes qui vont heriter 
};


#endif