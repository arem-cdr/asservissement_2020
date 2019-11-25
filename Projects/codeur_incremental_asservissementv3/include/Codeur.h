#pragma once

#include "mbed.h"

class Codeur
{

public:
  Codeur(PinName A, PinName B /*, PinName Z*/);
  ~Codeur();

  bool Init_Codeur();

  long int get_nbr_tick() const { return encoderValue; }

private:
  InterruptIn *phaseA;
  InterruptIn *phaseB;
  //InterruptIn *phaseZ;

  volatile long encoderValue = 0; //nombre de tics sur l'encodeur A
  volatile int lastEncoded = 0;
  long lastencoderValue = 0;
  int lastMSB = 0;
  int lastLSB = 0;

  void updateEncoder();
};
