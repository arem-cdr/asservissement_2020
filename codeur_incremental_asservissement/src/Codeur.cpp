#include "Codeur.h"

Codeur::Codeur(PinName A, PinName B /*, PinName Z*/)
{
    phaseA = new InterruptIn(A);
    phaseB = new InterruptIn(B);
    //phaseZ = new InterruptIn(Z);
    phaseA->mode(PullUp);
    phaseB->mode(PullUp);
    //phaseZ->mode(PullUp);
    phaseA->rise(callback(this, &Codeur::updateEncoder));
    phaseA->fall(callback(this, &Codeur::updateEncoder));
    phaseB->rise(callback(this, &Codeur::updateEncoder));
    phaseB->fall(callback(this, &Codeur::updateEncoder));
    //phaseZ->rise(callback())

    encoderValue = 0;
}

Codeur::~Codeur()
{
    delete phaseA;
    delete phaseB;
    //delete phaseZ;
}

bool Codeur::Init_Codeur()
{
    encoderValue = 0;
    return true;
}

void Codeur::updateEncoder()
{
    int MSB = phaseA->read(); //MSB = most significant bit
    int LSB = phaseB->read(); //LSB = least significant bit

    int encoded = (MSB << 1) | LSB;         //converting the 2 pin value to single number
    int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoderValue++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoderValue--;

    lastEncoded = encoded; //store this value for next time
}
