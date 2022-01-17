#include <Encoder.h>

Encoder myEnc(2, 3);

float Valmm;
float ValmmMem;
const float factConv = 0.23;
float G56;
const int BpPin = 4;
bool BpEtat = 0;
float difference;

//********P

//float consignePositionX = 2.35;

//delta = consignePositionX - Valmm;


// ******* I
//deltaIntegration += delta;


//***** Sortie PI
//puissance = (delta * 10)+(deltaIntegration * 0.2);
//puissance = constrain(puissance,0,255);
//if (delta > 0){
//  analogWrite(pinMoteurA, puissance);
//}else{
//  analogWrite(pinMoteurB, puissance);
//}
//
//void setup()  {
//  Serial.begin(9600);
//  pinMode(BpPin, INPUT_PULLUP);
//
//}

long oldPosition  = -999;

void loop(){

  ValeureG56();

  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  Valmm = newPosition*factConv;

if (Valmm != ValmmMem){
  ValmmMem = Valmm;
  Serial.print(Valmm);
  Serial.println(" mm");
  Serial.print("Valeure pupitre : "); 
  Serial.println( G56);
  //Serial.print(" difference : "); 
  //Serial.println( difference);
  }
}


void ValeureG56() {
   BpEtat = digitalRead(BpPin);
   if (BpEtat == LOW) {
        difference = Valmm;
   }
      G56 = Valmm - difference;
}