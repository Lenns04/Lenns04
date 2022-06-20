// Projet commande base 308
// Willemin-Macodel
// Juin 2022 / Lenny Lab

#include <Encoder.h>


 // pin nr. 0 Reservé Serial Monitor
 // pin nr. 1 Reservé Serial Monitor
const int IN_manivelleChA = 2;   // pin nr. 2 / PWM / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : manivelleChA
const int IN_axeZChA = 3;   // pin nr. 3 / PWM / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : axeZChA
const int OUT_VarVitesseX = 4;   // pin nr. 4 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarVitesseX
const int OUT_VarDirectionX = 5;   // pin nr. 5 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarDirectionX
const int OUT_VarEnableX = 6;   // pin nr. 6 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarEnableX
const int OUT_VarVitesseY = 7;   // pin nr. 7 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarVitesseY
const int OUT_VarDirectionY = 8;   // pin nr. 8 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarDirectionY
const int OUT_VarEnableY = 9;   // pin nr. 9 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarEnableY
const int OUT_VarVitesseZ = 10;   // pin nr. 10 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarVitesseZ
const int OUT_VarDirectionZ = 11;   // pin nr. 11 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarDirectionZ
const int OUT_VarEnableZ = 12;   // pin nr. 12 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VarEnableZ
const int IN_manivelleChB = 13;   // pin nr. 13 / PWM (LED) / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : manivelleChB
  // pin nr. 14  FREE
const int IN_axeXChB = 15;   // pin nr. 15 / RX / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : axeXChB
const int IN_axeYChB = 16;   // pin nr. 16 / TX / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : axeYChB
const int IN_axeZChB = 17;   // pin nr. 17 / RX / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : axeZChB
const int IN_axeXChA = 18;   // pin nr. 18 / TX / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : axeXChA
const int IN_axeYChA = 19;   // pin nr. 19 / RX / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : axeYChA
  // pin nr. 20  FREE
  // pin nr. 21  FREE
const int IN_BpAxeX = 22;   // pin nr. 22 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : BpAxeX
const int IN_BpAxeY = 23;   // pin nr. 23 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : BpAxeY
const int IN_BpAxeZ = 24;   // pin nr. 24 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : BpAxeZ
const int IN_BpMulti = 25;   // pin nr. 25 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : BpMulti
const int OUT_LedBpAxeX = 26;   // pin nr. 26 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LedBpAxeX
const int OUT_LedBpAxeY = 27;   // pin nr. 27 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LedBpAxeY
const int OUT_LedBpAxeZ = 28;   // pin nr. 28 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LedBpAxeZ
const int OUT_VerineRouge = 29;   // pin nr. 29 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VerineRouge
const int OUT_VerineJaune = 30;   // pin nr. 30 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VerineJaune
const int OUT_VerineVerte = 31;   // pin nr. 31 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : VerineVerte
const int OUT_LCD_RS_4 = 32;   // pin nr. 32 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LCD_RS_4
const int OUT_LCD_E_6 = 33;   // pin nr. 33 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LCD_E_6
const int OUT_LCD_DB4_11 = 34;   // pin nr. 34 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LCD_DB4_11
const int OUT_LCD_DB5_12 = 35;   // pin nr. 35 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LCD_DB5_12
const int OUT_LCD_DB6_13 = 36;   // pin nr. 36 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LCD_DB6_13
const int OUT_LCD_DB7_14 = 37;   // pin nr. 37 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : LCD_DB7_14
const int IN_FinCourseHaut = 38;   // pin nr. 38 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : FinCourseHaut
const int IN_FinCourseBas = 39;   // pin nr. 39 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : FinCourseBas
const int IN_FinCourseDroite = 40;   // pin nr. 40 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : FinCourseDroite
const int IN_FinCourseGauche = 41;   // pin nr. 41 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : FinCourseGauche
const int IN_FinCourseAvant = 42;   // pin nr. 42 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : FinCourseAvant
const int IN_FinCourseArriere = 43;   // pin nr. 43 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : FinCourseArriere
  // pin nr. 44  FREE
  // pin nr. 45  FREE
  // pin nr. 46  FREE
  // pin nr. 47  FREE
  // pin nr. 48  FREE
  // pin nr. 49  FREE
  // pin nr. 50  FREE
  // pin nr. 51  FREE
  // pin nr. 52  FREE
  // pin nr. 53  FREE
  // pin nr. A0  FREE
  // pin nr. A1  FREE
  // pin nr. A2  FREE
  // pin nr. A3  FREE
  // pin nr. A4  FREE
  // pin nr. A5  FREE
  // pin nr. A6  FREE
  // pin nr. A7  FREE

struct BP{ //Structure pour sub shortLong()
  bool BP = false;
  bool shortPress = false; //flanc montant, impulsion courte sur le BP
  bool longPress = false; //flanc montant, appuis long sur le BP
  long buttonTimer = 0L;
  bool buttonActive = false;
  bool longPressActive = false;
  bool outputState = false;
};    
BP BPStart, BPSelect; //declaration des variables pour chaque boutons

// réglage des librairies
Encoder manivelle(IN_manivelleChA, IN_manivelleChB);
Encoder codeurZ(IN_axeZChA, IN_axeZChB);

// Options du programme
const boolean debug = false; // on-off mode debug
const boolean graphic = true; // on-off mode graphic

boolean tempoDebug = false;
boolean tempoGraphic = false;

// Axe X
static float valmmCodeurX;
const float factConvCodeurX = 0.23;
// Axe Y
static float valmmCodeurY;
const float factConvCodeurY = 0.23;
// Axe Z
static float valmmCodeurZ;
const float factConvCodeurZ = 0.1;
static int vitMoteur;
// Manivelle
static float valmmManivelle;


// **************************************************************************************************
// ************************ SETUP *******************************************************************
// **************************************************************************************************

void setup() {

Serial.begin(115200);
if(tempoDebug)Serial.println("Serial.begin(115200);"); //S'affiche une fois, au démarage.

pinMode (IN_manivelleChA, INPUT_PULLUP); //pin nr. 2 / PWM / Interrupt / Câble : Codeur IHM CHA / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_manivelleChA
pinMode (IN_axeZChA, INPUT_PULLUP); //pin nr. 3 / PWM / Interrupt / Câble : Codeur axeZ CHA / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_axeZChA
pinMode (OUT_VarVitesseX, OUTPUT); //pin nr. 4 / PWM / Câble : PWM vers variateur Optidrive X / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarVitesseX
pinMode (OUT_VarDirectionX, OUTPUT); //pin nr. 5 / PWM / Câble : vers variateur Optidrive X / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarDirectionX
pinMode (OUT_VarEnableX, OUTPUT); //pin nr. 6 / PWM / Câble : vers variateur Optidrive X / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarEnableX
pinMode (OUT_VarVitesseY, OUTPUT); //pin nr. 7 / PWM / Câble : PWM vers variateur Optidrive Y / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarVitesseY
pinMode (OUT_VarDirectionY, OUTPUT); //pin nr. 8 / PWM / Câble : vers variateur Optidrive Y / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarDirectionY
pinMode (OUT_VarEnableY, OUTPUT); //pin nr. 9 / PWM / Câble : vers variateur Optidrive Y / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarEnableY
pinMode (OUT_VarVitesseZ, OUTPUT); //pin nr. 10 / PWM / Câble : PWM vers variateur Optidrive Z / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarVitesseZ
pinMode (OUT_VarDirectionZ, OUTPUT); //pin nr. 11 / PWM / Câble : vers variateur Optidrive Z / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarDirectionZ
pinMode (OUT_VarEnableZ, OUTPUT); //pin nr. 12 / PWM / Câble : vers variateur Optidrive Z / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VarEnableZ
pinMode (IN_manivelleChB, INPUT_PULLUP); //pin nr. 13 / PWM (LED) / Câble : Codeur IHM CHB / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_manivelleChB
pinMode (IN_axeXChB, INPUT_PULLUP); //pin nr. 15 / RX / Câble : Codeur axeX CHB / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_axeXChB
pinMode (IN_axeYChB, INPUT_PULLUP); //pin nr. 16 / TX / Câble : Codeur axeY CHB / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_axeYChB
pinMode (IN_axeZChB, INPUT_PULLUP); //pin nr. 17 / RX / Câble : Codeur axeZ CHB / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_axeZChB
pinMode (IN_axeXChA, INPUT_PULLUP); //pin nr. 18 / TX / Interrupt / Câble : Codeur axeX CHA / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_axeXChA
pinMode (IN_axeYChA, INPUT_PULLUP); //pin nr. 19 / RX / Interrupt / Câble : Codeur axeY CHA / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_axeYChA
pinMode (IN_BpAxeX, INPUT); //pin nr. 22 /  / Câble : Mode manuel bouton sélection Axe X / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_BpAxeX
pinMode (IN_BpAxeY, INPUT); //pin nr. 23 /  / Câble : Mode manuel bouton sélection Axe Y / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_BpAxeY
pinMode (IN_BpAxeZ, INPUT); //pin nr. 24 /  / Câble : Mode manuel bouton sélection Axe Z / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_BpAxeZ
pinMode (IN_BpMulti, INPUT); //pin nr. 25 /  / Câble : Bouton multifonctions / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_BpMulti
pinMode (OUT_LedBpAxeX, OUTPUT); //pin nr. 26 /  / Câble : Led indiquant que Axe X est sélectionné / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LedBpAxeX
pinMode (OUT_LedBpAxeY, OUTPUT); //pin nr. 27 /  / Câble : Led indiquant que Axe Y est sélectionné / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LedBpAxeY
pinMode (OUT_LedBpAxeZ, OUTPUT); //pin nr. 28 /  / Câble : Led indiquant que Axe Z est sélectionné / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LedBpAxeZ
pinMode (OUT_VerineRouge, OUTPUT); //pin nr. 29 /  / Câble : Verine 701  sur L293D / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VerineRouge
pinMode (OUT_VerineJaune, OUTPUT); //pin nr. 30 /  / Câble : Verine 701  sur L293D / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VerineJaune
pinMode (OUT_VerineVerte, OUTPUT); //pin nr. 31 /  / Câble : Verine 701  sur L293D / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_VerineVerte
pinMode (OUT_LCD_RS_4, OUTPUT); //pin nr. 32 /  / Câble : LCd 20x2 / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LCD_RS_4
pinMode (OUT_LCD_E_6, OUTPUT); //pin nr. 33 /  / Câble : LCd 20x2 / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LCD_E_6
pinMode (OUT_LCD_DB4_11, OUTPUT); //pin nr. 34 /  / Câble : LCd 20x2 / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LCD_DB4_11
pinMode (OUT_LCD_DB5_12, OUTPUT); //pin nr. 35 /  / Câble : LCd 20x2 / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LCD_DB5_12
pinMode (OUT_LCD_DB6_13, OUTPUT); //pin nr. 36 /  / Câble : LCd 20x2 / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LCD_DB6_13
pinMode (OUT_LCD_DB7_14, OUTPUT); //pin nr. 37 /  / Câble : LCd 20x2 / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : OUT_LCD_DB7_14
pinMode (IN_FinCourseHaut, INPUT); //pin nr. 38 /  / Câble : capteur fin de course Axe Z haut / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_FinCourseHaut
pinMode (IN_FinCourseBas, INPUT); //pin nr. 39 /  / Câble : capteur fin de course Axe Z bas / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_FinCourseBas
pinMode (IN_FinCourseDroite, INPUT); //pin nr. 40 /  / Câble : capteur fin de course Axe X droite / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_FinCourseDroite
pinMode (IN_FinCourseGauche, INPUT); //pin nr. 41 /  / Câble : capteur fin de course Axe X gauche / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_FinCourseGauche
pinMode (IN_FinCourseAvant, INPUT); //pin nr. 42 /  / Câble : capteur fin de course Axe Y avant / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_FinCourseAvant
pinMode (IN_FinCourseArriere, INPUT); //pin nr. 43 /  / Câble : capteur fin de course Axe Y arrière / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : IN_FinCourseArriere

}

// **************************************************************************************************
// ***************************** LOOP  **************************************************************
// **************************************************************************************************
void loop() {

  static unsigned long millisMemDebug = 0L;

  if((debug) && (millis()-millisMemDebug > 90)){
    tempoDebug = true;
    millisMemDebug = millis();
  }

Manivelle(10);
CodeurAxeZ();
PID();
Graphic();

//detection appui court / long sur BP
  BPStart.BP = shortLong(true, IN_BpAxeX, BPStart.shortPress, BPStart.longPress, BPStart.buttonTimer, BPStart.buttonActive, BPStart.longPressActive, BPStart.outputState);
  BPSelect.BP = shortLong(true, IN_BpAxeY, BPSelect.shortPress, BPSelect.longPress, BPSelect.buttonTimer, BPSelect.buttonActive, BPSelect.longPressActive, BPSelect.outputState);

}

// **************************************************************************************************
// ************************ Acquisition Manivelle  **************************************************
void Manivelle(int increment) {
  static unsigned long millisMemDebug = 0L;
  static float valmmManivelleMem;
  static long oldPosition  = -999;
  long newPosition = manivelle.read(); 

  if (newPosition != oldPosition) {
    millisMemDebug = millis();
      if(newPosition < oldPosition){
        valmmManivelle = valmmManivelle + increment;
      }else{
        valmmManivelle = valmmManivelle - increment;
      }
      oldPosition = newPosition;
  }else{
    if(millis()-millisMemDebug > 100){
      valmmManivelle = valmmCodeurZ;
    }
    if (valmmManivelle != valmmManivelleMem){
      valmmManivelleMem = valmmManivelle;
      if(tempoDebug)Serial.print("valeur Manivelle = ");
      if(tempoDebug)Serial.print(valmmManivelle);
      if(tempoDebug)Serial.println(" mm");
    }
  }
}  

// **************************************************************************************************
// ************************ Acquisition Codeur Axe Z  *****************************************************
void CodeurAxeZ (){
  static float valmmCodeurMem;
  static long oldPosition  = -999;
  long newPosition = codeurZ.read();
  if (newPosition != oldPosition) {
      oldPosition = newPosition;
  }

    valmmCodeurZ = newPosition*factConvCodeurZ;

  if (valmmCodeurZ != valmmCodeurMem){
      valmmCodeurMem = valmmCodeurZ;
    if(tempoDebug)Serial.print("valeur Codeur = ");
    if(tempoDebug)Serial.print(valmmCodeurZ);
    if(tempoDebug)Serial.println(" mm");
  }
}

// **************************************************************************************************
// ************************** PID *****************************************************
void PID(){
static int delta;
const int hysteresis = 10;
const float kP = 1.5;
  delta = valmmCodeurZ - valmmManivelle;

  if (delta > 0) {
    digitalWrite(OUT_VarDirectionZ, LOW); // moteur sens direction gauche
  }
  else{
    digitalWrite(OUT_VarDirectionZ, HIGH); // moteur sens direction droite
  }

  vitMoteur = abs(delta) * kP;
  constrain(vitMoteur, 0, 255); // contraint valeure entre 0 et 255

  if(abs(delta) < hysteresis){
    vitMoteur = 0;
    digitalWrite(OUT_VarEnableZ, LOW);
         if(tempoDebug)Serial.print("on est dans l'hysteresis");
  }
  else{
      digitalWrite(OUT_VarEnableZ, HIGH);
  }


  digitalWrite(OUT_VarVitesseZ, vitMoteur); // écriture pwm vitesse moteur

 // digitalWrite(OUT_VarEnableZ, HIGH);
}

//***************************************************************************************************
// ************************ short Long BP  **********************************************************
boolean shortLong(boolean pullup, int IN_BP,boolean &shortPress,boolean &longPress, long &buttonTimer, boolean &buttonActive, boolean &longPressActive, boolean &outputState){
  const long longPressTime = 550; //[ms]durée en miliseconde pour caractériser une pression "longue"
  shortPress = false;
  longPress = false;
    if(pullup){
      outputState = !digitalRead(IN_BP);
    }else{
      outputState = digitalRead(IN_BP);
    }

  if (outputState) {
    if (!buttonActive) {buttonActive = true;buttonTimer = millis();}
    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {longPressActive = true;longPress = true;}
  } else {
    if (buttonActive == true) {if (longPressActive == true) {longPressActive = false;} else {shortPress = true;}buttonActive = false;}
  }
  return outputState;
}

void Graphic(){
  static unsigned long millisMemGraphic = 0L;

  if((graphic) && (millis()-millisMemGraphic > 70)){
    tempoGraphic = 1;
    millisMemGraphic = millis();
  }
  if(tempoGraphic){
    Serial.print("Manivelle:");
    Serial.print(valmmManivelle);
    Serial.print(",");
    Serial.print("Codeur:");
    Serial.print(valmmCodeurZ);
    Serial.print(",");
    Serial.print("Moteur:");
    Serial.print(vitMoteur);
    Serial.print(",");
    Serial.println();

  }
}
