// Projet commande base 308
// Willemin-Macodel
// Juin 2022 / Lenny Lab

#include <Encoder.h>
#include <PID_v1.h>

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
const boolean graphic = false; // on-off mode graphic

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

// G code
String currentReceivedString = {""}; //received string via serial com
String currentReceivedStringMem = {""};

int adressNumber = 0;


const int recipesMax = 16;
boolean recipesGCode[recipesMax]={}; // ? (si G:)25 X 24 Y 23 Z 22 F100; (si M:) 25 S100;
int recipesParam[recipesMax]={};  //G 25 ? 24 ? 23 ? 22 ? 100 //0 = X command / 1 = Y command / 2 = Z command / 3 = F command / 4 = S command

boolean storageMode = false;
double recipesPosition[4][recipesMax]={{0,0,0,0},{}}; //G 25 X ?? Y ?? Z ?? F ??; //0 = X value / 1 = Y value / 2 = Z value/ 3 = F value / 4 = S
int recipesFunction[recipesMax]={}; //G ?? X 24 Y 23 Z 22 F 100; //GCode number
double currentXPosition = 00.00; //[mm]  current shown average Z axis absolut position 
double currentYPosition = 00.00; //[mm]  current shown Z1 axis absolut position 
double currentZPosition = 00.00; //[mm]  current shown Z2 axis absolut position 
double xWantedPosition = 000.00;  //[mm]  requested X axis absolut/relativ position 
double yWantedPosition = 000.00;  //[mm]  requested Y axis absolut/relativ position
double zWantedPosition = 000.00;  //[mm]  requested Z axis absolut/relativ position

int XYAxisSpeed = 0;  //[mm/s]  currentOrder requested axis speed --> [G0X200Y100F???;]

double zWantedSpeed = 0; //mm/ 10 cycles // speed regulation

int autoModeRepetition = 0; //M99 cycle repetion number a
boolean autoModeStatus = false;
boolean zReferenceTaken = false;

boolean xyReferenceTaken = true; 

int ToolSpeed = 0;  //[tr/s]  currentOrder requested tool speed --> [M3S???;]



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

  //Serial READING
  if (Serial.available()){                                  // Something is given via serial com 
    currentReceivedString = serialRead(currentReceivedString);   // Reading what is given on serial port // typically: G0X102.23Y-132.23Z132.1F1300; M = 077; G = 071; ";" = 059 ; X = 088; Y = 089; Z = 090; F = 070; "." = 046; " " = 032
  }
  //G CODE INTERPRETER   
  if(currentReceivedString != currentReceivedStringMem){
    adressNumber = GCodeInterpreter (currentReceivedString);              // Converting the given string into separate variables
    currentReceivedStringMem = currentReceivedString;
  }

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

//***************************************************************************************************
// *************************** GRAPHIC **************************************************************
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
//***************************************************************************************************
// ************************ SERIAL READ *************************************************************
String serialRead(String currentReceivedString){// Reading information over serial coms
    char byteRead;
    char serialBuffer[64];        // 64 = Maximum size of the arduino's serial buffer
    bool flag = false;
    int i = 0; 
    do{
      byteRead = Serial.read(); // Reading the current char via serial
      serialBuffer[i]=byteRead; // (re)constructing the incomming string
      delay(1);                 // Time for the serial com to load the next char
      i++;
    }while(((byteRead != 59)||(i>63))&&(Serial.available() > 0));      // 59 means ";", so we stop the string construction here 10 = Line feed 13= Carriage Return
    serialBuffer[i] = 0;         // Strings must end whith "0"
    if (i>63){                    // i > 63 --> Maximum 64 char!
        flag = true;
        Serial.println("Maximum 64 characters, command line ignored");          // **** SERIAL PRINT ****
    } 
    if (byteRead != 59){
        flag = true;
        Serial.println(" ; is missing");          // **** SERIAL PRINT ****
    }
    while(Serial.available() > 0){  // Emptying entrance Buffer
        Serial.read();
    }
   if (flag){
        flag = false;
        Serial.println(" Commande effacee");     // **** SERIAL PRINT ****
        return currentReceivedString;
   }else{
        Serial.print("SerialRead Done : ");      // **** SERIAL PRINT ****
        Serial.println(serialBuffer + String(i,DEC));
        return serialBuffer;
   }    
}

//***************************************************************************************************
// ************************ GCODE INTERPRETER *******************************************************
int GCodeInterpreter(String currentReceivedString){ // is only executed once per instruction received 
    int currentOrderXPosition=0;
    int currentOrderYPosition=0;
    int currentOrderZPosition=0;
    int currentOrderFPosition=0;
    int currentOrderSPosition=0;
    int GCode =0;
  static int autoModeCounter = 0;
    GCode = ParseINT(currentReceivedString, 0);   // Finding what numbers follows "G" or "M" char 
    Serial.println("***GCODE INTERPRETER***"); 
    if (!storageMode){
      switch(currentReceivedString[0]){               // Switching between G or M code
    case 71:                                      // G code (G == 71 ASCII)
      switch(GCode){                              // Switching between G code number
      case 0:
        currentOrderXPosition = currentReceivedString.lastIndexOf('X'); //where is located "X" char?
        currentOrderYPosition = currentReceivedString.lastIndexOf('Y'); //...
        currentOrderZPosition = currentReceivedString.lastIndexOf('Z');
        currentOrderFPosition = currentReceivedString.lastIndexOf('F');
        if (currentOrderXPosition != -1){xWantedPosition = ParseFloat(currentReceivedString,currentOrderXPosition);}
        if (currentOrderYPosition != -1){yWantedPosition = ParseFloat(currentReceivedString,currentOrderYPosition);}    
        if (currentOrderZPosition != -1){zWantedPosition = ParseFloat(currentReceivedString,currentOrderZPosition);} 
        if (currentOrderFPosition != -1){XYAxisSpeed = ParseINT(currentReceivedString, currentOrderFPosition);}
        zWantedSpeed  = XYAxisSpeed;
        zWantedSpeed = constrain(zWantedSpeed,20,1000);
        XYAxisSpeed = constrain(XYAxisSpeed,0,5000);
    
      break;
      case 54:
      /*
          selectedRelativOffset
      */      
      break;
      case 55:
      /*
          selectedRelativOffset
      */
      break;
      default: Serial.println("unknow G command number"); break;
      break;
      }
    break;
    case 77:                                      //M code (M == 77 ASCII)
      switch (GCode){                             //Switching between M code number
      case 0:                                   //M0: REFERENCE
        xyReferenceTaken = false;

        zReferenceTaken = false;        
      break;
      case 3:                                    //M3: ON OFF Function 1 + speed           
        currentOrderSPosition = currentReceivedString.lastIndexOf('S');
        if (currentOrderSPosition != -1)ToolSpeed = ParseINT(currentReceivedString,currentOrderSPosition);  
      break;  
      case 20:                                   //M3: ON OFF Function 1 + speed           
        autoModeStatus = !autoModeStatus;
        autoModeCounter = autoModeRepetition;
      break;        
      case 21:                                   //M21: Storage Mode On / Off          
        storageMode = true;
      break; 
      case 22:                                   //M21: Storage Mode On / Off          
        adressNumber = 0;
      break;  
      case 99:                   //M99: Auto mode - Cycle 
        currentOrderSPosition = currentReceivedString.lastIndexOf('S');
        if (currentOrderSPosition != -1)autoModeRepetition = ParseINT(currentReceivedString,currentOrderSPosition); 
        if (!autoModeStatus) autoModeStatus=true;   
        if (autoModeCounter >= autoModeRepetition){
          autoModeCounter = 0;
          autoModeStatus = false;
        }
        autoModeCounter ++;
      break;          
      default:                                  //M number not readeable  --> Error        
      break;
      }
    break;
    default:                                      //!G and !M --> Error
    break;
      }
  }else{ // STORAGE MODE ACTIVE : G or M Command given while being under storage Mode  *** STORAGE STORAGE STORAGE STORAGE STORAGE STORAGE STORAGE STORAGE ***
    if (currentReceivedString[0] == 71){recipesGCode[adressNumber] = true;}else{recipesGCode[adressNumber] = false;} // Storing if G or M code
      switch(currentReceivedString[0]){               // Switching between G or M code
      case 71: // G code (G == 71 ASCII)
      switch (GCode){
      case 0:
        currentOrderXPosition = currentReceivedString.lastIndexOf('X'); //where is located "X" char?
        currentOrderYPosition = currentReceivedString.lastIndexOf('Y'); //...
        currentOrderZPosition = currentReceivedString.lastIndexOf('Z');
        currentOrderFPosition = currentReceivedString.lastIndexOf('F');
        if (currentOrderXPosition != -1){ //S'il y a qqch, on applique
          recipesPosition[0][adressNumber]  = ParseFloat(currentReceivedString,currentOrderXPosition);
        }else{ //S'il n'y a rien, on considère les consignes précédente comme valable
          if (adressNumber == 0){
            recipesPosition[0][adressNumber]  = currentXPosition; //Pour la première entrée, on prend les positions actuelles
          }else{
            recipesPosition[0][adressNumber]  = recipesPosition[0][adressNumber-1]; //ensuite on prend les consignes précédentes
          } 
        } 
        if (currentOrderYPosition != -1){ //S'il y a qqch, on applique
          recipesPosition[1][adressNumber]  = ParseFloat(currentReceivedString,currentOrderYPosition);
        }else{ //S'il n'y a rien, on considère les consignes précédente comme valable
          if (adressNumber == 0){
            recipesPosition[1][adressNumber]  = currentYPosition; //Pour la première entrée, on prend les positions actuelles
          }else{
            recipesPosition[1][adressNumber]  = recipesPosition[1][adressNumber-1]; //ensuite on prend les consignes précédentes
          } 
        } 
        if (currentOrderZPosition != -1){ //S'il y a qqch, on applique
          Serial.print("//S'il y a qqch, on applique " + String(currentOrderZPosition, DEC)); //   ***SERIAL PRINT***
          Serial.println("  currentReceivedString"); //   ***SERIAL PRINT***
          recipesPosition[2][adressNumber]  = ParseFloat(currentReceivedString,currentOrderZPosition);
          delay(1);
        }else{ //S'il n'y a rien, on considère les consignes précédente comme valable
          if (adressNumber == 0){
            Serial.println("//Pour la première entrée, on prend les positions actuelles"); //   ***SERIAL PRINT***
            recipesPosition[2][adressNumber]  = currentZPosition; //Pour la première entrée, on prend les positions actuelles
          }else{
            Serial.println("//ensuite on prend les consignes précédentes"); //   ***SERIAL PRINT***         
            recipesPosition[2][adressNumber]  = recipesPosition[2][adressNumber-1]; //ensuite on prend les consignes précédentes
          } 
        }
        if (currentOrderFPosition != -1){ //S'il y a qqch, on applique
          recipesPosition[3][adressNumber]  = ParseINT(currentReceivedString,currentOrderFPosition);
        }else{ //S'il n'y a rien, on considère les consignes précédente comme valable
          if (adressNumber == 0){
            recipesPosition[3][adressNumber]  = 1000; //Pour la première entrée, on considère par défaut une vitesse d'avance à 1000.
          }else{
            recipesPosition[3][adressNumber]  = recipesPosition[3][adressNumber-1]; //ensuite on prend les consignes précédentes
          } 
        }          
          
              recipesFunction[adressNumber] = 0; //G ?? X 24 Y 23 Z 22 F 100; //GCode number
        Serial.println("Storing G0 commands "); //   ***SERIAL PRINT***
        Serial.println("X Stored Position : " + String(recipesPosition[0][adressNumber],DEC)); //     ***SERIAL PRINT***
        Serial.println("Y Stored Position : " + String(recipesPosition[1][adressNumber],DEC)); //     ***SERIAL PRINT***          
        Serial.println("Z Stored Position : " + String(recipesPosition[2][adressNumber],DEC)); //     ***SERIAL PRINT***
        Serial.println("F Stored Speed : " + String(recipesPosition[3][adressNumber],DEC)); //     ***SERIAL PRINT***         
        Serial.println("Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***
      break;
      case 4: //G4X12; 12 second waiting time
              recipesFunction[adressNumber] = 4;
        recipesPosition[5][adressNumber] = ParseFloat(currentReceivedString,currentOrderXPosition);
        Serial.print("Storing G4 commands "); //   ***SERIAL PRINT***
        Serial.print("  Waiting Time : " + String(recipesPosition[5][adressNumber],DEC)); //     ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***
      break;
          case 54: 
        recipesFunction[adressNumber] = 54;          
        Serial.print ("Storing G54 commands "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***
          break;
          case 55: 
        recipesFunction[adressNumber] = 55;
        Serial.print("Storing G55 commands "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***
          break;
      default:
        Serial.println("Storage Mode: Unknown GCode Command"); //   ***SERIAL PRINT***          
      break;
      }
    break;
    case 77: //M code (M == 77 ASCII)
      switch (GCode){                             //Switching between M code number
      case 0:                                   //M0:
        recipesFunction[adressNumber] = 0;
        Serial.print ("Storing M0 commands "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***       
      break;
      case 3:                                   //M3: ON OFF Function 1 + speed           
        recipesFunction[adressNumber] = 3; 
        Serial.print ("Storing M3 commands "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***         
      break; 
      case 20:                                   //M21: Storage Mode On / Off          
        recipesFunction[adressNumber] = 4;
        Serial.print ("Storing M20 commands "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***         
      break;          
      case 21:                                   //M21: Storage Mode On / Off          
        storageMode = false;
        Serial.print (" storageMode = false "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***         
      break;    
      case 22:                                   //M22: Storage buffer flush   
        Serial.print (" storageMode = false "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***              
        adressNumber = 0;
      break;  
      case 99:                                   //M99: Storage - Repeat cycle          
        recipesFunction[adressNumber] = 99; // if (!autoModeStatus) autoModeStatus=true;  
        recipesParam[adressNumber] = 4;  // "S"           
        currentOrderSPosition = currentReceivedString.lastIndexOf('S');
        if (currentOrderSPosition != -1)recipesPosition[4][adressNumber] = ParseINT(currentReceivedString,currentOrderSPosition);   
        Serial.print ("Storing M99 commands "); //   ***SERIAL PRINT***
        Serial.println("  Command number : " + String(adressNumber,DEC)); //         ***SERIAL PRINT***         
      break;          
      default:                                  //M number not readeable  --> Error        
      break;
      }
    break;
    }
        adressNumber++;
        if (adressNumber == (recipesMax-4)){
          Serial.println("Beware command buffer will be full soon!");  //     ***SERIAL PRINT***
    }
        if (adressNumber > recipesMax){
          adressNumber = recipesMax-1;
          Serial.println("Too much command entered.");  //     ***SERIAL PRINT***
    }
  }   
  return adressNumber;
}

//***************************************************************************************************
// ************************ PARSE FLOAT *************************************************************
double ParseFloat(String currentOrder, int searchFrom){
      String outPutStr = {""};
      int i=searchFrom;// searching from given position
      int decimalCount; 
      int decimalLimiter; // to have only 2 decimal after 0.00
      do{         
        i++;        
        outPutStr += currentOrder[i];
        if (currentOrder[i] == 46){
            decimalCount = i; // at the "i" position, there is a dot ".". We only want 2 decimal float numbers.
            decimalLimiter = i+2;
          }
        if (decimalLimiter == i)break;
       }while((isdigit(currentOrder[i])||(currentOrder[i]==46)||(currentOrder[i]==45)));  // searching for every numbers including "." (46) and "-" (45) ...
     double XYZPosition = outPutStr.toFloat(); //... then converting them into a floating var
     return XYZPosition;
}
//***************************************************************************************************
// ************************ PARSE INT ***************************************************************
int ParseINT(String currentOrder, int searchFrom){ 
      String outPutStr="";
      int i=searchFrom;// searching from xyz letter's given positions
      do{   
        i++;      
       outPutStr += currentOrder[i];
     }while(isdigit(currentOrder[i]));  // searching for every numbers   ...
     int GFValue = outPutStr.toInt(); //... then converting them into a integer var
     return GFValue;
}
