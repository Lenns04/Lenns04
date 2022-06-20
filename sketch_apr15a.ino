#include <Wire.h>
#include "Adafruit_ADS1015.h"
#include <Adafruit_MCP23017.h>
#include <Encoder.h>
#include <avr/wdt.h>

Adafruit_MCP23017 mcp1;
Adafruit_ADS1015 ads1(72);     /* Use this for the 12-bit version, 73 if the jumper is shorted) */


  // pin nr. 0  FREE
  // pin nr. 1  FREE
const int IN_ManivelleA = 2;   // pin nr. 2 / PWM / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_CodeurXA = 3;   // pin nr. 3 / PWM / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_ManivelleB = 4;   // pin nr. 4 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_CodeurXB = 5;   // pin nr. 5 / PWM / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
  // pin nr. 6  FREE
  // pin nr. 7  FREE
  // pin nr. 8  FREE
  // pin nr. 9  FREE
  // pin nr. 10  FREE
  // pin nr. 11  FREE
  // pin nr. 12  FREE
  // pin nr. 13  FREE
  // pin nr. 14  FREE
  // pin nr. 15  FREE
  // pin nr. 16  FREE
  // pin nr. 17  FREE
const int IN_CodeurYA = 18;   // pin nr. 18 / TX / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_CodeurZA = 19;   // pin nr. 19 / RX / Interrupt / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
  // pin nr. 20  FREE
  // pin nr. 21  FREE
const int IN_CodeurYB = 22;   // pin nr. 22 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_CodeurZB = 23;   // pin nr. 23 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_ReferenceX = 24;   // pin nr. 24 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_ReferenceY = 25;   // pin nr. 25 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_ReferenceZ = 26;   // pin nr. 26 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_HorsCourseX = 27;   // pin nr. 27 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_HorsCourseY = 28;   // pin nr. 28 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_HorsCourseZ = 29;   // pin nr. 29 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
  // pin nr. 30  FREE
  // pin nr. 31  FREE
  // pin nr. 32  FREE
  // pin nr. 33  FREE
  // pin nr. 34  FREE
  // pin nr. 35  FREE
  // pin nr. 36  FREE
  // pin nr. 37  FREE
const int OUT_EnableX = 38;   // pin nr. 38 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_EnableY = 39;   // pin nr. 39 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_EnableZ = 40;   // pin nr. 40 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_SensRotationX = 41;   // pin nr. 41 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_SensRotationY = 42;   // pin nr. 42 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_SensRotationZ = 43;   // pin nr. 43 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_VitesseX = 44;   // pin nr. 44 / (PWM mega) / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_VitesseY = 45;   // pin nr. 45 / (PWM mega) / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int OUT_VitesseZ = 46;   // pin nr. 46 / (PWM mega) / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_BPX = 47;   // pin nr. 47 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_BPY = 48;   // pin nr. 48 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_BPZ = 49;   // pin nr. 49 /  / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_BP_Ref = 50;   // pin nr. 50 / (spi) / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
const int IN_BP_Start = 51;   // pin nr. 51 / (spi) / Nom Câble :  / Couleur Fil sur Arduino :  / Couleur Fil Suivant :  / Couleur Fil Destination : 
  // pin nr. 52  FREE
  // pin nr. 53  FREE





long positionPulse = 0L;
double facteur = 4104/(50*3.141);  //nb pulse sur un tour, Øroue dentée, Pi. //4104 pulse pour 157.05mm (Ø50mm) soit 26.1318 pulse / mm
double position = positionPulse/facteur;
double wantedPos = 0;
long wantedPosPulse = wantedPos * facteur;
boolean sensRotation = false;
int vitesse = 0;
long oldPosition  = -999;

volatile long currentZ1PositionPulse = 0L;//[pulse]  current Z1 axis absolut position  / interrupt protected
volatile long currentZ2PositionPulse = 0L;//[pulse]  current Z2 axis absolut position  / interrupt protected
volatile long currentManivellePositionPulse = 0L; //[pulse]  current Man axis absolut position  / interrupt protected 

int xMotorPower = 100; //Z1 motor power (0-255 = 0-100%) Used to regulate position and speed between all Z Axis.
int yMotorPower = 100; //Z2 motor power (0-255 = 0-100%) Used to regulate position and speed between all Z Axis.
double motorZ1PwrAvg = 0; //used to determinate 500 cycle average motor power
double motorZ2PwrAvg = 0; //used to determinate 500 cycle average motor power  
double ecartPoursuiteX = 0; //used to determinate instant difference bewteen virtual position and current position
double ecartPoursuiteY = 0; //used to determinate instant difference bewteen virtual position and current position
double ecartPoursuiteXAvg = 0; //used to determinate 500 cycle average difference bewteen virtual position and current position
double ecartPoursuiteYAvg = 0; //used to determinate 500 cycle average difference bewteen virtual position and current position
int Z1State = 0;
int Z2State = 0;
int z1MemState = 0; //Acceleration mgmt
int z2MemState = 0; //Acceleration mgmt
int z1Counter = 0; //Acceleration mgmt
int z2Counter = 0; //Acceleration mgmt
boolean z1Accel = false; //Acceleration mgmt
boolean z2Accel = false; //Acceleration mgmt
boolean z1EnCours = false; //Acceleration mgmt
boolean z2EnCours = false; //Acceleration mgmt
int xRegulICounter = 0; //Movement mgmt
int yRegulICounter = 0; //Movement mgmt
double z1WantedPosition = 000.00;  //[mm]  requested Z1 axis absolut/relativ position
double z2WantedPosition = 000.00;  //[mm]  requested Z2 axis absolut/relativ position
double zWantedSpeed = 100; //mm/ 10 cycles // speed regulation
double xWantedSpeed = 1000; //mm/ 10 cycles // speed regulation
double yWantedSpeed = 300; //mm/ 10 cycles // speed regulation
double z2WantedSpeed = 50; //mm/ 10 cycles // speed regulation
double xDeltaVirtual = 00.00; //Movement mgmt
double z2deltaVirtual = 00.00; //Movement mgmt
double xDeltaVirtualMem = 00.00; //Movement mgmt
double z2deltaVirtualMem = 00.00; //Movement mgmt
boolean z1ReferenceTaken = false; 
boolean z2ReferenceTaken = false;  
boolean zReferenceTaken = false; 
boolean noZMotors = false;
boolean noXYMotors = false; 
boolean xCoderInversion = false;
boolean yCoderInversion = false;

long currentXPositionPulse = 0L;//[pulse]  current X axis absolut position
double currentXPositionMch = 00.00; //[mm]  current X axis absolut position 
long currentYPositionPulse = 0L;//[pulse]  current Y axis absolut position
double currentYPositionMch = 00.00; //[mm]  current Y axis absolut position 
long currentZPositionPulse = 0L;//[pulse]  current Y axis absolut position
double currentZPositionMch = 00.00; //[mm]  current Z axis absolut position 

double xVirtualPosition = 00.00; //[mm] virtual position for speed regulation
double yVirtualPosition = 00.00; //[mm] virtual position for speed regulation
double zVirtualPosition = 00.00; //[mm] virtual position for speed regulation
double currentManivellePosition = 00.00;       //[mm]  current Man axis absolut position  
double currentManivellePositionMch = 00.00; 			//[mm]  current Man axis absolut position
double currentXPosition = 00.00; //[mm]  current shown average Z axis absolut position 
double currentYPosition = 00.00; //[mm]  current shown Z1 axis absolut position 
double currentZPosition = 00.00; //[mm]  current shown Z2 axis absolut position  
double xPositionG55Offset =  000.00;//[mm] //distance between G55 position and reference Pin
double yPositionG55Offset =  000.00;//[mm] //distance between G55 position and reference Pin
double zPositionG55Offset =  000.00;//[mm] //distance between G55 position and reference Pin
double xPositionG56Offset =  000.00;//[mm] //position Memory
double yPositionG56Offset =  000.00;//[mm] //position Memory
double zPositionG56Offset =  000.00;//[mm] //position Memory
double xPositionG57Offset =  000.00;//[mm] //position Memory //used to zero position 
double yPositionG57Offset =  000.00;//[mm] //position Memory
double zPositionG57Offset =  000.00;//[mm] //position Memory
double xWantedPosition = 000.00;  //[mm]  requested X axis absolut/relativ position 
double yWantedPosition = 000.00;  //[mm]  requested Y axis absolut/relativ position
double zWantedPosition = 000.00;  //[mm]  requested Z axis absolut/relativ position

long wantedSteps[2]={0,0}; //{x,y,z} This what will be given to multistepper functions 
int XYAxisSpeed = 0;  //[mm/s]  currentOrder requested axis speed --> [G0X200Y100F???;]
int ToolSpeed = 0;  //[tr/s]  currentOrder requested tool speed --> [M3S???;]
int XYState = 0;
int ZState = 0;
int mainState = 4;
const double xTotalLength = 370.00;  //[mm]  total length of the X axis 
const double yTotalLength = 255.00;  //[mm]  total length of the Y axis
const double zTotalLength = 255.00;  //[mm]  total length of the Z axis
double xWantedPositionSteps = 0; //[steps] currentOrder requested X axis absolut position 
double yWantedPositionSteps = 0; //[steps] currentOrder requested Y axis absolut position 

double xMaxSteps = 1530*8; //total number of steps for this axis
double yMaxSteps = 1080*8; //total number of steps for this axis 

int xMotorSpeed = 1000;
int yMotorSpeed = 1000;
int zMotorSpeed = 100;
int xPulleyFactor = 200;
int yPulleyFactor = 200;
int zPulleyFactor = 1;

String currentReceivedString = {""}; //received string via serial com
String currentReceivedStringMem = {""};
boolean xyReferenceTaken = true; 

const int recipesMax = 16;
boolean recipesGCode[recipesMax]={}; // ? (si G:)25 X 24 Y 23 Z 22 F100; (si M:) 25 S100;
int recipesParam[recipesMax]={};  //G 25 ? 24 ? 23 ? 22 ? 100 //0 = X command / 1 = Y command / 2 = Z command / 3 = F command / 4 = S command
int recipesFunction[recipesMax]={}; //G ?? X 24 Y 23 Z 22 F 100; //GCode number
double recipesPosition[4][recipesMax]={{0,0,0,0},{}}; //G 25 X ?? Y ?? Z ?? F ??; //0 = X value / 1 = Y value / 2 = Z value/ 3 = F value / 4 = S
boolean storageMode = false;
int adressNumber = 0;
int orderNumber = 0;
boolean autoModeStatus = false;
int autoModeRepetition = 0; //M99 cycle repetion number a
int cycle500Counter = 0;
boolean debugMode = true;
int mode = 0; //mode: 0= Manivelle / 1: Auto / 2: G-Code 
int currentSelectedAxis=0; //current selected axis X = 0 / Y = 1 / Z = 2 .. Z1 = 11 / Z2 = 12 / Z3 = 13 / Z4 = 14
int selectedRelativOffset = 0; // 0 : Machine / 1: G55 / 2: G56
int manualZMvtSelection = 0; // idem state
int dataShown = 0; //0 = Position pulses / 1 = Position Mch / 2 = MotorPower / 3 = axis Delta / 4 = 
double cycleTime = 0;  //watchdog

Encoder codeurX(18, 16); //3,5
Encoder codeurY(51, 52); //18,22
//Encoder codeurZ(19, 23); //19,23
Encoder manivelle(15, 17); //2,4

//********************* pour verification de la mémoire restant
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
//*********************
//***************************************************************************************************
// ************************ SETUP *******************************************************************
void setup() {  
  mcp1.begin(1);      // use default address 0
  Serial.begin(115200);
  ads1.setGain(GAIN_ONE);
  ads1.begin();
  mcp1.pinMode(0, OUTPUT); //enableX
  mcp1.pinMode(1, OUTPUT); //Sens rotation X
  //mcp1.pinMode(3, OUTPUT); //analog X 0-10V (vitesse)
  pinMode (44, OUTPUT);  //pin nr. 44 / (PWM mega) / Câble :  / Couleur Nappe :  / Couleur Câble :  / Couleur Destination : 

  mcp1.pinMode(8, INPUT);
  wdt_enable(WDTO_500MS); // watch dog - Si le CPU ne réagit pas pendant 1seconde, il relance.
}
//***************************************************************************************************
// ************************ MAIN LOOP****************************************************************
void loop() {
	//Serial READING
	if (Serial.available()){                                  // Something is given via serial com 
		currentReceivedString = serialRead(currentReceivedString);   // Reading what is given on serial port // typically: G0X102.23Y-132.23Z132.1F1300; M = 077; G = 071; ";" = 059 ; X = 088; Y = 089; Z = 090; F = 070; "." = 046; " " = 032
	}
	//G CODE INTERPRETER   
	if(currentReceivedString != currentReceivedStringMem){
		adressNumber = GCodeInterpreter (currentReceivedString);              // Converting the given string into separate variables
		currentReceivedStringMem = currentReceivedString;
	}
	ihm();
	//delay(100);
	mode = 1;
	positionTranslation();
	xVirtualPosition = virtualPosition(xVirtualPosition, currentXPositionMch, xWantedPosition, xWantedSpeed);
	Z1State = ZMovementMgmt(xRegulICounter, xMotorPower, xWantedPosition, currentXPositionMch, xVirtualPosition, xDeltaVirtual, xDeltaVirtualMem, 0.05);
	//xMotorPower = zAccelMgmt(Z1State, z1MemState, z1Counter, z1Accel, z1EnCours, xMotorPower);
	Z1State = zMotorsMgmt(Z1State,xMotorPower,OUT_EnableX,OUT_SensRotationX,true/* IN_ContactZ1 */); 
	//Surveillance du temps de cycle UC et mis à jour des clocks et counter
	watchDog (cycleTime);
	//Debug I/O to serial monitor 
	IOTest();
	wdt_reset();
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
        Serial.println("le caractère ';' est manquant");          // **** SERIAL PRINT ****
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
    GCode = ParseINT(currentReceivedString, 0); 	// Finding what numbers follows "G" or "M" char 
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
				//if (currentOrderFPosition != -1){XYAxisSpeed = ParseINT(currentReceivedString, currentOrderFPosition);}
				XYAxisSpeed = 500;
				zWantedSpeed  = XYAxisSpeed;
			break;
			case 1:
				currentOrderXPosition = currentReceivedString.lastIndexOf('X'); //where is located "X" char?
				currentOrderYPosition = currentReceivedString.lastIndexOf('Y'); //...
				currentOrderZPosition = currentReceivedString.lastIndexOf('Z');
				currentOrderFPosition = currentReceivedString.lastIndexOf('F');
				if (currentOrderXPosition != -1){xWantedPosition = ParseFloat(currentReceivedString,currentOrderXPosition);}
				if (currentOrderYPosition != -1){yWantedPosition = ParseFloat(currentReceivedString,currentOrderYPosition);}    
				if (currentOrderZPosition != -1){zWantedPosition = ParseFloat(currentReceivedString,currentOrderZPosition);} 
				if (currentOrderFPosition != -1){XYAxisSpeed = ParseINT(currentReceivedString, currentOrderFPosition);}
				zWantedSpeed  = XYAxisSpeed;
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
			case 99:								   //M99: Auto mode - Cycle 
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
		Serial.print("X wanted position: ");
		Serial.println(String(xWantedPosition,DEC));
		Serial.print("Y wanted position: ");
		Serial.println(String(yWantedPosition,DEC));
		Serial.print("Z wanted position: ");
		Serial.println(String(zWantedPosition,DEC));
		Serial.print("Speed: ");
		Serial.println(String(XYAxisSpeed,DEC));		
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
				recipesParam[adressNumber] = 4;	 // "S" 					
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
// ************************ HMI Inputs acquisition **********************************************************
void ihm(){
	//Handwheel
	currentManivellePositionPulse = manivelle.read();
	currentManivellePositionMch = currentManivellePositionPulse / 1.0L;  
}
//*********************S******************************************************************************
// ************************ positionTranslation **********************************************************
void positionTranslation(){//everything about measures 
  //  byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
  //  noInterrupts ();   // turn interrupts off  
  // *** Z AXIS ***
  currentXPositionPulse = codeurX.read();
  currentYPositionPulse = codeurY.read();
  if (!xCoderInversion){currentXPositionMch = currentXPositionPulse / facteur;}else{currentXPositionMch = currentXPositionPulse / -(facteur);}   
  if (!yCoderInversion){currentYPositionMch = currentYPositionPulse / 120.0L;}else{currentYPositionMch = currentYPositionPulse / -120.0L;} 

  //  SREG = oldSREG;    // turn interrupts back on, if they were on before  /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/! 
 // z1WantedPosition = zWantedPosition;
  //z2WantedPosition = zWantedPossition;
	switch(mode){
		case 0:{ // mode MANU
			switch(currentSelectedAxis){
				case 0:{ // axe X
					xWantedPosition = currentManivellePositionMch;
				}break;
				case 1:{ // axe Y
					yWantedPosition = currentManivellePositionMch;
				}break;
				case 2:{ // axe Z
					zWantedPosition = currentManivellePositionMch;
				}break;
			}
		}break;
		case 1:{ // mode AUTO

		}break;
		case 2:{ // mode G-Code

		}break;
	}

  // *** X & Y AXIS *** Conversion step / [mm]   
  currentXPositionMch = currentXPositionPulse / facteur; 
  currentYPositionMch = currentYPositionPulse / facteur; 
  currentZPositionMch = currentZPositionPulse / facteur; 
  // X & Y Axis speed
  	switch(mode){
		case 0:{ // mode MANU
			switch(currentSelectedAxis){
				case 0:{ // axe X
					xWantedPosition = currentManivellePositionMch;
				}break;
				case 1:{ // axe Y
					yWantedPosition = currentManivellePositionMch;
				}break;
				case 2:{ // axe Z
					zWantedPosition = currentManivellePositionMch;
				}break;
			}
		}break;
		case 1:{ // mode AUTO
			xWantedSpeed=constrain(XYAxisSpeed,0,5000); 
			yWantedSpeed=constrain(XYAxisSpeed,0,5000);  
			zWantedSpeed=constrain(XYAxisSpeed,0,5000);  
		}break;
	}

  // General State overwatch
  if((Z1State == 2) && (Z2State == 2) && (XYState == 2)){mainState = 2;}else{mainState = 0;}
  // calcul de la moyenne de Z
  //currentZPositionMch = (currentXPositionMch + currentYPositionMch)/2; //position Z affichée, calcule de la moyenne
  // suivi de l'erreur de poursuite Z
  ecartPoursuiteX = currentXPositionMch - xVirtualPosition;
  ecartPoursuiteY = currentYPositionMch - yVirtualPosition;  
  ecartPoursuiteXAvg += ecartPoursuiteX; 
  ecartPoursuiteYAvg += ecartPoursuiteY;

  // Suivi de la moyenne de la puissance moteur
  motorZ1PwrAvg += xMotorPower;
  motorZ2PwrAvg += yMotorPower;
  
  // Calcul des moyennes sur 500 cycles
  if (cycle500Counter == 0){
    ecartPoursuiteXAvg = 0;
    ecartPoursuiteYAvg = 0;
    motorZ1PwrAvg = 0;
    motorZ2PwrAvg = 0;

  }
  // *** OFFSET ***
  switch (selectedRelativOffset){
    case 0: // machine
      currentXPosition = currentXPositionMch;
      currentYPosition = currentYPositionMch;
      currentZPosition = currentZPositionMch;
    break;
    case 1: // G55
      currentXPosition = currentXPositionMch - xPositionG55Offset;
      currentYPosition = currentYPositionMch - yPositionG55Offset; 
      currentZPosition = currentZPositionMch - zPositionG55Offset;  
    break;
    case 2: // G56
      currentXPosition = currentXPositionMch - xPositionG56Offset;
      currentYPosition = currentYPositionMch - yPositionG56Offset; 
      currentZPosition = currentZPositionMch - zPositionG56Offset;
    break;
    case 3: // IHM
      currentXPosition = currentXPositionMch - xPositionG57Offset;
      currentYPosition = currentYPositionMch - yPositionG57Offset;
      currentZPosition = currentZPositionMch - zPositionG57Offset;
    break;
  }
}
//***************************************************************************************************
// ************************  Virtual Position for speed regulation  ********************************
double virtualPosition (double virPos, double currentPosition, double wantedPosition, double wantedSpeed){
	static double counterb = 0;
	static double wantedPositionMem = wantedPosition;
	static double delta = currentZPosition - wantedPosition;
	static boolean wayToGo = false;
	static double distancePerCycle = 00.00;
	static double neededCycle = 0;
	static double travelTime = 00.00;
	// New position entered
	if (wantedPositionMem != wantedPosition){
		wantedPositionMem = wantedPosition;
		counterb = 0;
		//setting start position
		delta = currentPosition - wantedPosition;
		virPos = currentPosition;
		delta = abs(delta);
		// setting wayToGo
		if (wantedPosition > currentPosition){wayToGo = true;}else{wayToGo = false;}
		// Speed translation : 0-1000[] ==> 0-5.0 mm/sec
		wantedSpeed = ((wantedSpeed - 0.0)*(5.0-0.0) / (1000.0 - 0.0) + (0.0));// (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		// Travel time calculation
		travelTime = delta / wantedSpeed; //distance à parcourir divisé par vitesse demandé
		// How many cycle needed to reach this time
		neededCycle = travelTime / (cycleTime/1000);
		//what distance per cycle?
		distancePerCycle = delta / neededCycle;

		String bufferMonitor = {"***Virtual Position Calcultation*** \n"};
		bufferMonitor += {"wantedPosition :  " + String(wantedPosition,DEC)};
		bufferMonitor +=  "\n CurrentPosition " + String(currentPosition,DEC);   
		bufferMonitor +=  "\ndelta " + String(delta,DEC);
		bufferMonitor +=  "\n";   
		bufferMonitor +=  "wantedSpeed " + String(wantedSpeed,DEC);
		bufferMonitor +=  "\n";
		bufferMonitor +=  "travelTime " + String(travelTime,DEC);
		bufferMonitor +=  "\n";
		bufferMonitor +=  "neededCycle " + String(neededCycle,DEC);
		bufferMonitor +=  "\n";
		bufferMonitor +=  "distancePerCycle " + String(distancePerCycle,DEC);   
		bufferMonitor +=  "\n"; 
		bufferMonitor +=  "cycleTime " + String(cycleTime,DEC);   
		bufferMonitor +=  "\n"; 
		Serial.print (bufferMonitor);
	}
	if (counterb <= neededCycle){
		if (wayToGo){   // Going downward
			virPos += distancePerCycle;
			counterb ++;
		}else{  // Going upward
			virPos -= distancePerCycle;
			counterb ++;
		}   
	}
	return virPos;
}
//***************************************************************************************************
// ************************ Z axis Movements MGMT **********************************************************
int ZMovementMgmt(int &counter, int &motorSpeed, double wantedPosition, double currentPosition, double virtualPosition, double &delta, double &deltaMem, double hysteresys){ 
	int state = 4; // error mode per default
	double gainMoins = 0.9;
	double gainPlus = 1.2;
	boolean wayToGoPower = false;
	boolean wayToGoState = false;
	boolean flagDeltaChanging = false;  
	//***** 1 ***** Régulation de position autour d'une cible mouvante (virtualPosition) en jouant sur la puissance
	// Trouver quel sens
	if (virtualPosition > currentPosition){wayToGoPower = true;}else{wayToGoPower = false;} //la position reelle en relation avec la position virtuelle nous donne l'indication si l'on doit ralentir ou accelerer
	if (wantedPosition > currentPosition){wayToGoState = true;}else{wayToGoState = false;}  //la position reelle en relation avec la consigne nous donne le sens dans lequel aller
	// Quelle distance?
	delta = virtualPosition - currentPosition; // delta [mm]
	delta = abs(delta); 
	counter ++; // contre-productif de répéter cette opération trop souvent..
	if (counter > 100){  
		if (delta < deltaMem){ // Est-ce que la différence après 10 cycles diminue?
			flagDeltaChanging = true; // Oui, Bien! On se rapproche
		}else{
			flagDeltaChanging = false; // Non, Pas bien, on s'éloigne!
		}
		deltaMem = delta;
		
		if (!flagDeltaChanging){ // Il faut réagir
			if (!wayToGoPower){ // on est trop en avance, faut ralentir 	
				motorSpeed --;			
				//motorSpeed = motorSpeed * gainMoins;//gain Moins;
				//Serial.println("  /  motorSpeed --" + String(motorSpeed,DEC));
			}else{        // trop en retard, faut accelerer
				motorSpeed ++;
				//motorSpeed = motorSpeed * gainPlus;//gain Plus;
				//Serial.println("  /  motorSpeed ++" + String(motorSpeed,DEC));
			}
			motorSpeed = constrain(motorSpeed, 0,255);
		}
		counter = 0;
	}
	//***** 2 ***** Détermination de la sortie "state" ( => Sens haut (0)/ bas(1) / arret des moteurs (2)) *********************************************
	delta = wantedPosition - currentPosition; // delta [mm]
	delta = abs(delta); 

	if (delta > hysteresys){
		if (wayToGoState){
			state = 0; // going upward  
		}else{
			state = 1; // going downward
		}
	}else{state = 2;}

	// String bufferMonitor = "*******************************************************************************************";
	// bufferMonitor +=  "\n";
	// bufferMonitor +=  "*  Wanted mm : " + String(xWantedPosition,DEC);
	// bufferMonitor +=  " /  current mm : " + String(currentXPositionMch,DEC);
	// bufferMonitor +=  " /  virtualPosition mm : " + String(virtualPosition,DEC);	  
	// bufferMonitor +=  "\n";	   
    // bufferMonitor +=  "* Delta X : " + String(delta,DEC); 
    // bufferMonitor +=   " xMotorPower : " + String(xMotorPower,DEC);
	// bufferMonitor +=   " wayToGoPower : " + String(wayToGoPower,DEC);	
    // bufferMonitor +=  " wayToGoState : " + String(wayToGoState,BIN);   
    // bufferMonitor +=  " State : " + String(state,DEC);  
    // bufferMonitor +=   " FlagDeltaChanging : " + String(flagDeltaChanging,BIN);
	// bufferMonitor +=  "\n\n\n\n\n";
	// Serial.print (bufferMonitor);

	return state; 
}
//***************************************************************************************************
// ************************ z Accélération Management *********************************************** 
int zAccelMgmt (int state, int &memState,int &counter, boolean &accel, boolean &enCours, int motorPower){
  if (memState != state){    //changement d'état  
    enCours = true;
    if (state != 2){ // --> on accélère
        accel = true;
        counter = 1;
    }else{      
        accel = false;// --> on ralenti
    }
  }
  int motorPowerMem = motorPower+40;
  // cycle d'accélération en cours
  if (enCours){ 
    if (accel){
        motorPower = 5 * counter;
    }
    counter ++;
    if (counter > 20){counter = 1; enCours = false;} //fin des 20 cycles d'accélération .. 
  }     
  motorPower = constrain(motorPower, 0, motorPowerMem);
  memState = state;
  return motorPower;
}
//***************************************************************************************************
// ************************ zMotorsMgmt  ************************************************************
int zMotorsMgmt (int state, int &motorPower, int enable, int sensRotation, int contactPin){
 // static int counter = 0;
  boolean out = true;
  motorPower = constrain(motorPower, 0, 255);
    if (digitalRead(contactPin)){         //normal operation
		switch (state){
		case 0:                                   // going upward  
			mcp1.digitalWrite(0, HIGH); //enable
			mcp1.digitalWrite(1, HIGH); // sens rotation
			analogWrite(44, motorPower);
		break;
		case 1:	  // going downward
			mcp1.digitalWrite(0, HIGH); //enable
			mcp1.digitalWrite(1, LOW); // sens rotation
			analogWrite(44, motorPower);			
		break;
		case 2:                                  // in position
			mcp1.digitalWrite(0, LOW); //enable
			mcp1.digitalWrite(1, LOW); // sens rotation
			analogWrite(44, 0);
		break;
		}
    }else{ //in case mobile touching axis contact sensor
      out = false;
         // analogWrite (motorPinA, 255); 
         // analogWrite (motorPinB, 0);
    }
 /* if (!out){
      state = 4;                               //Contact sensor
      counter++;

      digitalWrite (motorPinA, LOW);
      digitalWrite (motorPinB, HIGH);
      if (counter > 200){
        counter = 0;
        out = true;
        digitalWrite (motorPinA, LOW); 
        digitalWrite (motorPinB, LOW);
      }
   }*/
   return state;
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
//***************************************************************************************************
// ************************  watchDog       *********************************************************
void watchDog (double &cycleTime){
	static int counter = 0;
	static unsigned long timeMem = 0L;  //watchdog
    if (counter == 99){timeMem = millis ();}
    if (counter >= 100){cycleTime = millis() - timeMem; counter = 0;}
	counter ++;
	cycle500Counter ++;
	cycleTime = constrain(cycleTime,0.1,999.9);
	if (cycle500Counter > 500){cycle500Counter = 0;}
}
//***************************************************************************************************
// ************************ Memory Check  ********************************************************
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
//***************************************************************************************************
// ************************  IOTest  ****************************************************************
void IOTest(){
//
//  if(digitalRead(IN_CodeurZ1ChB)){Serial.println("IN_CodeurZ1ChB");}
//  if(digitalRead(IN_CodeurZ2ChB)){Serial.println("IN_CodeurZ2ChB");}
//  if(digitalRead(IN_CodeurZ1ChA)){Serial.println("IN_CodeurZ1ChA");}
//  if(digitalRead(IN_CodeurZ2ChA)){Serial.println("IN_CodeurZ2ChA");}

  int memoryLeft = freeMemory();
  String bufferMonitor = {""};  
	if ((cycle500Counter == 500)/* ||(cycle500Counter == 100)||(cycle500Counter == 200)||(cycle500Counter == 300)||(cycle500Counter == 400) */){
  	bufferMonitor += "*******************************************************************************************";
	bufferMonitor +=  "\n";
	bufferMonitor +=  "* ---> Cycle Time (ms) :  " + String(cycleTime,DEC);	 
	bufferMonitor +=  "  <--- ---> Memory left : " + String(memoryLeft,DEC);	  
	bufferMonitor +=  " <---\n\n";
	bufferMonitor +=  "* Current Selected Axis : " + String(currentSelectedAxis,DEC);
	bufferMonitor +=  " / Current Mode : " + String(mode,DEC);
	bufferMonitor +=  "\n";
	bufferMonitor +=  "* XY state : " + String(XYState,DEC);
	bufferMonitor +=  "\n";
	bufferMonitor +=  "* X Wanted mm : " + String(xWantedPosition,DEC);
	bufferMonitor +=  " / X current mm : " + String(currentXPositionMch,DEC);
	bufferMonitor +=  "\n";	  
	bufferMonitor +=  "* Y Wanted mm : " + String(yWantedPosition,DEC);
	bufferMonitor +=  " / Y current mm : " + String(currentYPositionMch,DEC);   
    bufferMonitor +=  "\n";      
    bufferMonitor +=  "* Z Wanted mm : " + String(zWantedPosition,DEC);
    bufferMonitor +=  " / Z current mm : " + String(currentZPositionMch,DEC);   
    bufferMonitor +=  "\n";
    bufferMonitor +=  "* ecartPoursuite X : " + String(ecartPoursuiteX,DEC); 
    bufferMonitor +=  " Z1 State : " + String(Z1State,DEC);  
    bufferMonitor +=   " currentXPositionMch : " + String(currentXPositionMch,DEC);
    bufferMonitor +=   " motorXPwrAvg : " + String(xMotorPower,DEC);
    bufferMonitor +=  "\n* ecartPoursuite Y : " + String(ecartPoursuiteY,DEC); 
    bufferMonitor +=  " Z2 State : " + String(Z2State,DEC);   
    bufferMonitor +=   " currentYPositionMch : " + String(currentYPositionMch,DEC);
    bufferMonitor +=   " motorYPwrAvg : " + String(yMotorPower,DEC);
	  bufferMonitor +=  "\n\n\n\n\n";
	  Serial.print (bufferMonitor);
  }
}