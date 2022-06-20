  //*********************************************************************************************************************************
//*************/************
//************//************	GCodeAxes.V1.X Alpha						
//***********///************	
//***********///************	Soft de contrôle de "bidule", résultat du projet bancal "imprimante3D Version 5".	Starkius 08.2019
//*********//////*********** 
//**************************
//**************************	Matériel utilisé:
//**************************		
//**************************		Contrôleur:						      Arduino Mega 2560
//**************************		Pilote Moteur pas à Pas:		HY-DIV268N-5A Stepper driver
//**************************		Moteur pas à pas:				    Trinamic QSH4218-51-10-049 "NEMA 17"
//**************************		Encodeur incrémentaux:			HN3806-AB-400N (400 pulses)
//**************************		Pilotes Moteurs DC:				  SN754410
//**************************
//**********************************************************************************************************************************
#include <AccelStepper.h> //stepMotors control library
#include <MultiStepper.h> //stepMotors control library
#include <Wire.h>
#include <rgb_lcd.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Encoder.h>


byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 177);
EthernetServer server(80); // (port 80 HTTP):

rgb_lcd lcd;
int colorR = 255;
int colorG = 0;
int colorB = 0;

String projectName = "GCode3Axes";  

  // pin nr. 0  FREE
  // pin nr. 1  FREE
const int IN_ManivelleChA = 2;   // pin nr. 2 / PWM / Interrupt / Câble :  / Couleur Nappe : bu / Couleur Câble : gn / Couleur Destination : wh
  // pin nr. 3  FREE
  // pin nr. 4  FREE
  // pin nr. 5  FREE
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
const int IN_CodeurZ1ChB = 16;   // pin nr. 16 / TX / Câble : z1 / Couleur Nappe : og / Couleur Câble :  / Couleur Destination : 
const int IN_CodeurZ2ChB = 17;   // pin nr. 17 / RX / Câble : z2 / Couleur Nappe : ye / Couleur Câble :  / Couleur Destination : 
const int IN_CodeurZ1ChA = 18;   // pin nr. 18 / TX / Interrupt / Câble : z1 / Couleur Nappe : gn / Couleur Câble :  / Couleur Destination : 
const int IN_CodeurZ2ChA = 19;   // pin nr. 19 / RX / Interrupt / Câble : z2 / Couleur Nappe : bu / Couleur Câble :  / Couleur Destination : 
const int OUT_LCD_SDA = 20;   // pin nr. 20 / Interrupt / SDA / Câble : ihm / Couleur Nappe : vt / Couleur Câble : wh/gn / Couleur Destination : wh
const int OUT_LCD_SCL = 21;   // pin nr. 21 / Interrupt / SCL / Câble : ihm / Couleur Nappe : gy / Couleur Câble : wh/ye / Couleur Destination : ye
const int OUT_MoteurXEnable = 22;   // pin nr. 22 /  / Câble :  / Couleur Nappe : rd / Couleur Câble : rd / Couleur Destination : 
const int OUT_MoteurYEnable = 23;   // pin nr. 23 /  / Câble :  / Couleur Nappe : rd / Couleur Câble : rd / Couleur Destination : 
const int OUT_MoteurXPulse = 24;   // pin nr. 24 /  / Câble :  / Couleur Nappe : og / Couleur Câble : og / Couleur Destination : 
const int OUT_MoteurYPulse = 25;   // pin nr. 25 /  / Câble :  / Couleur Nappe : og / Couleur Câble : og / Couleur Destination : 
const int OUT_MoteurXDir = 26;   // pin nr. 26 /  / Câble :  / Couleur Nappe : ye / Couleur Câble : ye / Couleur Destination : 
const int OUT_MoteurYDir = 27;   // pin nr. 27 /  / Câble :  / Couleur Nappe : ye / Couleur Câble : ye / Couleur Destination : 
const int IN_BP_1 = 28;   // pin nr. 28 /  / Câble : ihm / Couleur Nappe : gn / Couleur Câble : ye/bn / Couleur Destination : 
const int IN_ContactX = 29;   // pin nr. 29 /  / Câble :  / Couleur Nappe : gn / Couleur Câble : gn / Couleur Destination : 
const int IN_BP_2 = 30;   // pin nr. 30 /  / Câble : ihm / Couleur Nappe : bu / Couleur Câble : gy/bn / Couleur Destination : 
const int IN_ContactY = 31;   // pin nr. 31 /  / Câble :  / Couleur Nappe : bu / Couleur Câble : bu / Couleur Destination : 
const int IN_BP_3 = 32;   // pin nr. 32 /  / Câble : ihm / Couleur Nappe : vt / Couleur Câble : wh/gy / Couleur Destination : 
const int IN_ContactZ1 = 33;   // pin nr. 33 /  / Câble :  / Couleur Nappe : vt / Couleur Câble : vt / Couleur Destination : 
const int IN_BP_4 = 34;   // pin nr. 34 /  / Câble : ihm / Couleur Nappe : gy / Couleur Câble : rd/bu / Couleur Destination : 
const int IN_ContactZ2 = 35;   // pin nr. 35 /  / Câble :  / Couleur Nappe : gy / Couleur Câble : gy / Couleur Destination : 
const int IN_BP_5 = 36;   // pin nr. 36 /  / Câble : ihm / Couleur Nappe : wh / Couleur Câble : gy/pk / Couleur Destination : 
  // pin nr. 37  FREE
const int IN_BP_6 = 38;   // pin nr. 38 /  / Câble : ihm / Couleur Nappe : bk / Couleur Câble : wh/pk / Couleur Destination : 
const int IN_ManivelleChB = 39;   // pin nr. 39 /  / Câble : ihm / Couleur Nappe : bk / Couleur Câble : ye / Couleur Destination : ye
  // pin nr. 40  FREE
  // pin nr. 41  FREE
const int OUT_MoteurZ1Dir = 42;   // pin nr. 42 /  / Câble :  / Couleur Nappe : rd / Couleur Câble :  / Couleur Destination : 
const int OUT_MoteurZ2Dir = 43;   // pin nr. 43 /  / Câble :  / Couleur Nappe : rd / Couleur Câble :  / Couleur Destination : 
const int OUT_MoteurZ1Pwm = 44;   // pin nr. 44 / (PWM mega) / Câble :  / Couleur Nappe : og / Couleur Câble :  / Couleur Destination : 
const int OUT_MoteurZ2Pwm = 45;   // pin nr. 45 / (PWM mega) / Câble :  / Couleur Nappe : og / Couleur Câble :  / Couleur Destination : 
  // pin nr. 46  FREE
  // pin nr. 47  FREE
  // pin nr. 48  FREE
  // pin nr. 49  FREE
  // pin nr. 50  FREE
  // pin nr. 51  FREE
  // pin nr. 52  FREE
  // pin nr. 53  FREE
  
volatile long currentZ1PositionPulse = 0L;//[pulse]  current Z1 axis absolut position  / interrupt protected
volatile long currentZ2PositionPulse = 0L;//[pulse]  current Z2 axis absolut position  / interrupt protected
volatile long currentManivellePositionPulse = 0L; //[pulse]  current Man axis absolut position  / interrupt protected 

int z1MotorPower = 50; //Z1 motor power (0-255 = 0-100%) Used to regulate position and speed between all Z Axis.
int z2MotorPower = 50; //Z2 motor power (0-255 = 0-100%) Used to regulate position and speed between all Z Axis.
double motorZ1PwrAvg = 0; //used to determinate 500 cycle average motor power
double motorZ2PwrAvg = 0; //used to determinate 500 cycle average motor power  
double ecartPoursuiteZ1 = 0; //used to determinate instant difference bewteen virtual position and current position
double ecartPoursuiteZ2 = 0; //used to determinate instant difference bewteen virtual position and current position
double ecartPoursuiteZ1Avg = 0; //used to determinate 500 cycle average difference bewteen virtual position and current position
double ecartPoursuiteZ2Avg = 0; //used to determinate 500 cycle average difference bewteen virtual position and current position
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
int z1RegulICounter = 0; //Movement mgmt
int z2RegulICounter = 0; //Movement mgmt
double z1WantedPosition = 000.00;  //[mm]  requested Z1 axis absolut/relativ position
double z2WantedPosition = 000.00;  //[mm]  requested Z2 axis absolut/relativ position
double zWantedSpeed = 0; //mm/ 10 cycles // speed regulation
double z1WantedSpeed = 50; //mm/ 10 cycles // speed regulation
double z2WantedSpeed = 50; //mm/ 10 cycles // speed regulation
double z1deltaVirtual = 00.00; //Movement mgmt
double z2deltaVirtual = 00.00; //Movement mgmt
double z1deltaVirtualMem = 00.00; //Movement mgmt
double z2deltaVirtualMem = 00.00; //Movement mgmt
boolean z1ReferenceTaken = false; 
boolean z2ReferenceTaken = false;  
boolean zReferenceTaken = false; 
boolean noZMotors = false;
boolean noXYMotors = false; 
boolean z1CoderInversion = false;
boolean z2CoderInversion = false;

long currentXPositionPulse = 0L;//[pulse]  current X axis absolut position
double currentXPositionMch = 00.00; //[mm]  current X axis absolut position 
long currentYPositionPulse = 0L;//[pulse]  current Y axis absolut position
double currentYPositionMch = 00.00; //[mm]  current Y axis absolut position 

double currentZPositionMch = 00.00; //[mm]  current Z axis absolut position 
double currentZ1PositionMch = 00.00; //[mm]  current Z1 axis absolut position 
double currentZ2PositionMch = 00.00; //[mm]  current Z2 axis absolut position

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

int xMotorSpeed = 300;
int yMotorSpeed = 300;
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
int mode = 0; //mode, auto (g-code) / manu / Aux_1 / Aux_2
int currentSelectedAxis=0; //current selected axis X = 0 / Y = 1 / Z = 2 .. Z1 = 11 / Z2 = 12 / Z3 = 13 / Z4 = 14
int selectedRelativOffset = 0; // 0 : Machine / 1: G55 / 2: G56
int manualZMvtSelection = 0; // idem state
int dataShown = 0; //0 = Position pulses / 1 = Position Mch / 2 = MotorPower / 3 = axis Delta / 4 = 
double cycleTime = 0;  //watchdog

#define NOT_AN_INTERRUPT -1

AccelStepper xStepperMotor(AccelStepper::DRIVER, OUT_MoteurXPulse, OUT_MoteurXDir);
AccelStepper yStepperMotor(AccelStepper::DRIVER, OUT_MoteurYPulse, OUT_MoteurYDir);
MultiStepper steppers;

Encoder manivelle(IN_ManivelleChA,IN_ManivelleChB);
Encoder codeurZ1(IN_CodeurZ1ChA,IN_CodeurZ1ChB);
Encoder codeurZ2(IN_CodeurZ2ChA,IN_CodeurZ2ChB);

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
void setup(){
	Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2); //Taille du LCD
  lcd.setRGB(colorR, colorG, colorB);
  pinMode (OUT_MoteurXEnable, OUTPUT);  //pin nr. 22 /  / Câble :  / Couleur Nappe : rd / Couleur Câble : rd / Couleur Destination : 
  pinMode (OUT_MoteurYEnable, OUTPUT);  //pin nr. 23 /  / Câble :  / Couleur Nappe : rd / Couleur Câble : rd / Couleur Destination : 
  pinMode (OUT_MoteurXPulse, OUTPUT);  //pin nr. 24 /  / Câble :  / Couleur Nappe : og / Couleur Câble : og / Couleur Destination : 
  pinMode (OUT_MoteurYPulse, OUTPUT);  //pin nr. 25 /  / Câble :  / Couleur Nappe : og / Couleur Câble : og / Couleur Destination : 
  pinMode (OUT_MoteurXDir, OUTPUT);  //pin nr. 26 /  / Câble :  / Couleur Nappe : ye / Couleur Câble : ye / Couleur Destination : 
  pinMode (OUT_MoteurYDir, OUTPUT);  //pin nr. 27 /  / Câble :  / Couleur Nappe : ye / Couleur Câble : ye / Couleur Destination : 
  pinMode (IN_BP_1, INPUT);  //pin nr. 28 /  / Câble : ihm / Couleur Nappe : gn / Couleur Câble : ye/bn / Couleur Destination : 
  pinMode (IN_ContactX, INPUT);  //pin nr. 29 /  / Câble :  / Couleur Nappe : gn / Couleur Câble : gn / Couleur Destination : 
  pinMode (IN_BP_2, INPUT);  //pin nr. 30 /  / Câble : ihm / Couleur Nappe : bu / Couleur Câble : gy/bn / Couleur Destination : 
  pinMode (IN_ContactY, INPUT);  //pin nr. 31 /  / Câble :  / Couleur Nappe : bu / Couleur Câble : bu / Couleur Destination : 
  pinMode (IN_BP_3, INPUT);  //pin nr. 32 /  / Câble : ihm / Couleur Nappe : vt / Couleur Câble : wh/gy / Couleur Destination : 
  pinMode (IN_ContactZ1, INPUT);  //pin nr. 33 /  / Câble :  / Couleur Nappe : vt / Couleur Câble : vt / Couleur Destination : 
  pinMode (IN_BP_4, INPUT);  //pin nr. 34 /  / Câble : ihm / Couleur Nappe : gy / Couleur Câble : rd/bu / Couleur Destination : 
  pinMode (IN_ContactZ2, INPUT);  //pin nr. 35 /  / Câble :  / Couleur Nappe : gy / Couleur Câble : gy / Couleur Destination : 
  pinMode (IN_BP_5, INPUT);  //pin nr. 36 /  / Câble : ihm / Couleur Nappe : wh / Couleur Câble : gy/pk / Couleur Destination : 
  pinMode (IN_BP_6, INPUT);  //pin nr. 38 /  / Câble : ihm / Couleur Nappe : bk / Couleur Câble : wh/pk / Couleur Destination : 
  pinMode (OUT_MoteurZ1Dir, OUTPUT);  //pin nr. 42 /  / Câble :  / Couleur Nappe : rd / Couleur Câble :  / Couleur Destination : 
  pinMode (OUT_MoteurZ2Dir, OUTPUT);  //pin nr. 43 /  / Câble :  / Couleur Nappe : rd / Couleur Câble :  / Couleur Destination : 
  pinMode (OUT_MoteurZ1Pwm, OUTPUT);  //pin nr. 44 / (PWM mega) / Câble :  / Couleur Nappe : og / Couleur Câble :  / Couleur Destination : 
  pinMode (OUT_MoteurZ2Pwm, OUTPUT);  //pin nr. 45 / (PWM mega) / Câble :  / Couleur Nappe : og / Couleur Câble :  / Couleur Destination : 
  xStepperMotor.setSpeed(xMotorSpeed);
  yStepperMotor.setSpeed(yMotorSpeed);
  xStepperMotor.setMaxSpeed(100);
  yStepperMotor.setMaxSpeed(100);
  xStepperMotor.setAcceleration(10);
  yStepperMotor.setAcceleration(10);
//  xStepperMotor.setEnablePin(OUT_MoteurXEnable); 
//  yStepperMotor.setEnablePin(OUT_MoteurYEnable); 
  steppers.addStepper(xStepperMotor);
  steppers.addStepper(yStepperMotor);
 // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
 // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.");
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  // start the server
  if (!(Ethernet.hardwareStatus() == EthernetNoHardware)&& !(Ethernet.linkStatus() == LinkOFF)){
    server.begin();
    Serial.print("Setup: server is at");
    Serial.println(Ethernet.localIP());
  }
  wdt_enable(WDTO_500MS); // watch dog - Si le CPU ne réagit pas pendant 1seconde, il relance.
}
//***************************************************************************************************
// ************************ MAIN LOOP****************************************************************
void loop (){   
// MODE auto
//	if (autoModeStatus){
//		currentReceivedString = autoMode (mainState,adressNumber);
//	}	   
	//Serial READING
	if (Serial.available()){                                  // Something is given via serial com 
		currentReceivedString = serialRead(currentReceivedString);   // Reading what is given on serial port // typically: G0X102.23Y-132.23Z132.1F1300; M = 077; G = 071; ";" = 059 ; X = 088; Y = 089; Z = 090; F = 070; "." = 046; " " = 032
	}
	//G CODE INTERPRETER   
	if(currentReceivedString != currentReceivedStringMem){
		adressNumber = GCodeInterpreter (currentReceivedString);              // Converting the given string into separate variables
		currentReceivedStringMem = currentReceivedString;
	}
	//HMI inputs acquisition
	HMIAcquisition();
	//Measured Position (absolut, relatives) and speed management
	positionTranslation();
	//Axes References 
//	if (!xyReferenceTaken){
//		XYState=referenceXY(xyReferenceTaken, IN_ContactX1, IN_ContactX2, IN_ContactY1, IN_ContactY2);
//	}
//	if (!zReferenceTaken){
 //		referenceZ();
//	}	
	//Movement calculation
	if (mode == 2){
//      debugManualMvt();
	}else{
		//Z virtual Position management
		zVirtualPosition = virtualPosition(zVirtualPosition, currentZPositionMch, zWantedPosition, zWantedSpeed);
		//XY Movement management
		XYState = XYMovementMgmt(xWantedPosition, yWantedPosition, currentXPositionMch, currentYPositionMch, wantedSteps[0], wantedSteps[1], xPulleyFactor, yPulleyFactor, 1.5);
		//Z1 - Z4 Movement management
		Z1State = ZMovementMgmt(z1RegulICounter, z1MotorPower, z1WantedPosition, currentZ1PositionMch, zVirtualPosition, z1deltaVirtual, z1deltaVirtualMem, 0.3); 
		Z2State = ZMovementMgmt(z2RegulICounter, z2MotorPower, z2WantedPosition, currentZ2PositionMch, zVirtualPosition, z2deltaVirtual, z2deltaVirtualMem, 0.3); //(int &zMotorPower, double wantedPosition, double currentPosition, double virtualPosition, double &delta, double hysteresys)
		//Z movement security synchronisation
		ZSyncroSecu(10.0, Z1State, Z2State);
	}
	//Z movement accélération management
	z1MotorPower = zAccelMgmt(Z1State, z1MemState, z1Counter, z1Accel, z1EnCours, z1MotorPower);
	z2MotorPower = zAccelMgmt(Z2State, z2MemState, z2Counter, z2Accel, z2EnCours, z2MotorPower);
	//XY Motors managements (via multiStepper)
	XYState = XYMotorsMgmt(wantedSteps[0], wantedSteps[1]);
	//Z Motors management 
	Z1State = zMotorsMgmt(Z1State,z1MotorPower,OUT_MoteurZ1Dir,OUT_MoteurZ1Pwm,IN_ContactZ1); 
	Z2State = zMotorsMgmt(Z2State,z2MotorPower,OUT_MoteurZ2Dir,OUT_MoteurZ2Pwm,IN_ContactZ2);  
	//LCD Management 
	if (!steppers.run()){LCDMgmt();}
	//WebServer
	//webServer();
	//Surveillance du temps de cycle UC et mis à jour des clocks et counter
	watchDog (cycleTime);
	//Debug I/O to serial monitor 
	IOTest();
	//Watch dog - Restart automatique de l'arduino si celui-ci plante.
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
// ************************ Mode Auto *************************************************************
String autoMode (int state, int &adressNumber){ //on recréer une string type "currentReceivedString" 
	String stringToSend="";
	static int adressNumber2 = 0;
	if ((state == 2)&&(adressNumber2 <= adressNumber-1)){
		if (recipesGCode[adressNumber2]){		
	    	stringToSend = "G";
			stringToSend += String(recipesFunction[adressNumber2],DEC);
	    	stringToSend += "X";
	    	stringToSend += String(recipesPosition[0][adressNumber2],DEC);
	    	stringToSend += "Y";
	    	stringToSend += String(recipesPosition[1][adressNumber2],DEC);
	    	stringToSend += "Z";
	    	stringToSend += String(recipesPosition[2][adressNumber2],DEC);
	    	stringToSend += "F";
	    	stringToSend += String(recipesPosition[3][adressNumber2],DEC);
	    	stringToSend += "S";
	    	stringToSend += String(recipesPosition[4][adressNumber2],DEC);			 
		}else{
	    	stringToSend = "M";
			stringToSend += String(recipesFunction[adressNumber2],DEC);
			stringToSend += "S";
	    	stringToSend += String(recipesPosition[4][adressNumber2],DEC);
		}
		adressNumber2++;
		stringToSend += ";";
		stringToSend += "0";
		Serial.println("Cycle mode:" + stringToSend); //              ****SERIAL PRINT**** 
		Serial.println("Cycle mode adressNumber:" + String(adressNumber, DEC)); // 		    				
	}
	if (adressNumber2 == adressNumber-1){
		Serial.println("Cycle mode FINISHED" + adressNumber2); //              ****SERIAL PRINT**** 
		adressNumber2=0;
		autoModeStatus = false;
	}
	return stringToSend;
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
				if (currentOrderFPosition != -1){XYAxisSpeed = ParseINT(currentReceivedString, currentOrderFPosition);}
				zWantedSpeed  = XYAxisSpeed;
				zWantedSpeed = constrain(zWantedSpeed,20,1000);
				XYAxisSpeed = constrain(XYAxisSpeed,0,5000);
//				xStepperMotor.setSpeed(XYAxisSpeed);
//				yStepperMotor.setSpeed(XYAxisSpeed);
        xStepperMotor.setMaxSpeed(5000);
        yStepperMotor.setMaxSpeed(5000);	
		
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
void HMIAcquisition(){//(mode manu)
  static boolean buttonXState = false;
  static boolean buttonYState = false;
  static boolean buttonZState = false;
  static boolean buttonSelectionState = false;
  static boolean buttonActionState = false;
  static boolean button6State = false;
  
  static int counter = 0;
  static boolean debounceBP1 = false;
  static boolean debounceBP2 = false;
  static boolean debounceBP3 = false;
  static boolean debounceBP4 = false;
  static boolean debounceBP5 = false;
  static boolean debounceBP6 = false;
  
  buttonXState = digitalRead(IN_BP_1) && debounceBP1;
  buttonYState = digitalRead(IN_BP_2) && debounceBP2;
  buttonZState = digitalRead(IN_BP_3) && debounceBP3;
  buttonSelectionState = digitalRead(IN_BP_4) && debounceBP4;
  buttonActionState = digitalRead(IN_BP_5) && debounceBP5;
  button6State = digitalRead(IN_BP_6) && debounceBP6;

  debounceBP1 = digitalRead(IN_BP_1);
  debounceBP2 = digitalRead(IN_BP_2);
  debounceBP3 = digitalRead(IN_BP_3);
  debounceBP4 = digitalRead(IN_BP_4);
  debounceBP5 = digitalRead(IN_BP_5);
  debounceBP6 = digitalRead(IN_BP_6);

  static boolean lastButtonXState = false;
  static boolean lastButtonYState = false;
  static boolean lastButtonZState = false;
  static boolean lastButtonSelectionState = false;
  static boolean lastButtonActionState = false;
  static boolean once1 = true;
  static boolean once2 = true; 
  static boolean once3 = true; 
  // *** Acquisition des touches Rapides
  switch (mode){ //les touches rapides changent de fonction selon le mode
    case 0://mode auto
      if (buttonXState != lastButtonXState){if (buttonXState){counter = 0; currentSelectedAxis=0;Serial.println("Appuis rapide (flanc montant) sur bouton X --> Mode Auto: Selection axe X"); }} //Appuis rapide (flanc montant) sur bouton X --> Mode Auto: Selection axe X
      if (buttonYState != lastButtonYState){if (buttonYState){counter = 0; currentSelectedAxis=1;Serial.println("Appuis rapide (flanc montant) sur bouton Y --> Mode Auto: Selection axe Y");  }} //Appuis rapide (flanc montant) sur bouton Y --> Mode Auto: Selection axe Y
      if (buttonZState != lastButtonZState){if (buttonZState){counter = 0; currentSelectedAxis=2;Serial.println("Appuis rapide (flanc montant) sur bouton Z --> Mode Auto: Selection axe Z");  }} //Appuis rapide (flanc montant) sur bouton Z --> Mode Auto: Selection axe Z
      if (buttonSelectionState != lastButtonSelectionState){if (buttonSelectionState){counter = 0;Serial.println("Appuis rapide (flanc montant) sur bouton 4 --> Mode Auto: ?");}} //Appuis rapide (flanc montant) sur bouton Selection --> Mode Auto:
      if (buttonActionState != lastButtonActionState){if (buttonActionState){counter = 0;autoModeStatus = !autoModeStatus;Serial.println("Appuis rapide (flanc montant) sur bouton 5 --> Mode Auto: ?");}} //Appuis rapide (flanc montant) sur bouton Action --> Mode Auto:  
    break;
    case 1: //mode manu
      if (buttonXState != lastButtonXState){ //Appuis rapide (flanc montant) sur bouton X --> Mode Manu: Selection axe X
        if (buttonXState){
          counter = 0;
          currentSelectedAxis=0;
            byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
            noInterrupts ();   // turn interrupts off         
            currentManivellePositionPulse = currentXPositionMch * 10L;
            SREG=oldSREG; //                           /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
            Serial.println("Appuis rapide (flanc montant) sur bouton X --> Mode Manu: Selection axe X");  
        }
      } 
      if (buttonYState != lastButtonYState){ //Appuis rapide (flanc montant) sur bouton Y --> Mode Manu: Selection axe Y
        if (buttonYState){
          counter = 0;
          currentSelectedAxis=1;
            byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
            noInterrupts ();   // turn interrupts off         
            currentManivellePositionPulse = currentYPositionMch * 10L;
            SREG=oldSREG; //                           /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
            Serial.println("Appuis rapide (flanc montant) sur bouton Y --> Mode Manu: Selection axe Y");  
        }
      } 
      if (buttonZState != lastButtonZState){ //Appuis rapide (flanc montant) sur bouton Z --> Mode Manu: Selection axe Z
        if (buttonZState){
          counter = 0;
          currentSelectedAxis=2;
            byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
          noInterrupts ();   // turn interrupts off         
            currentManivellePositionPulse = currentZPositionMch * 10L;
            SREG=oldSREG; //                           /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/! 
            Serial.println("Appuis rapide (flanc montant) sur bouton Z --> Mode Manu: Selection axe Z");          
        }
      } 
      if (buttonSelectionState != lastButtonSelectionState){
      if (buttonSelectionState){counter = 0;selectedRelativOffset++;if (selectedRelativOffset > 3)selectedRelativOffset=0;}} //Appuis rapide (flanc montant) sur bouton Selection --> Mode Manu: changement de refentiel
    break;
    case 2: //mode aux1
      if (buttonXState != lastButtonXState){ //Appuis rapide (flanc montant) sur bouton X --> Mode Aux1: Monter l'axe selectionné
        if (buttonXState){
          counter = 0;
          manualZMvtSelection=0;
        }else{
          manualZMvtSelection=2;
        }
      } 
      if (buttonYState != lastButtonYState){  //Appuis rapide (flanc montant) sur bouton Y --> Mode Aux1: Selection axe Y
        if (buttonYState){
          counter = 0;
          currentSelectedAxis++;
          if(currentSelectedAxis>6){
            currentSelectedAxis=3;
          }
         }
       }
      if (buttonZState != lastButtonZState){ //Appuis rapide (flanc montant) sur bouton Z --> Mode Aux1: Descendre l'axe selectionné
        if (buttonZState){
          counter = 0;
          manualZMvtSelection=1;
        }else{
          manualZMvtSelection=2;
        }
      } 
      if (buttonSelectionState != lastButtonSelectionState){ //Appuis rapide (flanc montant) sur bouton Selection --> Mode Aux1:
        if (buttonSelectionState){
          counter = 0;
        }
      }   
      if (buttonActionState != lastButtonActionState){  //Appuis rapide (flanc montant) sur bouton Action --> Mode Aux1:  
        if (buttonActionState){
          counter = 0;
        }
      }   
    break;
    case 3: //mode aux2
      if (buttonXState != lastButtonXState){if (buttonXState){counter = 0; }} //Appuis rapide (flanc montant) sur bouton X --> Mode Aux2: 
      if (buttonYState != lastButtonYState){if (buttonYState){counter = 0; }} //Appuis rapide (flanc montant) sur bouton Y --> Mode Aux2:
      if (buttonZState != lastButtonZState){if (buttonZState){counter = 0; }} //Appuis rapide (flanc montant) sur bouton Z --> Mode Aux2:
      if (buttonSelectionState != lastButtonSelectionState){if (buttonSelectionState){counter = 0; }} //Appuis rapide (flanc montant) sur bouton Selection --> Mode Aux2:
      if (buttonActionState != lastButtonActionState){if (buttonActionState){counter = 0;}} //Appuis rapide (flanc montant) sur bouton Action --> Mode Aux2: 
    break;
  }
  // *** Acquisition des touches longues
  if (buttonXState || buttonYState || buttonZState || buttonSelectionState || buttonActionState)counter++; //appuis long des boutons --> incrémentation du compteur 
  if (counter > 100){  //appuis long des boutons
      if (buttonXState && (mode == 1)){ // actif uniquement en mode manu
      switch(selectedRelativOffset){ //suivant la référence sélectionnée, on remet à zéro la valeur concernée
        case 0:
          xPositionG55Offset = 00.00;
          xPositionG56Offset = 00.00;
          xPositionG57Offset = 00.00;
          Serial.println("Long press on xPositionXXXOffset");         
        break;
        case 1:
          xPositionG55Offset = currentXPositionMch;
          Serial.println("Long press on xPositionG55Offset");         
        break;
        case 2:
          xPositionG56Offset = currentXPositionMch;
          Serial.println("Long press on xPositionG56Offset");       
        break;
        case 3:
          xPositionG57Offset = currentXPositionMch;
          Serial.println("Long press on xPositionG57Offset");
        break;        
      }
      }
      if (buttonYState && (mode == 1)){
      switch(selectedRelativOffset){ //suivant la référence sélectionnée, on remet à zéro la valeur concernée
        case 0:
          yPositionG55Offset = 00.00;
          yPositionG56Offset = 00.00;
          yPositionG57Offset = 00.00;
          Serial.println("Long press on yPositionXXXOffset");         
        break;
        case 1:
          yPositionG55Offset = currentYPositionMch;
          Serial.println("Long press on yPositionG55Offset");         
        break;
        case 2:
          yPositionG56Offset = currentYPositionMch;
          Serial.println("Long press on yPositionG56Offset");       
        break;
        case 3:
          yPositionG57Offset = currentYPositionMch;
          Serial.println("Long press on yPositionG57Offset");
        break;        
      }
      }
      if (buttonZState && (mode == 1)){
      switch(selectedRelativOffset){ //suivant la référence sélectionnée, on remet à zéro la valeur concernée
        case 0:
          zPositionG55Offset = 00.00;
          zPositionG56Offset = 00.00;
          zPositionG57Offset = 00.00;
          Serial.println("Long press on zPositionXXXOffset");         
        break;
        case 1:
          zPositionG55Offset = currentZPositionMch;
          Serial.println("Long press on zPositionG55Offset");         
        break;
        case 2:
          zPositionG56Offset = currentZPositionMch;
          Serial.println("Long press on zPositionG56Offset");       
        break;
        case 3:
          zPositionG57Offset = currentZPositionMch;
          Serial.println("Long press on zPositionG57Offset");
        break;        
      }
      }
      if (buttonSelectionState){
        Serial.println("Long press on buttonSelectionState");
        mode++; if(mode > 3)mode =0;
      }
      if (buttonActionState){
        Serial.println("Long press on buttonActionState");
      }
  counter = 0;
  }
  // *** Manivelle & mode
  switch(mode){ // interface entre manivelle et régulation de position
    case 0: //mode auto
      switch (currentSelectedAxis){
        case 0:
          //   xWantedPosition = currentManivellePositionMch;
        break;
        case 1:
          //   yWantedPosition = currentManivellePositionMch;
        break;
        case 2:
          //   zWantedPosition = currentManivellePositionMch;
        break;
      }
    break;
    case 1: //mode manu
      switch (currentSelectedAxis){
        case 0:
          if (once1){
  //            byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
  //            noInterrupts ();   // turn interrupts off         
              currentManivellePositionPulse = currentXPositionMch * 10L; //on remet a zéro la manivelle pour éviter un déplacement intempestif
  //            SREG=oldSREG; //                           /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/! 
          }
          once1 = false;
          once2 = true;
          once3 = true;     
          xWantedPosition = currentManivellePositionMch;
        break;
        case 1:
          if (once2){
  //            byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
  //            noInterrupts ();   // turn interrupts off         
              currentManivellePositionPulse = currentYPositionMch * 10L;
  //            SREG=oldSREG; //                           /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!  
          }
          once1 = true;
          once2 = false;
          once3 = true;
          yWantedPosition = currentManivellePositionMch;
        break;
        case 2:
          if (once3){
  //            byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
  //            noInterrupts ();   // turn interrupts off         
              currentManivellePositionPulse = currentZPositionMch * 10L;
  //            SREG=oldSREG; //                           /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/! 
          }
          once1 = true;
          once2 = true;
          once3 = false;
          zWantedPosition = currentManivellePositionMch;
          zWantedSpeed = 1000;
        break;
      }
    break;
    case 2:
    break;
    default:
    break;
  }
  lastButtonXState = buttonXState;
  lastButtonYState = buttonYState;
  lastButtonZState = buttonZState;
  lastButtonSelectionState = buttonSelectionState;
  lastButtonActionState = buttonActionState;
}
//***************************************************************************************************
// ************************ positionTranslation **********************************************************
void positionTranslation(){//everything about measures 
  //  byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
  //  noInterrupts ();   // turn interrupts off  
  // *** Z AXIS ***
  currentZ1PositionPulse = codeurZ1.read();
  currentZ2PositionPulse = codeurZ2.read();
  if (!z1CoderInversion){currentZ1PositionMch = currentZ1PositionPulse / 120.0L;}else{currentZ1PositionMch = currentZ1PositionPulse / -120.0L;}   
  if (!z2CoderInversion){currentZ2PositionMch = currentZ2PositionPulse / 120.0L;}else{currentZ2PositionMch = currentZ2PositionPulse / -120.0L;} 
   
  //Handwheel
  currentManivellePositionPulse = manivelle.read();
  currentManivellePositionMch = currentManivellePositionPulse / 10.0L;  
  //  SREG = oldSREG;    // turn interrupts back on, if they were on before  /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/! 
  z1WantedPosition = zWantedPosition;
  z2WantedPosition = zWantedPosition;

  // *** X & Y AXIS *** Conversion step / [mm]
  currentXPositionPulse = xStepperMotor.currentPosition();
  currentYPositionPulse = yStepperMotor.currentPosition();   
  currentXPositionMch = currentXPositionPulse * xPulleyFactor; //xPulleyFactor = xTotalLength / xMaxSteps; 
  currentYPositionMch = currentYPositionPulse * yPulleyFactor; 
  // X & Y Axis speed
  xMotorSpeed=constrain(XYAxisSpeed,0,500); 
  yMotorSpeed=constrain(XYAxisSpeed,0,500);  
  // General State overwatch
  if((Z1State == 2) && (Z2State == 2) && (XYState == 2)){mainState = 2;}else{mainState = 0;}
  // calcul de la moyenne de Z
  currentZPositionMch = (currentZ1PositionMch + currentZ2PositionMch)/2; //position Z affichée, calcule de la moyenne
  // suivi de l'erreur de poursuite Z
  ecartPoursuiteZ1 = currentZ1PositionMch - zVirtualPosition;
  ecartPoursuiteZ2 = currentZ2PositionMch - zVirtualPosition;  
  ecartPoursuiteZ1Avg += ecartPoursuiteZ1; 
  ecartPoursuiteZ2Avg += ecartPoursuiteZ2;

  // Suivi de la moyenne de la puissance moteur
  motorZ1PwrAvg += z1MotorPower;
  motorZ2PwrAvg += z2MotorPower;
  
  // Calcul des moyennes sur 500 cycles
  if (cycle500Counter == 0){
    ecartPoursuiteZ1Avg = 0;
    ecartPoursuiteZ2Avg = 0;
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
// ************************ XY REFERENCE ************************************************************
int referenceXY(boolean &referenceTaken, const int sensorXPin, const int sensorYPin){
    int state = 3;
	boolean X = digitalRead(sensorXPin); 
	boolean Y = digitalRead(sensorYPin);  
	if (X){
		digitalWrite(OUT_MoteurXEnable, LOW); //enable output
		xWantedPosition += 0.05; 
	}else{digitalWrite(OUT_MoteurXEnable, HIGH);} //disable output
	if (Y){
		digitalWrite(OUT_MoteurYEnable, LOW); //enable output
		yWantedPosition += 0.05; 
	}else{digitalWrite(OUT_MoteurYEnable, HIGH);} //disable output
	if ((!X) && (!Y)){
		referenceTaken = true;
		state = 2;
		xStepperMotor.setCurrentPosition(0);
		yStepperMotor.setCurrentPosition(0);  
		xStepperMotor.setSpeed(xMotorSpeed);
		yStepperMotor.setSpeed(yMotorSpeed);
		xWantedPosition = 0;
		yWantedPosition = 0;
		Serial.println("XY Reference Taken"); 
	}
    return state;
}
//***************************************************************************************************
// ************************ XY axis Movements MGMT ************************************************** 
int XYMovementMgmt(double xWantedPosition, double yWantedPosition, double xCurrentPosition, double yCurrentPosition, long &xWantedPulse, long &yWantedPulse, double xPulleyFactor, double yPulleyFactor, double hysteresis){   //XYState = XYMovementMgmt(xWantedPosition, yWantedPosition, currentXPositionMch, currentYPositionMch, 0.5);
  int XYState=2;
  
//double xDelta = xCurrentPosition - xWantedPosition;
//double yDelta = yCurrentPosition - yWantedPosition;
//if (abs(xDelta) > hysteresis){xWantedPulse = long(xDelta / xPulleyFactor) * -1.0;}else{xWantedPulse = 0;}   //how many pulses we need in order to reach the X destination?
//if (abs(yDelta) > hysteresis){yWantedPulse = long(yDelta / yPulleyFactor) * -1.0;}else{yWantedPulse = 0;} //how many pulses we need in order to reach the Y destination?
  
  xMotorSpeed = constrain(XYAxisSpeed,0,15000);
  yMotorSpeed = constrain(XYAxisSpeed,0,15000);
  xWantedPulse = long(xWantedPosition / xPulleyFactor);
  yWantedPulse = long(yWantedPosition / yPulleyFactor);

  
  //	if ((xWantedPulse == 0)&&(yWantedPulse == 0)){XYState=2;}else{XYState=0;}
  
  return XYState;
}
//***************************************************************************************************
// ************************ XYMotorsMgmt  ************************************************************
int XYMotorsMgmt(long wantedXSteps, long wantedYSteps){
	static long wantedSteps[2]={0,0};
	static int counter = 0;
	int state = 4;	
	wantedSteps[0]= wantedXSteps;
	wantedSteps[1]= wantedYSteps;

	if (!steppers.run()){
		counter ++;
	}else{
		state = 0;
	}
	if (counter > 200){
		state = 2;
		counter = 0;
	}
 
	steppers.moveTo(wantedSteps); 
  xStepperMotor.enableOutputs();
  yStepperMotor.enableOutputs();
	if (!noXYMotors){steppers.run();}
	return state;
}
//***************************************************************************************************
// ************************ debugManualMvt  ************************************************************
void debugManualMvt(){
  static boolean debugManualMvtResetBit = true;

  if (debugManualMvtResetBit){currentManivellePositionPulse=0;debugManualMvtResetBit = false;}
  z1MotorPower = 0; 
  z2MotorPower = 0;

  switch(currentSelectedAxis){
  case 3: //Z1
    Z1State = manualZMvtSelection;
    z1MotorPower = abs(currentManivellePositionMch)*10;
  zWantedPosition = currentZ1PositionMch;
  break;
  case 4: //Z2
    Z2State = manualZMvtSelection;
    z2MotorPower = abs(currentManivellePositionMch)*10;
  zWantedPosition = currentZ2PositionMch;
  break;
  }
}
//***************************************************************************************************
// ************************ Z REFERENCE *************************************************************
void referenceZ(){
  if (cycle500Counter==200){zWantedPosition = 300;zWantedSpeed = 1000;}
  if (!digitalRead(IN_ContactZ1)){
  //    byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
  //    noInterrupts ();   // turn interrupts off  
      currentZ1PositionPulse = 1000L;
      currentZ2PositionPulse = 1000L;
     
  //    SREG = oldSREG;    // turn interrupts back on, if they were on before  /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/! 
    z1ReferenceTaken = true;
    Z1State = 2;
  }
  if (!digitalRead(IN_ContactZ2)){
  //    byte oldSREG = SREG;   // remember if interrupts are on or off   /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!
  //    noInterrupts ();   // turn interrupts off 
      currentZ1PositionPulse = 1000L;
      currentZ2PositionPulse = 1000L;

  //    SREG = oldSREG;    // turn interrupts back on, if they were on before  /!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!/!   
    z2ReferenceTaken = true;
    Z2State = 2;  
  }

  zReferenceTaken = z1ReferenceTaken && z2ReferenceTaken;
  if (zReferenceTaken){
    zWantedSpeed = 500; //default
    zWantedPosition = 0;//default
    Serial.println("Z Reference Taken"); 
  } 
}
//***************************************************************************************************
// ************************ Z Virtual Position for speed regulation  ********************************
double virtualPosition (double virPos, double currentZPosition, double wantedPosition, double wantedSpeed){
  static double counterb = 0;
  static double wantedPositionMem = wantedPosition;
  static double delta = currentZPosition - wantedPosition;
  static boolean wayToGo = false;
  static double distancePerCycle = 00.00;
  static double neededCycle = 0;
  static double travelTime = 00.00;
  // New Z position entered
  if (wantedPositionMem != wantedPosition){
    wantedPositionMem = wantedPosition;
    counterb = 0;
    //setting start position
    delta = currentZPosition - wantedPosition;
    virPos = currentZPosition;
    delta = abs(delta);
    // setting wayToGo
    if (wantedPosition > currentZPosition){wayToGo = true;}else{wayToGo = false;}
    // Speed translation : 0-1000[] ==> 0-5.0 mm/sec
    wantedSpeed = ((wantedSpeed - 0.0)*(5.0-0.0) / (1000.0 - 0.0) + (0.0));// (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // Travel time calculation
    travelTime = delta / wantedSpeed; //distance à parcourir divisé par vitesse demandé
    
    // How many cycle needed to reach this time
    neededCycle = travelTime / (cycleTime/1000);
    //what distance per cycle?
    distancePerCycle = delta / neededCycle;
    
//    String bufferMonitor = {"***Virtual Position Calcultation*** \n"};
//    bufferMonitor += {"wantedPosition :  " + String(wantedPosition,DEC)};
//    bufferMonitor +=  "\n CurrentPosition " + String(currentPosition,DEC);   
//    bufferMonitor +=  "\ndelta " + String(delta,DEC);
//    bufferMonitor +=  "\n";   
//    bufferMonitor +=  "wantedSpeed " + String(wantedSpeed,DEC);
//    bufferMonitor +=  "\n";
//    bufferMonitor +=  "travelTime " + String(travelTime,DEC);
//    bufferMonitor +=  "\n";
//    bufferMonitor +=  "neededCycle " + String(neededCycle,DEC);
//    bufferMonitor +=  "\n";
//    bufferMonitor +=  "distancePerCycle " + String(distancePerCycle,DEC);   
//    bufferMonitor +=  "\n"; 
//    bufferMonitor +=  "cycleTime " + String(cycleTime,DEC);   
//    bufferMonitor +=  "\n"; 
//    Serial.print (bufferMonitor);
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
int ZMovementMgmt(int &counter, int &zMotorPower, double wantedPosition, double currentPosition, double virtualPosition, double &delta, double &deltaMem, double hysteresys){ 
    int state = 4; // error mode per default
  double gainMoins = 0.9;
  double gainPlus = 1.1;
  boolean wayToGoPower = false;
  boolean wayToGoState = false;
  boolean flagDeltaChanging = false;  
  //***** 1 ***** Régulation de position autour d'une cible mouvante (virtualPosition) en jouant sur la puissance
    // Trouver quel sens
    if (virtualPosition > currentPosition){wayToGoPower = true;}else{wayToGoPower = false;} 
    if (wantedPosition > currentPosition){wayToGoState = true;}else{wayToGoState = false;}  
    // Quelle distance?
    delta = virtualPosition - currentPosition; // delta [mm]
  delta = abs(delta); 
  counter ++; // contre-productif de répéter cette opération trop souvent..
  if (counter > 10){  
    if (delta < deltaMem){ // Est-ce que la différence après 10 cycles diminue?
      flagDeltaChanging = true; // Oui, Bien! On se rapproche
    }else{
      flagDeltaChanging = false; // Non, Pas bien, on s'éloigne!
    }
    deltaMem = delta;
    delta = constrain (delta, 0.2,1);
    delta = delta / 10.0;   
    if (!flagDeltaChanging){ // Il faut réagir
      if (wayToGoState){  // pendant que l'on monte
        if (!wayToGoPower){ // on est trop haut, faut baisser       
          zMotorPower = zMotorPower * (0.9-(delta));//gainMoins;
        }else{        // trop bas, faut monter
          zMotorPower = zMotorPower * (1.1+(delta));//gainPlus;
        }
      }else{        // pendant que l'on descend
        if (!wayToGoPower){ // on est trop haut, faut monter
          zMotorPower = zMotorPower * (1.1+(delta));//gainPlus;
        }else{        // trop bas, faut baisser
          zMotorPower = zMotorPower * (0.9-(delta));//gainMoins;
        } 
      }
      zMotorPower = constrain(zMotorPower, 5,255);
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
    return state; 
}
//***************************************************************************************************
// ************************ ZSyncroSecu  ************************************************************ 
void ZSyncroSecu(double deltaMax, int &Z1State, int &Z2State){
  double minValue = 00.00;
  double maxValue = 00.00;
  double deltaValue = 00.00;
  int minAxisName = 0; //minAxisName: {0 = Z1; 1 = Z2; 2 = Z3; 3 = Z4}
  int maxAxisName = 0; //maxAxisName: {0 = Z1; 1 = Z2; 2 = Z3; 3 = Z4}
  boolean wayToGo = false;
  static boolean z1Flag = false;
  static boolean z2Flag = false;

  if (currentZ1PositionMch > currentZ2PositionMch){minAxisName = 1; maxAxisName = 0; minValue = currentZ2PositionMch; maxValue = currentZ1PositionMch;}else{minAxisName = 0; maxAxisName = 1; minValue = currentZ1PositionMch; maxValue = currentZ2PositionMch;}

  deltaValue = maxValue - minValue;
  double lowValue = currentZPositionMch - minValue; //absolut distance btw min pos and average position
  double highValue = currentZPositionMch - maxValue;//absolut distance btw max pos and average position
  lowValue = abs(lowValue);
  highValue = abs(highValue);  
  if (lowValue < (deltaMax/2)){}; // if the distance between 
  if (highValue < (deltaMax/2)){}; //
  deltaValue = abs(deltaValue);
  deltaMax = abs(deltaMax);
  if (deltaValue  > deltaMax){
    if (zWantedPosition > currentZPositionMch){wayToGo = true;}else{wayToGo = false;}    
    if (wayToGo){ // going upward
      switch (maxAxisName){
        case 0://we need to pause Z1 motor while going upward, Z1 is the highest one by [deltaMax] mm
          if (!z1Flag)Serial.println("   Z SynchroSecu: Axis Z1 Blocked while doing upward !!!   ");
          z1Flag = true;    
        break;
        case 1:
          if (!z2Flag)Serial.println("   Z SynchroSecu: Axis Z2 Blocked while doing upward !!!   ");  
          z2Flag = true;      
        break;
      }
      switch (minAxisName){
        case 0://we need to release Z1 motor while going upward, Z1 is the lowest one by [deltaMax] mm
          z1Flag = false; 
          Serial.println("  Z SyncroSecu default FINISH Z1");     
        break;
        case 1:
          z2Flag = false; 
          Serial.println("  Z SyncroSecu default FINISH Z2");     
        break;
      }
    }else{ // going downward
      switch (minAxisName){
        case 0: //we need to pause Z1 motor while going downward, Z1 is the lowest one by [deltaMax] mm
          if (!z1Flag){Serial.println("   Z SynchroSecu: Axis Z1 Blocked while doing downward !!!   ");}
          z1Flag = true;      
        break;
        case 1:
          if (!z2Flag){Serial.println("   Z SynchroSecu: Axis Z2 Blocked while doing downward !!!   ");}
          z2Flag = true;      
        break;
      }
      switch (maxAxisName){
        case 0://we need to release Z1 motor while going downward, Z1 is the highest one by [deltaMax] mm
          z1Flag = false; 
          Serial.println("  Z SyncroSecu default FINISH Z1");   
        break;
        case 1:
          z2Flag = false; 
          Serial.println("  Z SyncroSecu default FINISH Z2");     
        break;
      }
    }
  }
  if (z1Flag){ //z1 is stopped while his position become closer to the average value
    Z1State = 2;
    double delta = currentZ1PositionMch - currentZPositionMch;
    delta = abs(delta);
    if (delta < 0.5){z1Flag = false; Serial.println("  Z SyncroSecu FINISH Z1 DeltaValue : " + String(delta,DEC));}  
  }
  if (z2Flag){
    Z2State = 2;
    double delta = currentZ2PositionMch - currentZPositionMch;
    delta = abs(delta);
    if (delta < 0.5){z2Flag = false; Serial.println("  Z SyncroSecu FINISH Z2 DeltaValue : " + String(delta,DEC));}   
  }
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
int zMotorsMgmt (int state, int &motorPower, int motorPinA, int motorPinB, int contactPin){
 // static int counter = 0;
  boolean out = true;
  motorPower = constrain(motorPower, 0, 80);
    if (!noZMotors){
    if (digitalRead(contactPin)){         //normal operation
      switch (state){
      case 0:                                   // going upward  
        analogWrite (motorPinA, motorPower);
        analogWrite (motorPinB, 0);
      break;
      case 1:                                   // going downward
        analogWrite (motorPinA, 0);
        analogWrite (motorPinB, motorPower);
      break;
      case 2:                                  // in position
          analogWrite (motorPinA, 0); 
          analogWrite (motorPinB, 0); 
      break;
      }
    }else{ //in case mobile touching axis contact sensor
      out = false;
          analogWrite (motorPinA, 255); 
          analogWrite (motorPinB, 0);
    }
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
// ************************ LCDMgmt *****************************************************************
void LCDMgmt(){
  static int counterLCDRefresh = 0;
  String mchStatusStr={""};
  String modeStr ={""};
  String currentSelectedAxisStr ={""};
  String selectedRelativOffsetStr ={""};
  String manualZMvtSelectionStr ={""};
  String dataShownStr ={""};
  int currentZMotorPowerStr = 0;
  double axisShown = 00.00;
  static String stringMem = {""};
  static int mode_mem = mode;
  colorR = 255;
  colorG = 255;
  colorB = 255;

  switch(selectedRelativOffset){ //Quel offset ? (mode manu)
	  case 0:
	    selectedRelativOffsetStr = "Mch";
	  break;
	  case 1:
	    selectedRelativOffsetStr = "G55";
	  break;
	  case 2:
	    selectedRelativOffsetStr = "G56";
	  break;
	  case 3:
	    selectedRelativOffsetStr = "G57";
	  break;
	  default:
	    selectedRelativOffsetStr = "???";
	  break;
  }
  switch (mode){ //Mode
	  case 0:
	    modeStr = "Auto  ";
	  break;
	  case 1:
	    modeStr = "Manu  ";
	  break;
	  case 2:
	    modeStr = "Aux_1";
	  break;
	  case 3:
	    modeStr = "Aux_2";
	  break;
	  default:
	    modeStr = "???   ";
	  break;
  } 
//  switch (manualZMvtSelection){ //Sens de marche pour le mode debug
//  case 0:
//    manualZMvtSelectionStr = "'";
//  break;
//  case 1:
//    manualZMvtSelectionStr = ",";
//  break;
//  case 2: 
//    manualZMvtSelectionStr = "-";
//  break;
//  }
  switch (currentSelectedAxis){  //Axe sélectionné
    case 0:
      currentSelectedAxisStr = "X ";
      axisShown = currentXPosition;
    break;
    case 1:
      currentSelectedAxisStr = "Y ";
      axisShown = currentYPosition;
    break;
    case 2:
      currentSelectedAxisStr = "Z ";
      axisShown = currentZPosition;
    break;
    default:
     currentSelectedAxisStr = "??";
     axisShown = 0;
    break;
  }
  switch (mode){ // AFFICHAGE DES ECRANS
  case 0: //auto
	if (mode_mem != mode){lcd.clear();} //refreshing screen a first launch
    lcd.setCursor(0, 0); 
    lcd.print(mchStatusStr); //Ready; Moving; Error; ...
    lcd.setCursor(0, 1);     
    lcd.print(modeStr); //Auto; Manu; ...
    lcd.setCursor(9, 0);
    lcd.print("Axe:"); 
    lcd.setCursor(14, 0);
    lcd.print(currentSelectedAxisStr); //X; Y; Z; Z1; Z2; Z3; Z4; ...
	//    lcd.setCursor(5, 1);
	//    lcd.print(selectedRelativOffsetStr);  //Mch; G55; G56; ..
    lcd.setCursor(9, 1);
    lcd.print(axisShown); // -00.00
    break;
  case 1: //manu
		if (mode_mem != mode){lcd.clear();} //refreshing screen at first launch
		lcd.setCursor(0, 0); 
		lcd.print(mchStatusStr); //Ready; Moving; Error; ...
		lcd.setCursor(0, 1);     
		lcd.print(modeStr); //Auto; Manu; ...
		lcd.setCursor(9, 0);
		lcd.print("Axe:"); 
		lcd.setCursor(14, 0);
		lcd.print(currentSelectedAxisStr); //X; Y; Z; Z1; Z2; Z3; Z4; ...
		lcd.setCursor(5, 1);
		lcd.print(selectedRelativOffsetStr);  //Mch; G55; G56; ..
		lcd.setCursor(9, 1);
		lcd.print(axisShown); // -00.00
    break;
    case 2: // aux1 - Z1 - Z4 axis movements
		if (mode_mem != mode){lcd.clear();} //refreshing screen at first launch
		lcd.setCursor(4, 1); 
		lcd.print(manualZMvtSelectionStr);
		lcd.setCursor(0, 1); 
		lcd.print(" UP ");
		lcd.setCursor(12, 1); 
		lcd.print("DOWN"); 
		lcd.setCursor(6, 0);     
		lcd.print(currentSelectedAxisStr); //Auto; Manu; ...
		lcd.setCursor(0, 0);     
		lcd.print(modeStr); //Auto; Manu; ...
		lcd.setCursor(9, 0);     
		lcd.print(axisShown); //00.00
		lcd.setCursor(6, 1);     
		lcd.print(currentZMotorPowerStr);
    break;
	case 3: // data access
		if (mode_mem != mode){lcd.clear();} //refreshing screen at first launch
		lcd.setCursor(0, 0);     
		lcd.print(modeStr); //Auto; Manu; ...
		lcd.setCursor(6, 0);
		lcd.print(currentSelectedAxisStr);	
    break;
    default:
		lcd.setCursor(0, 0); 
		lcd.print("Error: Unknown mode number"); //Ready; Moving; Error; ...
    break;
  }
  switch(mainState){
    case 0:
        colorR = 0;
        colorG = 255;
        colorB = 0;
    break;
    case 1:
        colorR = 0;
        colorG = 0;
        colorB = 255;
    break;
    case 2:
        colorR = 0;
        colorG = 255;
        colorB = 255;
    break;
    case 4:
        colorR = 255;
        colorG = 0;
        colorB = 0;
    break;
  }
  if (stringMem !=  (mchStatusStr + modeStr + currentSelectedAxisStr + selectedRelativOffsetStr + String(axisShown, DEC)+ String(currentZMotorPowerStr, DEC))){ //mis à jour du lcd à intervalle regulier, seulement si l'affichage différe du cycle précédent
    if (counterLCDRefresh > 20){lcd.clear();counterLCDRefresh=0;} 
  } 
  stringMem = mchStatusStr + modeStr + currentSelectedAxisStr + selectedRelativOffsetStr + String(axisShown, DEC)+ String(currentZMotorPowerStr, DEC); //mis en mémoire de l'affichage du cycle en cours
  counterLCDRefresh++;
  mode_mem = mode;
  lcd.setRGB(colorR, colorG, colorB);
}
//***************************************************************************************************
// ************************  webServer       *********************************************************
 void webServer(){
     // listen for incoming clients
   EthernetClient client = server.available();
   if (client) {
     Serial.println("new client");
     // an http request ends with a blank line
     bool currentLineIsBlank = true;
     while (client.connected()) {
       if (client.available()) {
         char c = client.read();
         Serial.write(c);
         // if you've gotten to the end of the line (received a newline
         // character) and the line is blank, the http request has ended,
         // so you can send a reply
         if (c == '\n' && currentLineIsBlank) {
           // send a standard http response header
           client.println("HTTP/1.1 200 OK");
           client.println("Content-Type: text/html");
           client.println("Connection: close");  // the connection will be closed after completion of the response
           client.println("Refresh: 5");  // refresh the page automatically every 5 sec
           client.println();
           client.println("<!DOCTYPE HTML>");
           client.println("<html>");
           // output the value of each analog input pin
           for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
             int sensorReading = analogRead(analogChannel);
             client.print("analog input ");
             client.print(analogChannel);
             client.print(" is ");
             client.print(sensorReading);
             client.println("<br />");
           }
           client.println("</html>");
           break;
         }
         if (c == '\n') {
           // you're starting a new line
           currentLineIsBlank = true;
         } else if (c != '\r') {
           // you've gotten a character on the current line
           currentLineIsBlank = false;
         }
       }
     }
     // give the web browser time to receive the data
     delay(1);
     // close the connection:
     client.stop();
     Serial.println("client disconnected");
   }
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
  if(digitalRead(IN_BP_1)){Serial.println("IN_BP_1");}
//  if(!digitalRead(IN_ContactX)){Serial.println("IN_ContactX");}
  if(digitalRead(IN_BP_2)){Serial.println("IN_BP_2");}
//  if(!digitalRead(IN_ContactY)){Serial.println("IN_ContactY");}
  if(digitalRead(IN_BP_3)){Serial.println("IN_BP_3");}
//  if(!digitalRead(IN_ContactZ1)){Serial.println("IN_ContactZ1");}
  if(digitalRead(IN_BP_4)){Serial.println("IN_BP_4");}
//  if(!digitalRead(IN_ContactZ2)){Serial.println("IN_ContactZ2");}
  if(digitalRead(IN_BP_5)){Serial.println("IN_BP_5");}
 // if(digitalRead(IN_ManivelleChA)){Serial.println("IN_ManivelleChA");}
  if(digitalRead(IN_BP_6)){Serial.println("IN_BP_6");}
//  if(digitalRead(IN_ManivelleChB)){Serial.println("IN_ManivelleChB");}

  int memoryLeft = freeMemory();
  String bufferMonitor = {""};  
	if ((cycle500Counter == 500)||(cycle500Counter == 100)||(cycle500Counter == 200)||(cycle500Counter == 300)||(cycle500Counter == 400)){
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
	  bufferMonitor +=  " / X Wanted steps : " + String(wantedSteps[0],DEC);  
	  bufferMonitor +=  "\n";	  
	  bufferMonitor +=  "* Y Wanted mm : " + String(yWantedPosition,DEC);
	  bufferMonitor +=  " / Y current mm : " + String(currentYPositionMch,DEC); 
	  bufferMonitor +=  " / Y Wanted steps : " + String(wantedSteps[1],DEC);    
    bufferMonitor +=  "\n";      
    bufferMonitor +=  "* Z Wanted mm : " + String(zWantedPosition,DEC);
    bufferMonitor +=  " / Z current mm : " + String(currentZPositionMch,DEC); 
    bufferMonitor +=  " / Z Wanted steps : " + String(wantedSteps[2],DEC);    
    bufferMonitor +=  "\n";
    bufferMonitor +=  "* Z1ecartPoursuiteZ1 : " + String(ecartPoursuiteZ1,DEC); 
    bufferMonitor +=  " Z1 State : " + String(Z1State,DEC);  
    bufferMonitor +=   " currentZ1PositionMch : " + String(currentZ1PositionMch,DEC);
    bufferMonitor +=   " motorZ1PwrAvg : " + String(z1MotorPower,DEC);
    bufferMonitor +=  "\n* Z1ecartPoursuiteZ2 : " + String(ecartPoursuiteZ2,DEC); 
    bufferMonitor +=  " Z2 State : " + String(Z2State,DEC);   
    bufferMonitor +=   " currentZ2PositionMch : " + String(currentZ2PositionMch,DEC);
    bufferMonitor +=   " motorZ2PwrAvg : " + String(z2MotorPower,DEC);
	  bufferMonitor +=  "\n\n\n\n\n";
	  Serial.print (bufferMonitor);
  }
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
// ************************  ArretUrgence  **********************************************************
void ArretUrgence (){
}
