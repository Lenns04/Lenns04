
int pwmPin = 3;
int const potPin = A0;
int potVal;
int pwmVal;


void setup()  {
  pinMode(pwmPin, OUTPUT); //  pin 3 c'est en sortie
  pinMode(potVal, INPUT);  //pin A0 c'est en entree
  //Serial.begin(9600);


}



void loop() {
potVal = analogRead(potPin);
//Serial.print("Valeure du pot : ");
//Serial.println(potVal);

pwmVal = map(potVal, 0, 1024, 0, 255);
//Serial.print("Valeure du pwm : ");
//Serial.println(pwmVal);

//analogWrite(pwmPin, pwmVal);

digitalWrite(pwmPin, HIGH);
delay(100);
digitalWrite(pwmPin, LOW);
delay(100);
}


