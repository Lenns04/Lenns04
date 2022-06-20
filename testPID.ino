//define Kp,Ki,Kd
float kp = 1, ki = 1, kd = 1;
//define PID variable (error, setpoint, mesure, error sum, error variation, last error and output)
float yprMesure[3], yprPidResult[3], yprOutput[3], yprError[3], yprAddError[3], yprVarError[3], yprLastError[3], yprSetpoint[3];
float rcControllerOutput[3];
float ypr[3];

void setup() 
{
  Serial.begin(9600);
}
inline void gyroPidAxe(float yprMesure[], float yprOutput[], float yprSetpoint[], char n)
{
  //Compute error for all the 3 axis
  yprError[n] = yprSetpoint[n] - yprMesure[n];
  //Compute error sum for all the 3 axis
  yprAddError[n] += yprError[n];
  // Compute error variation for all the 3 axis
  yprVarError[n] += yprError[n] - yprLastError[n];
  //Compute angle correction
  yprOutput[n] = kp * yprError[n] + ki * yprAddError[n] + kd * yprVarError[n];
  //Compute Last error
  yprLastError[n] = yprError[n];
}

void gyroPid(float yprMesure[], float yprOutput[], float yprSetpoint[])
{
  gyroPidAxe(yprMesure, yprOutput, yprSetpoint, 0);
  gyroPidAxe(yprMesure, yprOutput, yprSetpoint, 1);
  gyroPidAxe(yprMesure, yprOutput, yprSetpoint, 2);
}
void loop() 
{

  //Call the fonction every seconds (for good correction, you need at least a delay of 20ms or faster)
  gyroPid(ypr, yprPidResult, rcControllerOutput);

  //Print the corrections values for each angle
  Serial.println(yprPidResult[0]);
  Serial.println(yprPidResult[1]);
  Serial.println(yprPidResult[2]);
  Serial.println(" ");
  delay(1000);
}