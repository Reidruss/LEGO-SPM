const int flex_pin = A0;
const float VCC = 3.3;        
const float R_FIXED = 1000.0;
 
void setup() 
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Arduino ready for communication!");
}

void loop() 
{
  int sensorValue = analogRead(flex_pin);       // Read ADC value (0–1023)
  float Vout = sensorValue * (VCC / 1023.0);    // Convert to voltage
  float Rflex = R_FIXED * (VCC / Vout - 1.0);   // Calculate resistance (ohms)

  Serial.println(Rflex);

  delay(400);
}