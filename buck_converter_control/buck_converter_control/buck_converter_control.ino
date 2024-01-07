/* -----------------------------------------------------------------------------------------------------------------------------------------

EE463 Buck Converter Current Controlled Buck Converter Digital Controller Code
Written by: Erkin Atay Toka & Doruk Yazıcı & Elif Topaloğlu

--------------------------------------------------------------------------------------------------------------------------------------------*/

#include <PWM.h> // Library for pwm generation from arduino

// Set the Arduino Pins
int pwm_pin = 3;     // Gate of the MOSFET
int Iout_pin = A0;   // Output current value is measured through this pin
int Vout_pin = A1;   // Output voltage value is measured through this pin

// Initialize parameters 
float cutoff_voltage = 15.6;     // Max battery voltage that should not be exceeded set as 15.6V check:https://www.electriccarpartscompany.com/assets/images/12V100AH-LiFEPO4-Lithium-Module%20-LCD.pdf
float CV_set_voltage = 14.6;     // CV loop beginning voltage set as 14.6V check:https://www.electriccarpartscompany.com/assets/images/12V100AH-LiFEPO4-Lithium-Module%20-LCD.pdf
float R1 = 820;                  // R1 resistance in voltage divider at output (in kohm)
float R2 = 270;                  // R2 resistance in voltage divider at output (in kohm)
int32_t frequency = 50000;       // Desired frequency in Hertz

// Variable initialization
float desired_current = 0;       // Desired output current
float output_voltage = 0;        // Instant battery voltage at the output
float output_voltage_raw = 0;    // Raw voltage value directly read with analogRead at the voltage divider
float output_current = 0;        // Instant battery current at the output
float input_voltage = 0;         // Approximated source voltage
float pwm_value = 255*0.3;       // Set the duty cycle by arranging the pwm value
float duty = 0;                  // Duty cycle (%)
float current_error_P = 0;       // Error of the output current (for P controller)
float current_error_I = 0;       // Error of the output current (for I controller)
float u_current = 0;             // Control signal for PI
float kp = 0;                    // Proportional controller gain
float ki = 0;                    // Intergral controller gain  
long prevT = 0;                  // Previous time


void setup()
{
  Serial.begin(9600);
  InitTimersSafe();                         // Initialize all the timer except timer 0
  SetPinFrequencySafe(pwm_pin,frequency);   // Set the frequency
}

void loop()
{
// Set some parameters initially
  kp = 60;    // Proportional controller gain is set
  ki = 8;     // integral controller gain is set
  desired_current = 2.0; // Set the desired output current in Amps as 10 A


// Read the output voltage and current
  output_current = currentRead();
  output_voltage_raw = analogRead(Vout_pin); 
  output_voltage = ((output_voltage_raw/1024)*5)*((R1+R2)/R2); // Map the value read between 0-5V and find actual voltage using voltage divider values


// Over voltage protection for the battery output
  if(output_voltage>cutoff_voltage){ 
    pwmWrite(pwm_pin,0); // Stop charging the battery
    Serial.print("Over voltage protection is on !");
    return; // Rest of the code will not be run
  }


// Input is off
  if(output_current<0.01){  // If the current sensor cannot measure meaningful data basically
    current_error_I = 0; // Set zero so that error it is not accumulated when system is not energized
    pwmWrite(pwm_pin,0); // Do not charge the battery
    Serial.println("No input voltage is supplied !");
    return; // Rest of the code will not be run
  }


// CCCV charging 
  if(output_voltage>=CV_set_voltage){ 
    desired_current = 0.5; // Set as 2A as the battery datasheet suggests
    Serial.print("Constant voltage charging stage is ongiong...");
  }


// Time difference for integral error calculation
  long currT = micros(); // Current time in microseconds
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); // Time step in seconds
  prevT = currT; // Store current time in previous time


// Control algorithm with PI controller 
// For further explanation check: https://www.youtube.com/watch?v=dTGITLnYAY0&t=4s, this video is for motor drive but same logic applies 

  current_error_P = desired_current-output_current;
  current_error_I = current_error_I + current_error_P*deltaT;
  u_current = kp*current_error_P + ki*current_error_I;
  pwm_value = fabs(u_current); // floating absolute value of the control signal
  duty = pwm_value/255*100; // duty value (%) calculated just for our interpretation
  input_voltage = output_voltage/duty/1.35; // Multiplier 1.35 comes from 3-phase full-bridge rectifier (Vline-to-line)
  
  if(u_current<0){
    pwm_value= 25.0;
  }

/*
// High input voltage protection 
  if (input_voltage>30.0){ // High input voltage protection
    pwmWrite(pwm_pin,0);
    Serial.print("Input voltage is too high !");
    return; // Rest of the code will not be run
  }
*/

// Limit pwm value
  if(pwm_value < 1){    // Output pwm can never go below 0
    pwm_value = 0;
  }
  if(pwm_value > 250){   // Output pwm can never go above 255 we do not even give full duty just for safety
    pwm_value = 255*0.9;
  }
  duty = pwm_value/255*100;

// Print values to serial for interpretation
  Serial.print("output current: ");
  Serial.print(output_current);
  Serial.print("\tdesired current: ");
  Serial.print(desired_current);
  Serial.print("\tcurrent error_I: ");
  Serial.print(current_error_I);
  Serial.print("\tu_current: ");
  Serial.print(u_current);
  Serial.print("\tpwm: ");
  Serial.print(pwm_value);
  Serial.print("\tduty cycle: ");
  Serial.print(duty);
  Serial.print("\tinput voltage: ");
  Serial.print(input_voltage);
  Serial.print("  ");
  Serial.print("\n");


// Set the output pwm to the gate of MOSFET 
  pwmWrite(pwm_pin,pwm_value); 
}


// Function dor reading current from the ACS712 current sensor 
double currentRead()
{ 
  int mVperAmp = 66; // Set as 185 for 5A module, 100 for 20A module & 66 for 30A module
  int RawValue = 0;
  int ACSoffset = 2500; // Sensor has an offset value
  double Voltage = 0; 
  double Amps = 0;

  RawValue = analogRead(Iout_pin); // Read voltage feedback
  Voltage = (RawValue / 1024.0) * 5000; // Map the analog value between 0-1024 and map this to 5V as current sensor reads max 5V
  Amps = ((Voltage - ACSoffset) / mVperAmp); // Current calculation, 30A type generate 66mV for every amp current, check the datashett of the sensor
  return Amps;
}

// The End