#include <PWM.h>

// Set the Arduino Pins
int pwm_pin = 3; // Gate of the MOSFET
int Iout_pin = A0; // Output current value is measured through this pin
int Vout_pin = A1; // Output voltage value is measured through this pin


// Initialize parameters

float resistance = 0.5;             // Measured resistance of the sense resistor
long int cutoff_voltage = 14;       // Maximum battery voltage (in mV) that should not be exceeded
float output_voltage = 0;           // Current battery voltage at the output
float output_current = 0;           // Output current sensed with sense resistor
float pwm_value = 255*0.3;                // Set the duty cycle by arranging the pwm value
float duty = 0;                     // Duty cycle (%)
float desired_current = 3.0;       // Set the desired output current (in mA)
float current_error_P = 0;       // Error of the output current (for P controller)
float current_error_I = 0;          // Error of the output current (for I controller)
float u_current = 0;                // Control signal for PI
float kp = 0;                       // Proportional controller gain
float ki = 0;                       // Intergral controller gain  
long int CV_set_voltage = 13;       // CV loop beginning voltage
long prevT = 0;                     // Previous time
int32_t frequency = 50000;          // Desired frequency in Hertz

float v_out;
float i_out;


void setup()
{
  Serial.begin(9600);
  InitTimersSafe(); // Initialize all the timer except timer 0
  SetPinFrequencySafe(pwm_pin,frequency); // Set the frequency
  //CCCV(); // CC-CV charging function   // this should work but life is not that simple ATAY
}

void loop()
{

// PI constants
  kp = 60;
  ki = 8;

// Read the output voltage and current

  output_voltage = analogRead(Vout_pin);
  //v_out = output_voltage * 5.0 / 1023.0;
  
  output_current = currentRead();
  i_out = output_current * 5.0 / 1023 ;


// Time difference for integral error calculation

  long currT = micros(); // Current time in microseconds
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); // Time step in seconds
  prevT = currT; // Store current time in previous time


// Control algorithm with PI controller
  Serial.print("output current: ");
  Serial.print(output_current);
  Serial.print("\tdesired current: ");
  Serial.print(desired_current);
  current_error_P = desired_current-output_current;
  current_error_I = current_error_I + current_error_P*deltaT;
  Serial.print("\tcurrent error_I: ");
  Serial.print(current_error_I);
  u_current = kp*current_error_P + ki*current_error_I;
  Serial.print("\tu_current: ");
  Serial.print(u_current);
  pwm_value = constrain(pwm_value, 0, 255); 
  pwm_value = fabs(u_current); // floating absolute value of the control signal
// Set output limits

  if(pwm_value < 1){    // Output can never go below 0
    pwm_value = 0;
  }
  if(pwm_value > 250){     // Output can never go above 255
    pwm_value = 255*0.9;
  }

  duty = pwm_value/255*100; // duty value (%) for our interpretation
  Serial.print("\tpwm: ");
  Serial.print(pwm_value);
  Serial.print("\tduty cycle: ");
  Serial.print(duty);
  Serial.print("  ");
  Serial.print("\n");

// Further explanation check: https://www.youtube.com/watch?time_continue=4&v=dTGITLnYAY0&embeds_referring_euri=https%3A%2F%2Fwww.google.com%2Fsearch%3Fq%3Dmotor%2Bcontroller%2Bwith%2Bencoder%2Barduino%26oq%3Dmotor%2Bcontroller%2Bwith%2Bencoder%2Barduino%26gs_lcrp%3DEgZjaHJvb&source_ve_path=Mjg2NjY&feature=emb_logo
// This video is for motor drive but same logic applies 

// Over voltage protection for the battery output

 /* if(v_out>cutoff_voltage){ 
    pwmWrite(pwm_pin,0); // Stop charging the battery
    Serial.print("Max Voltage is Reached");
  }
*/

  pwmWrite(pwm_pin,pwm_value); // set the output pwm to the gate of MOSFET // yeri konusunda şüphelerim mevcut

}

// CC-CV charging  function

/*void CCCV () // CC-CV charging
{
  if(v_out>=CV_set_voltage){ 
    pwm_value = pwm_value*0.8; // If CV voltage is reached, decrease the output current
    delay(100);
  }
}
*/
double currentRead(){

  int mVperAmp = 66; // 185 5A MODÜL İÇİN , 100 20A MODÜL İÇİN VE 66  30A MODÜL İÇİN
  int RawValue = 0;
  int ACSoffset = 2500;
  double Voltage = 0; 
  double Amps = 0;

  RawValue = analogRead(Iout_pin);
  Voltage = (RawValue / 1024.0) * 5000; // Voltage calculation, it outputs max 5V
  Amps = ((Voltage - ACSoffset) / mVperAmp); // Current calculation
  return Amps; // 30A type generate 66mV every amp current
}



/* bazı notlarım var

şimdi öncelikle pi controller kısmında bir iki şüphem var burdan cortlama ihtimali yüksek
But lets see 

arduino pinlerinin giriş gerilimlerini kontol etmek ve sense direncinden ve outputtan gelen 
gerilimi ona göre ayarlamak lazım ki bu da set ettiğimiz değerleri değiştirmemiz lazım 
not compicated but needs to be done 
3.3 ila 5.5 voltage arası kabul ediyorlarmış dolayısıyla output voltajı için bi mapping
yapmak lazım gelir alizade gibi konuştum sfjvkdvbjbfv, neyse 5V gibi düşünsek
12V u 5 volta mapplicez
sense direnci düşük olduğu için 


*/
