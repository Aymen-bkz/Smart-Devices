#include <Arduino.h>
#include "rn2xx3.h"

#define RST 21
#define RX 18
#define TX 19
#define INTERRUPT_PIN 12

#define SIG 2
#define Buzz_pin 17
#define R_Poly_CMD 16
#define ADC_SENS 33
#define ADC_R_Alu 27




 // RX, TX !! labels on relay board is swapped !!
rn2xx3 myLora(Serial2);
byte payload[2];

float R0=0.45;

float sensor_volt;
float RS_gas; // Get value of RS in a GAS
float ratio; // Get ratio RS_GAS/RS_air

 int duty_cyle;
  int t_1 = 0;
  int t_2 = 0;
  int delta_t=t_2 - t_1;
  double Kp=0.8, Ki=10, Kd=0.1;
  float e_t = 0;
  float e_t_1 = 0;

uint32_t groove_data;
uint32_t aime_data;

int R1;
int R2;
int R3;
int R5;
int Vcc;
int R6;

 // the number of the LED pin
const uint32_t ledPin = 16;  // 16 corresponds to GPIO16

// setting PWM properties
const uint32_t freq = 20000;
const uint32_t ledChannel = 0;
const uint32_t resolution = 8;

void initialize_radio()
{
  //RST RN2xx3
  pinMode(RST, OUTPUT);
  digitalWrite(RST, LOW);
  delay(100);
  digitalWrite(RST, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  Serial2.flush();


  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());


  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  //join_result = myLora.initABP("260BC0E0", "DDA46D8053E8E61D270FC829FB8E3041", "91077773C17E66C8E99DAA4FE049DD7C");

  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = myLora.initOTAA("0000000000000000", "44E53245F1200CDEEC2525661C76BA38");

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
  
}

void manage_interrupt(){

}


void setup() {
    // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(9600);
  // Open serial communications and wait for port to open:
  config_PWM();
  Serial2.begin(57600, SERIAL_8N1, RX, TX);
  initialize_radio();
  Serial2.print("sys get hwei\r\n");
  Serial.println(Serial2.readStringUntil('\n'));

}

uint32_t read_groove_sensor (){
  int sensor_groove_Value = analogRead(SIG);
  sensor_volt=(float)sensor_groove_Value/4096*5.0;
  RS_gas = (5.0-sensor_volt)/sensor_volt; // omit *RL
  ratio = (RS_gas/R0);  // ratio = RS/R0
  /*-----------------------------------------------------------------------*/
  delay(1000);
  uint32_t data=ratio*100;

  return data;

}

uint32_t read_aime_sensor (){
  int sensor_aime_Value = analogRead(ADC_SENS);
  sensor_volt=(float)sensor_aime_Value/4096*5.0;
  /*float v_sens = sensor_aime_Value/(10^7);
  float i_sens= v_sens */

  float R_capteur=(R1*(R2+R3)*Vcc/(R2*sensor_volt)) -R1-R5;
 
  return R_capteur;

}

uint32_t read_r_alu (){
  int ADC_R_ALU = analogRead(ADC_R_Alu);
  float ADC_R_ALU_volt=(float)ADC_R_Alu/4096*5.0;
  /*float v_sens = sensor_aime_Value/(10^7);
  float i_sens= v_sens */

  float R_Alu=ADC_R_ALU_volt*R6/(3.3-ADC_R_ALU_volt);

 
  return R_Alu;

}
void config_PWM(){


// configure PWM functionalitites
ledcSetup(ledChannel, freq, resolution);

// attach the channel to the GPIO to be controlled
ledcAttachPin(ledPin, ledChannel);

}

float temperature (){
  float T = a*(read_r_alu()) + b;
  return T;
}
int manage_PID(){
 
  int t_2 = micros();
  e_t = 200 - temperature();
  float e_P = Kp * e_t;
  float e_I = Ki * e_t * delta_t; 
  float e_D = Kd * (e_t - e_t_1) / (delta_t);
  t_2 = t_2;
  e_t_1 = e_t;
  uint8_t C = e_P + e_D + e_I;
  if(C > 255) C = 255;
 
  return C;
}

void Cmd_PWM_Poly(){
int duty_cycle=manage_PID();
ledcWrite(ledChannel,duty_cyle)

}


void loop() {

  groove_data=read_groove_sensor();
  aime_data=read_aime_sensor();


  Serial.println("TXing");
  Serial.println(ratio);
  delay(1000);

  payload[0]=highByte(gas_data);  
  payload[1]=lowByte(gas_data);

  myLora.txBytes(payload,2); //one byte, blocking function
 

}