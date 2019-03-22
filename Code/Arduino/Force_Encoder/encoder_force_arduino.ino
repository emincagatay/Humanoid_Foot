// include the library code:
#include <LiquidCrystal.h>
#include <SPI.h>

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

const uint8_t CS = 10; // Cock PIN SPI

const double pi = 3.14159;

// PINs Encoder
const int pinA = 2;
const int pinB = 4;
const int pinZ = 3;

const int resolution = 4096/(2*pi);

// dynamic Values
double angle = 0;   //[-2pi, 2pi]
double revCounter = 0;
bool initialized = false;

bool toogle = false;


// Serial Communication
void send2Simulink(float tt) {
  FLOATUNION_t fa;
  fa.number = tt;
  for (int i = 0; i<4; i++)
  {
    Serial.write(fa.bytes[i]);
  }
}

// Called at every resolution step
void change_A(){
  if (initialized)
  {
    bool A = digitalRead(pinA);
    bool B = digitalRead(pinB);

    if ((A && !B ) || (!A && B)) {
      angle += 1.0/resolution;
    }else{
      angle -= 1.0/resolution;
    }
  }

  if (toogle){
    digitalWrite(LED_BUILTIN, HIGH); 
  }else{
    digitalWrite(LED_BUILTIN, LOW); 
  }
  toogle = !toogle;
}

// Called whenever full 2pi are passed
void rise_Z(){
  if (angle > pi){
    revCounter +=1;
  }else if(angle < - pi){
    revCounter -= 1;
  }else{
    //Do nothing as no full revolution assumed
  }

  // Reset Angle to zero
  angle = 0.0;

  initialized = true;
  
}

// From http://www.robert-fromm.info/?post=elec_spi_mcp3008
uint16_t mcp3008_read(uint8_t channel) {
  digitalWrite(CS, LOW);
  SPI.transfer(0x01);
  uint8_t msb = SPI.transfer(0x80 + (channel << 4));
  uint8_t lsb = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  return ((msb & 0x03) << 8) + lsb;
}

float forceSensor_Evaluator(uint16_t value){

  return (float)value;
//  uint16_t threshold = 80;
//
//  if (value < threshold){
//    return 0.0; 
//  }else{
//    return 1.0;
//  }
}

void setup() {
  //Serial Communication
  Serial.begin(230400);

  // SPI ################
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
 
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  // #### ENCODER ######################################
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinZ, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinA), change_A, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(pinZ), rise_Z, RISING);

   pinMode(LED_BUILTIN, OUTPUT);

   digitalWrite(LED_BUILTIN, HIGH);
}






void loop() {
   // SPI Force Sensor
  float reading_F0 = forceSensor_Evaluator(mcp3008_read(0));
  float reading_F1 = forceSensor_Evaluator(mcp3008_read(1));
  float reading_F2 = forceSensor_Evaluator(mcp3008_read(2));
  float reading_F3 = forceSensor_Evaluator(mcp3008_read(3));
  
  // Encoder SEND
  send2Simulink(angle);
  send2Simulink(revCounter);
  send2Simulink(initialized);

  // FORCE Sensor Send
  send2Simulink(reading_F0);
  send2Simulink(reading_F1);
  send2Simulink(reading_F2);
  send2Simulink(reading_F3);
  
  delay(100);
  
}

  

