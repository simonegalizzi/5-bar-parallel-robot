

//#######_USER VARIABLES_#######
byte pp = 14;                  //BLDC motor number of pole pairs
float phaseRes = 0.560;       //Phase winding resistance [ohms]
float sourceVoltage = 12;      //Voltage of your power source [Volts]
float maxCurrent = 2;        //Very rough approximation of max current [Amps]
String controlType = "C2";    //control type: C0 -> torque (voltage)
                                           // C1 -> velocity
                                           // C2 -> angular position

// Bellow are the control loops parameters, to obtain the desired response out of the controller 
// they need to be tuned. These parameters can be tuned via the "Commander" interface, which is 
// based on ASCII character command ids. There are two types of commands, a fetch and a set.
// A fetch command will have the following structure:
//                               motor ID ->  MLC <- Current
//                                             ^
//                                             |
//                                           Limits
//
// The example above will show the current limit set to the controller 
//
// A set command will have the following structure:
//                               motor ID ->  MVI3.5 <- value
//                                             ^^
//                                            /  \
//                          Velocity controller  Integral gain
//
// The example above will set the Integral gain value of the velocity control loop to 3.5
//
// Changing the set-point (target) looks like this:
//                               motor ID ->  M6.3  <- value
//
// The example above will rotate the motor's rotor one full turn, given angular position control. 
//
// These commands can be sent through the Serial Monitor, make sure to set the baud rate to 115200 and 
// "Newline". To the right of the variables bellow there are the characters that would be used to fetch
// or set each individual parameter. There is a cheat-sheet of the commands at the bottom of this tab.
// Make sure to write the final values of the parameters after tuning to update the firmware and 
// upload it again.



//########_EXTRA CONFIGURATON_##########
byte maxTemp = 80;            // Maximum temperature if the power-stage [°C]
float overTempTime = 3;       // Time in an over-temperature senario to disable the controller [seconds]
float sensorOffset = 0.0;     // Position offset, used to define an absolute 0 position on the motor's rotor [rads]
int motionDownSample = 5;     // Downsample the motion control loops with respect to the torque control loop [amoount of loops]
int callerFixedFreq = 1;      // Frequency of the fixed rate function caller in void loop [hertz]
float alignStrength = 0.5;    // Percentage of power used to calibrate the sensor on start-up
char motorID = 'M';           // Motor ID used for command can be any character, useful for multi board proyects
bool trueTorque = false;             // True torque mode, current sensing or voltage control mode.
bool focModulation = true;    // Field oriented control modulation type: true -> Sine PWM
                                                                     // false -> Space Vector PWM
bool skipCalibration = false; //Skip the calibration if the set-up won't change in the future
                              //electric angle offset and natural direction printed on start-up
const float elecOffset = 0.0; //Printed as: "MOT: Zero elec. angle: X.XX"
String natDirection = "CW";   //can be either CW or CCW   


//#######_LIST OF COMMANDS_#######

/*
V - Velocity PID controller
  P - proportional gain
  I - integral gain
  D - derivative gain
A - Angle PID controller
  P - proportional gain
  I - integral gain
  D - derivative gain
L - Limits
  C - Current
  U - Voltage
  V - Velocity
C - Motion control type config
  D - downsample motion loop
  0 - torque
  1 - velocity
  2 - angle
E - Motor status (enable/disable)
  0 - enable
  1 - disable
R - Motor phase resistance
S - Sensor offsets
  M - sensor offset
  E - sensor electrical zero
M - Monitoring control
  D - downsample monitoring
  C - clear monitor
  S - set monitoring variables
  G - get variable value
’’ - Target get/set
*/

// Visit https://docs.simplefoc.com/commander_interface to learn more about the Commander Interface.


//###########################################
// GLOBAL TAB
//###########################################

// Libraries, pin number assignment and instance initialization

//SimpleFOC Version 2.1
#include <SimpleFOC.h>
#include <SPI.h>
#include <FlexCAN_T4.h>

//#######_THREE PHASE DRIVER - DRV8305_########
// Datasheet: www.ti.com/lit/ds/symlink/drv8305.pdf
#define enGateA 9       //Chip Enable
#define nFaultA 25      //Fault reading
#define csA 38           //DRV8305 Chip-select
#define sa1 8
#define sa2 7
#define sa3 6
//#define maA 15
#define enGateB 5       //Chip Enable
#define nFaultB 40      //Fault reading
#define csB 10           //DRV8305 Chip-select
#define sb1 4
#define sb2 3
#define sb3 2
bool faultTrig = false;

//######_MAGNETIC SENSOR - AS5147_######
// Datasheet: https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf
           // interrupt 1

//######_TEMPERATURE SENSOR - STLM20_######
// Datasheet: https://datasheet.lcsc.com/szlcsc/1810010411_STMicroelectronics-STLM20W87F_C129796.pdf
#define vTempB 41
#define vTempA 24

FlexCAN_T4<CAN1, RX_SIZE_8, TX_SIZE_16> can1;

//#####_TIME MANAGEMENT_#####
float runTime, prevT = 0, timeDif, stateT;
int timeInterval = 1000, totalTempTime;
float target_angleA = 0;
float target_angleB = 100;
float angleA, angleB;

CAN_message_t msg;
char comandoA[7];
char comandoB[7];
char buffX[7];
char buffY[7];
bool check_x=false;
bool check_y=false;


String inA,inB;
char* invioA,invioB;

//Five Bar Parrallel Robot constants
const int l0 = 140;
const int l1 = 175;
const int l2 = 245;
const int l3 = 245;
const int l4 = 175;



//####_SIMPLEFOC INSTANCES_####
BLDCMotor motorA = BLDCMotor(pp);   //BLDCMotor instance
BLDCDriver3PWM driverA = BLDCDriver3PWM(8, 7, 6, enGateA);     //3PWM Driver instance
MagneticSensorSPI sensorA = MagneticSensorSPI(0,14,0x3FFF);

BLDCMotor motorB = BLDCMotor(pp);   //BLDCMotor instance
BLDCDriver3PWM driverB = BLDCDriver3PWM(4, 3, 2, enGateB);     //3PWM Driver instance
MagneticSensorSPI sensorB = MagneticSensorSPI(37,14,0x3FFF);

float gradi=0;
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.002, 20.0, so1, so2); 
//####_COMMANDER INTERFACE_####
Commander command = Commander(Serial);
void onMotorA(char* cmd){command.motor(&motorA, cmd);}
void onMotorB(char* cmd){command.motor(&motorB, cmd);}

//######_SETUP FUNCTIONS INIT_######
void SimpleFOCinit();
void drv_initA();
void drv_initB();
void PID_B();
void PID_A();
//void current_dc_calib(bool activate);

//######_LOOP FUNCTIONS INIT_######
void timeManagement();
void tempStatus(bool debug = false);
//void printCurrents(bool dcEquivalent = true);
float rotorPosition();
void faultStatus();
//String serialReceiveUserCommand();


//###########################################
// SETUP
//###########################################

void setup() {
  
  Serial.begin(115200);
  can1.begin();
  can1.setBaudRate(500000);
  SPI1.setMOSI(26);
  SPI1.setMISO(1);
  SPI1.setSCK(27);
  
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  //SPI.setClockDivider(SPI_CLOCK_DIV2); 
  SPISettings(4000000, MSBFIRST, SPI_MODE1);
  //Pinmodes assignment
 // pinMode(15,OUTPUT);
  //digitalWrite(15,HIGH);
  //pinMode(so1, INPUT);
 // pinMode(so2, INPUT);
 // pinMode(so3, INPUT);
  pinMode(vTempA, INPUT);
  pinMode(nFaultA, INPUT);
  pinMode(vTempB, INPUT);
  pinMode(nFaultB, INPUT);
//  pinMode(maA, INPUT);

  pinMode(enGateA, OUTPUT);
  digitalWrite(enGateA, LOW);
  pinMode(enGateB, OUTPUT);
  digitalWrite(enGateB, LOW);
  //SPI start up
  pinMode(csA, OUTPUT);
  digitalWrite(csA, HIGH);
  pinMode(csB, OUTPUT);
  digitalWrite(csB, HIGH);
  SPI.begin();
  SPI1.begin();
  
  delay(1000);
  
  drv_initA();
  drv_initB();
  
  SimpleFOCinit();
  inverseKinematics(70,160);
       Serial.print("angoloa:"+String(angleA));Serial.print("angolob:"); Serial.print(String(angleB));
       Serial.println();
}


//###########################################
// LOOP
//###########################################



void loop() {
  // These functions have to run as fast as possible
  
  ricevi();
  timeManagement();
  motorA.loopFOC();
  motorB.loopFOC();
 // motorB.loopFOC(); 
 // motor.monitor();
  command.run();
  //command.run(comandoA);
  //command.run(comandoB);
  // Fixed rate functions
  // Functions inside this "if" will execute at a 5hz rate. Un/ comment the functions that you wish to use.

  //motorA.target=target_angleA;
  //motorB.target=target_angleB;
 motorA.move();
  motorB.move();
  /*target_angle = (motor.target - angolo);
  if (target_angle >= 1) target_angle = voltageLimit;
  else if(target_angle <= -1) target_angle = -voltageLimit;
  motor.move(target_angle); // target set by the motor.command()  - or by variable motor.target*/
  //sensorA.update();
  //sensorB.update();
  if(stateT >= 1000000/callerFixedFreq){
    stateT = 0;

    // Un/ comment the functions bellow that you wish to use.
    tempStatus();
    faultStatus();
   // rotorPosition();
  }
  
}


//###########################################
// SETUP FUNCTIONS
//###########################################
//static void IRAM_ATTR isr_handler(void*);

// Initialization of SimpleFOC
// Do NOT remove the delays in this function.
void SimpleFOCinit(){
  //Motor driver initialization
  
  _delay(500);
  //inizializzazione sensore motore A
  sensorA.clock_speed = 4000000;
  sensorA.spi_mode=SPI_MODE1; 
  sensorA.init(&SPI1);
  //encoder.init();
  // hardware interrupt enable
  //encoder.enableInterrupts(doA, doB);
  motorA.linkSensor(&sensorA);      // Link sensor to motor instance  
  
  //inizializzazione sensore motore B
  sensorB.clock_speed = 4000000;
  sensorB.spi_mode=SPI_MODE1; 
  sensorB.init(&SPI);
  //encoder.init();
  // hardware interrupt enable
  //encoder.enableInterrupts(doA, doB);
  motorB.linkSensor(&sensorB);      // Link sensor to motor instance  
  
  // driver config, power supply voltage [V]
  //driver.pwm_frequency = 46000;
  //driver.voltage_limit = 12;
  //driver.pwm_frequency = 45000;
  
  // INIZIALIZZAZIONE DRIVER MOTOR A
  driverA.pwm_frequency = 45000;//50000
  driverA.voltage_power_supply = sourceVoltage;
  driverA.voltage_limit = sourceVoltage;
  driverA.init();
  //driver.enable();
  motorA.linkDriver(&driverA);

  // INIZIALIZZAZIONE DRIVER MOTOR B
  driverB.pwm_frequency = 45000;//50000
  driverB.voltage_power_supply = sourceVoltage;
  driverB.voltage_limit = sourceVoltage;
  driverB.init();
  //driver.enable();
  motorB.linkDriver(&driverB);

  
  


  //motor.velocity_index_search = 3;
  if (trueTorque){
//    motor.linkCurrentSense(&current_sense);
    motorA.torque_controller = TorqueControlType::foc_current;
    motorB.torque_controller = TorqueControlType::foc_current; 
  }
    else{
    motorA.torque_controller = TorqueControlType::voltage; 
    motorB.torque_controller = TorqueControlType::voltage;
   // motor.voltage_limit = phaseRes*maxCurrent; //voltageLimit
    // Measured phase resistance [ohms]
    //motor.phase_resistance = phaseRes;
    //motor.current_limit = maxPowersourceCurrent;
  }
  if (focModulation){ 
    motorA.foc_modulation = FOCModulationType::SinePWM;
    motorB.foc_modulation = FOCModulationType::SinePWM;
  } else {
    motorA.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motorB.foc_modulation = FOCModulationType::SpaceVectorPWM;
  }
  
  // set FOC loop to be used: MotionControlType::torque, velocity, angle, velocity_openloop, angle_openloop
  if (controlType == "C0"){
    motorA.controller = MotionControlType::torque;
    motorB.controller = MotionControlType::torque;
  }
  else if (controlType == "C1"){
    motorA.controller = MotionControlType::velocity;
    motorB.controller = MotionControlType::velocity;
  }
  else {
    motorA.controller = MotionControlType::angle;
    motorB.controller = MotionControlType::angle;
  }
  
  PID_A();
  PID_B();
 
  
  motorA.useMonitoring(Serial);      // use monitoring functionality
  
  motorB.useMonitoring(Serial);      // use monitoring functionality
  
  motorB.init();// initialise motor
  if (skipCalibration && natDirection == "CW") {
    motorA.initFOC(2.25, Direction::CW);              // start FOC
    motorB.initFOC(3.26,Direction::CCW);              // start FOC
  }
  else if (skipCalibration && natDirection == "CCW"){
    motorA.initFOC(elecOffset,CCW);       // start FOC
    motorB.initFOC(elecOffset,CCW);       // start FOC
  }
  else{
    motorA.init();
    motorA.initFOC();                         // align sensor/ encoder and start FOC
    motorA.init();
    motorB.initFOC(); 
  }
  //motorA.target= motorA.shaft_angle;
  //motorB.target= motorB.shaft_angle;
  motorA.sensor_offset = motorA.shaft_angle;
  motorB.sensor_offset = motorB.shaft_angle;
  Serial.println("DAGOR: Ready BLDC.");
  
  _delay(500);
    // define the motor id
  command.add('A', onMotorA, "BLDC");
  command.add('B', onMotorB, "BLDC");
  rotorPosition();
}

//Configure DRV8305 to desired operation mode
void drv_initA(){
  Serial.println("DRIVER: DRV8305 INIT");
  SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  //Set to three PWM inputs mode
  digitalWrite(csA, LOW);
  byte resp1 = SPI1.transfer(B00111010);
  byte resp2 = SPI1.transfer(B10000110);
  digitalWrite(csA, HIGH);
  Serial.println(resp1, BIN);
  Serial.println(resp2, BIN);
  
  //Clamp sense amplifier output to 3.3V
  digitalWrite(csA, LOW);
  byte resp7 = SPI1.transfer(B01001100);
  byte resp8 = SPI1.transfer(B10100000);
  digitalWrite(csA, HIGH);
  Serial.println(resp7, BIN);
  Serial.println(resp8, BIN);
   
  //mosfet time
  digitalWrite(csA, LOW);
  byte resp3 = SPI1.transfer(B00101000);
  byte resp4 = SPI1.transfer(B01110111);
  digitalWrite(csA, HIGH);
  Serial.println(resp3, BIN);
  Serial.println(resp4, BIN);
  
   //mosfet time
  digitalWrite(csA, LOW);
  byte resp5 = SPI1.transfer(B00110000);
  byte resp6 = SPI1.transfer(B01110111);
  digitalWrite(csA, HIGH);
  Serial.println(resp5, BIN);
  Serial.println(resp6, BIN);
  _delay(1000);
  
  SPI1.endTransaction();
  Serial.println("DRIVER: enGate Enabled");
  digitalWrite(enGateA, HIGH);
  
  
}

void drv_initB(){
  Serial.println("DRIVER: DRV8305 INIT");
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  //Set to three PWM inputs mode
  digitalWrite(csB, LOW);
  byte resp1 = SPI.transfer(B00111010);
  byte resp2 = SPI.transfer(B10000110);
  digitalWrite(csB, HIGH);
  Serial.println(resp1, BIN);
  Serial.println(resp2, BIN);
  
  //Clamp sense amplifier output to 3.3V
  digitalWrite(csB, LOW);
  byte resp7 = SPI.transfer(B01001100);
  byte resp8 = SPI.transfer(B10100000);
  digitalWrite(csB, HIGH);
  Serial.println(resp7, BIN);
  Serial.println(resp8, BIN);
   
  //mosfet time
  digitalWrite(csB, LOW);
  byte resp3 = SPI.transfer(B00101000);
  byte resp4 = SPI.transfer(B01110111);
  digitalWrite(csB, HIGH);
  Serial.println(resp3, BIN);
  Serial.println(resp4, BIN);
  
   //mosfet time
  digitalWrite(csB, LOW);
  byte resp5 = SPI.transfer(B00110000);
  byte resp6 = SPI.transfer(B01110111);
  digitalWrite(csB, HIGH);
  Serial.println(resp5, BIN);
  Serial.println(resp6, BIN);
  _delay(500);
  
  SPI.endTransaction();
  Serial.println("DRIVER: enGate Enabled");
  digitalWrite(enGateB, HIGH);
  
  
}

/*void current_dc_calib(bool activate){
  if (activate){
    digitalWrite(cs, LOW);
    byte resp5 = SPI.transfer(B01010111);
    byte resp6 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println(resp5, BIN);
    Serial.println(resp6, BIN);
    digitalWrite(cs, LOW);
    byte resp9 = SPI.transfer(B01001100);
    byte resp10 = SPI.transfer(B10000000);
    digitalWrite(cs, HIGH);
    Serial.println(resp5, BIN);
    Serial.println(resp6, BIN);
  }
  else if (!activate){
    digitalWrite(cs, LOW);
    byte resp7 = SPI.transfer(B01010000);
    byte resp8 = SPI.transfer(B00010101);
    digitalWrite(cs, HIGH);
    Serial.println(resp7, BIN);
    Serial.println(resp8, BIN);
  }
}*/


//###########################################
// LOOP FUNCTIONS
//###########################################

// Time management for fixed rate functions
void timeManagement(){
  runTime = micros();
  timeDif = runTime - prevT;
  prevT = runTime;
  stateT += timeDif;
}

// Temperature status and manager
void tempStatus(bool debug){
  static int tFlag;

  //Read voltage from temperature sensor and transform it to °C
  float voutA = analogRead(vTempA);
  float voutB = analogRead(vTempB);
  
  float tempA = (((voutA*3.3)/1024)-0.4)/0.0195;
  float tempB = (((voutB*3.3)/1024)-0.4)/0.0195;
  
  if (debug == true) {
    Serial.print("TempA: ");
    Serial.println(tempA,2);
     Serial.print("TempB: ");
    Serial.println(tempB,2);
  }
  
  if (tempA >= maxTemp && tFlag == false){
    int tempTime = micros();
    totalTempTime += tempTime;

    //If temperature is high for [overTempTime] seconds disable controller
    if(totalTempTime >= overTempTime*1000000){
      tFlag = true;
      digitalWrite(enGateA, LOW);
      Serial.print("motor A Disabled - Temperature protection: ");
      Serial.println(tempA);
    }
    
  }
  else if (tempA <= maxTemp && tFlag == false){
    totalTempTime = 0;
  }

  if (tempB >= maxTemp && tFlag == false){
    int tempTime = micros();
    totalTempTime += tempTime;

    //If temperature is high for [overTempTime] seconds disable controller
    if(totalTempTime >= overTempTime*1000000){
      tFlag = true;
      digitalWrite(enGateB, LOW);
      Serial.print("motor B Disabled - Temperature protection: ");
      Serial.println(tempB);
    }
    
  }
  else if (tempB <= maxTemp && tFlag == false){
    totalTempTime = 0;
  }
  
}


// Print the rotor position in radians
float rotorPosition(){

  Serial.println("Motor A "+(String)sensorA.getAngle()+"Motor B "+(String)sensorB.getAngle());
  Serial.println("Motor A "+(String)motorA.shaft_angle+"Motor B "+(String)motorB.shaft_angle);
}

// Fault status and manager for the DRV8305
// Datahseet pages 37 and 38
void faultStatus(){
  //Read nFault pin from DRV8305 - LOW == error / HIGH == normal operation
  int faulta = digitalRead(nFaultA);
  int faultb = digitalRead(nFaultB);
  if(faulta == LOW && faultTrig == false){
    Serial.println("Fault detected");
    faultTrig = true;
    //Check warning and watchdog reset (Address = 0x1)
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    digitalWrite(csA, LOW);
    byte ft1 = SPI1.transfer(B10001000);
    byte ft2 = SPI1.transfer(B00000000);
    digitalWrite(csA, HIGH);
    Serial.println("Address = 0x1");
    Serial.println(ft1,BIN);
    Serial.println(ft2,BIN);

    //Check OV/VDS Faults (Address = 0x2)
    digitalWrite(csA, LOW);
    byte ft3 = SPI1.transfer(B10010000);
    byte ft4 = SPI1.transfer(B00000000);
    digitalWrite(csA, HIGH);
    Serial.println("Address = 0x2");
    Serial.println(ft3,BIN);
    Serial.println(ft4,BIN);

    //Check IC Faults (Address = 0x3)
    digitalWrite(csA, LOW);
    byte ft5 = SPI1.transfer(B10011000);
    byte ft6 = SPI1.transfer(B00000000);
    digitalWrite(csA, HIGH);
    Serial.println("Address = 0x3");
    Serial.println(ft5,BIN);
    Serial.println(ft6,BIN);

    //Check VGS Faults (Address = 0x4)
    digitalWrite(csA, LOW);
    byte ft7 = SPI1.transfer(B10100000);
    byte ft8 = SPI1.transfer(B00000000);
    digitalWrite(csA, HIGH);
    Serial.println("Address = 0x4");
    Serial.println(ft7,BIN);
    Serial.println(ft8,BIN);
    SPI1.endTransaction();
  }
  if(faultb == LOW && faultTrig == false){
    Serial.println("Fault detected");
    faultTrig = true;
    //Check warning and watchdog reset (Address = 0x1)
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    digitalWrite(csB, LOW);
    byte ft1 = SPI.transfer(B10001000);
    byte ft2 = SPI.transfer(B00000000);
    digitalWrite(csB, HIGH);
    Serial.println("Address = 0x1");
    Serial.println(ft1,BIN);
    Serial.println(ft2,BIN);

    //Check OV/VDS Faults (Address = 0x2)
    digitalWrite(csB, LOW);
    byte ft3 = SPI.transfer(B10010000);
    byte ft4 = SPI.transfer(B00000000);
    digitalWrite(csB, HIGH);
    Serial.println("Address = 0x2");
    Serial.println(ft3,BIN);
    Serial.println(ft4,BIN);

    //Check IC Faults (Address = 0x3)
    digitalWrite(csB, LOW);
    byte ft5 = SPI.transfer(B10011000);
    byte ft6 = SPI.transfer(B00000000);
    digitalWrite(csB, HIGH);
    Serial.println("Address = 0x3");
    Serial.println(ft5,BIN);
    Serial.println(ft6,BIN);

    //Check VGS Faults (Address = 0x4)
    digitalWrite(csB, LOW);
    byte ft7 = SPI.transfer(B10100000);
    byte ft8 = SPI.transfer(B00000000);
    digitalWrite(csB, HIGH);
    Serial.println("Address = 0x4");
    Serial.println(ft7,BIN);
    Serial.println(ft8,BIN);
    SPI.endTransaction();
  }
}

void PID_A(){
  motorA.PID_velocity.P = 0.06;   // sx 0.7 dx 0.06
  motorA.PID_velocity.I = 1.2;
  motorA.PID_velocity.D = 0.0;
  motorA.PID_velocity.output_ramp = 1000.0;//
  motorA.PID_velocity.limit = 6;//10
  // Low pass filtering time constant 
  motorA.LPF_velocity.Tf = 0.001;
  // angle loop PID
  motorA.P_angle.P = 6.0;
  motorA.P_angle.I = 0.0;
  motorA.P_angle.D = 0.0;
  motorA.P_angle.output_ramp = 0.0;
  motorA.P_angle.limit = 6.0;//10
  // Low pass filtering time constant 
  motorA.LPF_angle.Tf = 0.0;
  // current q loop PID 
  motorA.PID_current_q.P = 3.0;
  motorA.PID_current_q.I = 300.0;
  motorA.PID_current_q.D = 0.0;
  motorA.PID_current_q.output_ramp = 1000.0;
  motorA.PID_current_q.limit = 12.0;
  // Low pass filtering time constant 
  motorA.LPF_current_q.Tf = 0.005;
  // current d loop PID
  motorA.PID_current_d.P = 3.0;
  motorA.PID_current_d.I = 300.0;
  motorA.PID_current_d.D = 0.0;
  motorA.PID_current_d.output_ramp = 0.0;
  motorA.PID_current_d.limit = 12.0;
  // Low pass filtering time constant 
  motorA.LPF_current_d.Tf = 0.005;
  // Limits 
  motorA.velocity_limit = 300.0;//200
  motorA.current_limit = 0.6;
  motorA.voltage_limit = 2.5;//4
  
  motorA.modulation_centered = 1.0;
  motorA.voltage_sensor_align = 2.5;
  motorA.motion_downsample = motionDownSample;
 
 
}

void PID_B(){
  motorB.PID_velocity.P = 0.06;   // sx 0.7 dx 0.06
  motorB.PID_velocity.I = 1.2;
  motorB.PID_velocity.D = 0.0;
  motorB.PID_velocity.output_ramp = 1000.0;//1000
  motorB.PID_velocity.limit = 6;//10
  // Low pass filtering time constant 
  motorB.LPF_velocity.Tf = 0.001;
  // angle loop PID
  motorB.P_angle.P = 6.0;
  motorB.P_angle.I = 0.0;
  motorB.P_angle.D = 0.0;
  motorB.P_angle.output_ramp = 0.0;
  motorB.P_angle.limit = 6.0;//10
  // Low pass filtering time constant 
  motorB.LPF_angle.Tf = 0.0;
  // current q loop PID 
  motorB.PID_current_q.P = 3.0;
  motorB.PID_current_q.I = 300.0;
  motorB.PID_current_q.D = 0.0;
  motorB.PID_current_q.output_ramp = 1000.0;
  motorB.PID_current_q.limit = 12.0;
  // Low pass filtering time constant 
  motorB.LPF_current_q.Tf = 0.005;
  // current d loop PID
  motorB.PID_current_d.P = 3.0;
  motorB.PID_current_d.I = 300.0;
  motorB.PID_current_d.D = 0.0;
  motorB.PID_current_d.output_ramp = 0.0;
  motorB.PID_current_d.limit = 12.0;
  // Low pass filtering time constant 
  motorB.LPF_current_d.Tf = 0.005;
  // Limits 
  motorB.velocity_limit = 300.0;//200
  motorB.current_limit = 0.6;
  motorB.voltage_limit = 2.5;//4
  
  motorB.modulation_centered = 1.0;
  motorB.voltage_sensor_align = 2.5;
  motorB.motion_downsample = motionDownSample;
 
 
}

void inverseKinematics(int posX, int posY){
   /* if (posX == 0 && posY == 0){
    posX = 330;
    posY = 390;
  }*/
  //if ((posX==0)&&(posY==190)){
  //  angleA=0;
  ////  angleA=0;
 // }else{
  //if (posY < 0) posY = 1;
  //posX = (posX - 330);
  //posY = 400 - posY;
  //if (posY <= 0) posY = 0;
  
 float c1_sqrd = (posX*posX)+(posY*posY);
 float c2_sqrd = (posX-l0)*(posX-l0)+(posY*posY);
 float alphaA = acos((c1_sqrd + l0*l0 -c2_sqrd)/(2*sqrt(c1_sqrd)*l0));
 float alphaB = acos((c2_sqrd + l0*l0 -c1_sqrd)/(2*sqrt(c2_sqrd)*l0));
 float betaA = acos(((l1*l1)-(l2*l2) + c1_sqrd)/(2*l1*sqrt(c1_sqrd)));
 float betaB = acos(((l1*l1)-(l2*l2) + c2_sqrd)/(2*l1*sqrt(c2_sqrd)));
  //if (posY >=50 || posX <= 0){
   // float thetaA = acos(argA);
    angleA = -(alphaA + betaA - PI+0.43) ;
    angleB = (alphaB + betaB - PI+0.43) ;
   
  //Serial.print("posX: ");
  //Serial.print(posX);
  //Serial.print(", posY: ");
  //Serial.print(posY);
  //Serial.println(thetaA);
  //Serial.println(dA);
  //Serial.println(argA);
  //Serial.println(phiA);
  //Serial.print(", angleA: ");
  //Serial.print(angleA);
  //Serial.print(", angleB: ");
  //Serial.println(angleB);
}

void ricevi() {
  

  if ( can1.read(msg) ) {
    if (msg.id == 0x01){
     // comandoA[0]='A';
       for ( uint8_t i = 0; i < 8; i++ ) {
          comandoA[i]=(char)msg.buf[i];
       }
       motorA.target=atof(comandoA); 
    }
    if (msg.id == 0x02){
       //comandoB[0]='B';
       for ( uint8_t i = 0; i < 8; i++ ) {
          comandoB[i]=(char)msg.buf[i];
       }
        motorB.target=atof(comandoB);
    }
    if (msg.id == 0x20){
     // comandoA[0]='A';
       for ( uint8_t i = 0; i < 7; i++ ) {
          buffX[i]=(char)msg.buf[i];
       }
       check_x=true;
    }
    if (msg.id == 0x40){
       //comandoB[0]='B';
       for ( uint8_t i = 0; i < 7; i++ ) {
          buffY[i]=(char)msg.buf[i];
       }
       check_y=true;
       
    }
    //if ((check_y)&&(check_y)){
       angleA=atoi(buffX);
       angleB=atoi(buffY);
       Serial.print("posx:"+String(angleA));Serial.print(" posy:"); Serial.print(String(angleB));
       Serial.println();
       //angleA=angleA-70;
       //angleB=angleB-160;
       inverseKinematics(angleA,angleB);
       Serial.print("angoloa:"+String(angleA));Serial.print("angolob:"); Serial.print(String(angleB));
       Serial.println();
       angleA=angleA*13.7;
       angleB=angleB*13.7;
       motorA.target=angleA; 
       motorB.target=angleB;
       
        
    //}
    
    
    rotorPosition();
    Serial.println();
   
   
  }
  
}
