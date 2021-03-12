/* Sketch to control the AK80-9 by Tmotor. Code from Ben Katz (HKC_MiniCheetah)
 * Uses an Arduino Nano 33 BLE Sense and Seeed studios Serial-CAN transciever
 * t_ret = kp(p_des-p_real) + kd(V_des-V_real) + t_in
 */

#include <ArduinoBLE.h>

 #define RED 22     
 #define BLUE 24     
 #define GREEN 23


//Complete Bluetooth test
const char* TMotorServiceUUID = "12341234-1212-EFDE-1523-785FEABCD120";
const char* enableCharUUID = "12341234-1212-EFDE-1523-785FEABCD121";
const char* setpointCharUUID = "12341234-1212-EFDE-1523-785FEABCD122";
const char* modeCharUUID = "12341234-1212-EFDE-1523-785FEABCD123";

BLEService TMotorService(TMotorServiceUUID);
BLEByteCharacteristic enableChar(enableCharUUID, BLERead | BLEWrite | BLEBroadcast); 
BLEDoubleCharacteristic setpointChar(setpointCharUUID, BLERead | BLEWrite | BLEBroadcast);
BLEIntCharacteristic modeChar(modeCharUUID, BLERead | BLEWrite | BLEBroadcast);

void setupBLE() {
  if (BLE.begin()) {

    //Advertised name and service
    BLE.setLocalName("TMOTOR");
    BLE.setDeviceName("TMOTOR");
    BLE.setAdvertisedService(TMotorService);
    //Add chars to service
    TMotorService.addCharacteristic(enableChar);
    TMotorService.addCharacteristic(setpointChar);
    TMotorService.addCharacteristic(modeChar);
    //Add service to BLE
    BLE.addService(TMotorService);

    //Add callbacks
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);
    enableChar.setEventHandler(BLEWritten, enableWritten);
    setpointChar.setEventHandler(BLEWritten, setpointWritten);
    modeChar.setEventHandler(BLEWritten, modeWritten);
    BLE.advertise();
  }
}


//==============BLE Callbacks=================
void onBLEConnected(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  BLE.stopAdvertise();
  digitalWrite(BLUE, LOW);
}
void onBLEDisconnected(BLEDevice central) {
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  BLE.advertise();
  digitalWrite(BLUE, HIGH);
}
void enableWritten(BLEDevice central, BLECharacteristic characteristic) {
  byte enable = enableChar.value();
  Serial.print("Enable written to: ");
  Serial.println(enable);
  if (enable) {
    Serial.println("Enable");
  } else {
    Serial.println("Disable");
  }
}
void setpointWritten(BLEDevice central, BLECharacteristic characteristic) {
  double setpoint = setpointChar.value();
  Serial.print("Setpoint written to: ");
  Serial.println(setpoint);
}
void modeWritten(BLEDevice central, BLECharacteristic characteristic) {
  int mode = modeChar.value();
  Serial.print("Mode written to: ");
  Serial.println(mode);
}



//Motor max/mins
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -25.0f //-65.0f
#define V_MAX 25.0f  //500.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// define motor vals
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 0.0f;  //100   10
float kd_in = 1.0f;  //1   10
float t_in = 0.0f;
// measured values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

//Library
#include "Serial_CAN_Nano.h"
//CAN instance
Serial_CAN can;

long unsigned int can_id = 0x1;
//Data buffer for CAN messages
unsigned char data[8] = {1,2,3,4,5,6,7,8};
int command = 0;
bool motor_on = false;

void setup() {
  Serial.begin(9600);
  can.begin(57600);
  while (!Serial) {};
  delay(500);
  pinMode(BLUE, OUTPUT);
  //can.baudRate('4'); //115200
  //can.canRate('18'); //1Mbps
  Serial.println("Begin!");
  //sendZero();
  //ExitMotorMode();
  setupBLE();
}

void loop() {
  BLE.poll();
  while (Serial.available()) {
    command = Serial.read();
    float p_step = 0.01;
    switch (command) {
      case '1':
        p_in = p_in + p_step;
        Serial.println("Increase Step.");
        break;
      case '2':
        Serial.println("Beware of Torque!");
        delay(1000);
        t_in = 5;
        pack_cmd();
        delay(1000);
        t_in = 0;
        pack_cmd();
        break;
      case '3':
        EnterMotorMode();
        Serial.println("Began Motor Mode!");
        break;
      case '4':
        ExitMotorMode();
        Serial.println("Stopped Motor Mode!");
        break;
      case '5':
        setZero();
        Serial.println("Zero Motor");
        break;
      case '6':
        pack_cmd();
        Serial.println("Sent Command");
        break;
      case '7':
        if (motor_on) 
        {
          Serial.println("Start Pos Ramp Sequence!");
          for (int i=0;i<85;i++)
          {
            p_in=p_in + p_step;
            pack_cmd();
            Serial.print("p_in: ");
            Serial.println(p_in);
            Serial.print("i: ");
            Serial.println(i);
            delay(1);
          }
        }
        else 
        {
          Serial.println("Motor must be on!");
        }
        break;
      case '8':
        sendZero();
        break;
    } 
  }
  if (motor_on) 
  {
    pack_cmd();
  }
  
  
  if (can.recv(&can_id, data) == '1') {
    unpack_reply();
  }
}

  void EnterMotorMode()
  {
    unsigned char enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    can.send(can_id, 0, 0, 8, enable_data);
    motor_on = true;
  }
  void ExitMotorMode()
  {
    unsigned char disable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    can.send(can_id, 0, 0, 8, disable_data);
    motor_on = false;
  }
  void setZero()
  {
    unsigned char zero_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    can.send(can_id, 0, 0, 8, zero_data);
  }
  void sendZero()
  {
    if (!motor_on) 
    {
      EnterMotorMode();
    }
    p_in = 0;
    pack_cmd();
  }
  
  bool pos_ctl(float pos, float kp, float kd)
  {
    if (kp<=0 || kd<0)
    {
      return false;
    }
    p_in = pos;
    v_in = 0;
    kp_in = kp;
    kd_in = kd;
    t_in = 0;
    pack_cmd();
    return true;
  }
  bool vel_ctrl(float vel, float kd)
  {
    if (kd<=0)
    {
      return false;
    }
    v_in = vel;
    kp_in = 0;;
    kd_in = kd;
    t_in = 0;
    pack_cmd();
    return true;
  }
  bool trq_ctrl(float tor, float kd)
  {
    if (kd<=0)
    {
      return false;
    }
    kp_in = 0;
    kd_in = kd;
    t_in = tor;
    pack_cmd();
  }












  
 void pack_cmd() {
  byte buf[8];
  //Bound input data
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);
  //Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  //pack ints into the can buffer
  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int &0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;
  //Send CAN message
  can.send(can_id, 0, 0, 8, buf);
 }
 void unpack_reply() {
  //Unpack global buffer
  unsigned int id = data[0];
  unsigned int p_int = (data[1] << 8) | data[2];
  unsigned int v_int = (data[3] << 4) | (data[4] >> 4);
  unsigned int i_int = ((data[4] & 0xF) << 8) | data[5];
  //Convert to floats
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
 }

//Type conversion funcs
 float float_to_uint(float x, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int) ((x-offset)*4095.0/span); 
  }
  if (bits == 16) {
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
 }
 float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
 }
  
//END FILE
