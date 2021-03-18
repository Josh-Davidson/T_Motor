#include "Serial_CAN_Nano.h"
Serial_CAN can;


TMotor::TMotor(int ID) {
  id = ID;
}
double pos()
{
  Serial.print(p_out);
}
double vel()
{
  Serial.print(v_out);
}
double tor()
{
  Serial.print(i_out);
}
