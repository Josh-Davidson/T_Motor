#ifndef TMOTORHEADER
#define TMOTORHEADER

class TMotor {
  public:
    //Constructor
    TMotor (int);
    //Methods
    enable (bool);
    changeMode(int);
    setpoint (double);
    setZero ();
    tick();
    double pos();
    double vel();
    double tor();
    //Properties
    int kp, kd;
  private:
    //Methods
    pack_cmd();
    unpack_reply();
    float float_to_uint(float, float, float, int)
    float uint_to_float(unsigned int, float, float, int)
    //Properties
    int id;
    double pos, vel, tor;
}

#endif
