#ifndef TMOTORHEADER
#define TMOTORHEADER

class TMotor {
  public:
    //Constructor
    TMotor(int);
    //Methods
    void enable(bool);
    void changeMode(int);
    void setpoint(double);
    void setZero();
    void tick();
    double pos();
    double vel();
    double tor();
    //Properties
    int kp, kd;
  private:
    //Methods
    void pack_cmd();
    void unpack_reply();
    float float_to_uint(float, float, float, int);
    float uint_to_float(unsigned int, float, float, int);
    //Properties
    int id;
    double pos_val, vel_val, tor_val;
};

#endif
