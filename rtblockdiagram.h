#ifndef rtblockdiagram_h
#define rtblockdiagram_h

#include "Arduino.h"

#define MAXLEN 10

int mysat_rtbd(int vin);

void shift_array_rtbd(float new_in, float vect_in[], int len_vect);
  
class block{
 public:
  int output;
  int read_output(float t);
  // pure virtual function
  virtual int find_output(float t);

};


class step_input: public block{
 public:
  float on_time;
  float off_time;
  int amp;
  step_input(float switch_on_time, int Amp);

  int find_output(float t);
};


class actuator{
 public:
  // pure virtual function
  virtual void send_command(int speed) = 0;
};


class h_bridge_actuator: public actuator{
 public:
  int in1, in2, pwm_pin;
  h_bridge_actuator(int IN1_PIN, int IN2_PIN, int PWM_PIN);

  void setup();
  void send_command(int speed);
};


class sensor{
 public:
  // pure virtual function
  virtual int get_reading() = 0;
};



class encoder: public sensor{
 public:
  volatile bool _EncoderBSet;
  volatile long encoder_count = 0;
  int encoderPinB;

  encoder(int ENCODER_PIN_B);
  
  void encoderISR();
  int get_reading();
};


class plant: public block{
 public:
  actuator* Actuator;
  sensor* Sensor;
  
  plant(actuator *myact, sensor *mysense);

  int get_reading();
  void send_command(int speed);
  //int read_output(float t);
  int find_output(float t);
};


class summing_junction: public block{
 public:
  block* input1;
  block* input2;
  int value1, value2;
  
  summing_junction(block *in1, block *in2);

  //int read_output(float t);
  int find_output(float t);
};


class P_control_block: public block{
 public:
  float Kp;
  block* input;
  int input_value;
  //int output;
  
  P_control_block(float KP, block *in);

  //int read_output(float t);
  int find_output(float t);
};



class PD_control_block: public block{
 public:
  float Kp;
  float Kd;
  float prev_t, cur_t;
  int prev_in;
  float dt, din_dt;
  int din;
  block* input;
  int input_value;
  //int output;
  
  PD_control_block(float KP, float KD, block *in);

  //int read_output(float t);
  int find_output(float t);
  void save_values(float t);
};


class digcomp_block: public block{
 public:
  block* input;
  int _len_out;
  int _len_in;
  float *_b_vect;
  float *_a_vect;
  float _in_vect[MAXLEN];
  float _out_vect[MAXLEN];
  int input_value;
  //int output;
  digcomp_block(float *b_vect, float *a_vect, int len_in, int len_out, block *in);
  //digcomp_block(float *b_vect, float *a_vect, block *in);
  //float calc_out(float new_in);
  //int read_output(float t);
  int find_output(float t);
};
  

class saturation_block: public block{
 public:
  block* input;
  int input_value;
  //int output;
  
  saturation_block(block *in);

  //int read_output(float t);
  int find_output(float t);
};


#endif
