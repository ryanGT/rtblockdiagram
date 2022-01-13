#ifndef rtblockdiagram_h
#define rtblockdiagram_h

#include "Arduino.h"

#define MAXLEN 10

int mysat_rtbd(int vin);

void shift_array_rtbd(float new_in, float vect_in[], int len_vect);
  
class block{
 public:
  int output;
  int read_output();//reading the output of a block can be done
                           //as often as needed by subsequent blocks;
                           //read_output just returns the variable
                           //output, set in the find_output method
  // pure virtual function
  virtual int find_output(float t);//find_output should only be called
                                   //once per time step, but must be
                                   //called for each block in the
                                   //system
};


class block_with_one_input: public block{
 public:
  block* input;
  void set_input(block* new_input){
    input = new_input;
  }
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


class pwm_output: public actuator{
public:
  int pwm_pin;
  pwm_output(int PWM_PIN);

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


class analog_sensor: public sensor{
 public:
  int analog_pin;
  int output;

  analog_sensor(int ANALOG_pin);
  
  int get_reading();
};


class plant: public block_with_one_input{
 public:
  actuator* Actuator;
  sensor* Sensor;
  // a plant block should still have an input block pointer  
  plant(actuator *myact, sensor *mysense);

  int get_reading();
  void send_command();
  void send_command(int speed);
  //int read_output(float t);
  int find_output(float t);
};


class plant_no_actuator: public block_with_one_input{
  // used to model a plant that has a sensor, but no actuator
  // - for example, the flexible beam used in EGR 345 lab
  //   vibrates because of the motion of the DC motor and
  //   its output is measured with an accelerometer
  //     - as far as the Arduino is concerned, nothing needs to
  //       be done to cause the beam to vibrate
  //          - there is not explicit actuator
public:
  sensor* Sensor;
  // a plant block should still have an input block pointer  
  plant_no_actuator(sensor *mysense);

  int get_reading();
  void send_command();//keeping these for consitency
  void send_command(int speed);
  //int read_output(float t);
  int find_output(float t);
};


class summing_junction: public block{
 public:
  block* input1;
  block* input2;
  int value1, value2;
  
  summing_junction(block *in1=NULL, block *in2=NULL);

  void set_inputs(block *IN1, block *IN2);
  //int read_output(float t);
  int find_output(float t);
};


class P_control_block: public block_with_one_input{
 public:
  float Kp;
  //block* input;
  int input_value;
  //int output;
  
  P_control_block(float KP, block *in=NULL);

  //int read_output(float t);
  int find_output(float t);
};



class PD_control_block: public block_with_one_input{
 public:
  float Kp;
  float Kd;
  float prev_t, cur_t;
  int prev_in;
  float dt, din_dt;
  int din;
  //block* input;
  int input_value;
  //int output;
  
  PD_control_block(float KP, float KD, block *in=NULL);

  //int read_output(float t);
  int find_output(float t);
  void save_values(float t);
};


class digcomp_block: public block_with_one_input{
 public:
  //block* input;
  int _len_out;
  int _len_in;
  float *_b_vect;
  float *_a_vect;
  float _in_vect[MAXLEN];
  float _out_vect[MAXLEN];
  int input_value;
  //int output;
  digcomp_block(float *b_vect, float *a_vect, int len_in, int len_out, block *in=NULL);
  //digcomp_block(float *b_vect, float *a_vect, block *in);
  //float calc_out(float new_in);
  //int read_output(float t);
  int find_output(float t);
};
  

class saturation_block: public block_with_one_input{
 public:
  //block* input;
  int input_value;
  //int output;
  
  saturation_block(block *in=NULL);

  //int read_output(float t);
  int find_output(float t);
};


#endif
