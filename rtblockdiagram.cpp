#include "Arduino.h"
#include "rtblockdiagram.h"
#include <Wire.h>
#include <math.h>

#define TWOPI 6.28318531

int mysat_rtbd(int vin){
  int mymax = 255;
  int mymin = -255;
  int vout;
  
  if ( vin > mymax ){
    vout = mymax;
  }
  else if ( vin < mymin ){
    vout = mymin;
  }
  else{
    vout = vin;
  }

  return(vout);
}


byte getsecondbyte(int input){
  byte output;
  output = (byte)(input >> 8);
  return output;
}


void shift_array_rtbd(float new_in, float vect_in[], int len_vect){
  int i;
  for(i=len_vect-1; i > 0;i--){
    vect_in[i]=vect_in[i - 1]; // copy
  }
  vect_in[0] = new_in;
}


int block::read_output(){
  return(output);
}


loop_count_block::loop_count_block(){};//empty constructor


int loop_count_block::find_output(int n){
  //this is basically just letting the Arduino loop count pass
  //through, but making it behave like a source block
  output = n;
  return(n);
}


int_constant_block::int_constant_block(int myvalue){
  value = myvalue;
};


int int_constant_block::find_output(){
  output = value;
  return(output);
}


step_input::step_input(float switch_on_time, int Amp){
    on_time = switch_on_time;
    amp = Amp;
};

int step_input::find_output(float t){
  //int output=0;
    if (t > on_time){
      output = amp;
    }
    else{
      output = 0;
    }
    return(output);
};


pulse_input::pulse_input(float switch_on_time, float switch_off_time, int Amp){
  on_time = switch_on_time;
  off_time = switch_off_time;
  amp = Amp;
}


int pulse_input::find_output(float t){
  //int output=0;
  if ((t > on_time) && (t < off_time)){
    output = amp;
  }
  else{
    output = 0;
  }
  return(output);
};

fixed_sine_input::fixed_sine_input(float myfreq, float myamp){
    freq = myfreq;
    amp = myamp;
}


int fixed_sine_input::find_output(float t){
    // do I want this to return a float or an int 
    // or does it matter?
    output = (int)amp*sin(TWOPI*freq*t);
    return(output);
};


swept_sine_input::swept_sine_input(float myslope, float myamp, float myt_end, float myt_on){
    slope = myslope;
    amp = myamp;
    t_end = myt_end;
    t_on = myt_on;
}

float swept_sine_input::set_t_on(float myt){
    t_on = myt;
}


float swept_sine_input::set_t_off(float stop_t){
    t_off = stop_t - t_end;
}

int swept_sine_input::find_output(float t){
    // do I want this to return a float or an int 
    // or does it matter?
    float t_shift;

    if ((t < t_on) || (t > t_off)){
	output = 0;
    }
    else{
	t_shift = t - t_on;
    	freq = slope*t_shift;
    	output = (int)(amp*sin(TWOPI*freq*t_shift));
    }

    return(output);
};



pwm_output::pwm_output(int PWM_PIN){
  pwm_pin = PWM_PIN;
};

void pwm_output::setup(){
  pinMode(pwm_pin, OUTPUT);
}

void pwm_output::send_command(int speed){
  analogWrite(pwm_pin, speed);
}


h_bridge_actuator::h_bridge_actuator(int IN1_PIN, int IN2_PIN, int PWM_PIN){
    in1 = IN1_PIN;
    in2 = IN2_PIN;
    pwm_pin = PWM_PIN;
};

void h_bridge_actuator::setup(){
  pinMode(pwm_pin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
};
			 
void h_bridge_actuator::send_command(int speed){
    speed = mysat_rtbd(speed);

    if (speed > 0){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(pwm_pin, speed);
    }
    else if (speed < 0){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(pwm_pin, abs(speed));
    }
    else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(pwm_pin, 0);
    }
};


encoder::encoder(int ENCODER_PIN_B){
    encoderPinB = ENCODER_PIN_B;
};

void encoder::encoderISR()
{
    // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
    //n++;
    _EncoderBSet = digitalRead(encoderPinB);   // read the input pin
  
    // and adjust counter + if A leads B
    if (_EncoderBSet){
      encoder_count ++;
    }
    else {
      encoder_count --;
    }
};


encoder_quad_sense::encoder_quad_sense(int PIN_A, int PIN_B){//: encoder(PIN_B){
   encoderPinA = PIN_A;
   encoderPinB = PIN_B;
};

void encoder_quad_sense::encoderISRA()
{
    // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
    //n++;
    _EncoderBSet = digitalRead(encoderPinB);
    _EncoderASet = digitalRead(encoderPinA);   // read the input pin
					       //
  
    // and adjust counter + if A leads B
    if (_EncoderASet != _EncoderBSet){
	//A changed first
      	encoder_count ++;
    }
    else {
      	encoder_count --;
    }
};

void encoder_quad_sense::encoderISRB()
{
    // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
    //n++;
    _EncoderBSet = digitalRead(encoderPinB);
    _EncoderASet = digitalRead(encoderPinA);   // read the input pin
					       //
  
    // and adjust counter + if A leads B
    if (_EncoderASet != _EncoderBSet){
	//B changed first
      	encoder_count --;
    }
    else {
      	encoder_count ++;
    }
};



int encoder::get_reading(){
  output = encoder_count;
  return(encoder_count);
};


int encoder_quad_sense::get_reading(){
  output = encoder_count;
  return(encoder_count);
};




analog_sensor::analog_sensor(int ANALOG_PIN){
  analog_pin = ANALOG_PIN;
};

int analog_sensor::get_reading(){
  output = analogRead(analog_pin);
  return(output);
};

plant::plant(actuator *myact, sensor *mysense){
    Actuator = myact;
    Sensor = mysense;
};

int plant::get_reading(){
    return Sensor->get_reading();
};

void plant::send_command(){
  int speed;
  speed = input->read_output();
  Actuator->send_command(speed);
};

void plant::send_command(int speed){
  // backward compatible version for when reading from input is not
  // prefered
  Actuator->send_command(speed);
};


int plant::find_output(float t){
    output = Sensor->get_reading();
    return(output);
};

plant_no_actuator::plant_no_actuator(sensor *mysense){
  Sensor = mysense;
};


int plant_no_actuator::get_reading(){
  return Sensor->get_reading();
};

void plant_no_actuator::send_command(){
  //do nothing
  // - kept for consistency with other plants that have actuators
};

void plant_no_actuator::send_command(int speed){
  //do nothing
};


int plant_no_actuator::find_output(float t){
  output = Sensor->get_reading();
  return(output);
};

plant_with_stepper::plant_with_stepper(stepper *MyStepper){
  mystepper = MyStepper;
};

int plant_with_stepper::get_reading(){
  // this is the main weird thing about a plant with a 
  // stepper motor: the actuator also has the pseudo
  // sensor value
  return mystepper->get_reading();
};


int plant_with_stepper::find_output(float t){
     output = mystepper->get_reading();
     return(output);
};

void plant_with_stepper::send_command(){
  int speed;
  speed = input->read_output();
  mystepper->send_command(speed);
};

void plant_with_stepper::send_command(int speed){
  mystepper->send_command(speed);
}

abs_block::abs_block(){};

int abs_block::find_output(){
    input_value = input->read_output();
    output = abs(input_value);
    return(output);
};



plant_with_double_actuator::plant_with_double_actuator(double_actuator *myact, sensor *mysense){
  dblActuator = myact;
  Sensor = mysense;
};

void plant_with_double_actuator::send_commands(){
  int speed1, speed2;
  speed1 = input1->read_output();
  speed2 = input2->read_output();
  dblActuator->send_commands(speed1, speed2);
};

void plant_with_double_actuator::send_commands(int in1, int in2){
  dblActuator->send_commands(in1, in2);
};


int plant_with_double_actuator::get_reading(){
  return Sensor->get_reading();
};

int plant_with_double_actuator::find_output(float t){
  output = Sensor->get_reading();
  return(output);
};

plant_with_double_actuator_two_sensors::plant_with_double_actuator_two_sensors(double_actuator *myact,sensor *mysense1, sensor *mysense2) : plant_with_double_actuator(myact, mysense1)

{
  dblActuator = myact;
  Sensor1 = mysense1;
  Sensor2 = mysense2;
};

// commenting out this teensy-only class
// - should be moved to a teensy specific library in the future 
//---------------------------
//
//plant_with_i2c_double_actuator_and_two_sensors::plant_with_i2c_double_actuator_and_two_sensors(int ACT_ADDR, sensor *mysense1, sensor *mysense2) : plant_with_double_actuator_two_sensors(NULL, mysense1, mysense2){
//  actuator_addr = ACT_ADDR;
//  Sensor1 = mysense1;
//  Sensor2 = mysense2;
//}
//
//
//void plant_with_i2c_double_actuator_and_two_sensors::send_commands(int i){
//  int speed1, speed2;
//  speed1 = input1->read_output();
//  speed2 = input2->read_output();
//  byte msb1, lsb1, msb2, lsb2, ilsb, imsb;
//  byte buf[7];
//
//  // on the mega side:
//  // n_msb = inArray[1];
//  // n_lsb = inArray[2];
//  // v1_msb = inArray[3];
//  // v1_lsb = inArray[4];
//  // v1 = reassemblebytes(v1_msb,v1_lsb);
//  // v2_msb = inArray[5];
//  // v2_lsb = inArray[6];
//  // v2 = reassemblebytes(v2_msb,v2_lsb);
//
//
//  // break into separate3 bytes for i2c transmission
//  ilsb = (byte)i;
//  imsb = getsecondbyte(i);
//  lsb1 = (byte)speed1;
//  msb1 = getsecondbyte(speed1);
//  lsb2 = (byte)speed2;
//  msb2 = getsecondbyte(speed2);
//
//  buf[0] = 3;
//  buf[1] = imsb;
//  buf[2] = ilsb;
//  buf[3] = msb1;
//  buf[4] = lsb1;
//  buf[5] = msb2;
//  buf[6] = lsb2;
//
//  // Serial.println("buf check");
//  // for (int k=0; k<7; k++){
//  //   Serial.print(buf[k]);
//  //   if (k<6){
//  //     Serial.print(",");
//  //   }
//  //   else{
//  //     Serial.print('\n');
//  //   }
//  // }
//  
//  Wire2.beginTransmission(actuator_addr);   // send the address and the write cmnd
//  Wire2.send(buf,7);                      // send three bytes
//  Wire2.endTransmission(); 
//};
//
//void plant_with_i2c_double_actuator_and_two_sensors::stop_motors(){
//  byte buf[7];
//  int num_bytes=7;
//  buf[0] = 3;
//  for (int k=1; k<num_bytes; k++){
//    buf[k] = 0;
//  }
//  Wire2.beginTransmission(actuator_addr);   // send the address and the write cmnd
//  Wire2.send(buf,7);                      // send three bytes
//  Wire2.endTransmission(); 
//};
//
//
//void plant_with_i2c_double_actuator_and_two_sensors::send_cal_cmd(){
//  byte buf[7];
//  int num_bytes=7;
//  buf[0] = 4;
//  for (int k=1; k<num_bytes; k++){
//    buf[k] = 0;
//  }
//  Wire2.beginTransmission(actuator_addr);   // send the address and the write cmnd
//  Wire2.send(buf,7);                      // send three bytes
//  Wire2.endTransmission(); 
//};
//
//---------------------------

int plant_with_double_actuator_two_sensors::find_output(){
  // in the loop code, the plant's find_output method is called, and
  // not the sensors.  In order to make the sensors behave as blocks
  // for the two sensors plant, I need to call the sensors find_output
  // methods to ensure that the output variable is set for each sensor.
  output1 = Sensor1->find_output();
  output2 = Sensor2->find_output();
  return(output1);
};

int plant_with_double_actuator_two_sensors::find_output(float t){
  // what is the right way to not have two find_output methods that are redundant?
  // - I probably never have a time dependent plant (LTI systems!)
  output1 = Sensor1->find_output();
  output2 = Sensor2->find_output();
  return(output1);
};

summing_junction::summing_junction(block *in1, block *in2){
    input1 = in1;
    input2 = in2;
};

int summing_junction::find_output(float t){
  //int output;
    value1 = input1->read_output();
    value2 = input2->read_output();
    output = value1 - value2;
    return(output);
};

void summing_junction::set_inputs(block *IN1, block *IN2){
  input1 = IN1;
  input2 = IN2;
}


int greater_than_block::find_output(){
  //int output;
  value1 = input1->read_output();
  value2 = input2->read_output();
  if (value1 > value2){
    output = 1;
  }
  else{
      output = 0;
  }
  return(output);
};

int greater_than_block::find_output(float t){
  int temp = find_output();
  return(temp);
};

//greater_than_block::greater_than_block(block *in1, block *in2){
//  input1 = in1;
//  input2 = in2;
//};



output_block::output_block(block *in){
    input = in;
};

int output_block::find_output(){
    input_value = input->read_output();
    output = (int)(input_value);
    return(output);
};


int logical_block::find_output(){
    // this is kind of like a virtual class, but I stink
    // at doing virtual classes correctly
    // - each derived class should provide its own
    //   version of this method
    // - this method exists in the pseudo-virtual class
    //   so that find_out(float t) can call this method
    output = -1;
    return(output);
};

int logical_block::find_output(float t){
  int temp = find_output();
  return(temp);
};

logical_block::logical_block(block *in1, block *in2){
  input1 = in1;
  input2 = in2;
};

int less_than_block::find_output(){
  //int output;
  value1 = input1->read_output();
  value2 = input2->read_output();
  if (value1 < value2){
    output = 1;
  }
  else{
      output = 0;
  }
  return(output);
};

int and_block::find_output(){
  //int output;
  value1 = input1->read_output();
  value2 = input2->read_output();
  if ((value1 > 0) && (value2 > 0)){
    output = 1;
  }
  else{
      output = 0;
  }
  return(output);
};


int or_block::find_output(){
  //int output;
  value1 = input1->read_output();
  value2 = input2->read_output();
  if ((value1 > 0) || (value2 > 0)){
    output = 1;
  }
  else{
      output = 0;
  }
  return(output);
};



int addition_block::find_output(){
  //int output;
  value1 = input1->read_output();
  value2 = input2->read_output();
  output = value1 + value2;
  return(output);
};


int subtraction_block::find_output(){
  //int output;
  value1 = input1->read_output();
  value2 = input2->read_output();
  output = value1 - value2;
  return(output);
};

//if 1
if_block::if_block(block *bool_in, block *in1, block *in2){
  bool_block = bool_in;
  input1 = in1;
  input2 = in2;  
}

void if_block::set_inputs(block *BOOLIN, block *IN1, block *IN2){
  bool_block = BOOLIN;
  input1 = IN1;
  input2 = IN2;
}

int if_block::find_output(){
  //int output;
  bool_value = bool_block->read_output();
  value1 = input1->read_output();
  value2 = input2->read_output();
  if (bool_value > 0){
    output = value1;
  }
  else{
    output = value2;
  }
  return(output);
};


P_control_block::P_control_block(float KP, block *in=NULL){
    input = in;
    Kp = KP;
};


int P_control_block::find_output(float t){
    input_value = input->read_output();
    output = (int)(Kp*input_value);
    return(output);
};

PD_control_block::PD_control_block(float KP, float KD, block *in=NULL){
    input = in;
    Kp = KP;
    Kd = KD;
    prev_t = -1.0;
};

int PD_control_block::find_output(float t){
    cur_t = t;
    input_value = input->read_output();
    dt = t - prev_t;
    din = input_value-prev_in;
    din_dt = ((float)din)/dt;
    output = (int)(Kp*input_value + Kd*din_dt);
    /* if (prev_t < 0){ */
    /*   output = (int)(Kp*input_value); */
    /* } */
    /* else{ */
    /*   
         /*   
         /*   output = (int)(Kp*input_value + Kd*din_dt); */
    /* } */
    prev_in = input_value;
    prev_t = cur_t;
    return(output);
};

void PD_control_block::save_values(float t){
    prev_t = t;
    prev_in = input_value;
};


digcomp_block::digcomp_block(float *b_vect, float *a_vect, int len_in, 
              int len_out, float GAIN, block *in){
  _a_vect = a_vect;
  _b_vect = b_vect;
  //_len_in = sizeof(_b_vect)/sizeof(_b_vect[0]);
  //_len_out = sizeof(_a_vect)/sizeof(_a_vect[0]);
  _len_in = len_in;
  _len_out = len_out;
  input = in;
  gain = GAIN;
}


int digcomp_block::find_output(float t){
  input_value = input->read_output();
  float new_out = 0.0;
  float float_in;
  float_in = (float)input_value;
  shift_array_rtbd(float_in, _in_vect, _len_in);
  int i;
  for(i=0; i<_len_in; i++){
    new_out += _in_vect[i]*_b_vect[i];
  }
  new_out *= gain;
  for(i=1; i<_len_out; i++){
    new_out -= _out_vect[i-1]*_a_vect[i];//out_vect hasn't been shifted yet, so the indices are off by 1
  }
  shift_array_rtbd(new_out, _out_vect, _len_out);
  output = (int)new_out;
  return(output);
}


saturation_block::saturation_block(block *in=NULL){
  input = in;
};

int saturation_block::find_output(float t){
  input_value = input->read_output();
  output = mysat_rtbd(input_value);
  return(output);
};


sat2_adjustable_block::sat2_adjustable_block(int max_in=255, block *in=NULL){
  mymax = max_in;
  mymin = -mymax;
  input = in;
};


sat2_adjustable_block::sat2_adjustable_block(int max_in=255, int min_in=-255, block *in=NULL){
  mymax = max_in;
  mymin = min_in;
  input = in;
};


int sat2_adjustable_block::find_output(float t){
  input_value = input->read_output();
  if ( input_value > mymax ){
    output = mymax;
  }
  else if ( input_value < mymin ){
    output = mymin;
  }
  else{
    output = input_value;
  }
  return(output);
};


switch_block::switch_block(){};//empty constructor


int switch_block::find_output(){
    // - if output is 1, leave it alone
    // - else, check to see if condition is satisfied
    // - output needs to be reset to 0 as a menu 
    //   parameter
    if (output == 0){
        input_value = input->read_output();
        if (input_value > 0){
            output = 1;
        }
    }
    return(output);
};

void switch_block::reset_switch(){
    output = 0;
};


void PI_control_block::initialize(){
    first_time = true;
    myint = 0;
    prev_in = 0;
    prev_t = 0;
}



int PI_control_block::find_output(float t){
    cur_t = t;
    input_value = input->read_output();
    if (first_time){
	first_time = false;
	output = (int)(Kp*input_value);
    }
    else{
        dt = t - prev_t;
        myint += input_value*dt;
        output = (int)(Kp*input_value + myint*Ki);
    }
    prev_in = input_value;
    prev_t = cur_t;
    return(output);
};





PID_control_block::PID_control_block(float KP, float KD, float KI, block *in){
    input = in;
    Kp = KP;
    Kd = KD;
    Ki = KI;
    prev_t = 0.0;
    myint = 0.0;
};

int PID_control_block::find_output(float t){
    cur_t = t;
    input_value = input->read_output();
    if (first_time){
	first_time = false;
	output = (int)(Kp*input_value);
    }
    else{
        dt = t - prev_t;
        din = input_value-prev_in;
        din_dt = ((float)din)/dt;
        myint += input_value*dt;
        output = (int)(Kp*input_value + Kd*din_dt + myint*Ki);
    }
    /* if (prev_t < 0){ */
    /*   output = (int)(Kp*input_value); */
    /* } */
    /* else{ */
    /*   
         /*   
         /*   output = (int)(Kp*input_value + Kd*din_dt); */
    /* } */
    prev_in = input_value;
    prev_t = cur_t;
    return(output);
};


PI_control_block::PI_control_block(float KP, float KI, block *in){
    input = in;
    Kp = KP;
    Ki = KI;
    prev_t = 0.0;
    myint = 0.0;
};




stair_input::stair_input(float sensor_interval_in, float readings_step_in, int clicks_step_in, int max_clicks_in) {
  sensor_interval = sensor_interval_in;
  readings_step = readings_step_in;
  clicks_step = clicks_step_in;
  max_clicks = max_clicks_in;
}

int stair_input::find_output(float t) {
  float stair_time = sensor_interval * readings_step;

  int steps = (int)(t / stair_time);                     
  int cycle_length = 2 * max_clicks;                     
  int cycle_pos = (steps * clicks_step) % cycle_length;  

  if (cycle_pos <= max_clicks) {
      output = cycle_pos;        // Ascending
  } else {
      output = cycle_length - cycle_pos;  // Descending
  }

  return output;
}




stepper::stepper(int DirPin, int StepPin){
 
  stepPin = StepPin;
  dirPin = DirPin;
  dir = 0;
  curClicks=0;
  outputState=0;

}

void stepper::setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
}

void stepper::send_command(int desClicks) {

  //int desClicks = input->read_output();  // Get desired click position from input block
  int error = desClicks - curClicks;

  if (error > 0) {
    dir = 1;
    curClicks++;
    toggleOutput();
  }
  else if (error < 0) {
    dir = 0;
    curClicks--;
    toggleOutput();
  }
}

void stepper::toggleOutput() {
  digitalWrite(dirPin, dir); // set direction
  outputState = !outputState; // toggle step pin state
  digitalWrite(stepPin, outputState); // pulse step
}

int stepper::get_reading(){
  output = curClicks;
  return(curClicks);
};


