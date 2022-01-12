#include "Arduino.h"
#include "rtblockdiagram.h"

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


int encoder::get_reading(){
    return(encoder_count);
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


digcomp_block::digcomp_block(float *b_vect, float *a_vect, int len_in, int len_out, block *in){
  _a_vect = a_vect;
  _b_vect = b_vect;
  //_len_in = sizeof(_b_vect)/sizeof(_b_vect[0]);
  //_len_out = sizeof(_a_vect)/sizeof(_a_vect[0]);
  _len_in = len_in;
  _len_out = len_out;
  input = in;
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
