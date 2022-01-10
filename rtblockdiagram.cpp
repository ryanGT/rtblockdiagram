#include "Arduino.h"
#include "rtblockdiagram.h"

int mysat(int vin){
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


step_input::step_input(float switch_on_time, int Amp){
    on_time = switch_on_time;
    amp = Amp;
};

int step_input::get_output(float t){
    int output=0;
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
    speed = mysat(speed);

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

void plant::send_command(int speed){
    Actuator->send_command(speed);
};

int plant::get_output(float t){
    return Sensor->get_reading();
};



summing_junction::summing_junction(block *in1, block *in2){
    input1 = in1;
    input2 = in2;
};

int summing_junction::get_output(float t){
    int output;
    value1 = input1->get_output(t);
    value2 = input2->get_output(t);
    output = value1 - value2;
    return(output);
};


P_control_block::P_control_block(float KP, block *in){
    input = in;
    Kp = KP;
};


int P_control_block::get_output(float t){
    input_value = input->get_output(t);
    output = (int)(Kp*input_value);
    return(output);
};

PD_control_block::PD_control_block(float KP, float KD, block *in){
    input = in;
    Kp = KP;
    Kd = KD;
    prev_t = -1.0;
};

int PD_control_block::get_output(float t){
    cur_t = t;
    input_value = input->get_output(t);
    dt = t - prev_t;
    din = input_value-prev_in;
    din_dt = din/dt;
    output = (int)(Kp*input_value + Kd*din_dt);
    /* if (prev_t < 0){ */
    /*   output = (int)(Kp*input_value); */
    /* } */
    /* else{ */
    /*   
         /*   
         /*   output = (int)(Kp*input_value + Kd*din_dt); */
    /* } */
    /* prev_in = input_value; */
    //prev_t = cur_t;
    return(output);
};

void PD_control_block::save_values(float t){
    prev_t = t;
    prev_in = input_value;
};


saturation_block::saturation_block(block *in){
    input = in;
};

int saturation_block::get_output(float t){
    input_value = input->get_output(t);
    output = mysat(input_value);
    return(output);
};
