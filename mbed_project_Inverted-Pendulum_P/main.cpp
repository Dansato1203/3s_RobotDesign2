#include "mbed.h"
#include "Motor.h"
#include "QEI.h"

BusOut led(LED1, LED2, LED3, LED4); 
AnalogIn pen(AD7); 
InterruptIn sw1(SW1);
InterruptIn sw2(SW2);
BusIn in(GPIO1,GPIO2,GPIO3,GPIO4);
Ticker pen_control; 
Serial pc(USBTX, USBRX);
Motor motor_left(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM); 
Motor motor_right(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM); 
QEI qei_right(GPIO3, GPIO4, NC, 48, QEI::X4_ENCODING);
QEI qei_left(GPIO2, GPIO1, NC, 48, QEI::X4_ENCODING);

//***************Cart speed control***************//
int pen_val; 
double goal_pen_val = 680.778; //Trget value of PID

double speed, last_speed; 
//PID Gain
const double pen_kp = 0.037; 
const double pen_ki = 0.06;
const double pen_kd = 0.028;
double p, i, d;

//State feedback Gain
double e,ed,ei,e0; 
double x, dx, theta,dtheta, x0, theta0;
const double K1 = -0.31131390654529;
const double K2 = -0.7110620772229;
const double K3 = -12.143656313343;
const double K4 = -1.537260393318;
double PULSE_TO_METER = 0.000507;
double ADV_TO_RAD = 0.220918860801;
double TARGET_THETA = 149.782987; //Target value of State feedback

int left;
int right;

int flag;

void pen_control_handler(){
switch(flag){
  case 0:
    pen_val = pen.read_u16()>>6;
	  printf("mode 0\r\n");
    speed = 0;
    break;
  case 1:
		pen_val = pen.read_u16()>>6;
    e = (double)(goal_pen_val - pen_val); 
    ed = (e - e0) / 0.001;
    ei += (e - e0)* 0.001;
    e0 = e;

    if(ei > 100) ei = 100;
    if(ei < -100) ei = -100;

    p = pen_kp * e;
    i = pen_ki * ei;
    d = pen_kd * ed;

    speed = (p + i + d)*100;
    printf("mode 1\r\n");	
	  if(pen_val > 800||pen_val < 610){
			flag = 0;
		}
    break;
  case 2:
    //Introduce x, dx, theta, dtheta
	  pen_val = pen.read_u16()>>6;
    left = qei_left.getPulses();
    right = qei_right.getPulses();
    qei_left.reset();
    qei_right.reset();
    x += (float)(left + right) / 2 * PULSE_TO_METER;
    dx = (x - x0) / 0.01;
    x0 = x;
    theta = (float)pen_val * ADV_TO_RAD;
    e = TARGET_THETA - theta;
    ed = (e - e0) / 0.01;
    e0 = e;
    theta0 = theta;

    speed = -(x*K1 + dx*K2 + e*K3 + ed*K4)*100;
    printf("mode 2\r\n");
	  if(pen_val > 800||pen_val < 610){
			flag = 0;
		}
	  break;
}

//Speed limit
if(speed > 1.0) speed = 1.0;
if(speed < -1.0) speed = -1.0;

//**** Change LED by speed ****//
if(speed > 0.8 ){
led = 8;
}else if(speed <= 0.8 && speed >= 0){
led = 4;
}else if(speed < 0 && speed >= -0.8){
led = 2;
}else if(speed < -0.8){
led = 1;
}
//*****************************//

last_speed = speed; 

//Reflects the calculation results in the motor
motor_left = speed;
motor_right = speed;

}
void sw1_rise(void){
	flag = 1;
}
void sw2_rise(void){
	flag = 2;
}
//***************Cart speed control is over***************//

void initialize(void){
  sw1.rise(sw1_rise);
  sw2.rise(sw2_rise);
  sw1.mode(PullUp);
  sw2.mode(PullUp);
  in.mode(PullUp);
  motor_left.setMaxRatio(1);
  motor_right.setMaxRatio(1);
  qei_left.reset();
  qei_right.reset();
  led = 0;
	flag = 0;
}

//***************main function***************//
int main() {
  initialize();
while(1) pen_control_handler();
}
