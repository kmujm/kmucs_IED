#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100 
#define _DIST_MAX 410 

// Distance sensor
#define _DIST_ALPHA 0.4

// configurable parameters
#define _DUTY_MIN 1160 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1550 // servo neutral position (90 degree)
#define _DUTY_MAX 1900 // servo full counterclockwise position (180 degree)

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 30 

//#define _SERVO_SPEED 60 // servo speed limit (unit: degree/second)

// Event periods
#define _INTERVAL_DIST 20  
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100

// global variables
unsigned long last_sampling_time; // unit: ms

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, alpha;
int a,b;
const float coE[] = {0.0000057, -0.0059973, 2.7321599, -109.1947594}; 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; //maximum duty difference per interval
int duty_target, duty_curr;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);

 // initialize global variables
  dist_target = _DIST_TARGET;
  alpha = _DIST_ALPHA;
  
  a = 69;
  b = 480;
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;

  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time_dist +  _INTERVAL_DIST) return;
  if(millis() < last_sampling_time_servo + _INTERVAL_SERVO) return;
  if(millis() < last_sampling_time_serial + _INTERVAL_SERIAL) return;

// calibrate distance reading from the IR sensor
  float x = ir_distance();
  float raw_dist =  coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  float dist_ema = alpha*dist_cali + (1-alpha)*dist_cali;

// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(dist_target > dist_ema) {
    duty_curr += duty_chg_per_interval;
    if(dist_ema > duty_target) dist_ema = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(dist_ema < duty_target) dist_ema = duty_target;
  }

// update servo position
  myservo.writeMicroseconds(duty_curr);

// output the read value to the serial port
  Serial.print("Min:100,dist_ema:");
  Serial.print(dist_ema);
  Serial.print(",duty_curr:");
  Serial.print(duty_curr);
  Serial.println(",Max:410");


// update last sampling time
  last_sampling_time_dist +=  _INTERVAL_DIST;
  last_sampling_time_servo += _INTERVAL_SERVO;
  last_sampling_time_serial += _INTERVAL_SERIAL;
}
