#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 
#define PIN_SERVO 10 
#define PIN_IR A0 

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100 
#define _DIST_MAX 410 

//filter
#define LENGTH 30
#define k_LENGTH 8

// Distance sensor
#define _DIST_ALPHA 0.65 

// Servo range
#define _DUTY_MIN 1160  // servo full clockwise position (0 degree)
#define _DUTY_NEU 1550  // servo neutral position (90 degree)
#define _DUTY_MAX 1900  // servo full counterclockwise position (180 degree)

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 100 

// Event periods
#define _INTERVAL_DIST 20  // 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // Serial제어주기 (ms)

// PID parameters
#define _KP 0.75 

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
int correction_dist, iter;
float dist_list[LENGTH], sum;
float dist_raw, dist_ema;  // 센서가 인식한 거리/ema 필터링된 거리
const float coE[] = {-0.0000004, -0.0004444, 1.2066632, 24.0073521};

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED, OUTPUT); // PIN_LED 핀을 OUTPUT으로 설정
myservo.attach(PIN_SERVO); // PIN_SERVO 핀을 servo를 쓰는 핀으로 설정

// initialize global variables
correction_dist = 0;
iter = 0; sum = 0;
dist_target = _DIST_TARGET;

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU); 
// initialize serial port
Serial.begin(57600); // serial port의 baud rate를 57600으로 초기화
// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0); // 서보 업데이트 1주기에 증감 가능한 duty 의 최댓값

  // 마지막 이벤트 발생 시간 초기화
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;
}
  
void loop() {
/////////////////////
// Event generator //
/////////////////////
// 거리 측정 주기가 되었는지 검사 
if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
        event_dist = true;
    
  // 서보 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
      event_servo= true;
    
  // Serial 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
      event_serial= true;



////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
      dist_raw = ir_distance_filtered();

  // PID control logic
    error_curr = dist_target - dist_raw; // 현재 읽어들인 데이터와 기준 값의 차이
    pterm = error_curr; // p게인 값인 kp와 error 값의 곱
    control = _KP * pterm; 

  // duty_target = f(duty_neutral, control)
  if (error_curr > 0){
    duty_target = map(control, 0, _DIST_TARGET - _DIST_MIN, _DUTY_NEU, _DUTY_MAX); 
  }
  else{
    if (error_curr == 0){
      duty_target = _DUTY_NEU;
    }
    duty_target = map(control, _DIST_TARGET - _DIST_MAX, 0, _DUTY_MIN, _DUTY_NEU);
  }

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  duty_target = min(max(duty_target, _DUTY_MIN), _DUTY_MAX); 
  
  // 마지막 샘플링 시각 업데이트
  last_sampling_time_dist += _INTERVAL_DIST;
  }
  
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target > duty_curr){
      duty_curr += duty_chg_per_interval;
      if (duty_target < duty_curr) duty_curr = duty_target;
    }
    else{
      duty_curr -= duty_chg_per_interval;
      if (duty_target > duty_curr) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);

    // 마지막 서보 조작 시각 업데이트
    last_sampling_time_servo += _INTERVAL_SERVO;
  }
  
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    // 마지막 Serial 업데이트 시각 업데이트
    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  sum = 0;
  iter = 0;
  float raw = ir_distance();
  while (iter < LENGTH)
  {
    dist_list[iter] = coE[0] * pow(raw, 3) + coE[1] * pow(raw, 2) + coE[2] * raw + coE[3];
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH);

  dist_ema =  (1.0 - _DIST_ALPHA) * dist_ema + _DIST_ALPHA * dist_cali;
  return dist_ema;
}
