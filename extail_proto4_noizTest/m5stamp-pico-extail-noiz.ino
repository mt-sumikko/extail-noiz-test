//Atomの代わりにpicoでも回るかとりあえずだけ確認しておくためのソース
//https://docs.m5stack.com/en/quick_start/stamp_pico/arduino

#include "Arduino.h"

// *--- Stepper ---
// Driver PIN

#define IN1 32  // モーター1正転信号端子
#define IN2 33  // モーター1逆転信号端子

#define CH_IN1 0  // PWM出力チャンネル設定（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15で周波数、分解能設定が共通）
#define CH_IN2 1  //  ※チャンネル7は液晶バックライトと共用になるため指定禁止
#define FREQ 500  // PWM出力周波数　ここを変えてみても意味はなかった
#define BIT_NUM 8  // PWM出力bit数（今回は12bit指定。分解能は2の12乗で4096）//FREQ500のとき、4でもまわるけど12は回らなかった

unsigned long programStartTime;  // プログラム開始時間を記録する変数


void setup() {

  Serial.begin(115200);

  pinMode(IN1, OUTPUT);              // PWM出力端子（INT1：正転用）を出力設定
  pinMode(IN2, OUTPUT);              // PWM出力端子（INT2：逆転用）を出力設定
  ledcSetup(CH_IN1, FREQ, BIT_NUM);  // PWM出力設定（チャンネル, 周波数, bit数）
  ledcSetup(CH_IN2, FREQ, BIT_NUM);
  ledcAttachPin(IN1, CH_IN1);  // PWMチャンネルを端子に割り当て（端子番号, チャンネル）
  ledcAttachPin(IN2, CH_IN2);

  delay(1000);

}

void loop() {
int switching = 100;

  //振幅を変えてく
  furifuri(10, 255, switching, 8);
  delay(100);
  furifuri(30, 255, switching, 8);
  delay(100);
  furifuri(50, 255, switching, 8);
  delay(1000);
  
  //速度を変えてく
   furifuri(50, 225, switching, 4);
  delay(100);
  furifuri(50, 205, switching, 4);
  delay(100);
  furifuri(50, 175, switching, 4);
  delay(100);
  furifuri(50, 135, switching, 4);
  delay(100);
  furifuri(50, 105, switching, 4);

  Serial.println("looped");
   delay(1000);
}


void forward(int pwm) {
  ledcWrite(CH_IN1, pwm);
  ledcWrite(CH_IN2, 0);
}

void reverse(int pwm) {
  ledcWrite(CH_IN1, 0);
  ledcWrite(CH_IN2, pwm);
}
void standby () {
  ledcWrite(CH_IN1, 0);  // 正転PWM出力（INT1）OFF（DUTY比0）
  ledcWrite(CH_IN2, 0);  // 逆転PWM出力（INT2）OFF（DUTY比0）
}
void brake () {
  ledcWrite(CH_IN1, 255);  // 正転PWM出力（INT1）OFF（DUTY比0）
  ledcWrite(CH_IN2, 255);  // 逆転PWM出力（INT2）OFF（DUTY比0）
}

void furifuri(int amp, int pwm, int switching, int count) {
  for (int i = 0; i < count; i++) {
    forward(pwm);
    delay(amp);
    brake();
    delay(switching);
    reverse(pwm);
    delay(amp);
    brake();
    delay(switching);
  }
}
