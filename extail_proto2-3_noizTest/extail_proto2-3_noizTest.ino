// ノイズ測定用ソース。m5atom・DRV8835構成でとりあえずバイポーラステッパーモータを回したい時に使うソース

#include "M5Atom.h"

int amp = 60;
int rotateSpeed = 800;
int switching = 100; // ここは固定で

// 1ステップの角度（ステップ角）
float stepAngle = 18;

// ステッパーモーターのステップ数（1回転あたりのステップ数）（計算でも出るけど一応定義
int stepsPerRevolution = 20; // 2号機なら103

// Stepper
int APHASE = 22;
int AENBL = 19;
int BPHASE = 23;
int BENBL = 33;

// IMUつかうときは21,25は使っちゃダメ！
#define MODE 32 // 25//32 //MODEピン　HIGHでPHASE／ENABLEモード
#define VCC 26  // 21//26 // VCCピン　0Vでスリープモード

// ## LED
uint8_t DisBuff[2 + 5 * 5 * 3]; // 表示バッファ RBGの色値を格納するために使用

// LED表示バッファへの値セット
void setBuff(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata)
{ // LEDの色を設定し、そのデータをDisBuff[]に保存する。
  DisBuff[0] = 0x05;
  DisBuff[1] = 0x05;
  for (int i = 0; i < 25; i++)
  {
    DisBuff[2 + i * 3 + 0] = Rdata;
    DisBuff[2 + i * 3 + 1] = Gdata;
    DisBuff[2 + i * 3 + 2] = Bdata;
  }
}

void setup()
{
  M5.begin(true, false, true); // Init Atom-Matrix(Initialize serial port, LED).
  delay(10);

  setBuff(0x40, 0x00, 0x00); // 緑
  M5.dis.displaybuff(DisBuff);

  // タスクの設定
  // xTaskCreatePinnedToCore(ledControl, "LEDTask0", 4096, NULL, 1, NULL, 0);

  // ステッピング用のピンの設定
  pinMode(APHASE, OUTPUT);
  pinMode(AENBL, OUTPUT);
  pinMode(BPHASE, OUTPUT);
  pinMode(BENBL, OUTPUT);
  digitalWrite(AENBL, HIGH);
  digitalWrite(BENBL, HIGH);

  pinMode(MODE, OUTPUT);
  digitalWrite(MODE, HIGH);
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH); // Current Start

  delay(1600);

  Serial.begin(115200);
}

void loop()
{
  // 振幅を変えてく
  furifuri(10, 800, switching, 4);
  delay(100);
  furifuri(30, 800, switching, 4);
  delay(100);
  furifuri(50, 800, switching, 4);
  delay(1000);

  // 速度を変えてく
  furifuri(50, 1000, switching, 4);
  delay(100);
  furifuri(50, 1200, switching, 4);
  delay(100);
  furifuri(50, 1500, switching, 4);
  delay(100);
  furifuri(50, 1800, switching, 4);
  delay(1000);

  M5.update();
}

/*---- Stepper Rotation ----*/

void DELAY_WAIT(int delayWait)
{
  delayMicroseconds(delayWait); // 700 3.7V以上　値が小さいほど待つ時間が短いので速く動く
}

void turn_forward(int stepNum, int waitTime) // 3号機:時計回り　2号機：反時計回り
{
  Serial.println("turn");

  for (int i = 0; i < stepNum; i++)
  {
    digitalWrite(APHASE, HIGH);
    DELAY_WAIT(waitTime);
    digitalWrite(BPHASE, HIGH);
    DELAY_WAIT(waitTime);
    digitalWrite(APHASE, LOW);
    DELAY_WAIT(waitTime);
    digitalWrite(BPHASE, LOW);
    DELAY_WAIT(waitTime);
  }
}

void turn_reverse(int stepNum, int waitTime) // 3号機:反時計回り　2号機：時計回り
{
  for (int i = 0; i < stepNum; i++)
  {
    digitalWrite(BPHASE, LOW);
    DELAY_WAIT(waitTime);
    digitalWrite(APHASE, LOW);
    DELAY_WAIT(waitTime);
    digitalWrite(BPHASE, HIGH);
    DELAY_WAIT(waitTime);
    digitalWrite(APHASE, HIGH);
    DELAY_WAIT(waitTime);
  }
}

void furifuri(int amp, int rotateSpeed, int switching, int count)
{

  // digitalWrite(VCC, HIGH);
  for (int i = 0; i < count; i++)
  {
    turn_forward(amp, rotateSpeed);
    delay(switching);
    turn_reverse(amp, rotateSpeed);
    delay(switching);
  }
  Serial.println("furi");
  // digitalWrite(VCC, LOW);
}