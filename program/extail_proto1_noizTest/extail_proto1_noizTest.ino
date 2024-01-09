// 2024.01.07 1号機 unipola 騒音測定用

#include "M5Atom.h"

#define DIR 25  // 回転方向ピン
#define STEP 21 // ステップピン
#define INH 22  // INHピン

int step_max = 70; // ステップ数（振幅）最大値

// LED表示バッファ
uint8_t DisBuff[2 + 5 * 5 * 3]; // RBGの色値を格納するために使用

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

// LED点灯タスクを渡す変数
int ledTask = 0;

// LEDの点灯をコントロール
void ledControl(void *arg)
{
  while (1)
  {

    if (ledTask == 1)
    {
      // 主電源ON：緑　1秒点灯
      ledTask = 0;
      setBuff(0x40, 0x00, 0x00); // 緑
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); // 消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 2)
    {

      // BLE接続 ：LED 青　1秒点灯
      ledTask = 0;
      setBuff(0x00, 0x00, 0x40); // 青
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); // 消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 3)
    {
      // BLE切断：LED 赤　1秒点灯
      ledTask = 0;
      setBuff(0x00, 0x40, 0x00); // 赤
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); // 消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 4)
    {
      // 心拍受信確認：LED 白　1秒点灯
      ledTask = 0;
      setBuff(0x20, 0x20, 0x20); // 白
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); // 消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 5)
    {
      // BLE受信確認：LED 青 2回点滅
      ledTask = 0;
      setBuff(0x0, 0x0, 0x30);
      M5.dis.displaybuff(DisBuff);
      delay(200);
      setBuff(0x0, 0x0, 0x0); // 消灯
      M5.dis.displaybuff(DisBuff);
      delay(200);
      setBuff(0x0, 0x0, 0x30);
      M5.dis.displaybuff(DisBuff);
      delay(200);
      setBuff(0x0, 0x0, 0x0);
      M5.dis.displaybuff(DisBuff);
    }
    else
    {
      // 適度に待つ
      delay(100);
    }
  }
}

void setup()
{
  M5.begin(true, false, true); // Init Atom-Matrix(Initialize serial port, LED).
  delay(10);
  setBuff(0x40, 0x00, 0x00);   // 緑
  M5.dis.displaybuff(DisBuff); // Display the DisBuff color on the LED.

  // タスクの設定
  xTaskCreatePinnedToCore(ledControl, "LEDTask0", 4096, NULL, 1, NULL, 0);

  // ステッピング用のピンの設定
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(INH, OUTPUT);
  delay(1600);

  Serial.begin(115200);
}

uint8_t FSM = 0; // Store the number of key presses.
void loop()
{
  furifuri_noiseTest();

  if (M5.Btn.wasPressed())
  {
    Serial.println("pressed");
  }

  M5.update();
}

void furifuri_noiseTest()
{
  int switching = 100;

  // 振幅を変えてく
  furifuri_base(10, 2, switching, 4);
  delay(100);
  furifuri_base(30, 2, switching, 4);
  delay(100);
  furifuri_base(50, 2, switching, 4);
  delay(1000);
  // 速度を変えてく
  furifuri_base(50, 4, switching, 4);
  delay(100);
  furifuri_base(50, 6, switching, 2);
  delay(100);
  furifuri_base(50, 8, switching, 2);
  delay(100);
  furifuri_base(50, 12, switching, 1);
  delay(100);
  furifuri_base(50, 20, switching, 1);
  delay(1000);
}

/*---- Basic --*/
// ステッピングモーターの一方高回転（片道分）を実現する基本関数
void step(int stepNum, int waitTime)
{
  for (int i = 0; i < stepNum; i++)
  {
    digitalWrite(STEP, HIGH);
    delay(waitTime);
    digitalWrite(STEP, LOW);
    delay(waitTime);
  }
}

void furifuri_base(int stepNum, int waitTime, int switching, int count)
{

  for (int i = 0; i < count; i++)
  {
    digitalWrite(DIR, HIGH); // 正転
    step(stepNum, waitTime);
    // digitalWrite(INH, HIGH);//Current Stop
    delay(switching);
    // digitalWrite(INH, LOW);//Current Start
    // delayMicroseconds(5); //立ち上がりを待つ

    digitalWrite(DIR, LOW); // 逆転
    step(stepNum, waitTime);
    // digitalWrite(INH, HIGH);//Current Stop
    delay(switching);
    // digitalWrite(INH, LOW);//Current Start
    // delayMicroseconds(5);
  }
}