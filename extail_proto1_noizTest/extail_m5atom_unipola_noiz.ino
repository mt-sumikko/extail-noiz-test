/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   接続を受けたら、定期的に通知を送るBLEサーバを作成します。
   このサービスは、自分自身を宣伝するものである。6E400001-B5A3-F393-E0A9-E50E24DCCA9E

   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"
   の特性を持つ。6E400002-B5A3-F393-E0A9-E50E24DCCA9E - "WRITE "でデータを受信するために使用されます。
   の特性を有する。6E400003-B5A3-F393-E0A9-E50E24DCCA9E - "NOTIFY "でデータを送信するために使用されます。

   BLEサーバーを作る設計は。
   1. BLEサーバーの作成
   2. BLEサービスを作成する
   3. サービス上にBLEキャラクタリスティックを作成する
   4. 特性にBLEディスクリプタを作成する
   5. サービスを開始する
   6. 広告を開始する

   この例では、rxValueが受信したデータです（この関数の内部でのみアクセス可能）。
   そしてtxValueは送信されるデータ。
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <BLE2902.h>
#include "M5Atom.h"

#define DIR 25  //回転方向ピン
#define STEP 21 //ステップピン
#define INH 22//INHピン

int step_max = 70;//ステップ数（振幅）最大値

int bpm = 70; //送られてきたbpm値をintに直して保管（初期値は標準的な値）
int rhr = 60; //送られてきたrhr値をintに直して保管（初期値は標準的な値）
int siri = 0;//送られてきたsiriのコマンド値をintに直して保管（初期値は実質なし）

//マニュアル入力初期値
int amp_manual = 70;
int speed_manual = 4;
int delay_manual = 500;

int bpm_max = 140;//bpm想定最大値
int bpm_min = 30;//bpm想定最小値

int runningMode = 1;//0:HeratBeat, 1:Auto 2:Manual

bool flg_ble = false;//ble接続してるか

//LED表示バッファ
uint8_t DisBuff[2 + 5 * 5 * 3]; //RBGの色値を格納するために使用

//LED表示バッファへの値セット
void setBuff(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata)
{ //LEDの色を設定し、そのデータをDisBuff[]に保存する。
  DisBuff[0] = 0x05;
  DisBuff[1] = 0x05;
  for (int i = 0; i < 25; i++)
  {
    DisBuff[2 + i * 3 + 0] = Rdata;
    DisBuff[2 + i * 3 + 1] = Gdata;
    DisBuff[2 + i * 3 + 2] = Bdata;
  }
}

//LED点灯タスクを渡す変数
int ledTask = 0;

//LEDの点灯をコントロール
void ledControl(void *arg)
{
  while (1)
  {

    if (ledTask == 1)
    {
      //主電源ON：緑　1秒点灯
      ledTask = 0;
      setBuff(0x40, 0x00, 0x00); //緑
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); //消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 2)
    {

      //BLE接続 ：LED 青　1秒点灯
      ledTask = 0;
      setBuff(0x00, 0x00, 0x40); //青
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); //消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 3)
    {
      //BLE切断：LED 赤　1秒点灯
      ledTask = 0;
      setBuff(0x00, 0x40, 0x00); //赤
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); //消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 4)
    {
      //心拍受信確認：LED 白　1秒点灯
      ledTask = 0;
      setBuff(0x20, 0x20, 0x20); //白
      M5.dis.displaybuff(DisBuff);
      delay(1000);
      setBuff(0x0, 0x0, 0x0); //消灯
      M5.dis.displaybuff(DisBuff);
    }
    else if (ledTask == 5)
    {
      //BLE受信確認：LED 青 2回点滅
      ledTask = 0;
      setBuff(0x0, 0x0, 0x30);
      M5.dis.displaybuff(DisBuff);
      delay(200);
      setBuff(0x0, 0x0, 0x0); //消灯
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
      //適度に待つ
      delay(100);
    }
  }
}



/*--- BLE ---*/
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;    //デバイスと繋がっているかのフラグ
bool oldDeviceConnected = false; //記録済みデバイスと繋がっているかのフラグ
uint8_t txValue = 0;

int LEDPinNo = 22;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    { //BLE接続した
      deviceConnected = true;
      ledTask = 2;
    };

    void onDisconnect(BLEServer *pServer)
    { //BLE接続解除した
      deviceConnected = false;
      ledTask = 3;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string rxValue = pCharacteristic->getValue();

      Serial.println("Receive");

      // 文字列として受信した値の読み取り開始位置
      int from = 0;

      // メッセージの処理が終了したことを示すフラグ
      boolean noMoreEvent = false;

      // メッセージの処理が終了するまで以下を繰り返す
      while (!noMoreEvent)
      {


        //①【Mode用】受信した文字列の中で「>」を探す
        int index_mode = rxValue.find(">", from);

        // もし見つからなければ
        if (index_mode < 0)
        {
          // 処理が終了したと判断してフラグをセット
          noMoreEvent = true;
          Serial.println("mode fine");
        }
        // もし見つかったら
        else
        {
          // 部分文字列をコマンドとして取り出し
          std::string command = rxValue.substr(from, index_mode);
          runningMode = atoi(command.c_str());
          Serial.println("mode："+String(runningMode));
          // 次に処理する読み取り開始位置を更新
          from = index_mode + 1;
        }


        //--- ▼ Manual Mode Only▼ ---
        if (runningMode == 2) {

          //【amp用】受信した文字列の中で「$」を探す
          int index_amp = rxValue.find("$", from);

          // もし見つからなければ
          if (index_amp < 0)
          {
            // 処理が終了したと判断してフラグをセット
            noMoreEvent = true;
            Serial.println("amp fine");
          }
          // もし見つかったら
          else
          {
            // 部分文字列をコマンドとして取り出し
            std::string command = rxValue.substr(from, index_amp);
            amp_manual = atoi(command.c_str());
            Serial.println("[manual] amp："+String(amp_manual));
            // 次に処理する読み取り開始位置を更新
            from = index_amp + 1;
          }


          //【speed用】受信した文字列の中で「$」を探す
          int index_speed = rxValue.find("%", from);

          // もし見つからなければ
          if (index_speed < 0)
          {
            // 処理が終了したと判断してフラグをセット
            noMoreEvent = true;
            Serial.println("speed_manual fine");
          }
          // もし見つかったら
          else
          {
            // 部分文字列をコマンドとして取り出し
            std::string command = rxValue.substr(from, index_speed);
            speed_manual = atoi(command.c_str());
           Serial.println("[manual] speed："+String(speed_manual));
            // 次に処理する読み取り開始位置を更新
            from = index_speed + 1;
          }


          //【delay用】受信した文字列の中で「$」を探す
          int index_delay = rxValue.find("@", from);

          // もし見つからなければ
          if (index_delay < 0)
          {
            // 処理が終了したと判断してフラグをセット
            noMoreEvent = true;
            Serial.println("delay_manual fine");
          }
          // もし見つかったら
          else
          {
            // 部分文字列をコマンドとして取り出し
            std::string command = rxValue.substr(from, index_delay);
            delay_manual = atoi(command.c_str());
           Serial.println("[manual] delay："+String(delay_manual));
            // 次に処理する読み取り開始位置を更新
            from = index_delay + 1;
          }

        }


        //【Siri（モーションパターン）用】受信した文字列の中で「¥」を探す
        int index_siri = rxValue.find("¥", from);
        // もし見つからなければ
        if (index_siri < 0)
        {
          // 処理が終了したと判断してフラグをセット
          noMoreEvent = true;
          Serial.println("siri fine");
        }
        // もし見つかったら
        else
        {
          // 部分文字列をコマンドとして取り出し
          std::string command = rxValue.substr(from, index_siri);
          siri = atoi(command.c_str());
          Serial.println("siri："+ String(siri));
          // 次に処理する読み取り開始位置を更新
          from = index_siri + 1;

          if (siri == 1) {
            buruburu();
          } else if (siri == 2) {
            glad();
          }
          else if (siri == 3) {
            rightFace();
          } else if (siri == 4) {
            leftFace();
          } else if (siri == 6) {
            pikupiku();
          }
        }


        //【RHR用】受信した文字列の中で「~」を探す
        int index_rhr = rxValue.find("~", from);

        // もし見つからなければ
        if (index_rhr < 0)
        {
          // 処理が終了したと判断してフラグをセット
          noMoreEvent = true;
          Serial.println("rhr fine");
        }
        // もし見つかったら
        else
        {
          // 部分文字列をコマンドとして取り出し
          std::string command = rxValue.substr(from, index_rhr);
          rhr = atoi(command.c_str());
          Serial.println("rhr："+ String(rhr));
          // 次に処理する読み取り開始位置を更新
          from = index_rhr + 1;
        }


        // 【bpm・接続確認用】受信した文字列の中で「*」を探す
        int index = rxValue.find("*", from);
        // もし見つからなければ
        if (index < 0)
        {
          // 処理が終了したと判断してフラグをセット
          noMoreEvent = true;
          Serial.println("bpm fine");
        }
        // もし見つかったら
        else
        {
          // 部分文字列をコマンドとして取り出し
          std::string command = rxValue.substr(from, index);
          //char com = rxValue.substr(from, index);

          // コマンドに応じて処理
          if (command == "cmf")//confirm
          {
            Serial.println("Confirm");
            ledTask = 5;
          }
          else //心拍数が送られてきた
          {
            bpm = atoi(command.c_str());
          Serial.println("bpm："+ String(bpm));
            ledTask = 4;
          }
          //M5.dis.displaybuff(DisBuff);

          // 次に処理する読み取り開始位置を更新
          from = index + 1;
        }
      }

    }
};




void setup()
{
  M5.begin(true, false, true); //Init Atom-Matrix(Initialize serial port, LED).
  delay(10);
  setBuff(0x40, 0x00, 0x00);   //緑
  M5.dis.displaybuff(DisBuff); //Display the DisBuff color on the LED.

  //タスクの設定
  xTaskCreatePinnedToCore(ledControl, "LEDTask0", 4096, NULL, 1, NULL, 0);

  //ステッピング用のピンの設定
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(INH, OUTPUT);
  delay(1600);

  Serial.begin(115200);

  /*--- BLE ---*/
  // Create the BLE Device
  BLEDevice::init("UART Service"); //BLE用のサービス名

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks()); //MyCallbacksクラスのインスタンス化

  // サービスの開始
  pService->start();

  // アドバタイズの開始
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}




uint8_t FSM = 0; //Store the number of key presses.
void loop()
{
  //    if (deviceConnected) {
  //        pTxCharacteristic->setValue(&txValue, 1);
  //        pTxCharacteristic->notify();
  //        txValue++;
  //    delay(10); // bluetooth stack will go into congestion, if too many packets are sent / Bluetoothスタックは、多くのパケットを送信すると輻輳（いろいろなものが同じ箇所に集中して混雑する状況）状態になります。
  //  }

  // disconnecting　接続が切れたら
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising

    //セントラルの探索を開始
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
    flg_ble = false;
    Serial.println("flg_ble："+String(flg_ble));

  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    setBuff(0x00, 0x00, 0x40); //blue：接続
    Serial.println("oldDeviceConnected");
    flg_ble = true;
   Serial.println("flg_ble："+String(flg_ble));
  }

  //Mode別の挙動について
  if (runningMode == 2) { // Mode: Manual
    digitalWrite(DIR, LOW);
    step(amp_manual, speed_manual);
    delay(delay_manual);
    digitalWrite(DIR, HIGH);
    step(amp_manual, speed_manual);
    delay(delay_manual);

  } else { /// Mode: BPM or Auto
    furifuri();
  }


  if (M5.Btn.wasPressed())
  {
    Serial.println("pressed");
  }

  M5.update();
}

// Motion Pattern（1）：基本のフリフリ、ランダム要素の実行も含む
void furifuri() {

  //BLE通信していないときは、furifuri開始時にAtom側でbpmをランダムにセット
  if (!flg_ble) {
    bpm = random(60, bpm_max);
    Serial.println("randam bpm："+String(bpm));
  }

  int diff = bpm - rhr;
  //bpmがrhrに対して高すぎる・低すぎるときは値を調整する（上限下限を決めておく）
  if (diff > 50) {
    diff = 50;
  } else if (diff < -10) {
    diff = -10;
  }

  int amp = map(diff, -10, 50, step_max, 20);
  int speed = map(diff, -10, 50, 25, 2);
  int switching = map(diff, -10, 50, 500, 2);
  int endDealy = map(diff, -10, 50, 50, 100);

  Serial.println("Amp："+String(amp)+", Speed"+String(speed)+", Switching"+String(switching));


  int randomCount = random(5, 20);
  Serial.println(randomCount);
  for (int i = 0; i < randomCount; i++) {
    digitalWrite(DIR, HIGH);
    step(amp, speed);
    digitalWrite(INH, HIGH);//Current Stop
    delay(switching);
    digitalWrite(INH, LOW);//Current Start
    delayMicroseconds(5); //立ち上がりを待つ

    digitalWrite(DIR, LOW); //逆転
    step(amp, speed);
    digitalWrite(INH, HIGH);//Current Stop
    delay(switching);
    digitalWrite(INH, LOW);//Current Start
    delayMicroseconds(5);
  }

  //最大15秒くらい静止する
    int randomNum = random(50, 150);
    Serial.println("Wait：" + String(endDealy * randomNum));
  digitalWrite(INH, HIGH);//Current Stop
  delay(endDealy * randomNum);
  digitalWrite(INH, LOW);//Current Start
  delayMicroseconds(5);



  //----たまたまある値が出たら違うパターンを実行する----
  int randomPattern = random(0, 15);
  if (randomPattern < 5) {
    Serial.println("randomPattern"+String(randomPattern));

    int diff = bpm - rhr;
    //bpmがrhrに対して高すぎる・低すぎるときは値を調整する（上限下限を決めておく）
    if (diff > 50) {
      diff = 50;
    } else if (diff < -10) {
      diff = -10;
    }

    int endDealy = map(diff, -10, 50, 50, 100);

    if (randomPattern == 0) {
      pikupiku();
    } else if (randomPattern == 1) {
      right2();
    } else if (randomPattern == 2) {
      left2();
    }else if (randomPattern == 3) {
      buruburu();
    } else if (randomPattern == 4) {
      glad();
    }
//---------------------------------------

    //最大15秒くらい静止する
    int randomNum = random(50, 150);
    Serial.println("Wait：" + String(endDealy * randomNum));
    //digitalWrite(INH, HIGH);//Current Stop
    delay(endDealy * randomNum);
    //digitalWrite(INH, LOW);//Current Start
    delayMicroseconds(5);

  }
  
}


// Motion Pattern（2）：ピクピクとした動き
void pikupiku() {

  int randomCount = random(5, 20);
    int randomSpeed = random(2, 6);
  Serial.print("randomCount："+String(randomCount)+", randomSpeed："+String(randomSpeed));


  int speed;

  if (bpm - rhr < 18) {
    speed = 20 - (bpm - rhr);
  } else {
    speed = 2;
  }



  for (int i = 0; i < randomCount; i++) {
    int randomAmp = random(8, 30);

    int randomSpan1 = random(50, 500) * speed;
   Serial.print("randomSpan1："+String(randomSpan1));
    digitalWrite(DIR, HIGH);
    step(randomAmp, randomSpeed);
    digitalWrite(INH, HIGH);//Current Stop
    delay(randomSpan1);
    digitalWrite(INH, LOW);//Current Start
    delayMicroseconds(5);

    int randomSpan2 = random(50, 500) * speed;
     Serial.print("randomSpan2："+String(randomSpan2));
    digitalWrite(DIR, LOW); //逆転
    step(randomAmp, randomSpeed);
    digitalWrite(INH, HIGH);//Current Stop
    delay(randomSpan2);
    digitalWrite(INH, LOW);//Current Start
    delayMicroseconds(5);

  }

}

// Motion Pattern（3）：右右
void right2() {

  int speed;

  if (bpm - rhr < 18) {
    speed = 20 - (bpm - rhr);
  } else {
    speed = 2;
  }

  digitalWrite(DIR, HIGH);
  step(40, speed);
  delay(0);

  digitalWrite(DIR, LOW);
  step(20, speed);
  delay(0);

  digitalWrite(DIR, HIGH);
  step(20, speed);
  digitalWrite(INH, HIGH);//Current Stop
  delay(1000);
  digitalWrite(INH, LOW);//Current Start
  delayMicroseconds(5);

  digitalWrite(DIR, LOW);
  step(40, speed);

  delay(10);


}

// Motion Pattern（4）：左左
void left2() {
Serial.println("left2");

  int speed;

  if (bpm - rhr < 18) {
    speed = 20 - (bpm - rhr);
  } else {
    speed = 2;
  }

  digitalWrite(DIR, LOW);
  step(40, speed);
  delay(0);

  digitalWrite(DIR, HIGH);
  step(20, speed);
  delay(0);

  digitalWrite(DIR, LOW);
  step(20, speed);
  digitalWrite(INH, HIGH);//Current Stop
  delay(1000);
  digitalWrite(INH, LOW);//Current Start
  delayMicroseconds(5);

  digitalWrite(DIR, HIGH);
  step(40, speed);

  delay(10);


}


/*---- For Siri Command --*/

// Motion Pattern（5）：ブルブル「Hey Siri, 寒いね」
void buruburu() {
  Serial.print("cold | ");

  int randomCount = random(20, 60);
  int randomAmp = random(8, 11);
  int randomSpeed = random(2, 4);
  Serial.println("Count：" + String(randomCount) + ", Amp：" + String(randomAmp) + ", Speed：" + String(randomSpeed));

  for (int i = 0; i < randomCount; i++) {
    digitalWrite(DIR, HIGH); //正転
    step(randomAmp, randomSpeed);

    digitalWrite(DIR, LOW); //逆転
    step(randomAmp, randomSpeed);

 //最後なるべく真ん中で終わるように調整
    if (i == randomCount - 1) {
      digitalWrite(DIR, HIGH);
      step(randomAmp / 2, randomSpeed);
    }
  }
}

// Motion Pattern（5）：ブンブン大振り, 切り返しゆっくりめ「Hey Siri, 嬉しいね」
void glad() {
  Serial.print("glad | ");

  int randomCount = random(7, 15);
  int randomAmp = random(40, step_max - 10);
  int randomSpeed = random(2, 4);
  int randomDelay = random(30, 150);
Serial.println("Count：" + String(randomCount) + ", Amp：" + String(randomAmp) + ", Speed："+ String(randomSpeed)+ ", Delay："+ String(randomDelay));

  for (int i = 0; i < randomCount; i++) {
    digitalWrite(DIR, HIGH);
    step(randomAmp, randomSpeed);

    delay(randomDelay);

    digitalWrite(DIR, LOW); //逆転
    step(randomAmp, randomSpeed);

    delay(randomDelay);

    //最後なるべく真ん中で終わるように調整
    if (i == randomCount - 1) {
      digitalWrite(DIR, HIGH);
      step(randomAmp / 2, randomSpeed);
    }
  }

}

// Motion Pattern（6）：ブンブン「Hey Siri, 右向け右」
void rightFace() {
Serial.println("rightFace");

  digitalWrite(DIR, HIGH);
  step(step_max - 20, 2);

  digitalWrite(INH, HIGH);//Current Stop
  delay(2000);
  digitalWrite(INH, LOW);//Current Start
  delayMicroseconds(5);

  digitalWrite(DIR, LOW);
  step(40, 10);

}


// Motion Pattern（7）：ブンブン「Hey Siri, 左向け左」
void leftFace() {
Serial.println("leftFace");

  digitalWrite(DIR, LOW);
  step(step_max - 20, 2);

  digitalWrite(INH, HIGH);//Current Stop
  delay(2000);
  digitalWrite(INH, LOW);//Current Start
  delayMicroseconds(5);

  digitalWrite(DIR, HIGH);
  step(40, 10);


}


/*---- Basic --*/
//ステッピングモーターの一方高回転（片道分）を実現する基本関数
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
