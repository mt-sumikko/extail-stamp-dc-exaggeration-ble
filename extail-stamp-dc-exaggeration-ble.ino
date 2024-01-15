/*MultiTaskにextail-stamp-exaggeration-dcを移植してstamp仕様のMultiTaskで動かしたソース
*******************************************************************************
  Copyright (c) 2021 by M5Stack
                   Equipped with STAMP-PICO sample source code
                           配套  STAMP-PICO 示例源代码
  Visit the website for more information：https://docs.m5stack.com/en/core/stamp_pico
  获取更多资料请访问：https://docs.m5stack.com/zh_CN/core/stamp_pico

  describe: MultiTask.  多线程
  date：2021/9/25
*******************************************************************************
*/
#include <Arduino.h>


// *--- 9軸センサ BNO055 ---
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // サンプル取得間のdelay

// I2C device address
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;



// *--- DC motor ---
#define IN1 32 // モーター1正転信号端子
#define IN2 33 // モーター1逆転信号端子

#define CH_IN1 0  // PWM出力チャンネル設定（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15で周波数、分解能設定が共通）
#define CH_IN2 1  //  ※チャンネル7は液晶バックライトと共用になるため指定禁止
#define FREQ 500  // PWM出力周波数　ここを変えてみても意味はなかった
#define BIT_NUM 8 // PWM出力bit数（今回は12bit指定。分解能は2の12乗で4096）//FREQ500のとき、4でもまわるけど12は回らなかった

int dir = 0; // 回転方向

int duty_max = 255; // 回転スピードに関わるが、delayだけで調整しそう。一旦MAXで一定にしておく。

// 回転スピード
int rotationSpeed = 10;
int rotationSpeed_max = 0;   // 最速
int rotationSpeed_min = 100; // 最遅

// 回転量の計算用
int targetAngle = 0;                 // 目標角度（-45〜45度の範囲）
int rotationQuantity = 0;            // センサ値に応じて動かすステップ数（角度からステップ数に変換）
int rotationQuantity_total = 0;      // のべステップ数（現在地）
int rotationQuantity_total_max = 50; // 振幅の最大値（片側分）

// *--- センサ値関係 ---
// 加速度
double accX = 0.0;
double accX_th_min = 1.5; // 閾値
double accX_th_max = 8;

// 姿勢角
double roll;     //-90~90の値を取る ただ普段人間の首はせいぜい-45~45くらいしか傾げないので、設計上は-45~45外は丸める
int roll_roundf; // rollを四捨五入した整数値
int roll_old;    // 1loop前のroll値
int roll_diff;   // 現roll値と1loop前のroll値の差（絶対値）0-180をとる

// diffの閾値 絶対値がこれ以上の場合回転させる roll
int roll_diff_th_min = 3;
int roll_diff_th_max = 20;

// 誇張係数（未使用）
int expand = 1;

bool back = false; // ホームポジションに戻す状態か



// --------------------------
// *--- Multithread tasks ---
// task1：センサ値に応じてステッパーを回す
void task1(void * pvParameters) { //Define the tasks to be executed in thread 1.  定义线程1内要执行的任务
  while (1) { //Keep the thread running.  使线程一直运行
    stepRoll();
    //stepAccX();
    vTaskDelay(5); // ステッパーがセンサ値に応じて回転する1セット分を待つインターバル
    // 0でもうごきはするが挙動が不安定になりやすいので5か10くらいにしておく。50だと流石にカクつく
  }
}

void task2(void * pvParameters) {
  while (1) {
    //Serial.print("task2 Uptime (ms): ");
    //Serial.println(millis());
    delay(200);
  }
}

void task3(void * pvParameters) {
  while (1) {
    //Serial.print("task3 Uptime (ms): ");
    //Serial.println(millis());
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);

  //DCモータ
  pinMode(IN1, OUTPUT);             // PWM出力端子（INT1：正転用）を出力設定
  pinMode(IN2, OUTPUT);             // PWM出力端子（INT2：逆転用）を出力設定
  ledcSetup(CH_IN1, FREQ, BIT_NUM); // PWM出力設定（チャンネル, 周波数, bit数）
  ledcSetup(CH_IN2, FREQ, BIT_NUM);
  ledcAttachPin(IN1, CH_IN1); // PWMチャンネルを端子に割り当て（端子番号, チャンネル）
  ledcAttachPin(IN2, CH_IN2);


  // Creat Task1.  创建线程1
  xTaskCreatePinnedToCore(
    task1,     //Function to implement the task.  线程对应函数名称(不能有返回值)
    "task1",   //线程名称
    4096,      // The size of the task stack specified as the number of * bytes.任务堆栈的大小(字节)
    NULL,      // Pointer that will be used as the parameter for the task * being created.  创建作为任务输入参数的指针
    1,         // Priority of the task.  任务的优先级
    NULL,      // Task handler.  任务句柄
    0);        // Core where the task should run.  将任务挂载到指定内核

  // Task 2
  xTaskCreatePinnedToCore(
    task2,
    "task2",
    4096,
    NULL,
    2,
    NULL,
    0);

  // Task 3
  xTaskCreatePinnedToCore(
    task3,
    "task3",
    4096,
    NULL,
    3,
    NULL,
    0);




  Wire.begin(21, 22);//pico
  while (!Serial)
    delay(50); // wait for serial port to open!

  // Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  printEvent(&orientationData);
  Serial.print("roll: ");
  Serial.println(roll);
  roll_old = roundf(roll);
  delay(1000);

  Serial.println("Ready.");



}

void loop() {


  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  printEvent(&orientationData);
  printEvent(&linearAccelData);

  // String str = "x:" + String(accX) + " target:" + String(rotationQuantity) + " rotationQuantity_total:" + String(rotationQuantity_total);
  // Serial.println(str);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}





// --------------------------
// *--- Functions ---

// *--- Stepper BaCsics ---
void turn_CW(int rotationQuantity, int rotationSpeed) // 時計回り
{
  ledcWrite(CH_IN1, rotationSpeed);
  ledcWrite(CH_IN2, 0);

  delay(rotationQuantity);
  rotationQuantity_total -= rotationQuantity;

  turn_brake();
}

void turn_CCW(int rotationQuantity, int rotationSpeed) // 反時計回り
{

  ledcWrite(CH_IN1, 0);
  ledcWrite(CH_IN2, rotationSpeed);

  delay(rotationQuantity);
  rotationQuantity_total += rotationQuantity;

  turn_brake();
}

void turn_standby()
{
  ledcWrite(CH_IN1, 0);
  ledcWrite(CH_IN2, 0);
}

void turn_brake()
{
  ledcWrite(CH_IN1, duty_max);
  ledcWrite(CH_IN2, duty_max);
}



// *--- tools ---
// 回転数をシリアルモニタに出力
void printSteps(int dir, int roundf_, int old, int diff)
{
  if (rotationQuantity != 0)
  {
    Serial.print(", roundf: ");
    Serial.print(roundf_);
    Serial.print(", old: ");
    Serial.print(old);
    Serial.print(", diff: ");
    if (dir == -1)
    {
      Serial.print("-");
    }
    Serial.print(diff);

    Serial.print(", rQuantity: ");
    if (dir == -1)
    {
      Serial.print("-");
    }
    Serial.print(rotationQuantity);

    /*Serial.print(", rSpeed: ");
      Serial.print(rotationSpeed);*/

    Serial.print(", total: ");
    Serial.print(rotationQuantity_total);

    Serial.print(", back: ");
    Serial.println(back);
    Serial.println();
  }
}

void printSteps_accX(int dir, double accX_absolute)
{
  if (rotationQuantity != 0)
  {
    String str;
    if (dir == 1)
    {
      str = "accX: " + String(accX_absolute) + " target: " + String(rotationQuantity) + " rQuantity_total:" + String(rotationQuantity_total) /*+ " rSpeed:" + String(rotationSpeed)*/;
    }
    else
    {
      str = "accX:-" + String(accX_absolute) + " target:- " + String(rotationQuantity) + " rQuantity_total:" + String(rotationQuantity_total) /*+ " rSpeed:" + String(rotationSpeed)*/;
    }
    Serial.print(str);
    Serial.print(" ");
    Serial.println(back);
  }
}







// 行った分戻る だと 正逆で回転量が違うとタチ悪いので，一旦使わず．ソースも未調整
//  回りっぱなしの軸をじわじわ中央へ戻す
void stepBack()
{
  // ステッパーを回すセンサ値になってるターンから最低1000msおいてから戻す
  //  じわじわ戻すために、戻す感覚は時間を空けたいが じわじわ戻すほど大変で最悪っぽいのでほどほどにする
  int diff = rotationQuantity_total - 0;
  if (rotationQuantity_total > 0)
  {
    Serial.print("back+ IN ");

    if (back == true)
    {
      Serial.print("差+");
      Serial.print(diff);

      turn_CW(diff, duty_max);

      Serial.print(" 戻した- total ");
      Serial.println(rotationQuantity_total);
    }
    else
    {
      delay(1000); // total==0でもここには入っているっぽいのはなぜだろう??気のせい
      Serial.print(" delayedOnly");
    }
    back = true;
    Serial.println(" Inside if block");  // 追加した行
  }
  else if (rotationQuantity_total < 0)
  {
    Serial.print("back- IN ");

    if (back == true)
    {
      Serial.print("差");
      Serial.print(diff);

      turn_CCW(diff, duty_max);

      Serial.print(" 戻した+ total ");
      Serial.println(rotationQuantity_total);
    }
    else
    {
      delay(1000);
      Serial.println(" delayedOnly");
    }
    back = true;
    Serial.println(" Inside else block");  // 追加した行
  }
  Serial.println(" Outside if-else block");  // 追加した行
}



// *--- センサー値 ---

// センサー値と現状に応じてステッパーを回す
void rotateWithSensorValue(int dir, int &rotationQuantity, int rotationSpeed)
{

  if (dir == 1) // CCW
  {
    // 振幅が既に最大値に至っている場合は回転しない
    if (rotationQuantity_total < rotationQuantity_total_max)
    {
      // ex 37<40
      // 回転量をそのまま足すと最大値を突破する場合は、最大値に収まる値を足す
      if (rotationQuantity_total + rotationQuantity > rotationQuantity_total_max)
      {
        // rotationQuantity_total + rotationQuantity <= rotationQuantity_total_max にしておきたいから
        // ex）37 + 5 > 42 これはオーバーなので補正する
        rotationQuantity = rotationQuantity_total_max - rotationQuantity_total;
        // 3 = 40 - 37
        // Serial.print("Corrected ");
      }

      turn_CCW(rotationQuantity, rotationSpeed);
      back = false;
    }
    else
    {
      //rotationQuantity = 0;

    }
  }
  else if (dir == -1) // CW
  {
    // 振幅が既に最小値に至っている場合は回転しない
    if (rotationQuantity_total > 0 - rotationQuantity_total_max)
    {
      // ex -37 > -40
      // 回転量をそのまま足すと最小値を突破する場合は、最小値に収まる値を足す
      if (rotationQuantity_total - rotationQuantity < 0 - rotationQuantity_total_max)
      {
        // rotationQuantity_total - rotationQuantity > 0 - rotationQuantity_total_max にしておきたいから
        // ex -37 -5 < -40
        rotationQuantity = rotationQuantity_total_max + rotationQuantity_total;
        // 3= 40+(-37)
        // Serial.print("Corrected ");
      }

      turn_CW(rotationQuantity, rotationSpeed);
      back = false;
    }
    else
    {
      //rotationQuantity = 0;
    }
  }
  else // dir==0
  {
    //rotationQuantity = 0;
  }
}

//  roll値の処理
void stepRoll()
{
  // 最新のroll値を確認
  roll_roundf = roundf(roll); // rollの四捨五入値
  // どのくらい動かせば良いかを知るために、1つ前の角度との差を計算
  // targetAngle = calcDiff(roll_roundf, roll_old); // ここでroll_oldからrollに動く方向がが正負のいずれかを見る
  calculateDirAndAbsDiff(roll_roundf, roll_old, dir, roll_diff);
  // Serial.print(roll_roundf);
  //  roll_diffが一定以上の場合は回転する
  if (roll_diff > roll_diff_th_min)
  {
    // 必要な回転量（ステップ数）を計算
    // 差を絶対値になるようにしているため、roll値が-90~90度をとるところを0~180度で考えている
    // 例）1ステップ3.44の場合90度動くには 90/3.44 = 26.16ステップ
    rotationQuantity = roundf(map(roll_diff, roll_diff_th_min, 20, 0, rotationQuantity_total_max * 2)); // 目標角度をステップ数に変換
    // rotationSpeed = roundf(map(roll_diff, roll_diff_th_min, 20, 100, 255));
    rotationSpeed = duty_max;                                    // 定数でよければ（仮）
    rotateWithSensorValue(dir, rotationQuantity, rotationSpeed); // 方向、回転量、スピード      // memo: 多分この3つはローカル変数にしておかないとごちゃごちゃになる
    // if (rotationQuantity != 0) {//コメントアウトしてあったけどする必要なくね？
    printSteps(dir, roll_roundf, roll_old, roll_diff);
    // }
    roll_old = roll_roundf; // 次のループに向けて古い値として保存 回転した時のみ更新
  }
  else
  {
    rotationQuantity = 0;
  }
}

// x軸方向の加速度値の処理
void stepAccX()
{
  // 回転方向決める
  if (accX < 0)
  {
    dir = -1; // CW
  }
  else if (accX > 0)
  {
    dir = 1; // CCW
  }
  else
  {
    dir = 0;
    return;
  }

  // accX_absolute の計算
  float accX_absolute = abs(accX);

  // accX_absolute の値に応じて rotationQuantity, rotationSpeed を割り当てるコードを書く
  if (accX_absolute > accX_th_min)
  {

    if (accX_absolute > accX_th_max) // 上限を8m/s^2として、それ以上の扱いは8にまるめる
    {
      accX_absolute = accX_th_max;
    }
    rotationQuantity = roundf(map(accX_absolute, accX_th_min, accX_th_max, 3, 20));
    // rotationSpeed = roundf(map(accX_absolute, accX_th_min, accX_th_max, 100, 255));
    rotationSpeed = duty_max; // 定数でよければ（仮）

    rotateWithSensorValue(dir, rotationQuantity, rotationSpeed);

    if (rotationQuantity != 0)
    {
      // accXには回ってる間にかなりの確率で次の値が代入される。printをaccXですると回転した時の値とずれるので、accX_absoluteでする。
      printSteps_accX(dir, accX_absolute);
    }
  }
  else
  {
    rotationQuantity = 0;
  }
}

// 差分計算・回転方向確定 暫定roll用の関数
// memo：値がひっくり返ってしまった時に無視する処理を入れておきたいが、なくても大丈夫そうかも
void calculateDirAndAbsDiff(int roll_roundf, int roll_old, int &dir, int &roll_diff)
{
  // 値の範囲を -90 から 90 に制限
  roll_roundf = std::max(-90, std::min(90, roll_roundf));
  roll_old = std::max(-90, std::min(90, roll_old));

  if (roll_roundf > roll_old)
  {
    dir = 1;
    roll_diff = roll_roundf - roll_old;
  }
  else if (roll_roundf < roll_old)
  {
    dir = -1;
    roll_diff = roll_old - roll_roundf;
  }
  else
  {
    dir = 0;
    roll_diff = 0;
  }

  if (roll_diff > roll_diff_th_max)
  {
    roll_diff = roll_diff_th_max; //
  }
}

// センサ値の取得
void printEvent(sensors_event_t *event)
{
  double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  {
    // Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    accX = x;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION)
  {
    // Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    roll = y;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
  {
    // Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE)
  {
    // Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
  {
    // Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    // Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    accX = x;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY)
  {
    // Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    // accX = x;
  }
  else
  {
    Serial.print("Unk:");
  }

  String str = "X:" + String(x) + "," + "Y:" + String(y) + "," + "Z:" + String(z);
  // Serial.println(str);
}
