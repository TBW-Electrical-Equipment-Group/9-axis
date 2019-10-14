/*9軸センサープログラム
 （モジュール名）BMX055、MPU9250
 9軸センサーからの加速度のx,y,z,ジャイロ（角速度）のx,y,z,方位のx,y,zの情報からロール角、ピッチ角、ヨー角を算出することで機体の姿勢を推定します。
 (URL)watako-lab.com/2019/01/01/m5stack_sens_9axis/からある程度の概要を理解できます。
BMX055からはジャイロと方位を、MPU9250から加速度の情報を使っています。BMX055は加速度の精度がよくなく、MPU9250はジャイロの精度がよくなかったのでお互いの精度
 の良いほうを活用しています。
 BMX055はakizukidenshi.com/catalog/g/gK-13010/（サンプルプログラム）とhttps://www.mouser.jp/datasheet/2/783/BST-BMX055-DS000-1509552.pdf
を参考にジャイロ、方位の情報を取得。（※秋月は方位情報のプログラム部分にミスがあったのでhttps://www.mouser.jp/datasheet/2/783/BST-BMX055-DS000-1509552.pdf
の説明書を読んで修正しました。）
 MPU9250はakiracing.com/を基に加速度の情報を取得。
*/

/*BMX055*/
#include<Wire.h>
// BMX055　加速度センサのI2Cアドレス
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)
#define MPU9250_ADDRESS 0x68//I2CでのMPU9250のスレーブアドレス
#define PWR_MGMT_1 0x6b//電源管理のアドレス，スリープモード解除用
#define INT_PIN_CFG 0x37//磁気センサのバイパスモード設定用のアドレス


/*MPU9250*/
#define ACCEL_CONFIG 0x1c//加速度センサ設定用のアドレス
#define ACCEL_FS_SEL_2G 0x00//加速度センサのレンジ(2G)
#define ACCEL_FS_SEL_4G 0x08//加速度センサのレンジ(4G)
#define ACCEL_FS_SEL_8G 0x10//加速度センサのレンジ(8G)
#define ACCEL_FS_SEL_16G 0x18//加速度センサのレンジ(16G)
#define GYRO_CONFIG 0x1b//ジャイロセンサ設定用のアドレス
#define GYRO_FS_SEL_250DPS 0x00//ジャイロセンサのレンジ(250DPS)
#define GYRO_FS_SEL_500DPS 0x08//ジャイロセンサのレンジ(500DPS)
#define GYRO_FS_SEL_1000DPS 0x10//ジャイロセンサのレンジ(1000DPS)
#define GYRO_FS_SEL_2000DPS 0x18//ジャイロセンサのレンジ(2000DPS)
#define AK8963_ADDRESS 0x0c//磁気センサのスレーブアドレス
#define CNTL1 0x0a//磁気センサ設定用のアドレス
#define CNTL1_MODE_SEL_8HZ 0x12//磁気センサの出力周期(8Hz)
#define CNTL1_MODE_SEL_100HZ 0x16//磁気センサの出力周期(100Hz)
#define ST1 0x02//データ読み込み用フラッグのアドレス


//センサーの値を保存するグローバル関数BMX055
float Gyro[3] = {0.00};//ジャイロの変数（x,y,z)
float Angle[3]= {0.00};//角度の変数（x,y,z)
int   Mag[3]  = {0};//方位の変数 （x,y,z)
int Lev_Mag[3] = {0};//水平座標に変換したあとの方位の変数（x,y,z)
unsigned long timer_Gyro = 0.00;//時間積分し角度を算出するための変数
unsigned long timer_Gyro_old =0.00;//時間積分し角度を算出するための変数
unsigned long timer_Gyro_interval = 0.00;//時間積分し角度を算出するための変数

//センサーの値を保存するグローバル関数MPU9250
volatile float accRange;//計算で使用するので，選択したレンジを入力する定数
volatile float gyroRange;//計算で使用するので，選択したレンジを入力する定数
volatile uint8_t accGyroTempData[14];//センサからのデータ格納用配列
volatile uint8_t magneticData[7];//センサからのデータ格納用配列
volatile uint8_t ST1Bit;//磁気センサのフラッグ
volatile int16_t a[3] = {0};//16bitの出力データ
volatile float MPU_Acc_Angle[2]={0};//MPU9250の加速度から求めたロール角とピッチ角の変数
volatile float acc[3] = {0};//加速度センサから求めた重力加速度
//表示
unsigned long timer =0.00;//一秒ごとに表示するための変数
unsigned long timer_old =0.00;//一秒ごとに表示するための変数
unsigned long second1_timer =0.00;//一秒ごとに表示するための変数

int i=0;
volatile float Axisch = 0;

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバック用シリアル通信は9600bps
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(300);
  //MPU9250　初期化
  i2cWriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);//スリープモードを解除
  i2cWriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_FS_SEL_16G);//加速度センサの測定レンジの設定
  accRange = 16.0;//計算で使用するので，選択したレンジを入力する
  i2cWriteByte(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);//ジャイロセンサの測定レンジの設定
  gyroRange = 2000.0;//計算で使用するので，選択したレンジを入力する
  i2cWriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);//bypass mode(磁気センサが使用出来るようになる)
  i2cWriteByte(AK8963_ADDRESS, CNTL1, CNTL1_MODE_SEL_100HZ);//磁気センサのAD変換開始
}

void loop()
{//1秒の計測
  timer = millis();
  second1_timer += timer - timer_old;
  timer_old= timer;

  //MPU9250関数
  MPU9250();
  //BMX055 ジャイロの読み取り
  BMX055_Gyro();
   //BMX055 磁気の読み取り
  BMX055_Mag();

  if(second1_timer > 1000){
    Serial.println("--------------------------------------");
    //重力加速度+機体加速度、速度の表示
    Serial.print("Gravity+Accl= ");
    for(i=0;i<3;i++){
      Serial.print(acc[i]);
      Serial.print(",");
    }
    Serial.println("");

   //角速度、角度の表示
   Serial.print("Gyro= ");
   for(i=0;i<3;i++){
      Serial.print(Gyro[i]);
      Serial.print(",");
    }
   Serial.println("");
   Serial.print("Angle = ");
    for(i=0;i<3;i++){
      Serial.print(Angle[i]);
      Serial.print(",");
    }
   Serial.println("");
  //磁気の表示
  Serial.print("Mag= ");
   for(i=0;i<3;i++){
      Serial.print(Lev_Mag[i]);
      Serial.print(",");
    }
  Serial.println("");
  Serial.println("");

  //タイマーの初期化
  second1_timer -=1000;
  }
  delay(1);
}

//=====================================================================================//
//BMX055の初期化する関数（詳しくはhttps://www.mouser.jp/datasheet/2/783/BST-BMX055-DS000-1509552.pdf）
void BMX055_Init()
{
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

//=====================================================================================//
void BMX055_Gyro()
{
  //時間積分のための一定時間の算出
    timer_Gyro = micros();
    timer_Gyro_interval = timer_Gyro - timer_Gyro_old;
    timer_Gyro_old = timer_Gyro;

   //ジャイロ情報
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  for(i=0;i<3;i++){
    Gyro[i] = (data[2*i+1] * 256) + data[i*2];
  if (Gyro[i] > 32767)  Gyro[i] -= 65536;
  }
  for(i=0;i<3;i++){
  Gyro[i] = (Gyro[i] * 0.0038); //  Full scale = +/- 125 degree/s
   if(Gyro[i]<1.1 && Gyro[i]>-1.1){
    Gyro[i] = 0;
  }
}
//軸の統一
  Axisch = Gyro[0];
  Gyro[0] = Gyro[1];
  Gyro[1] = Axisch;
 //加速度ベクトルの大きさに応じてローパスフィルター（LPF）の係数を変更https://garchiving.com/gyro-drift-correction/
if(sqrt(pow(acc[0],2)+pow(acc[1],2)+pow(acc[2],2))<10.0){
  for(i=0;i<2;i++){
    Angle[i] = (Angle[i] + (Gyro[i]* timer_Gyro_interval/1000000))*0.85+ ((-1)*MPU_Acc_Angle[i])*0.15;
  }
 }
 else {
   for(i=0;i<2;i++){
    Angle[i] = (Angle[i] + (Gyro[i]* timer_Gyro_interval/1000000))*0.95+ ((-1)*MPU_Acc_Angle[i])*0.05;
  }
 }
}
//=====================================================================================//
void BMX055_Mag()
{
  //方位情報の取得
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  for(i=0;i<2;i++){
    Mag[i] = ((data[i*2+1] <<5) | (data[i*2]>>3));
    if (Mag[i] > 4095)  Mag[i] -= 8192;
  }
   Mag[2] = ((data[5] <<7) | (data[4]>>1));
  if (Mag[2] > 16383)  Mag[2] -= 32768;

//軸の統一
  Mag[1] = -1 * Mag[1];
  Mag[2] = -1 * Mag[2];

  //水平座標に戻す座標変換（参考：https://watako-lab.com/2019/02/20/3axis_cmps/）
 Lev_Mag[0]= Mag[0]*cosf((3.1415/180)*Angle[0]) + Mag[1]*sinf((3.14/180)*Angle[0])*sinf((3.14/180)*Angle[1]) + Mag[2]*sinf((3.14/180)*Angle[0])*cosf((3.14/180)*Angle[1]);
  Lev_Mag[1] = (0 + Mag[1]*cosf((3.14/180)*Angle[1]) - Mag[2]*sinf((3.14/180)*Angle[1]));
  Lev_Mag[2] = -Mag[0]*sinf((3.14/180)*Angle[0]) + Mag[1]*cosf((3.14/180)*Angle[0])*sinf((3.14/180)*(-1)*Angle[1]) + Mag[2]*cosf((3.14/180)*Angle[0])*cosf((3.14/180)*Angle[1]);
  //角度に応じてローパスフィルターの係数を変更（傾きが大きいほどジャイロの信用性を重視）、(atan2f(Lev_Mag[1],Lev_Mag[0]))*(180/3.14)から方位情報からのヨー角を算出
  if((Angle[0]<1 && Angle[0]> -1) && (Angle[1]<1 && Angle[1]> -1)){
    Angle[2] = (Angle[2] + (Gyro[2] * timer_Gyro_interval/1000000))*0.99 + (atan2f(Lev_Mag[1],Lev_Mag[0]))*(180/3.14)*0.01;
  }
  else if((Angle[0]<0.5&& Angle[0]> -0.5) && (Angle[1]<0.5 && Angle[1]> -0.5)){
    Angle[2] = (Angle[2] + (Gyro[2] * timer_Gyro_interval/1000000))*0.9 + (atan2f(Lev_Mag[1],Lev_Mag[0]))*(180/3.14)*0.1;
  }
  else{
    Angle[2] =Angle[2] + (Gyro[2] * timer_Gyro_interval/1000000);
  }
}
//=====================================================================================//
//ラジアンから度（degree)に変換
float to_deg(float r) {
    return r * 180.0 / 3.141592;
}


//===============================================================================================//
//MPU9250から加速度の取得
void MPU9250() {
  //ACC&GRYO///////////////////////////////////////////////////////////////////////////
  i2cRead(MPU9250_ADDRESS, 0x3b, 14, accGyroTempData); //0x3bから，14バイト分をaccGyroDataにいれる

 //Acc
  for(i=0;i<3;i++){
  a[i] = (accGyroTempData[i*2] << 8) | accGyroTempData[i*2+1];//accGyroTempData[i*2]を左に8シフトし(<<)，accGyroTempData[i*2+1]を足し合わせる(|)
  }
  for(i=0;i<3;i++){
  acc[i] = a[i] * accRange / 32768.0;//[G]に変換
  acc[i] = acc[i] * 9.8;
}

  acc[0] = -1 * acc[0];//軸の統一

  //ノイズの除去
 if(acc[0]>-0.2&&acc[0]<0.4){
    acc[0] = (int)acc[0];
  }
  if(acc[1]>-0.2&&acc[1]<0.2){
    acc[1] = (int)acc[1];
  }
  if(acc[2]>-0.2&&acc[2]<0.2){
    acc[2] = (int)acc[2];
  }

  if(acc[1]==0.00 && acc[2] ==0.00){
    MPU_Acc_Angle[0]= 90;
  }
  else {
    MPU_Acc_Angle[0]= atanf( acc[1]/ acc[2]);
  }
  //加速度情報からロール、ピッチ角を算出
  MPU_Acc_Angle[1] =atanf((-1)*acc[0]/sqrt(acc[1]*acc[1]+acc[2]*acc[2]));
  MPU_Acc_Angle[0] = to_deg(MPU_Acc_Angle[0]);
  MPU_Acc_Angle[1] = to_deg(MPU_Acc_Angle[1]);
}

//指定したアドレスのデータを読む関数
void i2cRead(uint8_t Address, uint8_t Register, uint8_t NBytes, volatile uint8_t* Data) {
  Wire.beginTransmission(Address);//指定したアドレスと通信を始める
  Wire.write(Register);//レジスタを書き込む
  Wire.endTransmission();//通信を終了する
  Wire.requestFrom(Address, NBytes);//スレーブからNByteのデータを要求する
  uint8_t index = 0;
  while (Wire.available()) {
    Data[index++] = Wire.read();//データを読み込む
  }
 }

//指定したアドレスにデータを書き込む関数
void i2cWriteByte(uint8_t Address, uint8_t Register, volatile uint8_t Data) {
  Wire.beginTransmission(Address);//指定したアドレスと通信を始める
  Wire.write(Register);//指定するレジスタを書き込む
  Wire.write(Data);//データを書き込む
  Wire.endTransmission();//通信を終了する
}
