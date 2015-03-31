#include <Wire.h>

//Регистры L3G4200D
#define CTRL_REG1_GYRO 0x20
#define CTRL_REG2_GYRO 0x21
#define CTRL_REG3_GYRO 0x22
#define CTRL_REG4_GYRO 0x23

//Регистры LSM303
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M 0x02
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define LSM303_MAG  0x1E  // assuming SA0 grounded
#define LSM303_ACC  0x18  // assuming SA0 grounded

int AddrGyro = 105; // x69 в десятичном виде
int AddrLSMAccel = 24;//
int AddrLSMMag = 30;//
int gx, gy, gz, ax, ay, az, mx, my, mz;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  //инициализация L3g4200D
  writeI2C(AddrGyro, CTRL_REG1_GYRO, 0x1F);    // Включаем все оси, какой байт отсылать смотрим даташит к L3G4200d
  writeI2C(AddrGyro, CTRL_REG3_GYRO, 0x08);    // Режм чтения сигнала
  writeI2C(AddrGyro, CTRL_REG4_GYRO, 0x80);    // Выбираем диапозон измерений (500 град/сек)
  delay(100);
  //инициализация LSM303
 initLSM303(2);
 delay(100); 
}

void loop(){
  getGyroValues();
  getAccelValues();  // Получаем значения
  getMagValues();
  Serial.print(gx*0.00762);
  Serial.print(",");  
  Serial.print(gy*0.00762);
  Serial.print(",");
  Serial.print(gz*0.00762);
  Serial.print(",");
  Serial.print(ax);
  Serial.print(",");  
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(mx);
  Serial.print(",");  
  Serial.print(my);
  Serial.print(",");
  Serial.println(mz);
  delay(1);                   // Short delay between reads
}

void getGyroValues () {
  byte MSB, LSB;

  MSB = readI2C(AddrGyro,0x29); // ось X
  LSB = readI2C(AddrGyro,0x28);
  gx = (int16_t)((MSB << 8) | LSB);

  MSB = readI2C(AddrGyro,0x2B); // ось Y
  LSB = readI2C(AddrGyro,0x2A);
  gy = (int16_t)((MSB << 8) | LSB);

  MSB = readI2C(AddrGyro,0x2D);  // ось Z
  LSB = readI2C(AddrGyro,0x2C);
  gz = (int16_t)((MSB << 8) | LSB);
  //z = ((int16_t)((MSB << 8) | LSB))*(0.00762);
}

void getAccelValues () {
  byte MSB, LSB;

  MSB = readI2C(AddrLSMAccel,0x29); // ось X
  LSB = readI2C(AddrLSMAccel,0x28);
  ax = (int16_t)((MSB << 8) | LSB);

  MSB = readI2C(AddrLSMAccel,0x2B); // ось Y
  LSB = readI2C(AddrLSMAccel,0x2A);
  ay = (int16_t)((MSB << 8) | LSB);

  MSB = readI2C(AddrLSMAccel,0x2D);  // ось Z
  LSB = readI2C(AddrLSMAccel,0x2C);
  az = (int16_t)((MSB << 8) | LSB);
 
}

void getMagValues () {
  byte MSB, LSB;

  MSB = readI2C(AddrLSMMag,0x03); // ось X
  LSB = readI2C(AddrLSMMag,0x04);
  mx = (int16_t)((MSB << 8) | LSB);

  MSB = readI2C(AddrLSMMag,0x07); // ось Y
  LSB = readI2C(AddrLSMMag,0x08);
  my = (int16_t)((MSB << 8) | LSB);

  MSB = readI2C(AddrLSMMag,0x05);  // ось Z
  LSB = readI2C(AddrLSMMag,0x06);
  mz = (int16_t)((MSB << 8) | LSB);
 
}

//функция чтения с шины I2C
int readI2C (int device, byte regAddr) {
    Wire.beginTransmission(device);
    Wire.write(regAddr);                // Register address to read
    Wire.endTransmission();             // Terminate request
    Wire.requestFrom(device, 1);          // Read a byte
    while(!Wire.available()) { };       // Wait for receipt
    return(Wire.read());                // Get result
}
//функция записи в шину I2C
void writeI2C (byte device, byte regAddr, byte val) {
    Wire.beginTransmission(device);
    Wire.write(regAddr);
    Wire.write(val);
    Wire.endTransmission();
}

void initLSM303(int fs)
{
  LSM303_write(0x27, CTRL_REG1_A);  // 0x27 нормальный режим, работают все оси.
  if ((fs==8)||(fs==4))
    LSM303_write((0x00 | (fs-fs/2-1)<<4), CTRL_REG4_A);  // выставляем масштабный коэффициент
  else
  LSM303_write(0x00, CTRL_REG4_A);
  LSM303_write(0x14, CRA_REG_M);  // 0x14  частота выдачи данных для магнитометра 30 Hz
  LSM303_write(0x00, MR_REG_M);  // 0x00 норм. режим
}

void LSM303_write(byte data, byte address)
{
  if (address >= 0x20)
    Wire.beginTransmission(LSM303_ACC);
  else
    Wire.beginTransmission(LSM303_MAG);
    
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}
