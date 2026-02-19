/*
semakin ke barat Y- 55
semakin ke timur Y+ 37
semakin ke utara X- 20
semakin ke selatan X+ 20
A-->Selatan
B-->Utara
C-->Timur
D-->Barat
*/
#define MINBARAT -55
#define MAXTIMUR 37
#define MINUTARA -20
#define MAXSELATAN 20
#define DEBUG 1
#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
// #include "I2CKeyPad.h"
#define xyoffset = 1;

// Pin TA6586
const int M0_IN1 = 4;
const int M0_IN2 = 5;
const int M1_IN1 = 6;
const int M1_IN2 = 7;

// Tombol kontrol
const int BTN_M0_RIGHT = A0;
const int BTN_M0_LEFT = A1;
const int BTN_M1_RIGHT = A2;
const int BTN_M1_LEFT = A3;

byte eepromDay = 0;  // Menyimpan hari terakhir yang dimuat ke EEPROM

const uint8_t KEYPAD_ADDRESS = 0x21;

// I2CKeyPad keyPad(KEYPAD_ADDRESS);

int lastminute = 0;
int lastsecond = 0;
int lastday = 0;

float valX = 0;
float valY = 0;

// === Konfigurasi Perangkat ===
RTC_DS1307 rtc;

const int chipSelect = 10;  // pin CS modul SD Card


DateTime now;

unsigned long startMillisRTC;  //some global variables available anywhere in the program
unsigned long currentMillisRTC;

const int MPU_ADDR = 0x69;     // MPU-6050 I2C address
float angleX = 0, angleY = 0;  // Current angles from gyro
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
unsigned long lastTime = 0;
double dt = 0.01;  // Time step for integration


const int rs = 2, en = 3, d4 = A0, d5 = A1, d6 = A2, d7 = A3;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

void calibrateMPU6050() {
  return;
  const int samples = 1000;
  long gx = 0, gy = 0, gz = 0;

  // Serial.println(F("Calibrating MPU6050... Keep it still."));
  // for (int i = 0; i < samples; i++) {
  //   Wire.beginTransmission(MPU_ADDR);
  //   Wire.write(0x43);  // starting register for gyro
  //   Wire.endTransmission(false);
  //   Wire.requestFrom(MPU_ADDR, 6, true);
  //   int16_t gx_raw = Wire.read() << 8 | Wire.read();
  //   int16_t gy_raw = Wire.read() << 8 | Wire.read();
  //   int16_t gz_raw = Wire.read() << 8 | Wire.read();
  //   gx += gx_raw;
  //   gy += gy_raw;
  //   gz += gz_raw;
  //   delay(2);
  // }

  gyroXoffset = gx / (float)samples / 131.0;
  gyroYoffset = gy / (float)samples / 131.0;
  gyroZoffset = gz / (float)samples / 131.0;

  Serial.print(F("Gyro offsets: "));
  Serial.print(gyroXoffset);
  Serial.print(", ");
  Serial.print(gyroYoffset);
  Serial.print(", ");
  Serial.println(gyroZoffset);
}

bool initMPU6050() {
  return true;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU-6050
  byte error = Wire.endTransmission();

  if (error != 0) {
    return false;
  }

  // Configure gyroscope range (±250°/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission();

  // Configure accelerometer range (±2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g
  Wire.endTransmission();

  return true;
}

void readRTC() {
  // Inisialisasi RTC
  if (!rtc.begin()) {
    Serial.println(F("RTC ERR"));
    while (1)
      ;
  }
  now = rtc.now();
  Serial.println(F("RTC OK"));
  lastminute = now.minute();
  lastsecond = now.second();
  lastday = now.day();
  eepromDay = now.day();
  startMillisRTC = millis();  //initial start time
}
/*
void readSD() {
  // Inisialisasi SD Card
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD ERR"));
    while (1)
      ;
  }
  Serial.println(F("SD OK"));
  lastTime = millis();

  char filename[9];
  sprintf(filename, "%02d%02d.CSV", now.month(), now.day());
  // Baca file CSV harian
  File dataFile = SD.open(filename);
  if (dataFile) {
    String line;
    bool found = false;

    // Skip header (baris pertama)
    if (dataFile.available()) {
      dataFile.readStringUntil('\n');
      while (dataFile.available()) {
        line = dataFile.readStringUntil('\n');
        line.trim();

        int idx1 = line.indexOf(',');
        int idx2 = line.indexOf(',', idx1 + 1);
        int idx3 = line.indexOf(',', idx2 + 1);
        if (idx1 < 0 || idx2 < 0 || idx3 < 0) continue;

        String hourCSV = line.substring(idx1 + 1, idx2);

        if (hourCSV.toInt() == now.hour()) {
          Serial.println(line);
          String xStr = line.substring(idx2 + 1, idx3);
          String yStr = line.substring(idx3 + 1);
          // Serial.println(line);

          // Serial.println(xStr);
          // Serial.println(yStr);

          valX = xStr.toFloat();
          valY = yStr.toFloat();

          // Tampilkan ke Serial
          Serial.print("File: ");
          Serial.print(filename);
          Serial.print(" Jam: ");
          Serial.println(now.hour());
          Serial.print(" => X: ");
          Serial.print(valX, 2);
          Serial.print(" , Y: ");
          Serial.print(valY, 2);
          Serial.print(" => X: ");
          Serial.print(angleX, 2);
          Serial.print(" , Y: ");
          Serial.println(angleY, 2);

          found = true;
          break;
        }
      }
    }
  } else {
    Serial.print(F("!File "));
  }
}
*/
void initLCD() {
  lcd.begin(16, 2);
}

uint32_t lastKeyPressed = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  initLCD();

  readRTC();

  if (!SD.begin(chipSelect)) {
    //Serial.println("SD ERR");
    lcd.println(F("Card Error"));
    while (1)
      ;
  }
  
  loadCSVtoEEPROM();

  if (!lookupEEPROM(now.hour(), valX, valY)) { Serial.println(F("Hour not found in EEPROM!")); }
  //readSD();

  if (!initMPU6050()) {
    //Serial.println("MPU-6050 tidak terdeteksi!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU error!");
    while (1)
      ;
  }
  calibrateMPU6050();

  //Serial.println("MPU-6050 OK");
  lastTime = millis();

  // if (keyPad.begin() == false) {
  //   Serial.println("\nERROR: cannot communicate to keypad.\nPlease reboot.\n");
  //   while (1)
  //     ;
  // }

  // Motor pins
  pinMode(M0_IN1, OUTPUT);
  pinMode(M0_IN2, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);


  // Stop motor awal
  stopMotor0();
  stopMotor1();
}
#define OFFSETSUDUT 2
void loop() {
  // Update current angles from MPU-6050
  updateAngles();
  angleX=30; angleY=30;

#if DEBUG
  //Serial.print("X:");
  Serial.print(valX, 2);
  Serial.print(",");
  //Serial.print("\tY:");
  Serial.print(valY, 2);
  Serial.print(";");
  Serial.print(angleX, 2);
  Serial.print(",");
  //Serial.print("\tY:");
  Serial.println(angleY, 2);
#endif

  // if (angleY < -60 || angleY > 60) {
  //   Serial.println("outofangle");
  //   stopMotor0();
  //   //wait go to 00
  // }
  if (angleY <= valY - OFFSETSUDUT) {  //kurang timur
    if (angleY < MAXTIMUR) {           //masih bisa gerak
      motor0Left();
    } else {
      Serial.println("MTimur");
    }
  } else if (angleY >= valY + OFFSETSUDUT) {  //Kurang ke barat
    if (angleY > MINBARAT) {                  //masih bisa gerak
      motor0Right();
    } else {
      Serial.println("MBarat");
    }
  } else {
    stopMotor0();
  }

  // === Kontrol Sumbu X (Utara-Selatan) ===
  if (angleX <= valX - OFFSETSUDUT) {  // kurang ke Selatan
    if (angleX < MAXSELATAN) {         // masih bisa gerak ke Selatan
      motor1Right();                   // gerak ke Selatan
    } else {
      Serial.println("MSelatan");
    }
  } else if (angleX >= valX + OFFSETSUDUT) {  // kurang ke Utara
    if (angleX > MINUTARA) {                  // masih bisa gerak ke Utara
      motor1Left();                           // gerak ke Utara
    } else {
      Serial.println("MUtara");
    }
  } else {
    stopMotor1();  // Sudut sudah sesuai
  }



  // char keys[] = "DCBA#9639852*741";  //  N = NoKey, F = Fail
  // uint32_t nowkp = millis();
  // if (nowkp - lastKeyPressed >= 100) {
  //   lastKeyPressed = nowkp;

  //   // start = micros();
  //   uint8_t index = keyPad.getKey();
  //   // stop = micros();

  //   if (index < 16) {
  //     //3:A. 2:B. 1:C. 0:D.
  //     if (index == 3) {
  //       Serial.print("X+");
  //       motor1Left();
  //       //motor0Right();

  //       delay(500);
  //     }
  //     if (index == 2) {
  //       Serial.print("X-");
  //       motor1Right();
  //       //motor0Left();
  //       delay(500);
  //     }
  //     if (index == 1) {
  //       Serial.print("Y+");
  //       motor0Left();
  //       //motor1Right();
  //       delay(500);
  //     }
  //     if (index == 0) {
  //       Serial.print("Y-");
  //       motor0Right();
  //       //motor1Left();
  //       delay(500);
  //     }
  //     stopMotor0();
  //     stopMotor1();
  //     Serial.println();
  //   }
  //   // Serial.print("\t");
  //   // Serial.println(stop - start);
  // }


  currentMillisRTC = millis();                    //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillisRTC - startMillisRTC >= 1000)  //test whether the period has elapsed
  {
    if (lastday != now.day()) {
      Serial.println("Reset");
      //Serial.println(F("=== Hari berganti, perbarui EEPROM dari CSV ==="));
      lastday = now.day();    // Update pencatatan hari sekarang
      eepromDay = now.day();  // Simpan juga hari ke EEPROM tracking
      loadCSVtoEEPROM();
    }
    now = rtc.now();
    char dateStr[9];
    sprintf(dateStr, "%02d/%02d/%02d", now.day(), now.month(), now.year() % 100);
    char timeStr[9];  //+ \0
    sprintf(timeStr, "%02d.%02d.%02d", now.hour(), now.minute(), now.second());


    lcd.setCursor(0, 0);
    lcd.print(dateStr);
    lcd.setCursor(0, 1);
    lcd.print(timeStr);
    lcd.setCursor(9, 0);
    lcd.print("X:");
    lcd.print(valX, 1);
    lcd.setCursor(9, 1);
    lcd.print("Y:");
    lcd.print(valY, 1);

    if (lastminute != now.minute()) {
      lastminute = now.minute();
      //readSD();

      //hack
      //if (lookupEEPROM(18, valX, valY)) {

      if (lookupEEPROM(now.hour(), valX, valY)) {
        Serial.print("Hour=");
        Serial.print(now.hour());
        Serial.print(" X=");
        Serial.print(valX, 2);
        Serial.print(" Y=");
        Serial.println(valY, 2);
      } else {
        Serial.println(F("Hour not found in EEPROM!"));
        valX = valY = 0;
      }
    }
    //hack
    //valX = valY = 0;
  }
}


//EEPROM
// === EEPROM helper ===
void writeEEPROMRecord(int index, byte hour, float x, float y) {
  int addr = index * 9;
  EEPROM.update(addr, hour);
  EEPROM.put(addr + 1, x);
  EEPROM.put(addr + 5, y);
}

void readEEPROMRecord(int index, byte &hour, float &x, float &y) {
  int addr = index * 9;
  hour = EEPROM.read(addr);
  EEPROM.get(addr + 1, x);
  EEPROM.get(addr + 5, y);
}

// === Load CSV into EEPROM once at boot ===
void loadCSVtoEEPROM() {
  char filename[9];
  sprintf(filename, "%02d%02d.CSV", now.month(), now.day());

  File dataFile = SD.open(filename);
  if (!dataFile) {
    Serial.println(F("CSV not found!"));
    return;
  }

  // skip header
  dataFile.readStringUntil('\n');

  int idx = 0;
  while (dataFile.available() && idx < 24) {  // max 24 hours
    String line = dataFile.readStringUntil('\n');
    line.trim();

    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1);
    if (idx1 < 0 || idx2 < 0 || idx3 < 0) continue;

    byte hour = line.substring(idx1 + 1, idx2).toInt();
    float x = line.substring(idx2 + 1, idx3).toFloat();
    float y = line.substring(idx3 + 1).toFloat();

    writeEEPROMRecord(idx, hour, x, y);
    idx++;
  }

  dataFile.close();
  Serial.print(F("CSV loaded into EEPROM, rows="));
  Serial.println(idx);
}

// === Lookup by RTC hour ===
bool lookupEEPROM(byte hourTarget, float &x, float &y) {
  for (int i = 0; i < 24; i++) {
    byte h;
    float xx, yy;
    readEEPROMRecord(i, h, xx, yy);
    if (h == hourTarget) {
      x = xx;  //MPU6050 compensation here
      y = yy;
      return true;
    }
  }
  return false;  // not found
}

///MPU
void readMPU6050() { return;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  // Read accelerometer data
  int16_t accX_raw = Wire.read() << 8 | Wire.read();
  int16_t accY_raw = Wire.read() << 8 | Wire.read();
  int16_t accZ_raw = Wire.read() << 8 | Wire.read();

  // Skip temperature data
  Wire.read();
  Wire.read();

  // Read gyroscope data
  int16_t gyroX_raw = Wire.read() << 8 | Wire.read();
  int16_t gyroY_raw = Wire.read() << 8 | Wire.read();
  int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();

  // Convert to physical units
  accX = accX_raw / 16384.0;  // ±2g range
  accY = accY_raw / 16384.0;
  accZ = accZ_raw / 16384.0;

  // gyroX = gyroX_raw / 131.0;  // ±250°/s range
  // gyroY = gyroY_raw / 131.0;
  // gyroZ = gyroZ_raw / 131.0;

  gyroX = gyroX_raw / 131.0 - gyroXoffset;
  gyroY = gyroY_raw / 131.0 - gyroYoffset;
  gyroZ = gyroZ_raw / 131.0 - gyroZoffset;
}

void updateAngles() { return;
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  readMPU6050();
  //return;
  // Calculate angles from accelerometer (for reference/calibration)
  float accAngleX = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
  float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Integrate gyroscope data
  angleX += gyroX * dt;
  angleY += gyroY * dt;

  // Apply complementary filter (optional, for better accuracy)
  float alpha = 0.96;  //0.98;
  angleX = alpha * angleX + (1 - alpha) * accAngleX;
  angleY = alpha * angleY + (1 - alpha) * accAngleY;
  //Serial.println(angleX);
}
bool update = false;


// ===== Motor 0 control =====
void motor0Right() {
  digitalWrite(M0_IN1, HIGH);
  digitalWrite(M0_IN2, LOW);
  Serial.println("M0R");
}
void motor0Left() {
  digitalWrite(M0_IN1, LOW);
  digitalWrite(M0_IN2, HIGH);
  Serial.println("M0L");
}
void stopMotor0() {
  digitalWrite(M0_IN1, LOW);
  digitalWrite(M0_IN2, LOW);
}

// ===== Motor 1 control =====
void motor1Right() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  Serial.println("M1R");
}
void motor1Left() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  Serial.println("M1L");
}
void stopMotor1() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
}
