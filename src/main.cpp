/*
 * DPSHIP_FUSION - MOT ESP32 doc ca hai cam bien cua tau mo hinh DP:
 *   1) GY-271 (la ban, I2C SDA=GPIO32 / SCL=GPIO33) -> huong mui tau that
 *   2) RPLIDAR A1M8 (Serial2: RX=GPIO16 <- TX lidar, TX=GPIO17 -> RX lidar,
 *      MOTOCTL=GPIO25 muc CAO = motor quay; lidar cap nguon 5V rieng, chung GND)
 * Mot ket noi Wi-Fi/MQTT-TLS duy nhat (task mang, core 0) phat ca hai luong:
 *   - MQTT_TOPIC trong config (bas/DP_SHIP_001/telemetry), 1 Hz:
 *     {"x":<tu lidar>,"y":<tu lidar>,"hdg":<huong that>,"rot":<suy tu huong>,"src":"esp32-fusion","pos":0/1,"cal":0/1,...}
 *   - bas/BAS_LIDAR_001/telemetry, khi co vong quet hoan chinh moi:
 *     {"sensor":"RPLIDAR-A1M8","status":"ok","unit":"mm","step_deg":1,...,"ranges":[360 so, 0 = khong co phan hoi]}
 * Doc lidar chay o task rieng (core 1, uu tien 2); loop Arduino (core 1) doc la ban.
 * USB Serial chi 115200: log trang thai [NET]/[LIDAR]/[POS] + lenh c/r/h va p/t/d (dinh vi).
 * Ghep tu DPSHIP_COMPASS (la ban + mang) va firmware lidar goc cua repo (parser RPLIDAR).
 */

/*
 * Doc cam bien la ban GY-271 (HMC5883L hoac QMC5883L) bang ESP32, khong can thu vien ngoai.
 *
 * Day noi:
 *   GY-271 VCC -> 3V3 ESP32
 *   GY-271 GND -> GND ESP32
 *   GY-271 SCL -> GPIO33
 *   GY-271 SDA -> GPIO32
 *   GY-271 DRDY -> de ho
 *
 * Chuong trinh tu nhan chip:
 *   HMC5883L: dia chi I2C 0x1E (thanh ghi ID = 'H','4','3')
 *   QMC5883L: dia chi I2C 0x0D (thanh ghi ID 0x0D = 0xFF)
 *
 * Lenh qua Serial Monitor (115200 baud, ket thuc dong bat ky):
 *   c : hieu chuan 20 giay - xoay cam bien du moi huong (ve so 8 trong khong gian)
 *   r : xoa hieu chuan
 *   h : in lai huong dan
 * Ket qua hieu chuan duoc luu vao bo nho flash, giu lai sau khi tat nguon.
 *
 * Luu y: huong (heading) chi dung khi cam bien nam NGANG, khong co bu nghieng.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#if __has_include("config.h")
#include "config.h"            // cau hinh rieng (Wi-Fi, MQTT) - khong dua len git
#else
#include "config.example.h"
#endif
#include "ca_cert.h"

#define NET_LOG(...) do { Serial.print("[NET] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#include <math.h>

// ---------------- Cau hinh ----------------
#define I2C_SDA_PIN 32
#define I2C_SCL_PIN 33
#define I2C_FREQ_HZ 100000       // 100 kHz: ben hon 400 kHz khi day dai/tiep xuc kem

// Do lech tu truong tai dia diem cua ban (do, dong la +). Tra cuu "magnetic declination" theo toa do.
// Viet Nam khoang -1 do. De 0 neu khong can chinh xac tuyet doi.
#define DECLINATION_DEG 0.0f
// Cong them vao huong neu gan cam bien lech truc so voi huong "phia truoc" cua ban (do).
#define HEADING_OFFSET_DEG 0.0f

#define PRINT_INTERVAL_MS 200
#define CALIBRATION_MS 20000

// ---------------- RPLIDAR A1M8 (Serial2) ----------------
#define LIDAR_RX_PIN     16      // ESP32 RX2  <- RPLIDAR TX
#define LIDAR_TX_PIN     17      // ESP32 TX2  -> RPLIDAR RX
#define LIDAR_MOTOR_PIN  26
#define LIDAR_BAUD       115200
#define LIDAR_TOPIC      "bas/BAS_LIDAR_001/telemetry"

// ---------------- Chip ----------------
enum ChipType { CHIP_NONE, CHIP_HMC5883L, CHIP_QMC5883L };
static ChipType chip = CHIP_NONE;
static uint8_t  chipAddr = 0;
static float    uTperLsb = 0;          // he so doi so tho sang microTesla

// HMC5883L
#define HMC_ADDR 0x1E
#define HMC_REG_CONFIG_A 0x00
#define HMC_REG_CONFIG_B 0x01
#define HMC_REG_MODE     0x02
#define HMC_REG_DATA     0x03          // X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
#define HMC_REG_STATUS   0x09
#define HMC_REG_ID_A     0x0A

// QMC5883L
#define QMC_ADDR 0x0D
#define QMC_REG_DATA     0x00          // X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
#define QMC_REG_STATUS   0x06
#define QMC_REG_CONTROL1 0x09
#define QMC_REG_CONTROL2 0x0A
#define QMC_REG_PERIOD   0x0B
#define QMC_REG_ID       0x0D

// ---------------- Khoi phuc bus I2C ----------------
// Khi day chap chon, cam bien co the giu SDA o muc thap va treo bus. Day 9 xung len SCL de no nha SDA,
// roi tao dieu kien STOP, sau do khoi dong lai Wire.
static void i2cRestart() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  }
  // STOP: SDA len cao khi SCL cao
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(10);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQ_HZ);
  Wire.setTimeOut(50);
}

// Chan doan day: module GY-271 co dien tro keo len 3V3 san, nen khi nguon + day tot thi SDA va SCL
// doc duoc muc CAO ngay ca khi ESP32 khong bat pull-up noi.
static void diagnoseLines() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  delay(5);
  int sdaHigh = 0, sclHigh = 0;
  for (int i = 0; i < 20; i++) {
    sdaHigh += digitalRead(I2C_SDA_PIN);
    sclHigh += digitalRead(I2C_SCL_PIN);
    delay(1);
  }
  Serial.printf("Chan doan (khong pull-up noi): SDA=%s SCL=%s\n", sdaHigh >= 18 ? "CAO" : "THAP", sclHigh >= 18 ? "CAO" : "THAP");
  if (sdaHigh < 18 || sclHigh < 18)
    Serial.println("  -> Day SDA/SCL (hoac 1 trong 2) khong toi module, hoac module khong co nguon 3V3/GND.");
  else
    Serial.println("  -> Nguon va day tin hieu co ve tot nhung chip khong tra loi: kiem tra chip co bi hong/gia khong.");
}

// ---------------- I2C co ban ----------------
static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool devicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void scanBus() {
  Serial.print("Quet I2C: ");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (devicePresent(a)) {
      Serial.printf("0x%02X ", a);
      found++;
    }
  }
  if (!found) Serial.print("(khong thay thiet bi nao)");
  Serial.println();
  if (!found) {
    // Ma loi Wire: 0 = OK, 2 = NACK dia chi (khong co thiet bi / khong tra loi), 3 = NACK du lieu,
    //              4 = loi khac, 5 = TIMEOUT (bus bi treo, thuong do SCL/SDA bi giu muc thap)
    const uint8_t addrs[2] = {HMC_ADDR, QMC_ADDR};
    for (uint8_t a : addrs) {
      Wire.beginTransmission(a);
      Serial.printf("  dia chi 0x%02X -> ma loi Wire %d\n", a, Wire.endTransmission());
    }
  }
}

// ---------------- Khoi tao chip ----------------
static bool initHMC() {
  uint8_t id[3];
  if (!readRegs(HMC_ADDR, HMC_REG_ID_A, id, 3)) return false;
  if (id[0] != 'H' || id[1] != '4' || id[2] != '3') return false;
  bool ok = writeReg(HMC_ADDR, HMC_REG_CONFIG_A, 0x70)   // trung binh 8 mau, 15 Hz
         && writeReg(HMC_ADDR, HMC_REG_CONFIG_B, 0x20)   // +-1.3 Ga: 1090 LSB/Gauss
         && writeReg(HMC_ADDR, HMC_REG_MODE, 0x00);      // do lien tuc
  if (!ok) return false;
  chip = CHIP_HMC5883L;
  chipAddr = HMC_ADDR;
  uTperLsb = 100.0f / 1090.0f;                           // 1 Gauss = 100 microTesla
  return true;
}

static bool initQMC() {
  uint8_t id;
  if (!readRegs(QMC_ADDR, QMC_REG_ID, &id, 1)) return false;
  if (id != 0xFF) return false;
  bool ok = writeReg(QMC_ADDR, QMC_REG_CONTROL2, 0x80)   // soft reset
         && (delay(10), writeReg(QMC_ADDR, QMC_REG_PERIOD, 0x01))
         && writeReg(QMC_ADDR, QMC_REG_CONTROL1, 0x1D);  // OSR512, +-8G, 200 Hz, lien tuc
  if (!ok) return false;
  chip = CHIP_QMC5883L;
  chipAddr = QMC_ADDR;
  uTperLsb = 100.0f / 3000.0f;                           // +-8G: 3000 LSB/Gauss
  return true;
}

static bool initSensor() {
  chip = CHIP_NONE;
  if (devicePresent(HMC_ADDR) && initHMC()) return true;
  if (devicePresent(QMC_ADDR) && initQMC()) return true;
  return false;
}

// ---------------- Doc du lieu ----------------
// Tra ve true neu co mau moi; x,y,z la so tho
static bool readRaw(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6], st;
  if (chip == CHIP_HMC5883L) {
    if (!readRegs(chipAddr, HMC_REG_STATUS, &st, 1) || !(st & 0x01)) return false;
    if (!readRegs(chipAddr, HMC_REG_DATA, b, 6)) return false;
    x = (int16_t)((b[0] << 8) | b[1]);
    z = (int16_t)((b[2] << 8) | b[3]);                   // HMC: thu tu X, Z, Y
    y = (int16_t)((b[4] << 8) | b[5]);
    if (x == -4096 || y == -4096 || z == -4096) return false;   // tran thang do
    return true;
  }
  if (chip == CHIP_QMC5883L) {
    if (!readRegs(chipAddr, QMC_REG_STATUS, &st, 1) || !(st & 0x01)) return false;
    if (!readRegs(chipAddr, QMC_REG_DATA, b, 6)) return false;
    x = (int16_t)((b[1] << 8) | b[0]);
    y = (int16_t)((b[3] << 8) | b[2]);
    z = (int16_t)((b[5] << 8) | b[4]);
    return true;
  }
  return false;
}

// ---------------- Hieu chuan (hard-iron + soft-iron don gian) ----------------
static Preferences prefs;
static float offs[3] = {0, 0, 0};       // microTesla
static float scale[3] = {1, 1, 1};
static bool  calibrated = false;

static bool calibrating = false;
static uint32_t calStart = 0;
static float calMin[3], calMax[3];

static void loadCalibration() {
  prefs.begin("gy271", true);
  calibrated = prefs.getBool("cal", false);
  if (calibrated) {
    for (int i = 0; i < 3; i++) {
      offs[i]  = prefs.getFloat(i == 0 ? "ox" : i == 1 ? "oy" : "oz", 0);
      scale[i] = prefs.getFloat(i == 0 ? "sx" : i == 1 ? "sy" : "sz", 1);
    }
  }
  prefs.end();
  Serial.println(calibrated ? "Da nap hieu chuan da luu." : "Chua hieu chuan (go 'c' de hieu chuan).");
}

static void saveCalibration() {
  prefs.begin("gy271", false);
  prefs.putBool("cal", calibrated);
  for (int i = 0; i < 3; i++) {
    prefs.putFloat(i == 0 ? "ox" : i == 1 ? "oy" : "oz", offs[i]);
    prefs.putFloat(i == 0 ? "sx" : i == 1 ? "sy" : "sz", scale[i]);
  }
  prefs.end();
}

static void startCalibration() {
  calibrating = true;
  calStart = millis();
  for (int i = 0; i < 3; i++) { calMin[i] = 1e9f; calMax[i] = -1e9f; }
  Serial.printf("HIEU CHUAN %d giay: xoay cam bien cham theo moi huong (ve so 8, lat nghieng), dung dung tu...\n",
                CALIBRATION_MS / 1000);
}

static void finishCalibration() {
  calibrating = false;
  float avgRadius = 0, radius[3];
  for (int i = 0; i < 3; i++) {
    radius[i] = (calMax[i] - calMin[i]) / 2.0f;
    avgRadius += radius[i];
  }
  avgRadius /= 3.0f;
  if (radius[0] < 1 || radius[1] < 1 || radius[2] < 1) {
    Serial.println("Hieu chuan that bai: du lieu khong doi - co dang xoay cam bien khong?");
    return;
  }
  for (int i = 0; i < 3; i++) {
    offs[i]  = (calMax[i] + calMin[i]) / 2.0f;
    scale[i] = avgRadius / radius[i];
  }
  calibrated = true;
  saveCalibration();
  Serial.printf("Hieu chuan xong va da luu. offset = (%.1f, %.1f, %.1f) uT, scale = (%.2f, %.2f, %.2f)\n",
                offs[0], offs[1], offs[2], scale[0], scale[1], scale[2]);
}

static void clearCalibration() {
  calibrated = false;
  for (int i = 0; i < 3; i++) { offs[i] = 0; scale[i] = 1; }
  saveCalibration();
  Serial.println("Da xoa hieu chuan.");
}

static void printHelp() {
  Serial.println("Lenh: c = hieu chuan la ban | r = xoa hieu chuan | p = hieu chuan khung lidar | t <rong_m> <sau_m> = kich thuoc be | d = xem dinh vi | h = huong dan");
}

// ---------------- Huong ----------------
static const char *compassName(float deg) {
  static const char *names[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return names[(int)((deg + 22.5f) / 45.0f) % 8];
}


// ---------------- RPLIDAR A1M8: giao thuc + phan tich vong quet ----------------
// Port tu firmware lidar goc (src/main.cpp cua repo), cac khac biet co chu dich:
//   - bo PRINT_RAW_POINTS: USB Serial giu 115200, khong in tung diem
//   - printRevolution chi in dong REV tom tat (bo dong min theo cung 10 do)
//   - payload ranges: o trong ghi 0 (ban goc ghi null); chi dung payload khi
//     co vong quet hoan chinh MOI va con tuoi, khong phat trang thai stale/no_data
#define LIDAR_LOG(...) do { Serial.print("[LIDAR] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)

// Lenh RPLIDAR
#define CMD_SYNC   0xA5
#define CMD_STOP   0x25
#define CMD_RESET  0x40
#define CMD_SCAN   0x20
#define CMD_INFO   0x50
#define CMD_HEALTH 0x52

#define SECTOR_DEG 10
#define NUM_SECTORS (360 / SECTOR_DEG)

HardwareSerial &lidar = Serial2;

// Payload lidar dung chung jsonBuf voi payload tau (chi net task dung nen khong tranh chap)
static char jsonBuf[4096];

static void sendCmd(uint8_t cmd) {
  lidar.write((uint8_t)CMD_SYNC);
  lidar.write(cmd);
}

static void clearRx() {
  while (lidar.available()) lidar.read();
}

static bool waitBytes(size_t n, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while ((size_t)lidar.available() < n) {
    if (millis() - t0 > timeoutMs) return false;
    delay(1);
  }
  return true;
}

// Doc response descriptor 7 byte: A5 5A | len(30bit)+mode(2bit) | type
static bool readDescriptor(uint32_t &len, uint8_t &type, uint32_t timeoutMs = 1000) {
  uint32_t t0 = millis();
  uint8_t d[7];
  uint8_t i = 0;
  while (i < 7) {
    if (millis() - t0 > timeoutMs) return false;
    if (!lidar.available()) { delay(1); continue; }
    uint8_t b = lidar.read();
    if (i == 0 && b != 0xA5) continue;
    if (i == 1 && b != 0x5A) { i = (b == 0xA5) ? 1 : 0; continue; }
    d[i++] = b;
  }
  len  = (uint32_t)d[2] | ((uint32_t)d[3] << 8) | ((uint32_t)d[4] << 16) | ((uint32_t)(d[5] & 0x3F) << 24);
  type = d[6];
  return true;
}

static void motorOn()  { digitalWrite(LIDAR_MOTOR_PIN, HIGH); }
static void motorOff() { digitalWrite(LIDAR_MOTOR_PIN, LOW); }

static bool printDeviceInfo() {
  clearRx();
  sendCmd(CMD_INFO);
  uint32_t len; uint8_t type;
  if (!readDescriptor(len, type) || len != 20 || !waitBytes(20, 500)) return false;
  uint8_t r[20];
  for (int i = 0; i < 20; i++) r[i] = lidar.read();
  LIDAR_LOG("Model: %u, Firmware: %u.%02u, Hardware: %u", r[0], r[2], r[1], r[3]);
  Serial.print("[LIDAR] Serial: ");
  for (int i = 4; i < 20; i++) Serial.printf("%02X", r[i]);
  Serial.println();
  return true;
}

// 0 = tot, 1 = canh bao, 2 = loi, -1 = khong phan hoi
static int getHealth() {
  clearRx();
  sendCmd(CMD_HEALTH);
  uint32_t len; uint8_t type;
  if (!readDescriptor(len, type) || len != 3 || !waitBytes(3, 500)) return -1;
  uint8_t status = lidar.read();
  uint8_t lo = lidar.read();
  uint8_t hi = lidar.read();
  LIDAR_LOG("Health status=%u error_code=0x%04X", status, (uint16_t)(lo | (hi << 8)));
  return status;
}

static bool startScan() {
  clearRx();
  sendCmd(CMD_SCAN);
  uint32_t len; uint8_t type;
  if (!readDescriptor(len, type) || len != 5 || type != 0x81) return false;
  return true;
}

// ---------------- Xu ly du lieu quet ----------------
static uint32_t revCount = 0;
static uint16_t pointsInRev = 0;
static float    nearestDist = 0, nearestAngle = 0;
static uint16_t sectorMin[NUM_SECTORS];
static uint16_t binMm[360];             // khoang cach MIN (mm) theo tung 1 do cua vong dang quet

// Vong quay hoan chinh gan nhat, chia se cho task mang qua mutex
static SemaphoreHandle_t snapMutex;
static uint16_t snapMm[360];
static uint32_t snapRev = 0, snapPoints = 0, snapTimeMs = 0;
static float    snapNearest = 0, snapNearestAngle = 0;
static bool     snapValid = false;

static void resetRevolution() {
  pointsInRev = 0;
  nearestDist = 0;
  nearestAngle = 0;
  for (int i = 0; i < NUM_SECTORS; i++) sectorMin[i] = 0;
  memset(binMm, 0, sizeof binMm);
}

static void printRevolution() {
  if (pointsInRev == 0) return;
  revCount++;
  if (snapMutex && xSemaphoreTake(snapMutex, 0) == pdTRUE) {   // khong bao gio cho: uu tien doc lidar
    memcpy(snapMm, binMm, sizeof snapMm);
    snapRev = revCount;
    snapPoints = pointsInRev;
    snapNearest = nearestDist;
    snapNearestAngle = nearestAngle;
    snapTimeMs = millis();
    snapValid = true;
    xSemaphoreGive(snapMutex);
  }
  LIDAR_LOG("REV %lu | points=%u | nearest=%.0f mm @ %.1f deg",
            (unsigned long)revCount, pointsInRev, nearestDist, nearestAngle);
}

static void handlePoint(bool startFlag, uint8_t quality, float angle, float dist) {
  (void)quality;              // chi dung de in tung diem o ban goc (PRINT_RAW_POINTS), ban nay khong in
  if (startFlag) {            // bat dau vong quay moi
    printRevolution();
    resetRevolution();
  }
  if (dist <= 0) return;      // diem khong hop le (khong do duoc)

  pointsInRev++;
  if (nearestDist == 0 || dist < nearestDist) {
    nearestDist = dist;
    nearestAngle = angle;
  }
  int s = (int)(angle / SECTOR_DEG) % NUM_SECTORS;
  uint16_t d = (uint16_t)dist;
  if (sectorMin[s] == 0 || d < sectorMin[s]) sectorMin[s] = d;
  int b = (int)angle % 360;
  if (binMm[b] == 0 || d < binMm[b]) binMm[b] = d;
}

// Moi diem 5 byte:
//  b0: quality[7:2] | !S[1] | S[0]
//  b1: angle_q6[6:0] | C[0] (C luon = 1)
//  b2: angle_q6[14:7]
//  b3,b4: distance_q2 (little endian)
static void parseScanStream() {
  static uint8_t buf[5];
  static uint8_t idx = 0;

  while (lidar.available()) {
    uint8_t b = lidar.read();

    if (idx == 0) {
      if ((b & 0x01) == ((b >> 1) & 0x01)) continue;   // S va !S phai khac nhau
    } else if (idx == 1) {
      if (!(b & 0x01)) {                               // bit C phai = 1 -> lech byte
        idx = 0;
        if ((b & 0x01) != ((b >> 1) & 0x01)) buf[idx++] = b;
        continue;
      }
    }

    buf[idx++] = b;
    if (idx < 5) continue;
    idx = 0;

    bool    startFlag = buf[0] & 0x01;
    uint8_t quality   = buf[0] >> 2;
    float   angle     = (((uint16_t)buf[2] << 8) | buf[1]) >> 1;
    angle            /= 64.0f;
    float   dist      = (((uint16_t)buf[4] << 8) | buf[3]) / 4.0f;

    handlePoint(startFlag, quality, angle, dist);
  }
}

// Dung payload ranges chi khi co vong quet hoan chinh MOI (rev khac lastRev)
// va con tuoi (<= 3 s). Tra ve do dai JSON, 0 = chu ky nay khong phat.
// Ghi vao jsonBuf dung chung (chi net task goi ham nay).
static size_t buildLidarJson(uint32_t lastRev, uint32_t &outRev) {
  uint32_t points = 0, age = 0;
  float nearest = 0, nearestAngle = 0;
  bool fresh = false;
  static uint16_t mm[360];
  if (snapMutex && xSemaphoreTake(snapMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (snapValid && snapRev != lastRev) {
      age = millis() - snapTimeMs;
      if (age <= 3000) {
        fresh = true;
        memcpy(mm, snapMm, sizeof mm);
        outRev = snapRev; points = snapPoints; nearest = snapNearest; nearestAngle = snapNearestAngle;
      }
    }
    xSemaphoreGive(snapMutex);
  }
  if (!fresh) return 0;
  int n = snprintf(jsonBuf, sizeof jsonBuf,
    "{\"sensor\":\"RPLIDAR-A1M8\",\"status\":\"ok\",\"unit\":\"mm\",\"step_deg\":1,"
    "\"rev\":%lu,\"points\":%lu,\"nearest_mm\":%.0f,\"nearest_deg\":%.1f,"
    "\"age_ms\":%lu,\"rssi\":%d,\"uptime_s\":%lu,\"ranges\":[",
    (unsigned long)outRev, (unsigned long)points, nearest, nearestAngle,
    (unsigned long)age, (int)WiFi.RSSI(), (unsigned long)(millis() / 1000));
  if (n < 0 || (size_t)n >= sizeof jsonBuf) return 0;
  size_t len = (size_t)n;
  for (int i = 0; i < 360; i++) {
    if (sizeof jsonBuf - len < 8) return 0;
    len += snprintf(jsonBuf + len, sizeof jsonBuf - len, "%u%s", (unsigned)mm[i], i < 359 ? "," : "");
  }
  if (sizeof jsonBuf - len < 3) return 0;
  jsonBuf[len++] = ']';
  jsonBuf[len++] = '}';
  jsonBuf[len] = 0;
  return len;
}

// Task doc lidar: core 1, uu tien 2 (cao hon loop Arduino) - hut byte lien tuc
// khoi Serial2 de khong tran buffer ke ca khi loop chinh ban doc I2C / in log.
// Kem watchdog cua ban goc: mat du lieu qua 2 s thi khoi dong lai lenh quet.
static void lidarTask(void *) {
  uint32_t lastData = millis();
  uint32_t lastCount = 0;
  for (;;) {
    parseScanStream();
    if (pointsInRev != lastCount) { lastCount = pointsInRev; lastData = millis(); }
    if (millis() - lastData > 2000) {
      Serial.println("[LIDAR] Mat du lieu, khoi dong lai quet...");
      sendCmd(CMD_STOP);
      delay(10);
      startScan();
      lastData = millis();
    }
    delay(1);
  }
}

static bool initLidar() {
  // RESET de dua lidar ve trang thai sach (dac biet khi ESP32 reset ma lidar van dang quet)
  sendCmd(CMD_RESET);
  delay(2000);
  clearRx();

  if (!printDeviceInfo()) {
    Serial.println("[LIDAR] Khong doc duoc INFO - kiem tra day RX/TX va nguon 5V");
    return false;
  }
  int h = getHealth();
  if (h < 0) { Serial.println("[LIDAR] Khong doc duoc HEALTH"); return false; }
  if (h == 2) { Serial.println("[LIDAR] Lidar bao LOI (protection stop) - thu cap nguon lai"); return false; }
  return true;
}

// ---------------- Wi-Fi + MQTT (task rieng tren core 0) ----------------
// Phat huong la ban THAT len web: cung topic/dinh dang voi DPSHIP_SIM.
// x/y tinh tu lidar boi bo dinh vi [POS] ben duoi, hdg doc tu GY-271, rot suy tu dao ham cua huong.
static volatile float g_hdg = 0;
static volatile float g_mag = 0;
static volatile bool  g_haveSample = false;
static volatile bool  g_calibrated = false;
// jsonBuf[4096] khai bao o phan RPLIDAR phia tren, dung chung cho ca 2 payload

// ---------------- Dinh vi tu lidar [POS] ----------------
// Y tuong: chieu cac diem quet len 4 huong chinh cua KHUNG (khung = he toa do
// co truc Bac theo la ban). Khoang cach toi "vach" moi huong = phan vi 85 cua
// cac hinh chieu diem cuoi trong cung +-40 do quanh huong do (chong nhieu vat
// can gan: vat can chi lam giam khoang cach cua vai tia, phan vi cao giu lai
// cac tia cham vach that). Lidar gan lech so voi la ban mot goc beta va co the
// nguoc chieu quet (sign) - hai hang so nay tim bang lenh 'p' (hieu chuan
// khung: quet moi (sign,beta) tren 1 vong quet, chon bo lam tong 2 cap vach
// doi dien nho nhat - khung lech thi tia cheo dai ra, tong phinh len).
// Cau hinh luu trong Preferences "fusion": fsign, fbeta, tw, td.
static volatile int   posSign = 1;
static volatile float posBeta = 0, tankCfgW = 0, tankCfgD = 0;
static volatile bool  posCalibReq = false, posTankReq = false;
static volatile float pendTankW = 0, pendTankD = 0;
// Trang thai dinh vi moi nhat (net task ghi, lenh 'd' doc)
static volatile float g_posE = 0, g_posW = 0, g_posN = 0, g_posS = 0;
static volatile float g_posX = 0, g_posY = 0;
static volatile bool  g_posOk = false;
static float posSmoothX = 0, posSmoothY = 0;
static bool  posHave = false;

static void loadFusionConfig() {
  prefs.begin("fusion", true);
  posSign  = prefs.getInt("fsign", 1) < 0 ? -1 : 1;
  posBeta  = prefs.getFloat("fbeta", 0);
  tankCfgW = prefs.getFloat("tw", 0);
  tankCfgD = prefs.getFloat("td", 0);
  prefs.end();
  Serial.printf("[POS] Cau hinh khung: sign=%+d beta=%.1f | be: %.2f x %.2f m%s\n",
                (int)posSign, posBeta, tankCfgW, tankCfgD,
                tankCfgW > 0 ? "" : " (chua dat kich thuoc be)");
}

static void storeTankConfig() {
  tankCfgW = pendTankW;
  tankCfgD = pendTankD;
  prefs.begin("fusion", false);
  prefs.putFloat("tw", tankCfgW);
  prefs.putFloat("td", tankCfgD);
  prefs.end();
  Serial.printf("[POS] Da luu kich thuoc be: %.2f x %.2f m\n", tankCfgW, tankCfgD);
}

// Phan vi 85 co noi suy (v da sap xep tang dan tai cho, n nho)
static float pct85(float *v, int n) {
  for (int i = 1; i < n; i++) {
    float k = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  float idx = 0.85f * (float)(n - 1);
  int lo = (int)idx;
  float fr = idx - (float)lo;
  return (lo + 1 < n) ? v[lo] + fr * (v[lo + 1] - v[lo]) : v[lo];
}

// Uoc luong khoang cach 4 vach (mm) tu 1 vong quet. Tra ve false neu co
// huong nao qua it tia (< 10). Goc bin i -> theta = hdg + beta + sign*i
// (do, chieu kim dong ho tu Bac - cung chieu la ban).
static bool computeWalls(const uint16_t *mm, float hdgDeg, float betaDeg, int sign,
                         float &outE, float &outW, float &outN, float &outS) {
  static float secE[100], secW[100], secN[100], secS[100];
  int cE = 0, cW = 0, cN = 0, cS = 0;
  for (int i = 0; i < 360; i++) {
    uint16_t r = mm[i];
    if (!r) continue;
    float th = hdgDeg + betaDeg + (float)(sign * i);
    while (th >= 360.0f) th -= 360.0f;
    while (th < 0.0f) th += 360.0f;
    float rad = th * (float)M_PI / 180.0f;
    float ex = r * sinf(rad), ny = r * cosf(rad);
    if (th >= 50.0f && th <= 130.0f)       { if (cE < 100) secE[cE++] = ex; }
    else if (th >= 230.0f && th <= 310.0f) { if (cW < 100) secW[cW++] = -ex; }
    else if (th >= 320.0f || th <= 40.0f)  { if (cN < 100) secN[cN++] = ny; }
    else if (th >= 140.0f && th <= 220.0f) { if (cS < 100) secS[cS++] = -ny; }
  }
  if (cE < 10 || cW < 10 || cN < 10 || cS < 10) return false;
  outE = pct85(secE, cE);
  outW = pct85(secW, cW);
  outN = pct85(secN, cN);
  outS = pct85(secS, cS);
  return true;
}

static bool grabScan(uint16_t *dst) {
  if (!snapMutex) return false;
  bool ok = false;
  if (xSemaphoreTake(snapMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (snapValid) { memcpy(dst, snapMm, 360 * sizeof(uint16_t)); ok = true; }
    xSemaphoreGive(snapMutex);
  }
  return ok;
}

// Kiem tra 1 cap kich thuoc khung uoc luong (m) co hop ly khong:
// co cau hinh be thi phai trong [0.6, 1.35] lan kich thuoc that,
// chua cau hinh thi dung dai mac dinh [0.5, 9] m.
static bool sizeOk(float estM, float cfgM) {
  if (cfgM > 0) return estM >= 0.6f * cfgM && estM <= 1.35f * cfgM;
  return estM >= 0.5f && estM <= 9.0f;
}

// Tinh vi tri cho tick phat nay; tra ve true neu SO DO TICK NAY hop le.
// Ket qua phat = posSmooth (lam muot 0.5/tick tren cac tick hop le,
// tick hong thi giu nguyen gia tri muot cuoi).
static bool updatePosition(float hdg) {
  static uint16_t scan[360];
  static bool prevOk = false;
  static uint32_t tickNo = 0;
  tickNo++;
  bool ok = false;
  float E = 0, W = 0, N = 0, S = 0;
  if (grabScan(scan) && computeWalls(scan, hdg, posBeta, (int)posSign, E, W, N, S)) {
    float wEst = (E + W) / 1000.0f, dEst = (N + S) / 1000.0f;
    if (sizeOk(wEst, tankCfgW) && sizeOk(dEst, tankCfgD)) {
      ok = true;
      float x = (E - W) / 2000.0f, y = (N - S) / 2000.0f;
      if (!posHave) { posSmoothX = x; posSmoothY = y; posHave = true; }
      else {
        posSmoothX += (x - posSmoothX) * 0.5f;
        posSmoothY += (y - posSmoothY) * 0.5f;
      }
      g_posE = E / 1000.0f; g_posW = W / 1000.0f;
      g_posN = N / 1000.0f; g_posS = S / 1000.0f;
    }
  }
  g_posOk = ok;
  g_posX = posSmoothX;
  g_posY = posSmoothY;
  if (ok != prevOk || tickNo % 10 == 0) {
    Serial.printf("[POS] x=%+.2f y=%+.2f pos_ok=%d | E=%.2f W=%.2f N=%.2f S=%.2f m\n",
                  posSmoothX, posSmoothY, ok ? 1 : 0, g_posE, g_posW, g_posN, g_posS);
  }
  prevOk = ok;
  return ok;
}

// Lenh 'p': quet (sign, beta) tren MOT vong quet da chup, muc tieu =
// tong 2 cap vach doi dien nho nhat; tinh chinh beta +-1 do buoc 0.2.
// Chay trong net task (khoang vai tram ms), tau can dung yen.
static void runFrameCalibration() {
  static uint16_t scan[360];
  if (!g_haveSample) { Serial.println("[POS] Hieu chuan khung THAT BAI: chua co huong la ban"); return; }
  if (!grabScan(scan)) { Serial.println("[POS] Hieu chuan khung THAT BAI: chua co vong quet lidar"); return; }
  float hdg = g_hdg;
  Serial.println("[POS] Dang hieu chuan khung (giu yen tau)...");
  float E = 0, W = 0, N = 0, S = 0;
  float bestObj = 1e18f, bE = 0, bW = 0, bN = 0, bS = 0, bestBeta = 0;
  int bestSign = 0;
  for (int si = 0; si < 2; si++) {
    int sign = si == 0 ? 1 : -1;
    for (int b = 0; b < 360; b++) {
      if (!computeWalls(scan, hdg, (float)b, sign, E, W, N, S)) continue;
      if (!sizeOk((E + W) / 1000.0f, tankCfgW) || !sizeOk((N + S) / 1000.0f, tankCfgD)) continue;
      float obj = (E + W) + (N + S);
      if (obj < bestObj) { bestObj = obj; bestSign = sign; bestBeta = (float)b; bE = E; bW = W; bN = N; bS = S; }
    }
  }
  if (bestSign != 0) {
    for (float bb = bestBeta - 1.0f; bb <= bestBeta + 1.001f; bb += 0.2f) {
      float b2 = bb;
      while (b2 >= 360.0f) b2 -= 360.0f;
      while (b2 < 0.0f) b2 += 360.0f;
      if (!computeWalls(scan, hdg, b2, bestSign, E, W, N, S)) continue;
      float obj = (E + W) + (N + S);
      if (obj < bestObj) { bestObj = obj; bestBeta = b2; bE = E; bW = W; bN = N; bS = S; }
    }
  }
  if (bestSign == 0) {
    Serial.println("[POS] Hieu chuan khung THAT BAI: khong ung vien nao hop le, giu gia tri cu");
    return;
  }
  posSign = bestSign;
  posBeta = bestBeta;
  prefs.begin("fusion", false);
  prefs.putInt("fsign", (int)posSign);
  prefs.putFloat("fbeta", posBeta);
  prefs.end();
  Serial.printf("[POS] sign=%+d beta=%.1f | be uoc luong %.2f x %.2f m | x=%+.2f y=%+.2f m (da luu)\n",
                (int)posSign, posBeta, (bE + bW) / 1000.0f, (bN + bS) / 1000.0f,
                (bE - bW) / 2000.0f, (bN - bS) / 2000.0f);
}

// ---- Wi-Fi: mang chinh + mang du phong (hotspot), tu thu lai lien tuc ----
// Thu tu: mang dung duoc gan nhat truoc, roi toi mang con lai. Moi lan thu cho
// toi da ~12 s; het cac mang ma chua vao duoc thi nghi 3 s roi lap lai tu dau.
// Khong ESP.restart(): board phai song de tu vao hotspot khi hotspot xuat hien.
struct WifiNetDef { const char *ssid; const char *pass; };
static const WifiNetDef WIFI_NETS[] = {
  { WIFI_SSID, WIFI_PASS },
#ifdef WIFI_SSID2
  { WIFI_SSID2, WIFI_PASS2 },
#endif
};
static const int WIFI_NET_COUNT = (int)(sizeof(WIFI_NETS) / sizeof(WIFI_NETS[0]));
static int wifiLastGood = 0;   // index mang dung duoc gan nhat (chi luu trong RAM)

static bool wifiTryOne(const WifiNetDef &net) {
  NET_LOG("Thu WiFi: %s", net.ssid);
  WiFi.disconnect();
  WiFi.begin(net.ssid, net.pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(500);
  if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(); return false; }
  NET_LOG("WiFi OK: %s | IP: %s", net.ssid, WiFi.localIP().toString().c_str());
  return true;
}

static bool wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);   // vong lap duoi tu quan ly viec thu lai
  for (;;) {
    for (int k = 0; k < WIFI_NET_COUNT; k++) {
      int idx = (wifiLastGood + k) % WIFI_NET_COUNT;
      if (wifiTryOne(WIFI_NETS[idx])) { wifiLastGood = idx; return true; }
    }
    NET_LOG("Chua vao duoc mang Wi-Fi nao, thu lai sau 3 s...");
    delay(3000);
  }
}

static bool timeSync() {
  if (time(nullptr) > 1735689600) return true;
  NET_LOG("Dong bo gio NTP...");
  configTime(TZ_OFFSET_S, 0, "pool.ntp.org", "time.google.com");
  uint32_t t0 = millis();
  while (time(nullptr) < 1735689600 && millis() - t0 < 20000) delay(250);
  if (time(nullptr) < 1735689600) { NET_LOG("Khong dong bo duoc gio"); return false; }
  NET_LOG("Gio OK");
  return true;
}

static void netTask(void *) {
  WiFiClientSecure tls;
  PubSubClient mqtt(tls);
  uint32_t lastPublish = 0, publishCount = 0;
  uint32_t lastLidarRev = 0;
  float prevHdg = 0, prevT = 0, rotSmooth = 0;
  bool havePrev = false;
  for (;;) {
    if (!wifiConnect() || !timeSync()) { delay(5000); continue; }
    if (!mqtt.connected()) {
      tls.stop();
      tls.setCACert(CA_CERT_PEM);
      tls.setHandshakeTimeout(30);
      mqtt.setServer(MQTT_HOST, MQTT_PORT);
      mqtt.setSocketTimeout(20);
      mqtt.setKeepAlive(60);
      mqtt.setBufferSize(4096);   // payload ranges lidar ~2.4 KB
      NET_LOG("MQTT TLS toi %s:%d (client %s)", MQTT_HOST, MQTT_PORT, MQTT_CLIENT_ID);
      bool ok = mqtt.connect(MQTT_CLIENT_ID, strlen(MQTT_USER) ? MQTT_USER : nullptr,
                             strlen(MQTT_PASS) ? MQTT_PASS : nullptr);
      if (!ok) {
        char err[128] = "";
        int code = tls.lastError(err, sizeof err);
        NET_LOG("MQTT that bai: state %d, TLS %d %s. Thu lai sau 5 s", mqtt.state(), code, err);
        delay(5000);
        continue;
      }
      NET_LOG("MQTT da ket noi, topic %s", MQTT_TOPIC);
      lastPublish = millis() - PUBLISH_INTERVAL_MS;
      havePrev = false;
    }
    mqtt.loop();
    if (millis() - lastPublish >= PUBLISH_INTERVAL_MS) {
      lastPublish = millis();
      if (posTankReq) { posTankReq = false; storeTankConfig(); }
      if (posCalibReq) { posCalibReq = false; runFrameCalibration(); }
      if (!g_haveSample) { NET_LOG("Chua co du lieu cam bien, chua phat"); }
      else {
        float t = millis() / 1000.0f;
        float hdg = g_hdg;
        if (havePrev && t > prevT) {
          float dh = hdg - prevHdg;
          while (dh > 180.0f) dh -= 360.0f;
          while (dh < -180.0f) dh += 360.0f;
          float rotInst = dh / (t - prevT) * 60.0f;
          if (rotInst > 45.0f) rotInst = 45.0f;
          if (rotInst < -45.0f) rotInst = -45.0f;
          rotSmooth += (rotInst - rotSmooth) * 0.4f;
        }
        prevHdg = hdg; prevT = t; havePrev = true;
        bool posOk = updatePosition(hdg);
        int n = snprintf(jsonBuf, sizeof jsonBuf,
          "{\"x\":%.2f,\"y\":%.2f,\"hdg\":%.1f,\"rot\":%.1f,\"src\":\"esp32-fusion\",\"pos\":%d,\"cal\":%d,\"uptime_s\":%lu}",
          posSmoothX, posSmoothY, hdg, rotSmooth, posOk ? 1 : 0, g_calibrated ? 1 : 0, (unsigned long)(millis() / 1000));
        if (n < 0 || (size_t)n >= sizeof jsonBuf) { NET_LOG("Payload khong vua buffer, bo qua"); }
        else if (mqtt.publish(MQTT_TOPIC, (const uint8_t *)jsonBuf, (size_t)n, false)) {
          publishCount++;
          NET_LOG("#%lu hdg=%.1f rot=%+.1f |B|=%.1f cal=%d -> OK",
                  (unsigned long)publishCount, hdg, rotSmooth, g_mag, g_calibrated ? 1 : 0);
        } else {
          NET_LOG("Publish that bai, state %d: ket noi lai", mqtt.state());
          mqtt.disconnect();
        }
      }
      // --- RPLIDAR: chi phat khi co vong quet HOAN CHINH moi, khong phat mot phan ---
      {
        uint32_t newRev = 0;
        size_t llen = buildLidarJson(lastLidarRev, newRev);
        if (llen) {
          if (mqtt.publish(LIDAR_TOPIC, (const uint8_t *)jsonBuf, llen, false)) {
            lastLidarRev = newRev;
            NET_LOG("[LIDAR] Publish rev %lu OK (%u byte) -> %s",
                    (unsigned long)newRev, (unsigned)llen, LIDAR_TOPIC);
          } else {
            NET_LOG("[LIDAR] Publish that bai, state %d: ket noi lai", mqtt.state());
            mqtt.disconnect();
          }
        }
      }
    }
    delay(20);
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== DPSHIP FUSION: GY-271 + RPLIDAR A1M8 ===");
  i2cRestart();

  while (!initSensor()) {
    Serial.println("Khong tim thay HMC5883L (0x1E) hay QMC5883L (0x0D). Kiem tra day SDA=GPIO32, SCL=GPIO33, VCC=3V3, GND.");
    scanBus();
    diagnoseLines();
    // Thu dao chan: neu thay thiet bi thi SDA va SCL dang bi cam nguoc
    Wire.end();
    Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    if (devicePresent(HMC_ADDR) || devicePresent(QMC_ADDR))
      Serial.println("!!! THAY cam bien khi DAO chan: day SDA va SCL dang bi cam NGUOC (SDA phai vao GPIO32, SCL vao GPIO33).");
    delay(2000);
    i2cRestart();            // thu giai phong bus neu bi treo
  }
  Serial.printf("Phat hien %s tai 0x%02X\n", chip == CHIP_HMC5883L ? "HMC5883L" : "QMC5883L", chipAddr);
  loadCalibration();
  printHelp();
  loadFusionConfig();

  // --- RPLIDAR A1M8 ---
  snapMutex = xSemaphoreCreateMutex();
  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  motorOn();                                   // MOTOCTL = HIGH ngay tu dau: motor quay
  lidar.setRxBufferSize(4096);
  lidar.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(1000);                                 // cho motor dat toc do
  while (!initLidar()) {
    Serial.println("[LIDAR] Khoi tao that bai, thu lai sau 1 s...");
    delay(1000);
  }
  resetRevolution();
  while (!startScan()) {
    Serial.println("[LIDAR] Start scan that bai, thu lai...");
    delay(1000);
  }
  Serial.println("[LIDAR] Dang quet...");

  // Mang o core 0; doc lidar la task rieng tren core 1 (cung core voi loop la ban)
  xTaskCreatePinnedToCore(netTask, "net", 16384, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(lidarTask, "lidar", 8192, nullptr, 2, nullptr, 1);
}

void loop() {
  static uint32_t lastPrint = 0, lastGood = millis();
  static float lastX = 0, lastY = 0, lastZ = 0;
  static bool haveSample = false;

  // Lenh tu Serial
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') startCalibration();
    else if (ch == 'r' || ch == 'R') clearCalibration();
    else if (ch == 'h' || ch == 'H') printHelp();
    else if (ch == 'p' || ch == 'P') {
      posCalibReq = true;
      Serial.println("[POS] Da nhan lenh hieu chuan khung, task mang se chay trong tick phat ke tiep...");
    }
    else if (ch == 'd' || ch == 'D') {
      Serial.printf("[POS] sign=%+d beta=%.1f | be %.2f x %.2f m | E=%.2f W=%.2f N=%.2f S=%.2f m | x=%+.2f y=%+.2f pos_ok=%d\n",
                    (int)posSign, posBeta, tankCfgW, tankCfgD,
                    g_posE, g_posW, g_posN, g_posS, g_posX, g_posY, g_posOk ? 1 : 0);
    }
    else if (ch == 't' || ch == 'T') {
      float w = Serial.parseFloat();
      float d = Serial.parseFloat();
      if (w > 0.2f && w < 30.0f && d > 0.2f && d < 30.0f) {
        pendTankW = w; pendTankD = d; posTankReq = true;
        Serial.printf("[POS] Da nhan kich thuoc be %.2f x %.2f m, task mang se luu...\n", w, d);
      } else {
        Serial.println("[POS] Kich thuoc be khong hop le (can 2 so, vi du: t 3.60 2.40)");
      }
    }
  }

  int16_t rx, ry, rz;
  if (readRaw(rx, ry, rz)) {
    lastGood = millis();
    float x = rx * uTperLsb, y = ry * uTperLsb, z = rz * uTperLsb;   // uT tho
    if (calibrating) {
      float v[3] = {x, y, z};
      for (int i = 0; i < 3; i++) {
        if (v[i] < calMin[i]) calMin[i] = v[i];
        if (v[i] > calMax[i]) calMax[i] = v[i];
      }
    }
    lastX = (x - offs[0]) * scale[0];
    lastY = (y - offs[1]) * scale[1];
    lastZ = (z - offs[2]) * scale[2];
    haveSample = true;
  }

  if (calibrating && millis() - calStart >= CALIBRATION_MS) finishCalibration();

  // Mat cam bien (rut day, nhieu...) -> thu khoi tao lai
  if (millis() - lastGood > 2000) {
    Serial.println("Mat du lieu tu cam bien, thu khoi phuc bus I2C va khoi tao lai...");
    i2cRestart();
    scanBus();
    if (initSensor()) Serial.println("Da ket noi lai cam bien.");
    lastGood = millis();
  }

  if (haveSample && millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    float heading = atan2f(lastY, lastX) * 180.0f / (float)M_PI + DECLINATION_DEG + HEADING_OFFSET_DEG;
    while (heading < 0) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    float magnitude = sqrtf(lastX * lastX + lastY * lastY + lastZ * lastZ);
    g_hdg = heading; g_mag = magnitude; g_calibrated = calibrated; g_haveSample = true;
    if (calibrating) {
      Serial.printf("[HIEU CHUAN %2lu s] X=%7.1f Y=%7.1f Z=%7.1f uT\n",
                    (unsigned long)((CALIBRATION_MS - (millis() - calStart)) / 1000), lastX, lastY, lastZ);
    } else {
      Serial.printf("X=%7.1f Y=%7.1f Z=%7.1f uT | |B|=%5.1f uT | Huong=%6.1f deg (%s)%s\n",
                    lastX, lastY, lastZ, magnitude, heading, compassName(heading),
                    calibrated ? "" : "  [chua hieu chuan]");
    }
  }
  delay(2);
}
