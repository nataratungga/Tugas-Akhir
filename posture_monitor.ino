// ============================================================
//  SISTEM PEMANTAUAN POSTUR DUDUK - DATA UJI (KNN)
//  Sensor: 2x ADXL345 (punggung atas & bawah) + VL53L0X
//  Label: 1=Ergonomis, 2=Kurang Ergonomis, 3=Tidak Ergonomis
//  Flow : Sensor → KNN → POST Firebase langsung (tiap 10 detik)
// ============================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#define BUZZER_PIN 18

// ================= VARIABEL BUZZER - TIDAK ERGONOMIS =================
unsigned long tidakErgonomisSejak = 0;
bool          sedangTidakErgonomis = false;

// ================= VARIABEL BUZZER - POSISI STATIS =================
unsigned long statisSejak       = 0;
bool          sedangStatis      = false;
bool          statisAlertAktif  = false;

// Nilai sensor window sebelumnya (untuk perbandingan statis)
int   prevR1   = -9999;
int   prevP1   = -9999;
int   prevR2   = -9999;
int   prevP2   = -9999;
float prevVL   = -9999;
bool  prevValid = false; // apakah sudah ada data window sebelumnya

// Threshold statis
#define THRESHOLD_ANGLE 3      // derajat
#define THRESHOLD_DIST  3.0f   // cm
#define STATIS_DURASI   240000UL // 4 menit = 240000 ms

// ================= WIFI =================
const char* ssid     = "Postur_corrector";
const char* password = "ratuu uu6";
WebServer server(80);

// ================= FIREBASE =================
const char* FIREBASE_PROJECT_ID = "posture-monitor-4f413";

String sessionId    = "";
int    totalRecords = 0;

// ================= SENSOR =================
Adafruit_VL53L0X        lox;
Adafruit_ADXL345_Unified adxl1 = Adafruit_ADXL345_Unified(1);
Adafruit_ADXL345_Unified adxl2 = Adafruit_ADXL345_Unified(2);
LiquidCrystal_I2C        lcd(0x27, 16, 2);

// ================= PIN =================
#define BUTTON_PIN 14

// ================= KALIBRASI OFFSET =================
float rollOffset1 = 0, pitchOffset1 = 0;
float rollOffset2 = 0, pitchOffset2 = 0;
bool  lastButton  = HIGH;

// ================= PARAMETER NORMALISASI =================
float xmin[5] = {-30.0, -37.0, -53.0, -19.0,  1.5};
float xmax[5] = { 30.0,  31.0,  33.0,  65.0, 21.6};

// ================= PARAMETER KNN =================
#define K_VALUE   5
#define N_TRAIN 300
#define N_FITUR   5

// ================= DATA LATIH =================
float data_train[N_TRAIN][N_FITUR] = {
  {0.23333,0.76471,0.26744,0.61905,0.28358},
  {0.96667,0.76471,0.88372,0.67857,0.50746},
  {0.50000,0.26471,0.59302,0.14286,0.09453},
  {0.53333,0.60294,0.59302,0.48810,0.43284},
  {0.48333,0.51471,0.63953,0.14286,0.14925},
  {0.51667,0.58824,0.62791,0.34524,0.40796},
  {0.53333,0.52941,0.61628,0.09524,0.43781},
  {0.50000,0.52941,0.55814,0.40476,0.71144},
  {0.81667,0.76471,0.70930,0.54762,0.34328},
  {0.16667,0.92647,0.18605,0.71429,0.76119},
  {0.45000,0.69118,0.60465,0.35714,0.36816},
  {0.45000,0.79412,0.60465,0.44048,0.45274},
  {0.43333,0.63235,0.33721,0.50000,0.52239},
  {0.50000,0.38235,0.59302,0.21429,0.04975},
  {0.46667,0.58824,0.61628,0.23810,0.39801},
  {0.18333,0.69118,0.19767,0.58333,0.43781},
  {0.43333,0.61765,0.37209,0.51190,0.45274},
  {0.50000,0.52941,0.59302,0.34524,1.00000},
  {0.53333,0.45588,0.58140,0.30952,0.38308},
  {0.50000,0.80882,0.60465,0.40476,0.42786},
  {0.53333,0.48529,0.61628,0.14286,0.28358},
  {0.51667,0.60294,0.61628,0.39286,0.89055},
  {0.28333,0.76471,0.38372,0.67857,0.36816},
  {0.46667,0.77941,0.54651,0.69048,0.35323},
  {0.51667,0.70588,0.55814,0.58333,0.37811},
  {0.50000,0.50000,0.58140,0.26190,0.27363},
  {0.51667,0.51471,0.62791,0.17857,0.22388},
  {0.46667,0.66176,0.61628,0.57143,0.30348},
  {0.48333,0.92647,0.60465,0.63095,0.98507},
  {0.50000,0.58824,0.59302,0.35714,0.11940},
  {0.50000,0.54412,0.61628,0.22619,0.33333},
  {0.50000,0.54412,0.58140,0.35714,0.57711},
  {0.16667,0.91176,0.18605,0.70238,0.76119},
  {0.86667,0.73529,0.96512,0.64286,0.86070},
  {0.65000,0.61765,0.80233,0.57143,0.71144},
  {0.48333,0.66176,0.58140,0.47619,0.43284},
  {0.48333,0.57353,0.61628,0.22619,0.67662},
  {0.48333,0.69118,0.61628,0.42857,0.45771},
  {0.48333,0.45588,0.55814,0.35714,0.46766},
  {0.26667,0.91176,0.25581,0.76190,0.45274},
  {0.50000,0.54412,0.61628,0.22619,0.30348},
  {0.40000,0.61765,0.29070,0.48810,0.51741},
  {0.70000,0.54412,0.74419,0.58333,0.30348},
  {0.25000,0.36765,0.40698,0.28571,0.11940},
  {0.50000,0.54412,0.55814,0.41667,0.72637},
  {0.53333,0.48529,0.61628,0.10714,0.38308},
  {0.50000,0.50000,0.60465,0.16667,0.05970},
  {0.46667,0.54412,0.53488,0.44048,0.77114},
  {0.50000,0.54412,0.61628,0.23810,0.37313},
  {0.45000,0.00000,0.61628,0.02381,0.07960},
  {0.50000,0.50000,0.56977,0.47619,0.19900},
  {0.46667,0.83824,0.60465,0.40476,0.55721},
  {0.66667,0.60294,0.75581,0.46429,0.41294},
  {0.45000,0.85294,0.43023,0.63095,0.37811},
  {0.15000,0.70588,0.41860,0.53571,0.29353},
  {0.51667,0.52941,0.61628,0.20238,0.63184},
  {0.46667,0.57353,0.53488,0.48810,0.88557},
  {0.48333,0.95588,0.50000,0.60714,0.57214},
  {0.70000,0.38235,0.74419,0.34524,0.00000},
  {0.48333,0.79412,0.56977,0.45238,0.43781},
  {0.05000,0.75000,0.39535,0.61905,0.27861},
  {0.21667,0.89706,0.53488,0.65476,0.37313},
  {0.36667,1.00000,0.61628,0.60714,0.48756},
  {0.70000,0.60294,0.77907,0.47619,0.41791},
  {0.50000,0.54412,0.61628,0.27381,0.15920},
  {0.45000,0.00000,0.61628,0.02381,0.08458},
  {0.50000,0.51471,0.54651,0.36905,0.08955},
  {0.50000,0.61765,0.61628,0.34524,0.45771},
  {0.50000,0.54412,0.61628,0.08333,0.75124},
  {0.28333,0.67647,0.26744,0.59524,0.42786},
  {0.48333,0.69118,0.61628,0.42857,0.52736},
  {0.13333,0.73529,0.37209,0.65476,0.85572},
  {0.63333,0.76471,0.77907,0.69048,0.45771},
  {0.81667,0.67647,0.83721,0.58333,0.29851},
  {0.58333,0.55882,0.67442,0.46429,0.27861},
  {0.53333,0.48529,0.61628,0.15476,0.27363},
  {0.98333,0.63235,1.00000,0.72619,0.32338},
  {0.50000,0.58824,0.63953,0.28571,0.73134},
  {0.38333,0.61765,0.32558,0.51190,0.44279},
  {0.35000,0.66176,0.38372,0.72619,0.44279},
  {0.28333,0.32353,0.43023,0.35714,0.03483},
  {0.65000,0.70588,0.79070,0.58333,0.24876},
  {0.76667,0.67647,0.74419,0.55952,0.55224},
  {0.40000,0.70588,0.60465,0.52381,0.32836},
  {0.51667,0.60294,0.56977,0.41667,0.60697},
  {0.48333,0.58824,0.61628,0.25000,0.39801},
  {0.33333,0.66176,0.37209,0.55952,0.33333},
  {0.46667,0.55882,0.53488,0.44048,0.77114},
  {0.50000,0.75000,0.52326,0.75000,0.47264},
  {0.66667,0.60294,0.74419,0.47619,0.41294},
  {0.48333,0.95588,0.50000,0.59524,0.57214},
  {0.50000,0.57353,0.63953,0.26190,0.73632},
  {0.48333,0.51471,0.58140,0.42857,0.22388},
  {0.51667,0.58824,0.62791,0.33333,0.40796},
  {0.50000,0.52941,0.61628,0.27381,0.49254},
  {0.41667,0.89706,0.56977,1.00000,0.54726},
  {0.70000,0.61765,0.76744,0.57143,0.24876},
  {0.51667,0.64706,0.61628,0.47619,0.55721},
  {0.46667,0.86765,0.60465,0.39286,0.56219},
  {0.23333,0.79412,0.25581,0.72619,0.37811},
  {0.50000,0.54412,0.61628,0.22619,0.33333},
  {0.61667,0.64706,0.73256,0.55952,0.40796},
  {0.48333,0.60294,0.61628,0.25000,0.39801},
  {0.48333,0.45588,0.58140,0.36905,0.62189},
  {0.46667,0.67647,0.58140,0.50000,0.43781},
  {0.60000,0.79412,0.74419,0.66667,0.32338},
  {0.50000,0.41176,0.58140,0.27381,0.03483},
  {0.46667,0.57353,0.53488,0.47619,0.89055},
  {0.50000,0.57353,0.58140,0.32143,0.08458},
  {0.46667,0.50000,0.61628,0.11905,0.17413},
  {0.40000,0.61765,0.31395,0.48810,0.51741},
  {0.50000,0.57353,0.55814,0.57143,0.28856},
  {0.31667,0.58824,0.39535,0.50000,0.73134},
  {0.53333,0.50000,0.58140,0.36905,0.36318},
  {0.48333,0.75000,0.61628,0.46429,0.53731},
  {0.48333,0.58824,0.61628,0.21429,0.68159},
  {0.58333,0.66176,0.74419,0.57143,0.26866},
  {0.43333,0.63235,0.38372,0.51190,0.45274},
  {0.41667,0.86765,0.54651,0.97619,0.53731},
  {0.50000,0.23529,0.60465,0.11905,0.08955},
  {0.36667,0.64706,0.39535,0.70238,0.44776},
  {0.16667,0.58824,0.34884,0.53571,0.41294},
  {0.53333,0.48529,0.61628,0.13095,0.28358},
  {0.50000,0.51471,0.61628,0.14286,0.15423},
  {0.50000,0.51471,0.60465,0.36905,0.98507},
  {0.28333,0.72059,0.33721,0.73810,0.44279},
  {0.80000,0.79412,0.83721,0.79762,0.49751},
  {0.50000,0.35294,0.61628,0.00000,0.14428},
  {0.46667,0.70588,0.56977,0.53571,0.43781},
  {0.46667,0.85294,0.45349,0.61905,0.38308},
  {0.50000,0.55882,0.61628,0.11905,0.34826},
  {0.50000,0.54412,0.61628,0.07143,0.75622},
  {0.46667,0.75000,0.55814,0.65476,0.33333},
  {0.46667,0.86765,0.43023,0.64286,0.36816},
  {0.83333,0.29412,0.74419,0.02381,0.30846},
  {0.50000,0.32353,0.61628,0.08333,0.04478},
  {0.11667,0.64706,0.20930,0.75000,0.32338},
  {0.45000,0.86765,0.43023,0.64286,0.36816},
  {0.46667,0.86765,0.60465,0.38095,0.56219},
  {0.48333,0.60294,0.61628,0.25000,0.39801},
  {0.43333,0.63235,0.37209,0.51190,0.45274},
  {0.50000,0.91176,0.47674,0.82143,0.55721},
  {0.66667,0.60294,0.75581,0.46429,0.41294},
  {0.50000,0.35294,0.61628,0.00000,0.14428},
  {0.50000,0.70588,0.61628,0.44048,0.43284},
  {0.50000,0.55882,0.59302,0.28571,0.11443},
  {0.45000,0.60294,0.61628,0.21429,0.45771},
  {0.50000,0.54412,0.61628,0.22619,0.30348},
  {0.53333,0.50000,0.59302,0.38095,0.36816},
  {0.48333,0.57353,0.61628,0.22619,0.67662},
  {0.65000,0.57353,0.79070,0.46429,0.47264},
  {0.66667,0.61765,0.82558,0.57143,0.71144},
  {0.50000,0.55882,0.60465,0.26190,0.15423},
  {0.50000,0.47059,0.61628,0.36905,0.36318},
  {0.23333,0.76471,0.26744,0.61905,0.28856},
  {0.45000,0.85294,0.44186,0.63095,0.37811},
  {0.50000,0.35294,0.61628,0.00000,0.14925},
  {0.71667,0.61765,0.79070,0.48810,0.42289},
  {0.46667,0.77941,0.54651,0.69048,0.35323},
  {0.50000,0.77941,0.60465,0.48810,0.42786},
  {0.46667,0.66176,0.61628,0.57143,0.30348},
  {0.70000,0.60294,0.76744,0.57143,0.24378},
  {0.16667,0.91176,0.19767,0.70238,0.76119},
  {0.46667,0.58824,0.53488,0.47619,0.89552},
  {0.50000,0.95588,0.50000,0.60714,0.57214},
  {0.81667,0.69118,0.83721,0.59524,0.30348},
  {0.53333,0.48529,0.61628,0.14286,0.28856},
  {0.50000,0.50000,0.56977,0.47619,0.17413},
  {0.46667,0.66176,0.61628,0.55952,0.30348},
  {0.60000,0.79412,0.74419,0.65476,0.32338},
  {0.66667,0.60294,0.74419,0.46429,0.41294},
  {0.46667,0.79412,0.59302,0.45238,0.45771},
  {0.50000,0.91176,0.52326,0.57143,0.56219},
  {0.48333,0.66176,0.47674,0.52381,0.35821},
  {0.53333,0.60294,0.59302,0.48810,0.43284},
  {0.50000,0.54412,0.54651,0.41667,0.72637},
  {0.48333,0.66176,0.47674,0.53571,0.35821},
  {0.80000,0.30882,0.70930,0.35714,0.16915},
  {0.18333,0.69118,0.19767,0.57143,0.43284},
  {0.23333,0.98529,0.30233,0.97619,0.60199},
  {0.56667,0.82353,0.70930,0.79762,0.48756},
  {0.50000,0.27941,0.61628,0.22619,0.09453},
  {0.50000,0.61765,0.59302,0.26190,0.91045},
  {0.78333,0.67647,0.74419,0.55952,0.55224},
  {0.70000,0.54412,0.73256,0.57143,0.29353},
  {0.40000,0.54412,0.47674,0.45238,0.33333},
  {0.53333,0.48529,0.59302,0.32143,0.38308},
  {0.00000,0.54412,0.38372,0.42857,0.47761},
  {0.50000,0.58824,0.62791,0.32143,0.51741},
  {0.00000,0.73529,0.00000,0.71429,0.35821},
  {0.50000,0.50000,0.58140,0.47619,0.19900},
  {0.50000,0.61765,0.60465,0.27381,0.91045},
  {0.45000,0.85294,0.44186,0.63095,0.37811},
  {0.33333,0.79412,0.45349,0.61905,0.36318},
  {0.85000,0.69118,0.75581,0.30952,0.36816},
  {0.23333,0.97059,0.23256,0.80952,0.48756},
  {0.30000,0.76471,0.39535,0.66667,0.36816},
  {1.00000,0.76471,0.90698,0.65476,0.51244},
  {0.50000,0.54412,0.61628,0.44048,0.40299},
  {0.46667,0.55882,0.61628,0.22619,0.34328},
  {0.51667,0.55882,0.61628,0.02381,0.60697},
  {0.50000,0.55882,0.61628,0.39286,0.07463},
  {0.66667,0.60294,0.75581,0.46429,0.41294},
  {0.01667,0.75000,0.37209,0.64286,0.28358},
  {0.48333,0.61765,0.55814,0.52381,0.56716},
  {0.50000,0.70588,0.56977,0.58333,0.31343},
  {0.50000,0.54412,0.61628,0.08333,0.75124},
  {0.45000,0.77941,0.54651,0.70238,0.34826},
  {0.51667,0.54412,0.61628,0.41667,0.65174},
  {0.56667,0.80882,0.70930,0.78571,0.48259},
  {0.71667,0.60294,0.77907,0.47619,0.41791},
  {1.00000,0.63235,1.00000,0.72619,0.32836},
  {0.50000,0.61765,0.54651,0.35714,0.58706},
  {0.45000,0.01471,0.60465,0.01190,0.08955},
  {0.51667,0.55882,0.61628,0.39286,0.05970},
  {0.50000,0.54412,0.61628,0.22619,0.32338},
  {0.50000,0.57353,0.59302,0.30952,0.08458},
  {0.23333,0.76471,0.26744,0.61905,0.28856},
  {0.61667,0.63235,0.73256,0.55952,0.40299},
  {0.31667,0.51471,0.38372,0.55952,0.21891},
  {0.98333,0.76471,0.88372,0.67857,0.51244},
  {0.60000,0.66176,0.76744,0.55952,0.26866},
  {0.50000,0.51471,0.61628,0.23810,0.07463},
  {0.48333,0.63235,0.56977,0.52381,0.27861},
  {0.43333,0.63235,0.38372,0.52381,0.45771},
  {0.75000,0.98529,0.89535,0.98810,0.61194},
  {0.48333,0.77941,0.58140,0.46429,0.44279},
  {0.53333,0.50000,0.61628,0.14286,0.29353},
  {0.48333,0.80882,0.54651,0.51190,0.42786},
  {0.53333,0.67647,0.59302,0.63095,0.38308},
  {0.50000,0.51471,0.61628,0.25000,0.25373},
  {0.63333,0.57353,0.77907,0.52381,0.67164},
  {0.45000,0.54412,0.61628,0.22619,0.33831},
  {0.48333,0.51471,0.59302,0.38095,0.97015},
  {0.55000,0.55882,0.61628,0.45238,0.39801},
  {0.43333,0.63235,0.33721,0.50000,0.52239},
  {0.48333,0.54412,0.53488,0.48810,0.85075},
  {0.28333,0.76471,0.38372,0.67857,0.36816},
  {0.50000,0.45588,0.60465,0.35714,0.34328},
  {0.23333,0.76471,0.26744,0.64286,0.26368},
  {0.86667,0.72059,0.96512,0.64286,0.85572},
  {0.53333,0.64706,0.61628,0.47619,0.55721},
  {0.75000,1.00000,0.89535,1.00000,0.62189},
  {0.98333,0.76471,0.89535,0.66667,0.51244},
  {0.48333,0.82353,0.59302,0.48810,0.41791},
  {0.46667,0.76471,0.59302,0.46429,0.45274},
  {0.55000,0.75000,0.55814,0.63095,0.45771},
  {0.61667,0.64706,0.72093,0.66667,0.42786},
  {0.48333,0.51471,0.63953,0.21429,0.14428},
  {0.50000,0.54412,0.61628,0.08333,0.35323},
  {0.55000,0.55882,0.61628,0.45238,0.39801},
  {0.48333,0.50000,0.62791,0.10714,0.51244},
  {0.50000,0.52941,0.61628,0.28571,0.48756},
  {0.50000,0.32353,0.61628,0.08333,0.04975},
  {0.50000,0.45588,0.56977,0.36905,0.62687},
  {0.50000,0.54412,0.61628,0.22619,0.14428},
  {0.60000,0.64706,0.70930,0.57143,0.40796},
  {0.48333,0.64706,0.53488,0.54762,0.29851},
  {0.50000,0.50000,0.61628,0.20238,0.05970},
  {0.66667,0.58824,0.74419,0.46429,0.41294},
  {0.33333,0.79412,0.45349,0.63095,0.36318},
  {0.46667,0.77941,0.54651,0.69048,0.34826},
  {0.50000,0.92647,0.51163,0.57143,0.56219},
  {0.48333,0.54412,0.61628,0.09524,0.74627},
  {0.50000,0.27941,0.61628,0.22619,0.10448},
  {0.30000,0.72059,0.33721,0.73810,0.44776},
  {0.50000,0.73529,0.53488,0.59524,0.32338},
  {0.38333,0.60294,0.32558,0.51190,0.43781},
  {0.50000,0.58824,0.63953,0.28571,0.73632},
  {0.50000,0.54412,0.61628,0.20238,0.67164},
  {0.51667,0.57353,0.58140,0.44048,0.40796},
  {0.48333,0.51471,0.65116,0.17857,0.12935},
  {0.65000,0.57353,0.77907,0.46429,0.46766},
  {0.53333,0.57353,0.60465,0.45238,0.40796},
  {0.53333,0.54412,0.61628,0.11905,0.44279},
  {0.50000,0.51471,0.60465,0.38095,0.98010},
  {0.53333,0.50000,0.58140,0.36905,0.37313},
  {0.10000,0.64706,0.18605,0.76190,0.32338},
  {0.23333,1.00000,0.31395,0.98810,0.60697},
  {0.48333,0.63235,0.58140,0.52381,0.28358},
  {0.71667,0.38235,0.74419,0.35714,0.01990},
  {0.63333,0.64706,0.75581,0.65476,0.42289},
  {0.00000,0.55882,0.37209,0.42857,0.48259},
  {0.51667,0.54412,0.61628,0.16667,0.61692},
  {0.50000,0.54412,0.61628,0.22619,0.70149},
  {0.25000,0.97059,0.23256,0.80952,0.48259},
  {0.50000,0.23529,0.59302,0.13095,0.09453},
  {0.50000,0.61765,0.55814,0.35714,0.59204},
  {0.70000,0.38235,0.74419,0.34524,0.00995},
  {0.26667,0.36765,0.39535,0.28571,0.11940},
  {0.28333,0.67647,0.26744,0.59524,0.42786},
  {0.50000,0.30882,0.61628,0.22619,0.03980},
  {0.48333,0.63235,0.60465,0.41667,0.41294},
  {0.71667,0.60294,0.77907,0.47619,0.41791},
  {0.48333,0.80882,0.59302,0.47619,0.41791},
  {0.61667,0.64706,0.73256,0.57143,0.40796},
  {0.70000,0.60294,0.77907,0.46429,0.41294},
  {0.48333,0.51471,0.63953,0.14286,0.13433},
  {0.80000,0.79412,0.83721,0.79762,0.49751},
  {0.48333,0.45588,0.56977,0.36905,0.62687}
};

int label_train[N_TRAIN] = {
  3,3,1,2,1,1,1,2,2,3,1,1,2,1,1,3,2,2,1,1,1,2,3,3,2,1,1,2,3,1,1,2,3,3,3,2,2,1,1,3,
  1,3,2,2,2,1,1,2,1,3,2,2,1,3,2,2,3,3,1,1,3,3,3,2,1,3,1,1,2,3,2,3,3,2,1,1,3,2,2,3,
  2,2,3,2,2,1,2,2,3,2,3,2,1,1,1,3,2,3,3,3,1,2,1,2,2,3,1,3,1,1,3,2,3,1,2,2,2,2,3,2,
  3,2,1,1,2,3,3,1,2,3,1,2,3,3,2,1,3,3,3,1,2,3,1,1,1,1,1,1,1,2,1,3,1,1,3,3,1,2,3,2,
  2,2,3,3,3,3,1,2,2,3,1,1,3,2,2,2,2,2,3,3,3,1,2,3,2,1,1,3,1,3,2,2,3,3,2,3,3,3,1,1,
  2,1,1,3,3,2,2,3,2,3,2,3,2,3,1,1,1,3,2,2,3,2,1,2,2,3,1,1,2,3,1,3,1,2,1,2,3,3,1,3,
  3,3,3,3,2,1,3,3,1,1,1,1,1,1,2,1,2,2,1,1,3,3,3,2,1,3,3,2,2,2,1,1,1,1,1,2,1,3,3,2,
  1,3,3,2,2,3,2,2,1,2,3,1,1,2,2,2,1,1,3,2
};

// ================= VARIABEL SAMPLING =================
static long   sumR1 = 0, sumP1 = 0, sumR2 = 0, sumP2 = 0;
static float  sumVL = 0;
static int    count = 0;
static unsigned long startTime = 0;

int   feat_r1 = 0, feat_p1 = 0, feat_r2 = 0, feat_p2 = 0;
float feat_vl = 0;
float feat_norm[N_FITUR];
int   hasilKNN = 0;

// ================= STRUCT HASIL KNN =================
struct HasilKNN {
  int   prediksi;
  float similarity;
  float bobot[4];
  float totalBobot;
  float kDist[K_VALUE];
  int   kLabel[K_VALUE];
};
HasilKNN hasilDetail;

// ================= FUNGSI KALIBRASI =================
void kalibrasi() {
  sensors_event_t e1, e2;
  adxl1.getEvent(&e1);
  adxl2.getEvent(&e2);
  rollOffset1  = atan2(e1.acceleration.y, e1.acceleration.z) * 57.3;
  pitchOffset1 = atan2(-e1.acceleration.x,
                   sqrt(e1.acceleration.y*e1.acceleration.y +
                        e1.acceleration.z*e1.acceleration.z)) * 57.3;
  rollOffset2  = atan2(e2.acceleration.y, e2.acceleration.z) * 57.3;
  pitchOffset2 = atan2(-e2.acceleration.x,
                   sqrt(e2.acceleration.y*e2.acceleration.y +
                        e2.acceleration.z*e2.acceleration.z)) * 57.3;
  lcd.clear(); lcd.print("Kalibrasi OK");
  Serial.println("=== KALIBRASI OK ===");
  delay(800); lcd.clear();
}

// ================= FUNGSI BACA ADXL =================
void bacaADXL(Adafruit_ADXL345_Unified &sensor,
              float rollOffset, float pitchOffset,
              int &rollOut, int &pitchOut) {
  sensors_event_t e;
  sensor.getEvent(&e);
  float roll_f  = atan2(e.acceleration.y, e.acceleration.z) * 57.3;
  float pitch_f = atan2(-e.acceleration.x,
                    sqrt(e.acceleration.y*e.acceleration.y +
                         e.acceleration.z*e.acceleration.z)) * 57.3;
  rollOut  = round(roll_f  - rollOffset);
  pitchOut = round(-(pitch_f - pitchOffset));
}

// ================= NORMALISASI =================
float normalisasi(float nilai, float minVal, float maxVal) {
  if (maxVal - minVal == 0) return 0.0;
  float n = (nilai - minVal) / (maxVal - minVal);
  if (n < 0.0) n = 0.0;
  if (n > 1.0) n = 1.0;
  return n;
}

// ================= EUCLIDEAN =================
float euclidean(float *test, int idxTrain) {
  float sum = 0;
  for (int i = 0; i < N_FITUR; i++) {
    float diff = test[i] - data_train[idxTrain][i];
    sum += diff * diff;
  }
  return sqrt(sum);
}

// ================= KLASIFIKASI KNN =================
int knn(float *test) {
  float kDist[K_VALUE];
  int   kLabel[K_VALUE];
  for (int i = 0; i < K_VALUE; i++) {
    kDist[i]  = 1e9;
    kLabel[i] = -1;
  }

  for (int i = 0; i < N_TRAIN; i++) {
    float d = euclidean(test, i);
    for (int k = 0; k < K_VALUE; k++) {
      if (d < kDist[k]) {
        for (int m = K_VALUE - 1; m > k; m--) {
          kDist[m]  = kDist[m - 1];
          kLabel[m] = kLabel[m - 1];
        }
        kDist[k]  = d;
        kLabel[k] = label_train[i];
        break;
      }
    }
  }

  float bobot[4]   = {0, 0, 0, 0};
  float totalBobot = 0;

  for (int k = 0; k < K_VALUE; k++) {
    float w;
    if (kDist[k] == 0) {
      w = 1e9;
    } else {
      w = 1.0f / (kDist[k] * kDist[k]);
    }
    if (kLabel[k] >= 1 && kLabel[k] <= 3) {
      bobot[kLabel[k]] += w;
      totalBobot        += w;
    }
  }

  int prediksi = 1;
  for (int c = 2; c <= 3; c++) {
    if (bobot[c] > bobot[prediksi]) prediksi = c;
  }

  float sim = (totalBobot > 0) ? (bobot[prediksi] / totalBobot * 100.0f) : 0;

  hasilDetail.prediksi   = prediksi;
  hasilDetail.similarity = sim;
  hasilDetail.totalBobot = totalBobot;
  for (int c = 0; c <= 3; c++) hasilDetail.bobot[c] = bobot[c];
  for (int k = 0; k < K_VALUE; k++) {
    hasilDetail.kDist[k]  = kDist[k];
    hasilDetail.kLabel[k] = kLabel[k];
  }

  return prediksi;
}

// ================= NAMA POSTUR =================
String namaPostur(int label) {
  switch (label) {
    case 1: return "ERGONOMIS";
    case 2: return "KURANG ERGONOMIS";
    case 3: return "TIDAK ERGONOMIS";
    default: return "???";
  }
}

// ================= BUAT SESSION =================
void buatSession() {
  if (WiFi.status() != WL_CONNECTED) return;

  sessionId = String(esp_random()) + String(esp_random() % 9000 + 1000);

  HTTPClient http;
  String url = "https://firestore.googleapis.com/v1/projects/";
  url += FIREBASE_PROJECT_ID;
  url += "/databases/(default)/documents/sessions/" + sessionId;

  String body = "{\"fields\":{";
  body += "\"startTime\":{\"integerValue\":\"" + String(millis()) + "\"},";
  body += "\"status\":{\"stringValue\":\"active\"},";
  body += "\"totalRecords\":{\"integerValue\":\"0\"}";
  body += "}}";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.sendRequest("PATCH", body);

  if (code == 200 || code == 201) {
    Serial.println("✅ Session Firebase dibuat: " + sessionId);
    lcd.clear(); lcd.print("Firebase OK!");
    delay(800); lcd.clear();
  } else {
    Serial.println("❌ Gagal buat session. HTTP: " + String(code));
    lcd.clear(); lcd.print("Firebase FAIL");
    delay(1000); lcd.clear();
  }
  http.end();
}

// ================= POST DATA KE FIREBASE =================
void postKeFirebase() {
  if (WiFi.status() != WL_CONNECTED || sessionId == "") {
    Serial.println("⚠️ Skip POST: WiFi/session belum siap");
    return;
  }

  HTTPClient http;
  String url = "https://firestore.googleapis.com/v1/projects/";
  url += FIREBASE_PROJECT_ID;
  url += "/databases/(default)/documents/sessions/";
  url += sessionId;
  url += "/records";

  String body = "{\"fields\":{";
  body += "\"timestamp\":{\"integerValue\":\"" + String(millis()) + "\"},";
  body += "\"posture_label\":{\"integerValue\":\"" + String(hasilKNN) + "\"},";
  body += "\"posture_name\":{\"stringValue\":\"" + namaPostur(hasilKNN) + "\"},";
  body += "\"similarity\":{\"doubleValue\":" + String(hasilDetail.similarity, 2) + "},";
  body += "\"roll1\":{\"integerValue\":\"" + String(feat_r1) + "\"},";
  body += "\"pitch1\":{\"integerValue\":\"" + String(feat_p1) + "\"},";
  body += "\"roll2\":{\"integerValue\":\"" + String(feat_r2) + "\"},";
  body += "\"pitch2\":{\"integerValue\":\"" + String(feat_p2) + "\"},";
  body += "\"distance_cm\":{\"doubleValue\":" + String(feat_vl, 2) + "}";
  body += "}}";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);

  if (code == 200 || code == 201) {
    totalRecords++;
    Serial.println("✅ Data #" + String(totalRecords) + " → Firebase OK");

    HTTPClient http2;
    String urlSession = "https://firestore.googleapis.com/v1/projects/";
    urlSession += FIREBASE_PROJECT_ID;
    urlSession += "/databases/(default)/documents/sessions/" + sessionId;
    urlSession += "?updateMask.fieldPaths=totalRecords&updateMask.fieldPaths=status";

    String bodyUpdate = "{\"fields\":{";
    bodyUpdate += "\"totalRecords\":{\"integerValue\":\"" + String(totalRecords) + "\"},";
    bodyUpdate += "\"status\":{\"stringValue\":\"active\"}";
    bodyUpdate += "}}";

    http2.begin(urlSession);
    http2.addHeader("Content-Type", "application/json");
    http2.sendRequest("PATCH", bodyUpdate);
    http2.end();

  } else {
    Serial.println("❌ POST Firebase gagal. HTTP: " + String(code));
    Serial.println(http.getString());
  }
  http.end();
}

// ================= HANDLER /data =================
void handleData() {
  String json = "{";
  json += "\"r1\":"           + String(feat_r1)                   + ",";
  json += "\"p1\":"           + String(feat_p1)                   + ",";
  json += "\"r2\":"           + String(feat_r2)                   + ",";
  json += "\"p2\":"           + String(feat_p2)                   + ",";
  json += "\"vl\":"           + String(feat_vl, 1)                + ",";
  json += "\"postur\":"       + String(hasilKNN)                  + ",";
  json += "\"label\":\""     + namaPostur(hasilKNN)               + "\",";
  json += "\"similarity\":"   + String(hasilDetail.similarity, 1) + ",";
  json += "\"sessionId\":\"" + sessionId                          + "\",";
  json += "\"totalRecords\":" + String(totalRecords);
  json += "}";
  server.send(200, "application/json", json);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  lcd.init();
  lcd.backlight();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  if (!adxl1.begin(0x53)) { lcd.print("ADXL1 ERROR"); while (1); }
  adxl1.setRange(ADXL345_RANGE_16_G);

  if (!adxl2.begin(0x1D)) { lcd.print("ADXL2 ERROR"); while (1); }
  adxl2.setRange(ADXL345_RANGE_16_G);

  if (!lox.begin()) { lcd.print("VL53 ERROR"); while (1); }

  lcd.clear();
  lcd.print("Semua Sensor OK");
  delay(1000);

  lcd.clear();
  lcd.print("WiFi connect...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(800);
  lcd.clear();
  lcd.print("IP:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());

  server.on("/data", handleData);
  server.begin();

  delay(1500);
  lcd.clear();

  kalibrasi();
  buatSession();

  startTime = millis();
  lcd.print("Siap baca...");
  delay(500); lcd.clear();
}

// ================= LOOP =================
void loop() {

  // === CEK TOMBOL KALIBRASI ===
  bool btn = digitalRead(BUTTON_PIN);
  if (lastButton == HIGH && btn == LOW) {
    kalibrasi();
    sumR1 = sumP1 = sumR2 = sumP2 = 0;
    sumVL = 0;
    count = 0;
    startTime = millis();
    // Reset statis saat kalibrasi
    prevValid        = false;
    sedangStatis     = false;
    statisSejak      = 0;
    statisAlertAktif = false;
  }
  lastButton = btn;

  // === BACA SENSOR ===
  int r1_raw, p1_raw, r2_raw, p2_raw;
  bacaADXL(adxl1, rollOffset1, pitchOffset1, r1_raw, p1_raw);
  bacaADXL(adxl2, rollOffset2, pitchOffset2, r2_raw, p2_raw);

  int r1 = p1_raw;
  int p1 = r1_raw;
  int r2 = p2_raw;
  int p2 = r2_raw;

  // === BACA VL53L0X ===
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  float jarak_cm = -1;
  if (measure.RangeStatus == 0) {
    float jarak_mm = (0.8886f * measure.RangeMilliMeter) - 9.8f;
    jarak_cm = jarak_mm / 10.0f;
  }

  // === AKUMULASI ===
  sumR1 += r1;
  sumP1 += p1;
  sumR2 += r2;
  sumP2 += p2;
  if (jarak_cm >= 0) sumVL += jarak_cm;
  count++;

  // === TIAP 10 DETIK → KLASIFIKASI ===
  if (millis() - startTime >= 10000 && count > 0) {

    // ----- 1. RATA-RATA WINDOW -----
    feat_r1 = (int)round((float)sumR1 / count);
    feat_p1 = (int)round((float)sumP1 / count);
    feat_r2 = (int)round((float)sumR2 / count);
    feat_p2 = (int)round((float)sumP2 / count);
    feat_vl = sumVL / count;

    // ----- 2. NORMALISASI -----
    feat_norm[0] = normalisasi((float)feat_r1, xmin[0], xmax[0]);
    feat_norm[1] = normalisasi((float)feat_p1, xmin[1], xmax[1]);
    feat_norm[2] = normalisasi((float)feat_r2, xmin[2], xmax[2]);
    feat_norm[3] = normalisasi((float)feat_p2, xmin[3], xmax[3]);
    feat_norm[4] = normalisasi(feat_vl,        xmin[4], xmax[4]);

    // ----- 3. KLASIFIKASI KNN -----
    hasilKNN = knn(feat_norm);

    // ----- 4. OUTPUT SERIAL -----
    Serial.println("==============================");
    Serial.println("=== HASIL KLASIFIKASI KNN ===");
    Serial.print("Raw  : R1="); Serial.print(feat_r1);
    Serial.print(" P1="); Serial.print(feat_p1);
    Serial.print(" R2="); Serial.print(feat_r2);
    Serial.print(" P2="); Serial.print(feat_p2);
    Serial.print(" VL="); Serial.println(feat_vl, 1);
    Serial.print(">>> Prediksi  : "); Serial.println(namaPostur(hasilKNN));
    Serial.print(">>> Similarity: "); Serial.print(hasilDetail.similarity, 1);
    Serial.println("%");
    Serial.println("==============================");

    // ----- 5. LOGIKA BUZZER - TIDAK ERGONOMIS -----
    if (hasilKNN == 3) {
      if (!sedangTidakErgonomis) {
        sedangTidakErgonomis = true;
        tidakErgonomisSejak  = millis();
      }
      if (millis() - tidakErgonomisSejak >= 240000UL) {
        digitalWrite(BUZZER_PIN, HIGH);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("POSTUR TIDAK");
        lcd.setCursor(0, 1);
        lcd.print("ERGONOMIS!!!!!");
      } else {
        // Belum 4 menit → tampilkan info normal
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(namaPostur(hasilKNN));
        lcd.setCursor(0, 1);
        lcd.print("Sim:");
        lcd.print(hasilDetail.similarity, 1);
        lcd.print("%");
      }
    } else {
      // Bukan tidak ergonomis → reset timer tidak ergonomis & matikan buzzer tidak ergonomis
      // (buzzer statis ditangani terpisah di bawah)
      sedangTidakErgonomis = false;
      tidakErgonomisSejak  = 0;

      // Tampilkan info normal di LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(namaPostur(hasilKNN));
      lcd.setCursor(0, 1);
      lcd.print("Sim:");
      lcd.print(hasilDetail.similarity, 1);
      lcd.print("%");
    }

    // ----- 6. LOGIKA BUZZER - POSISI STATIS -----
    // Bandingkan rata-rata window sekarang vs window sebelumnya
    if (prevValid) {
      int selisihR1   = abs(feat_r1 - prevR1);
      int selisihP1   = abs(feat_p1 - prevP1);
      int selisihR2   = abs(feat_r2 - prevR2);
      int selisihP2   = abs(feat_p2 - prevP2);
      float selisihVL = fabs(feat_vl - prevVL);

      bool tidakBerubah =
          selisihR1   <= THRESHOLD_ANGLE &&
          selisihP1   <= THRESHOLD_ANGLE &&
          selisihR2   <= THRESHOLD_ANGLE &&
          selisihP2   <= THRESHOLD_ANGLE &&
          selisihVL   <= THRESHOLD_DIST;

      if (tidakBerubah) {
        if (!sedangStatis) {
          sedangStatis = true;
          statisSejak  = millis();
          Serial.println("⏱️ Posisi statis mulai dihitung...");
        }
        if (millis() - statisSejak >= STATIS_DURASI && !statisAlertAktif) {
          statisAlertAktif = true;
          digitalWrite(BUZZER_PIN, HIGH);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("POSTUR STATIS!");
          lcd.setCursor(0, 1);
          lcd.print("Gerak sekarang!");
          Serial.println("🔔 ALERT: Posisi statis > 4 menit!");
        }
      } else {
        // Ada perubahan → reset timer statis
        sedangStatis     = false;
        statisSejak      = 0;
        statisAlertAktif = false;
        Serial.println("✅ Posisi berubah, timer statis direset.");

        // Matikan buzzer statis (kecuali buzzer tidak ergonomis masih aktif)
        if (!(sedangTidakErgonomis && millis() - tidakErgonomisSejak >= 240000UL)) {
          digitalWrite(BUZZER_PIN, LOW);
        }
      }
    }

    // Simpan nilai window sekarang sebagai prev untuk window berikutnya
    prevR1    = feat_r1;
    prevP1    = feat_p1;
    prevR2    = feat_r2;
    prevP2    = feat_p2;
    prevVL    = feat_vl;
    prevValid = true;

    // ----- 7. POST KE FIREBASE -----
    postKeFirebase();

    // ----- 8. RESET WINDOW -----
    sumR1 = sumP1 = sumR2 = sumP2 = 0;
    sumVL = 0;
    count = 0;
    startTime = millis();

    // Matikan buzzer jika kedua kondisi sudah tidak aktif
    if (!statisAlertAktif &&
        !(sedangTidakErgonomis && millis() - tidakErgonomisSejak >= 240000UL)) {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  server.handleClient();
  delay(100);
}
