#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>

float timestamp_ms1, noisy_distance_m1, Angle_d1;
float timestamp_ms2, noisy_distance_m2, Angle_d2;
float timestamp_ms3, noisy_distance_m3, Angle_d3;
float timestamp_ms4, noisy_distance_m4, Angle_d4;
float calibX = 0.0;
float calibY = 0.0;

double loc1[] = {0.25, 0.25, 0};
double loc2[] = { -0.25, 0.25, 0};
double loc3[] = { -0.25,  -0.25, 0};
double loc4[] = {0.25,  -0.25, 0};

int count = 1;

bool got1 = false, got2 = false, got3 = false, got4 = false;

const int LED_PIN = 13;
unsigned long lastBlink = 0;
bool ledState = false;
const unsigned long BLINK_INTERVAL = 500;  // 0.5 second


struct TeensyPacket {
  float angle;      
  uint16_t distance; 
};

bool readPacket(HardwareSerial &port, TeensyPacket &pkt) {
  if (port.available() >= 6) {
    port.readBytes((uint8_t*)&pkt.angle, sizeof(pkt.angle));
    port.readBytes((uint8_t*)&pkt.distance, sizeof(pkt.distance));
    return true;
  }
  return false;
}

void doTriangulation(float &outX, float &outY) {
// float a1 = Angle_d1;
// float a2 = Angle_d2;
// float a3 = Angle_d3;
// float a4 = Angle_d4;

// BLA::Matrix<3, 3> GRSQ1 = {-1,0,0,0,1,0,0,0,1};
// BLA::Matrix<3, 3> GRSQ2 = {-1,0,0,0,1,0,0,0,1};
// BLA::Matrix<3, 3> GRSQ3 = {1,0,0,0,-1,0,0,0,1};
// BLA::Matrix<3, 3> GRSQ4 = {1,0,0,0,-1,0,0,0,1};

// BLA::Matrix<3, 1> vl1 = {cos(a1), sin(a1), 0};
// BLA::Matrix<3, 1> vl2 = {cos(a1), sin(a1), 0};
// BLA::Matrix<3, 1> vl3 = {cos(a1), sin(a1), 0};
// BLA::Matrix<3, 1> vl4 = {cos(a1), sin(a1), 0};
}
void doTrilateration(float &outX, float &outY) {
  float d1 = noisy_distance_m1;
  float d2 = noisy_distance_m2;
  float d3 = noisy_distance_m3;
  float d4 = noisy_distance_m4;

  float d21 = d2 - d1;
  float d31 = d3 - d1;
  float d32 = d3 - d2;
  float d42 = d4 - d2;
  float d43 = d4 - d3;
  float d13 = d1 - d3;
  float d14 = d1 - d4;
  float d24 = d2 - d4;

  float k1 = (loc1[0] * loc1[0]) + (loc1[1] * loc1[1]);
  float k2 = (loc2[0] * loc2[0]) + (loc2[1] * loc2[1]);
  float k3 = (loc3[0] * loc3[0]) + (loc3[1] * loc3[1]);
  float k4 = (loc4[0] * loc4[0]) + (loc4[1] * loc4[1]);

  float x21 = loc2[0]-loc1[0];
  float y21 = loc2[1]-loc1[1];
  float x31 = loc3[0]-loc1[0];
  float y31 = loc3[1]-loc1[1];  
  float x32 = loc3[0]-loc2[0];
  float y32 = loc3[1]-loc2[1];
  float x42 = loc4[0]-loc2[0];
  float y42 = loc4[1]-loc2[1]; 
  float x43 = loc4[0]-loc3[0];
  float y43 = loc4[1]-loc3[1];
  float x13 = loc1[0]-loc3[0];
  float y13 = loc1[1]-loc3[1]; 
  float x14 = loc1[0]-loc4[0];
  float y14 = loc1[1]-loc4[1];
  float x24 = loc2[0]-loc4[0];
  float y24 = loc2[1]-loc4[1]; 

  BLA::Matrix<2, 2> Bone = {x21, y21, x31, y31};
  BLA::Matrix<2, 2> B2   = {x32, y32, x42, y42};
  BLA::Matrix<2, 2> B3   = {x43, y43, x13, y13};
  BLA::Matrix<2, 2> B4   = {x14, y14, x24, y24};

  BLA::Matrix<2, 2> B1_inv = Inverse(Bone);
  BLA::Matrix<2, 2> B2_inv = Inverse(B2);
  BLA::Matrix<2, 2> B3_inv = Inverse(B3);
  BLA::Matrix<2, 2> B4_inv = Inverse(B4);

  BLA::Matrix<2, 1> D1 = {d21, d31};
  BLA::Matrix<2, 1> Calc1 = {(d21 * d21) - k2 + k1,
                             (d31 * d31) - k3 + k1};
  BLA::Matrix<2, 1> ANS1 = -(B1_inv) * (D1 * d1 + 0.5f * Calc1);

  BLA::Matrix<2, 1> D2 = {d32, d42};
  BLA::Matrix<2, 1> Calc2 = {(d32 * d32) - k3 + k2,
                             (d42 * d42) - k4 + k2};
  BLA::Matrix<2, 1> ANS2 = -(B2_inv) * (D2 * d2 + 0.5f * Calc2);

  BLA::Matrix<2, 1> D3 = {d43, d13};
  BLA::Matrix<2, 1> Calc3 = {(d43 * d43) - k4 + k3,
                             (d13 * d13) - k1 + k3};
  BLA::Matrix<2, 1> ANS3 = -(B3_inv) * (D3 * d3 + 0.5f * Calc3);

  BLA::Matrix<2, 1> D4 = {d14, d24};
  BLA::Matrix<2, 1> Calc4 = {(d14 * d14) - k1 + k4,
                             (d24 * d24) - k2 + k4};
  BLA::Matrix<2, 1> ANS4 = -(B4_inv) * (D4 * d4 + 0.5f * Calc4);

  outX = (ANS1(0)+ANS2(0)+ANS3(0)+ANS4(0))/4;
  outY = (ANS1(1)+ANS2(1)+ANS3(1)+ANS4(1))/4;
}

void setup() {
  Serial.begin(115200);     
  Serial1.begin(115200);    
  Serial2.begin(115200);    
  Serial3.begin(115200);    
  Serial4.begin(115200);    

  pinMode(LED_PIN, OUTPUT);
  Serial.println("Calibrating. Please Wait...");

  const int CALIB_SAMPLES = 100;
  double sumX = 0, sumY = 0;
  int collected = 0;

  while (collected < CALIB_SAMPLES) {
    TeensyPacket pkt;

    if (readPacket(Serial1, pkt)) {
      noisy_distance_m1 = pkt.distance / 1000.0;
      Angle_d1 = pkt.angle;
      got1 = true;
    }

    if (readPacket(Serial2, pkt)) {
      noisy_distance_m2 = pkt.distance / 1000.0;
      Angle_d2 = pkt.angle;
      got2 = true;
    }

    if (readPacket(Serial3, pkt)) {
      noisy_distance_m3 = pkt.distance / 1000.0;
      Angle_d3 = pkt.angle;
      got3 = true;
    }

    if (readPacket(Serial4, pkt)) {
      noisy_distance_m4 = pkt.distance / 1000.0;
      Angle_d4 = pkt.angle;
      got4 = true;
    }

    if (got1 && got2 && got3 && got4) {
      float x, y;
      Serial.println(collected);
      doTrilateration(x, y);
      sumX += x;
      sumY += y;
      collected++;

      got1 = got2 = got3 = got4 = false;
    }
  }

  calibX = sumX / CALIB_SAMPLES;
  calibY = sumY / CALIB_SAMPLES;

  Serial.print("Calibration complete. Origin (x,y) = ");
  Serial.print(calibX, 4);
  Serial.print(", ");
  Serial.println(calibY, 4);

  Serial.println("Calibration Complete.");
}


void loop() {
  if (millis() - lastBlink >= BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
  }

  TeensyPacket pkt;

  if (readPacket(Serial1, pkt)) {
    noisy_distance_m1 = pkt.distance / 1000.0;
    Angle_d1 = pkt.angle;
    // Serial.println("got1");
    // Serial.println(noisy_distance_m1);
    timestamp_ms1 = millis();
    got1 = true;
  }
  if (readPacket(Serial2, pkt)) {
    noisy_distance_m2 = pkt.distance / 1000.0;
    Angle_d2 = pkt.angle;
    // Serial.println("got2");
    // Serial.println(noisy_distance_m2);
    timestamp_ms2 = millis();
    got2 = true;
  }
  if (readPacket(Serial3, pkt)) {
    noisy_distance_m3 = pkt.distance / 1000.0;
    Angle_d3 = pkt.angle;
    // Serial.println("got3");
    // Serial.println(noisy_distance_m3);
    timestamp_ms3 = millis();
    got3 = true;
  }
  if (readPacket(Serial4, pkt)) {
    noisy_distance_m4 = pkt.distance / 1000.0;
    Angle_d4 = pkt.angle;
    // Serial.println("got4");
    // Serial.println(noisy_distance_m4);
    timestamp_ms4 = millis();
    got4 = true;
  }

  if (got1 && got2 && got3 && got4) {
    float rawX, rawY;
    doTrilateration(rawX, rawY);
    
    float adjX = rawX - calibX;
    float adjY = rawY - calibY;

    Serial.print(count);
    Serial.print("\t");
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(adjX, 4);
    Serial.print("\t");
    Serial.println(adjY, 4);

    count++;

    got1 = got2 = got3 = got4 = false;
  }

}