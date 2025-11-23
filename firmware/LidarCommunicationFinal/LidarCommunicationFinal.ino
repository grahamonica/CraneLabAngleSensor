#define PACKET_LEN 47

uint8_t packet[PACKET_LEN];

float closest_angle = 0;
uint16_t closest_distance = 65535;

float last_angle = 0;
bool has_started = false;
int count = 1;

const int LED_PIN = 13;
unsigned long lastBlink = 0;
bool ledState = false;
const unsigned long BLINK_INTERVAL = 1000;

void setup() {
  Serial.begin(115200);

  Serial1.begin(921600);
  Serial2.begin(115200);

  Serial.println("Teensy: Sending closest point once per 360°");
}

void loop() {
    if (millis() - lastBlink >= BLINK_INTERVAL) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastBlink = millis();
    }

    if (readPacket(Serial1, packet)) {
        uint16_t startRaw = packet[4] | (packet[5] << 8);
        uint16_t endRaw = packet[42] | (packet[43] << 8);
        float startAngle = startRaw / 100.0;
        float endAngle = endRaw / 100.0;
        if (endAngle < startAngle) endAngle += 360;

        float angles[12];
        uint16_t distances[12];
        int valid = parsePacket(packet, angles, distances);

        for (int i = 0; i < valid; i++) {
            float angle = angles[i];
            uint16_t dist = distances[i];

            if (angle >= 180 && angle <= 270 && dist < closest_distance) {
                closest_distance = dist;
                closest_angle = angle;
            }
        }

        if (!has_started) {
            last_angle = startAngle;
            has_started = true;
        }

        if (last_angle > 300 && endAngle < 60) {
            if (closest_distance < 65535) {
                Serial2.write((uint8_t*)&closest_angle, sizeof(closest_angle));
                Serial2.write((uint8_t*)&closest_distance, sizeof(closest_distance));

                Serial.print("Rotation ");
                Serial.print(count);
                Serial.print(": ");
                Serial.print(closest_angle);
                Serial.print("° -> ");
                Serial.println(closest_distance);
            }

            closest_distance = 65535;
            closest_angle = 0;
            count++;
        }

        last_angle = endAngle;
    }
}

// --- Helpers ---

bool readPacket(HardwareSerial &port, uint8_t *packet) {
  while (port.available()) {
    uint8_t b = port.read();
    if (b != 0x54) continue;

    packet[0] = b;
    int i = 1;
    unsigned long start = millis();

    while (i < PACKET_LEN && millis() - start < 5) {
      if (port.available()) {
        packet[i++] = port.read();
      }
    }

    if (i == PACKET_LEN && packet[1] == 0x2C) {
      return true;
    }
  }
  return false;
}

uint8_t computeCRC(const uint8_t *data) {
  uint8_t crc = 0;
  for (int i = 0; i < PACKET_LEN - 1; i++) {
    crc ^= data[i];
  }
  return crc;
}

int parsePacket(const uint8_t *packet, float *angles, uint16_t *distances) {
  uint16_t startRaw = packet[4] | (packet[5] << 8);
  uint16_t endRaw = packet[42] | (packet[43] << 8);
  float startAngle = startRaw / 100.0;
  float endAngle = endRaw / 100.0;

  if (startAngle >= 360 || endAngle > 360) return 0;
  if (endAngle < startAngle) endAngle += 360;
  if ((endAngle - startAngle) > 90) return 0;

  float angleStep = (endAngle - startAngle) / 12.0;

  int valid = 0;
  for (int i = 0; i < 12; i++) {
    int offset = 6 + i * 3;
    uint16_t distance = packet[offset] | (packet[offset + 1] << 8);
    uint8_t confidence = packet[offset + 2];
    float angle = fmod((startAngle + i * angleStep), 360.0);

    if (distance > 0 && distance < 1000 && confidence > 0) {
      distances[valid] = distance;
      angles[valid] = angle;
      valid++;
    }
  }

  if (valid == 0) return 0;

  bool allClose = true, allFar = true, allSame = true;
  for (int i = 1; i < valid; i++) {
    if (distances[i] != distances[0]) allSame = false;
    if (distances[i] >= 30) allClose = false;
    if (distances[i] <= 700) allFar = false;
  }
  if (allClose || allFar || allSame) return 0;

  return valid;
}
