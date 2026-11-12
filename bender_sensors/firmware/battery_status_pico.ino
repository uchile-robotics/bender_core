#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t LED_PIN = 14;
constexpr uint16_t LED_COUNT = 16;
constexpr uint8_t LED_BRIGHTNESS = 38;
constexpr size_t SERIAL_LINE_SIZE = 24;
constexpr unsigned long DATA_TIMEOUT_MS = 3000;

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
char serial_line[SERIAL_LINE_SIZE];
size_t serial_line_length = 0;
unsigned long last_data_ms = 0;
unsigned long last_status_toggle_ms = 0;
bool has_battery_data = false;
bool status_led_on = false;

uint32_t color_for_soc(float soc) {
  if (soc < 20.0f) {
    return pixels.Color(255, 0, 0);
  }
  if (soc < 50.0f) {
    return pixels.Color(255, 128, 0);
  }
  if (soc < 80.0f) {
    return pixels.Color(255, 255, 0);
  }
  return pixels.Color(0, 255, 0);
}

void show_color(uint32_t color) {
  for (uint16_t i = 0; i < LED_COUNT; ++i) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void handle_line() {
  serial_line[serial_line_length] = '\0';
  if (strncmp(serial_line, "B,", 2) != 0) {
    return;
  }

  char *end = nullptr;
  const float soc = strtof(serial_line + 2, &end);
  if (end == serial_line + 2 || *end != '\0' || !isfinite(soc)) {
    return;
  }

  const float bounded_soc = constrain(soc, 0.0f, 100.0f);
  has_battery_data = true;
  last_data_ms = millis();
  show_color(color_for_soc(bounded_soc));
}

void read_serial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      handle_line();
      serial_line_length = 0;
    } else if (c != '\r') {
      if (serial_line_length < SERIAL_LINE_SIZE - 1) {
        serial_line[serial_line_length++] = c;
      } else {
        serial_line_length = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS);
  show_color(pixels.Color(0, 0, 0));
}

void loop() {
  read_serial();

  if (has_battery_data && millis() - last_data_ms > DATA_TIMEOUT_MS) {
    has_battery_data = false;
  }

  if (!has_battery_data && millis() - last_status_toggle_ms >= 500) {
    last_status_toggle_ms = millis();
    status_led_on = !status_led_on;
    show_color(status_led_on ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0));
  }
}
