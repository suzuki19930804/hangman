#include "esp_camera.h"
#include <WiFi.h>
#include <SPI.h>
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
#define CAMERA_MODEL_WROVER_KIT  // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"

const uint8_t SCLK_OLED = 18;         //SCLK
const uint8_t MOSI_OLED = 23;         //MOSI (Master Output Slave Input)
const uint8_t Fsync_PIN = 14;         //CS (Chip Select)
const int SINE = 0b0010000000000000;  //0x2000
const float refFreq = 25000000.0;
const char* ssid = "ASUS_69";
const char* password = "nedoamed309";


void startCameraServer();
SPIClass spi(HSPI);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  spi.begin(SCLK_OLED, 12, MOSI_OLED, Fsync_PIN);
  spi.setFrequency(40000000);
  spi.setDataMode(SPI_MODE2);
  pinMode(Fsync_PIN, OUTPUT);
  digitalWrite(Fsync_PIN, HIGH);
  Control_Resister_Write(0b0000000100000000);
  delay(50);
  AD9833_SetFrequency(200, SINE);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
  ///*
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  //startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  //*/
  /*
  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("IP Address as Access Point: ");
  Serial.println(ip);
*/
  pinMode(33, ANALOG);
  pinMode(32, ANALOG);


  startCameraServer();
}



void loop() {
  // put your main code here, to run repeatedly:

  delay(10000);
}

void AD9833_SetFrequency(uint32_t frequency, int Waveform) {
  uint32_t FreqWord = (frequency * pow(2, 28)) / refFreq;

  uint16_t MSB = (uint16_t)((FreqWord & 0xFFFC000) >> 14);
  uint16_t LSB = (uint16_t)(FreqWord & 0x3FFF);

  LSB |= 0b0100000000000000;  //=0x4000
  MSB |= 0b0100000000000000;  //=0x4000

  Control_Resister_Write(0b0010000000000000);  //制御ワード書き込み
  Control_Resister_Write(LSB);
  Control_Resister_Write(MSB);

  Control_Resister_Write(0b1100000000000000);  //位相シフトはゼロ
  Control_Resister_Write(Waveform);
}

void Control_Resister_Write(uint16_t b) {
  digitalWrite(Fsync_PIN, LOW);
  spi.transfer16(b);
  digitalWrite(Fsync_PIN, HIGH);
}



