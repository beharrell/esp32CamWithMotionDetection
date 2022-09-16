#include <Arduino.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "MotionDetector.h"

#define CAMERA_MODEL_AI_THINKER
#include "EloquentVision.h"
const char *ssid = "Kiwilink2.4GHz";
const char *password = "kiwi01wifi";

String serverName = "192.168.1.118";          // REPLACE WITH YOUR Raspberry Pi IP ADDRESS
String serverPath = "/profile-upload-single"; // The default serverPath should be upload.php

const int serverPort = 3000;

WiFiClient client;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

RTC_DATA_ATTR int imageNum = 0;
String imageName = "time";
String imageNameStart = "/im_";
String uniqueId;
constexpr uint64_t detectionSleepTimeUs = 1000000;

//#define DEBUG
#ifdef DEBUG
constexpr uint64_t monitoringSleepTimeUs = 2000000;
constexpr int monitor_numberOfFramesSavedBeforeUpload = 5;
constexpr int detection_numberOfFramesSavedBeforeUpload = 10;
#define LOGD(format, ...) Serial.printf((format), ##__VA_ARGS__)
#define LOGE(format, ...) Serial.printf((format), ##__VA_ARGS__)
#else
uint64_t monitoringSleepTimeUs = 2000000;
constexpr int monitor_numberOfFramesSavedBeforeUpload = 5;
constexpr int detection_numberOfFramesSavedBeforeUpload = 10;
#define LOGD(format, ...)
#define LOGE(format...)
#endif

#define FRAMESIZE FRAMESIZE_QQVGA
#define FRAMESIZE_WIDTH 160
#define FRAMESIZE_HEIGHT 120
static constexpr int mTotalPixels = FRAMESIZE_WIDTH * FRAMESIZE_HEIGHT;
bool detectedMotion{false};
MotionDetector *motionDetector;

bool setupWifi()
{
  if (!WiFi.mode(WIFI_STA))
  {
    LOGE("Setting wifi mode failed\n");
    return false;
  }
  LOGD("Connecting to %s\n", ssid);
  WiFi.begin(ssid, password);
  int retryCount{0};
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    if (++retryCount > 30)
    {
      LOGE("Wifi failed to start\n");
      return false;
    }
  }

  LOGD("ESP32-CAM IP Address: %s", WiFi.localIP().toString().c_str());
  return true;
}

void stopWifi()
{
  WiFi.disconnect(true);
}

void setupCamera()
{
  auto frameSize = detectedMotion ? FRAMESIZE_VGA : FRAMESIZE;

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

  // init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
    config.frame_size = frameSize;
    config.jpeg_quality = 10; // 0-63 lower number means higher quality
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = frameSize;
    config.jpeg_quality = 12; // 0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    LOGD("Camera init failed with error %d", err);
    delay(1000);
    ESP.restart();
  }
  sensor_t *s = esp_camera_sensor_get();

  s->set_whitebal(s, 1);                   // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);                   // 0 = disable , 1 = enable
  s->set_aec2(s, 1);                       // 0 = disable , 1 = enable
  s->set_ae_level(s, 2);                   // -2 to 2
  s->set_gain_ctrl(s, 1);                  // 0 = disable , 1 = enable
  s->set_gainceiling(s, (gainceiling_t)1); // 0 to 6
  // s->set_bpc(s, 1);                        // 0 = disable , 1 = enable
  // s->set_wpc(s, 1);                        // 0 = disable , 1 = enable
}

void stopCamera()
{
  esp_camera_deinit();
}

void setupSdCard()
{
  if (!SD_MMC.begin("/sdcard", true))
  {
    LOGE("SD Card Mount Failed\n");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    LOGE("No SD Card attached\n");
    return;
  }
}

void stopSdCard()
{
  SD_MMC.end();
}

int dummyLogSink(const char *, va_list)
{
  return 0;
}

void setupSerial()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.setDebugOutput(1);
  Serial.setDebugOutput(0);
  delay(10);
#else
  esp_log_set_vprintf(dummyLogSink);
#endif
}

void stopSerial()
{
#ifdef DEBUG
  Serial.end();
#endif
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  setupSerial();
  setupWifi();
  setupCamera();
  setupSdCard();
  motionDetector = new MotionDetector(FRAMESIZE_WIDTH, FRAMESIZE_HEIGHT);

  uint8_t macAddress[6];
  esp_efuse_mac_get_default(macAddress);
  char macAsString[(sizeof(macAddress) * 2) + 1];
  sprintf(macAsString, "%x%x%x%x%x%x", macAddress[0], macAddress[1], macAddress[2],
          macAddress[3], macAddress[4], macAddress[5]);
  uniqueId = macAsString;
}

camera_fb_t *CaptureFrame()
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    LOGE("Camera capture failed");
    // delay(1000);
    // ESP.restart();
  }

  return fb;
}

bool sendImage(File &imageFile)
{
  String getAll;
  String getBody;

  LOGD("Connecting to server: %s", serverName.c_str());
  bool connectedOk = client.connect(serverName.c_str(), serverPort);
  if (connectedOk)
  {
    LOGD("Connection successful!");
    String fileName = imageFile.name();
    String headStart = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"profile-file\"; filename=\"";
    String headEnd = "\"\r\nContent-Type: image/jpg\r\n\r\n";
    String head = headStart + fileName + headEnd;
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = imageFile.size();
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;

    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);

    constexpr size_t bufferSize{1024};
    char buffer[bufferSize];
    size_t bytesRead = imageFile.readBytes(buffer, bufferSize);
    while (bytesRead == bufferSize)
    {
      client.write(buffer, bufferSize);
      bytesRead = imageFile.readBytes(buffer, bufferSize);
    }
    client.write(buffer, bytesRead);
    client.print(tail);

    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + timoutTimer) > millis())
    {
      delay(100);
      while (client.available())
      {
        char c = client.read();
        if (c == '\n')
        {
          if (getAll.length() == 0)
          {
            state = true;
          }
          getAll = "";
        }
        else if (c != '\r')
        {
          getAll += String(c);
        }
        if (state == true)
        {
          getBody += String(c);
        }
        startTimer = millis();
      }
      if (getBody.length() > 0)
      {
        break;
      }
    }
    client.stop();
    LOGD("%s\n", getBody.c_str());
    imageName = getBody;
    imageName.trim();
  }
  else
  {
    getBody = "Connection to " + serverName + " failed.";
    LOGE("%s\n", getBody.c_str());
  }
  return connectedOk;
}

bool uploadImages()
{
  if (!setupWifi())
  {
    return false;
  }
  bool uploadedOk{true};
  fs::FS &fs = SD_MMC;
  File dir = fs.open("/");
  File file = dir.openNextFile();
  while (file)
  {
    String fileToDelete = file.name();
    bool sendFile = fileToDelete.startsWith(imageNameStart);
    if (sendFile)
    {
      uploadedOk = sendImage(file);
      file.close();
      if (!uploadedOk)
      {
        break;
      }
      bool removed = fs.remove(fileToDelete);
      LOGD("%s %s\n", (removed ? "Deleted" : "Failed to delete"), fileToDelete.c_str());
    }
    else
    {
      file.close();
    }

    file = dir.openNextFile();
  }
  dir.close();
  stopWifi();
  return uploadedOk;
}

void saveJpgImage(camera_fb_t *frame)
{
  if (frame->format == PIXFORMAT_JPEG)
  {
    saveJpgImage(frame->buf, frame->len, "dect");
  }
}

void saveJpgImage(const uint8_t *buffer, const int32_t bufferLen, const String type)
{
  imageNum++;
  String path = imageNameStart + uniqueId + imageName + String(imageNum) + type + ".jpg";

  fs::FS &fs = SD_MMC;
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file)
  {
    LOGE("Failed to open file in writing mode");
  }
  else
  {
    file.write(buffer, bufferLen);
    LOGD("Saved file to path: %s\n", path.c_str());
  }
  file.close();
}

void sleep()
{
  stopCamera();
  stopSdCard();
  // stopSerial();

  auto sleepTime = detectedMotion ? detectionSleepTimeUs : monitoringSleepTimeUs;
  esp_sleep_enable_timer_wakeup(sleepTime);
  esp_light_sleep_start();

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // setupSerial();
  setupSdCard();
  setupCamera();

  for (int i = 0; i < 10; ++i)
  {
    auto frame = CaptureFrame();
    if (frame)
    {
      esp_camera_fb_return(frame);
    }
    delay(100);
  }
}

void SaveIntermidiateImages()
{
      uint8_t *jpgBuffer;
      size_t jpgBufferLen;
      fmt2jpg(motionDetector->LastImage(), mTotalPixels, FRAMESIZE_WIDTH, FRAMESIZE_HEIGHT, PIXFORMAT_GRAYSCALE, 48, &jpgBuffer, &jpgBufferLen);
      saveJpgImage(jpgBuffer, jpgBufferLen, "Last");
      free(jpgBuffer);
      fmt2jpg(motionDetector->DiffImage(), mTotalPixels, FRAMESIZE_WIDTH, FRAMESIZE_HEIGHT, PIXFORMAT_GRAYSCALE, 10, &jpgBuffer, &jpgBufferLen);
      saveJpgImage(jpgBuffer, jpgBufferLen, "Diff");
      free(jpgBuffer);
      fmt2jpg(motionDetector->ThresholdImage(), mTotalPixels, FRAMESIZE_WIDTH, FRAMESIZE_HEIGHT, PIXFORMAT_GRAYSCALE, 10, &jpgBuffer, &jpgBufferLen);
      saveJpgImage(jpgBuffer, jpgBufferLen, "Thresh");
      free(jpgBuffer);
      fmt2jpg(motionDetector->ErrodedImage(), mTotalPixels, FRAMESIZE_WIDTH, FRAMESIZE_HEIGHT, PIXFORMAT_GRAYSCALE, 10, &jpgBuffer, &jpgBufferLen);
      saveJpgImage(jpgBuffer, jpgBufferLen, "Errode");
      free(jpgBuffer);
      fmt2jpg(motionDetector->IgnoreMask(), mTotalPixels, FRAMESIZE_WIDTH, FRAMESIZE_HEIGHT, PIXFORMAT_GRAYSCALE, 10, &jpgBuffer, &jpgBufferLen);
      saveJpgImage(jpgBuffer, jpgBufferLen, "Ignore");
      free(jpgBuffer);
    }

void loop()
{
  camera_fb_t *frame = CaptureFrame();
  if (frame != NULL)
  {
    detectedMotion = detectedMotion || motionDetector->foundMovement(frame);
    saveJpgImage(frame);
    const auto numFramesBeforeUpload = detectedMotion ? detection_numberOfFramesSavedBeforeUpload : monitor_numberOfFramesSavedBeforeUpload;
    if (imageNum >= numFramesBeforeUpload)
    {
      if (detectedMotion)
      {
        LOGD("End of motion sequence\n");
        detectedMotion = false;
        motionDetector->Reset();
      }

      if (uploadImages())
      {
        imageNum = 0;
      }
    }
    esp_camera_fb_return(frame);
  }

  sleep();
}
