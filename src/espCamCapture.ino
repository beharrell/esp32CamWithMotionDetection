#include <Arduino.h>
#include <ArduinoJson.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#define DEBUG
#include "Logging.h"
#include "WiFiComms.h"
#include "MotionDetector.h"

#define CAMERA_MODEL_AI_THINKER

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
String imageNameStart = "/im_";
String uniqueId;

struct Config
{
  uint64_t mDetectionSleepTimeUs{1000000};
  uint64_t mMonitoringSleepTimeUs{2000000};
  int mMonitor_numberOfFramesSavedBeforeUpload{5};
  int mDetection_numberOfFramesSavedBeforeUpload{10};
  bool mSaveIntermidiateImages{true};
  String mImageName{"time"};
};
Config config;

#define FRAMESIZE FRAMESIZE_QQVGA
#define FRAMESIZE_WIDTH 160
#define FRAMESIZE_HEIGHT 120
static constexpr int mTotalPixels = FRAMESIZE_WIDTH * FRAMESIZE_HEIGHT;
bool detectedMotion{false};
MotionDetector *motionDetector;
WifiComms comms;

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
    config.jpeg_quality = detectedMotion ? 10 : 5;
    config.fb_count = 1;
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
#ifdef DEBUG
  config.mMonitoringSleepTimeUs = 2000000;
  config.mMonitor_numberOfFramesSavedBeforeUpload = 5;
  config.mDetection_numberOfFramesSavedBeforeUpload = 10;
#endif
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  setupSerial();
  comms.setupWifi();
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

bool uploadImages()
{
  if (!comms.setupWifi())
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
      uploadedOk = comms.sendImage(file);
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

  UpdateParams();
  comms.stopWifi();
  return uploadedOk;
}

void UpdateParams()
{
  String params = comms.GetParams();
  auto &motionConfig = motionDetector->mConfig;
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, params.c_str());
  JsonObject obj = doc.as<JsonObject>();
  LOGD("Updating params\n");
  if (obj.containsKey("time"))
  {
    config.mImageName = obj["time"].as<String>();
    LOGD("Set mImageName %d\n", config.mImageName);
  }

  if (obj.containsKey("saveIntermidiate"))
  {
    config.mSaveIntermidiateImages = obj["saveIntermidiate"].as<bool>();
    LOGD("Set mSaveIntermidiateImages %d\n", config.mSaveIntermidiateImages);
  }

  if (obj.containsKey("monitorUploadCount"))
  {
    config.mMonitor_numberOfFramesSavedBeforeUpload = obj["monitorUploadCount"].as<int>();
    LOGD("Set monitorUploadCount %d\n", config.mMonitor_numberOfFramesSavedBeforeUpload);
  }

  if (obj.containsKey("monitorSleepTime"))
  {
    config.mMonitoringSleepTimeUs = obj["monitorSleepTime"].as<int>();
    LOGD("Set monitorSleepTime %d\n", config.mMonitoringSleepTimeUs);
  }

  if (obj.containsKey("detectionUploadCount"))
  {
    config.mDetection_numberOfFramesSavedBeforeUpload = obj["detectionUploadCount"].as<int>();
    LOGD("Set detectionUploadCount %d\n", config.mDetection_numberOfFramesSavedBeforeUpload);
  }

  if (obj.containsKey("detectionSleepTime"))
  {
    config.mDetectionSleepTimeUs = obj["detectionSleepTime"].as<int>();
    LOGD("Set detectionSleepTime %d\n", config.mDetectionSleepTimeUs);
  }

  if (obj.containsKey("threshold"))
  {
    motionConfig.mThreshold = obj["threshold"].as<int>();
    LOGD("Set threshold %d\n", motionConfig.mThreshold);
  }

  if (obj.containsKey("movementPixelCount"))
  {
    motionConfig.mMovementPixelCount = obj["movementPixelCount"].as<int>();
    LOGD("Set movementPixelCount %d\n", motionConfig.mMovementPixelCount);
  }

  if (obj.containsKey("ignorePixelCount"))
  {
    motionConfig.mIgnorePixelCount = obj["ignorePixelCount"].as<int>();
    LOGD("Set ignorePixelCount %d\n", motionConfig.mIgnorePixelCount);
  }

  if (obj.containsKey("halfDilationKernelSize"))
  {
    motionConfig.mHalfDilationKernelSize = obj["halfDilationKernelSize"].as<int>();
    LOGD("Set halfDilationKernelSize %d\n", motionConfig.mHalfDilationKernelSize);
  }

  if (obj.containsKey("halfErrodeKernelSize"))
  {
    motionConfig.mHalfErrodeKernelSize = obj["halfErrodeKernelSize"].as<int>();
    LOGD("Set halfErrodeKernelSize %d\n", motionConfig.mHalfErrodeKernelSize);
  }

  LOGD("params response .... %s\n", params.c_str());
}

void saveJpgImage(camera_fb_t *frame, bool detectedImage)
{
  if (frame->format == PIXFORMAT_JPEG)
  {
    saveJpgImage(frame->buf, frame->len, detectedImage ? "dect" : "current");
  }
}

void saveJpgImage(const uint8_t *buffer, const int32_t bufferLen, const String type)
{
  imageNum++;
  String path = imageNameStart + uniqueId + config.mImageName + String(imageNum) + type + ".jpg";

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

  auto sleepTime = detectedMotion ? config.mDetectionSleepTimeUs : config.mMonitoringSleepTimeUs;
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
    auto wasDetected = detectedMotion;
    detectedMotion = detectedMotion || motionDetector->foundMovement(frame);
    if (config.mSaveIntermidiateImages && !wasDetected)
    {
      SaveIntermidiateImages();
    }
    saveJpgImage(frame, detectedMotion);
    const auto numFramesBeforeUpload = detectedMotion ? config.mDetection_numberOfFramesSavedBeforeUpload : config.mMonitor_numberOfFramesSavedBeforeUpload;
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
