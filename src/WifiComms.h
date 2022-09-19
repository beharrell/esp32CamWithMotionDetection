#ifndef WIFI_COMMS_H
#define WIFI_COMMS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "FS.h"

#include "Logging.h"
#include "MotionDetector.h"

struct Config
{
  uint64_t mDetectionSleepTimeUs{1000000};
  uint64_t mMonitoringSleepTimeUs{2000000};
  int mMonitor_numberOfFramesSavedBeforeUpload{5};
  int mDetection_numberOfFramesSavedBeforeUpload{10};
  bool mSaveIntermidiateImages{true};
  String mImageName{"time"};
};

class WifiComms
{
    WiFiClient client;
    const char *ssid = "Kiwilink2.4GHz";
    const char *password = "kiwi01wifi";

    String serverName = "192.168.1.118";
    String serverPath = "/profile-upload-single";
    const int serverPort = 3000;

public:
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
            LOGD(".");
            if (++retryCount > 30)
            {
                LOGE("Wifi failed to start\n");
                return false;
            }
        }

        LOGD("ESP32-CAM IP Address: %s", WiFi.localIP().toString().c_str());
        return true;
    }

    void GetParams(Config &config, MotionDetector::Config &motionConfig)
    {   
        String getAll;
        String getBody;
        LOGD("Getting params\n");
        bool connectedOk = client.connect(serverName.c_str(), serverPort);
        if (connectedOk)
        {
            LOGD("Connection successful!\n");
            client.println("GET /params HTTP/1.1");
            client.println("Host: " + serverName);
            client.println("Connection: close");
            client.println("");

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
            DynamicJsonDocument doc(1024);
            deserializeJson(doc, getBody.c_str());
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

            LOGD("params response .... %s\n", getBody.c_str());
        }
        else
        {
            getBody = "Connection to " + serverName + " failed.";
            LOGE("%s\n", getBody.c_str());
        }
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
            // imageName = getBody;
            // imageName.trim();
        }
        else
        {
            getBody = "Connection to " + serverName + " failed.";
            LOGE("%s\n", getBody.c_str());
        }
        return connectedOk;
    }

    void stopWifi()
    {
        WiFi.disconnect(true);
    }
};
#endif