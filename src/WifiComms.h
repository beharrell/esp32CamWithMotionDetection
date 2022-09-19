#ifndef WIFI_COMMS_H
#define WIFI_COMMS_H

#include <Arduino.h>
#include <WiFi.h>
#include "FS.h"

#include "Logging.h"

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

    String GetParams()
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
        }
        else
        {
            getBody = "Connection to " + serverName + " failed.";
            LOGE("%s\n", getBody.c_str());
            getBody = "{}";
        }

        return getBody;
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