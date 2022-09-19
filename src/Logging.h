#ifndef LOGGING_H
#define LOGGING_H

// must be placed before other project includes

#ifdef DEBUG
#define LOGD(format, ...) Serial.printf((format), ##__VA_ARGS__)
#define LOGE(format, ...) Serial.printf((format), ##__VA_ARGS__)
#else
#define LOGD(format, ...)
#define LOGE(format...)
#endif

#endif