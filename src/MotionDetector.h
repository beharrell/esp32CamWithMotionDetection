#ifndef MOTIONDETECTOR
#define MOTIONDETECTOR
#include <Arduino.h>
#include "Kernels.h"
#include "esp_camera.h"

class MotionDetector
{
private:
  const int mTotalPixels;
  uint8_t *mLastImage;
  uint8_t *mCurrentImage;
  uint8_t *mThresholdImage; // ptr into rgb
  uint8_t *mDiffImage;      // ptr into rgb
  uint8_t *mErrodedImage;   // ptr into rgb
  uint8_t *mRgbFrame;
  uint8_t *mIgnoreMask;
  bool mHaveBacking{false};
  Convolve mConvolve;

  void BlurCurrentImage()
  {
    Gaussian kernel;
    mConvolve.Process(&kernel, mCurrentImage, mThresholdImage);

    memcpy(mCurrentImage, mThresholdImage, mTotalPixels);
    memset(mThresholdImage, 0, mTotalPixels);
  }

  void UpdateIgnoreMask()
  {
    LayeredDilate kernel(3);
    mConvolve.Process(&kernel, mErrodedImage, mIgnoreMask);
  }

  void ErrodeThreshold()
  {
    BinaryErrode kernel(2);
    mConvolve.Process(&kernel, mThresholdImage, mErrodedImage);
  }

  int Count1sInByte(uint8_t b)
  {
    int count{0};
    while (b)
    {
      count += (b & 0x01);
      b = b >> 1;
    }

    return count;
  }

public:
  MotionDetector(int maxI, int maxJ) : mTotalPixels{maxI * maxJ}, mConvolve{maxI, maxJ}
  {
    mRgbFrame = reinterpret_cast<uint8_t *>(malloc(mTotalPixels * 3));
    mThresholdImage = mRgbFrame;
    mDiffImage = mRgbFrame + mTotalPixels;
    mErrodedImage = mDiffImage + mTotalPixels;
    mLastImage = reinterpret_cast<uint8_t *>(malloc(mTotalPixels));
    mIgnoreMask = reinterpret_cast<uint8_t *>(malloc(mTotalPixels));
    mCurrentImage = reinterpret_cast<uint8_t *>(malloc(mTotalPixels));
    memset(mIgnoreMask, 0, mTotalPixels);

    if (!mRgbFrame || !mLastImage || !mIgnoreMask || !mCurrentImage)
    {
     // LOGD("Failed to alloc frames\n");
    }
    Reset();
  }

  ~MotionDetector()
  {
    free(mRgbFrame);
    free(mLastImage);
    free(mIgnoreMask);
    free(mCurrentImage);
  }

  void Reset()
  {
    mHaveBacking = false;
    memset(mLastImage, 0, mTotalPixels);
  }

  bool foundMovement(camera_fb_t *frame)
  {
    assert(frame->format == PIXFORMAT_JPEG);
    fmt2rgb888(frame->buf, frame->len, frame->format, mRgbFrame);
    auto currentPixel = mCurrentImage;
    auto currentPixelEnd = mCurrentImage + mTotalPixels;
    auto rgbByte = mRgbFrame;
    while (currentPixel < currentPixelEnd)
    {
      // luminosity method
      *(currentPixel++) = round((0.3f * static_cast<float>(*(rgbByte++))) +
                              (0.59f * static_cast<float>(*(rgbByte++))) +
                              (0.11f * static_cast<float>(*(rgbByte++))));
    }
    memset(mRgbFrame, 0, mTotalPixels * 3);
    BlurCurrentImage();

    if (mHaveBacking)
    {
      auto currentPixel = mCurrentImage;
      auto currentPixelEnd = mCurrentImage + mTotalPixels;
      auto diffPixel = mDiffImage;
      auto lastPixel = mLastImage;
      auto thresholdPixel = mThresholdImage;
      while (currentPixel < currentPixelEnd)
      {
        *diffPixel = abs(static_cast<int>(*(lastPixel++)) - static_cast<int>(*(currentPixel++)));
        *(thresholdPixel++) = (*(diffPixel++) > 50) ? 255 : 0;
      }
    }

    bool foundMovement{false};
    if (mHaveBacking)
    {
      ErrodeThreshold();
      int numPixelsIndicatingMovement{0};
      auto errodedPixel = mErrodedImage;
      auto errodedPixelEnd = mErrodedImage + mTotalPixels;
      auto ignorePixel = mIgnoreMask;
      while (errodedPixel < errodedPixelEnd)
      {
        if (*(errodedPixel++) != 0 && Count1sInByte(*(ignorePixel++)) < 2)
        {
          ++numPixelsIndicatingMovement;
          foundMovement = numPixelsIndicatingMovement > 10;
          if (foundMovement)
          {
            break;
          }
        }
      }
      UpdateIgnoreMask();
    }
    memcpy(mLastImage, mCurrentImage, mTotalPixels);
    mHaveBacking = true;
    return foundMovement;
  }

  uint8_t * LastImage(){return mLastImage;}
  uint8_t * CurrentImage(){return mCurrentImage;}
  uint8_t * ThresholdImage(){return mThresholdImage;}
  uint8_t * DiffImage(){return mDiffImage;}
  uint8_t * ErrodedImage(){return mErrodedImage;}
  uint8_t * IgnoreMask(){return mIgnoreMask;}
};


#endif