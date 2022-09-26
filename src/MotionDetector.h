#ifndef MOTIONDETECTOR
#define MOTIONDETECTOR
#include <Arduino.h>
#include "esp_camera.h"

#include "Logging.h"
#include "Kernels.h"


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
  bool mHaveLastImage{false};
  Convolve mConvolve;

  void BlurCurrentImage();
  void ConvertJpegToGrayScale(camera_fb_t *frame);
  void DiffCurrentWithLastAndThreshold();
  bool FindMovementPixels();
  void UpdateIgnoreMask();
  void ErrodeThreshold();
  int ByteHasAtLeastN1s(uint8_t b, uint8_t numberOf1s);
  void CopyCurrentImageToLast();

public:
  struct Config
  {
    int mThreshold{50};
    int mMovementPixelCount{10};
    int mIgnorePixelCount{2};
    int mHalfDilationKernelSize{3};
    int mHalfErrodeKernelSize{2};
  };

  Config mConfig;
 
  MotionDetector(int maxI, int maxJ);
  ~MotionDetector();
  void Reset();
  bool FoundMovement(camera_fb_t *frame);

  uint8_t * LastImage(){return mLastImage;}
  uint8_t * CurrentImage(){return mCurrentImage;}
  uint8_t * ThresholdImage(){return mThresholdImage;}
  uint8_t * DiffImage(){return mDiffImage;}
  uint8_t * ErrodedImage(){return mErrodedImage;}
  uint8_t * IgnoreMask(){return mIgnoreMask;}
};


#endif