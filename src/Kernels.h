#ifndef KERNELS
#define KERNELS
#include <Arduino.h>

class Kernel
{
private:
   int16_t mHalfKernelSize{1};

public:
  Kernel(int16_t halfKernelSize = 1):mHalfKernelSize{halfKernelSize}{}
  virtual bool Process(uint8_t pixel) = 0;
  virtual uint8_t Result() = 0;
  virtual void Reset() = 0;
  virtual void UpdatePixel(uint8_t &pixel) {pixel = Result();}
  virtual int16_t HalfKernelSize(){return mHalfKernelSize;};
};

class BinaryErrode : public Kernel
{
private:
  uint8_t mResult{0};

public:
  using Kernel::Kernel;

  bool Process(uint8_t pixel) override
  {
    mResult = pixel == 0 ? 0 : 255;
    return mResult == 0;
  }

  uint8_t Result() override { return mResult; }

  void Reset() override { mResult = 0; }
};

class LayeredDilate : public Kernel
{
private:
  uint8_t mResult{0};

public:
  using Kernel::Kernel;

  bool Process(uint8_t pixel) override
  {
    mResult = pixel != 0 ? 0x80 : 0;
    return mResult != 0;
  }

  uint8_t Result() override { return mResult; }

  void UpdatePixel(uint8_t &pixel) {
    pixel = pixel >> 1 | Result();
  }

  void Reset() override { mResult = 0; }
};

class Gaussian : public Kernel
{
private:
  uint16_t mResult{0};
  uint8_t mGausIdx{0};
  const int mGaussian[9] = {1, 2, 1,
                           2, 4, 2,
                           1, 2, 1};

public:
  Gaussian():Kernel(1){}

  bool Process(uint8_t pixel) override
  {
    assert(mGausIdx < 9);
    mResult += mGaussian[mGausIdx] * pixel;
    return ++mGausIdx == 9;
  }

  uint8_t Result() override { return mResult / 16; }

  void Reset() override
  {
    mResult = 0;
    mGausIdx = 0;
  }
};

class Convolve
{
private:
  const int mMaxI;
  const int mMaxJ;
  Kernel *mKernel;
  uint8_t *mSourceImage;
  uint8_t *mDestImage;
  int16_t halfKernelSize;

  void Process(const uint16_t i, const uint16_t j)
  {
    mKernel->Reset();

    for (int y = j - halfKernelSize; y <= j + halfKernelSize; ++y)
    {
      auto row = mSourceImage + (y * mMaxI);
      for (int x = i - halfKernelSize; x <= i + halfKernelSize; ++x)
      {
        auto stop = mKernel->Process(row[x]);
        if (stop)
        {
          break;
        }
      }
    }
  }

  public:
  Convolve(int maxI, int maxJ):mMaxI{maxI}, mMaxJ{maxJ}{}

  void Process(Kernel *kernel, uint8_t *sourceImage, uint8_t *destImage)
  {
    mKernel = kernel;
    mSourceImage = sourceImage;
    mDestImage = destImage;
    halfKernelSize = mKernel->HalfKernelSize();

    for (int j = halfKernelSize; j < mMaxJ - halfKernelSize; ++j)
    {
      auto row = mDestImage + (j * mMaxI);
      for (int i = halfKernelSize; i < mMaxI - halfKernelSize; ++i)
      {
        Process(i, j);
        mKernel->UpdatePixel(row[i]);
      }
    }

  }
};


#endif
