#ifndef KERNELS
#define KERNELS
#include <Arduino.h>

class Kernel
{
private:
   int16_t mHalfKernelSize{1};

protected:
  uint8_t mResult{0};

public:
  Kernel(int16_t halfKernelSize = 1):mHalfKernelSize{halfKernelSize}{}
  virtual ~Kernel(){};

  virtual bool Process(uint8_t pixel) = 0;
  virtual uint8_t Result() { return mResult; }
  virtual void Reset(){mResult = 0;}
  virtual void UpdatePixel(uint8_t &pixel) {pixel = Result();}
  virtual int16_t HalfKernelSize(){return mHalfKernelSize;};
};

class BinaryErrode : public Kernel
{
public:
  using Kernel::Kernel;

  bool Process(uint8_t pixel) override;
};

class LayeredDilate : public Kernel
{
public:
  using Kernel::Kernel;

  bool Process(uint8_t pixel) override;
  void UpdatePixel(uint8_t &pixel);
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

  bool Process(uint8_t pixel) override;
  uint8_t Result() override;
  void Reset() override;
};

class Convolve
{
private:
  const int mMaxI;
  const int mMaxJ;
  Kernel *mKernel;
  uint8_t *mSourceImage;
  uint8_t *mDestImage;
  int16_t mHalfKernelSize;

  void Process(const uint16_t i, const uint16_t j);

  public:
  Convolve(int maxI, int maxJ):mMaxI{maxI}, mMaxJ{maxJ}{}

  void Process(Kernel *kernel, uint8_t *sourceImage, uint8_t *destImage);
};


#endif
