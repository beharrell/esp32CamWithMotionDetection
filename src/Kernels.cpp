#include "Kernels.h"

bool BinaryErrode::Process(uint8_t pixel)
{
    mResult = pixel == 0 ? 0 : 255;
    return mResult == 0;
}

bool LayeredDilate::Process(uint8_t pixel)
{
    mResult = pixel != 0 ? 0x80 : 0;
    return mResult != 0;
}

void LayeredDilate::UpdatePixel(uint8_t &pixel)
{
    pixel = pixel >> 1 | Result();
}

bool Gaussian::Process(uint8_t pixel)
{
    assert(mGausIdx < 9);
    mResult += mGaussian[mGausIdx] * pixel;
    return ++mGausIdx == 9;
}

uint8_t Gaussian::Result()
{
    return mResult / 16;
}

void Gaussian::Reset()
{
    Kernel::Reset();
    mGausIdx = 0;
}

void Convolve::Process(const uint16_t i, const uint16_t j)
{
    mKernel->Reset();

    for (int y = j - mHalfKernelSize; y <= j + mHalfKernelSize; ++y)
    {
        auto row = mSourceImage + (y * mMaxI);
        for (int x = i - mHalfKernelSize; x <= i + mHalfKernelSize; ++x)
        {
            auto stop = mKernel->Process(row[x]);
            if (stop)
            {
                break;
            }
        }
    }
}

void Convolve::Process(Kernel *kernel, uint8_t *sourceImage, uint8_t *destImage)
{
    mKernel = kernel;
    mSourceImage = sourceImage;
    mDestImage = destImage;
    mHalfKernelSize = mKernel->HalfKernelSize();

    for (int j = mHalfKernelSize; j < mMaxJ - mHalfKernelSize; ++j)
    {
        auto row = mDestImage + (j * mMaxI);
        for (int i = mHalfKernelSize; i < mMaxI - mHalfKernelSize; ++i)
        {
            Process(i, j);
            mKernel->UpdatePixel(row[i]);
        }
    }
}