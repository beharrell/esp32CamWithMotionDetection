#include "MotionDetector.h"

MotionDetector::MotionDetector(int maxI, int maxJ) : mTotalPixels{maxI * maxJ}, mConvolve{maxI, maxJ}
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
        LOGD("Failed to alloc frames\n");
    }
    Reset();
}

MotionDetector::~MotionDetector()
{
    free(mRgbFrame);
    free(mLastImage);
    free(mIgnoreMask);
    free(mCurrentImage);
}

void MotionDetector::BlurCurrentImage()
{
    Gaussian kernel;
    mConvolve.Process(&kernel, mCurrentImage, mThresholdImage);

    memcpy(mCurrentImage, mThresholdImage, mTotalPixels);
    memset(mThresholdImage, 0, mTotalPixels);
}

void MotionDetector::UpdateIgnoreMask()
{
    LayeredDilate kernel(mConfig.mHalfDilationKernelSize);
    mConvolve.Process(&kernel, mErrodedImage, mIgnoreMask);
}

void MotionDetector::ErrodeThreshold()
{
    BinaryErrode kernel(mConfig.mHalfErrodeKernelSize);
    mConvolve.Process(&kernel, mThresholdImage, mErrodedImage);
}

int MotionDetector::Count1sInByte(uint8_t b)
{
    int count{0};
    while (b)
    {
        count += (b & 0x01);
        b = b >> 1;
    }

    return count;
}

void MotionDetector::Reset()
{
    mHaveBacking = false;
    memset(mLastImage, 0, mTotalPixels);
}

bool MotionDetector::FoundMovement(camera_fb_t *frame)
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
            *(thresholdPixel++) = (*(diffPixel++) > mConfig.mThreshold) ? 255 : 0;
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
            if (*(errodedPixel++) != 0 && Count1sInByte(*(ignorePixel++)) < mConfig.mIgnorePixelCount)
            {
                ++numPixelsIndicatingMovement;
                foundMovement = numPixelsIndicatingMovement > mConfig.mMovementPixelCount;
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