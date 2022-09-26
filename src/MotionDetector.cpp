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

int MotionDetector::ByteHasAtLeastN1s(uint8_t b, uint8_t numberOf1s)
{
    while (b && numberOf1s)
    {
        numberOf1s -= (b & 0x01);
        b = b >> 1;
    }

    return numberOf1s == 0;
}

void MotionDetector::Reset()
{
    mHaveLastImage = false;
    memset(mLastImage, 0, mTotalPixels);
}

void MotionDetector::DiffCurrentWithLastAndThreshold()
{
    for (int i = 0; i < mTotalPixels; ++i)
    {
        mDiffImage[i] = abs(static_cast<int>(mLastImage[i]) - static_cast<int>(mCurrentImage[i]));
        mThresholdImage[i] = (mDiffImage[i] > mConfig.mThreshold) ? 255 : 0;
    }
}

bool MotionDetector::FindMovementPixels()
{
    int numPixelsIndicatingMovement{0};
    bool foundMovement{false};

    for (int i = 0; i < mTotalPixels; ++i)
    {
        if (mErrodedImage[i] != 0 && ByteHasAtLeastN1s(mIgnoreMask[i], mConfig.mIgnorePixelCount))
        {
            ++numPixelsIndicatingMovement;
            foundMovement = numPixelsIndicatingMovement > mConfig.mMovementPixelCount;
            if (foundMovement)
            {
                break;
            }
        }
    }

    return foundMovement;
}

void MotionDetector::ConvertJpegToGrayScale(camera_fb_t *frame)
{
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
}

void MotionDetector::CopyCurrentImageToLast()
{
    memcpy(mLastImage, mCurrentImage, mTotalPixels);
    mHaveLastImage = true;
}

bool MotionDetector::FoundMovement(camera_fb_t *frame)
{
    assert(frame->format == PIXFORMAT_JPEG);
    ConvertJpegToGrayScale(frame);
    BlurCurrentImage();

    bool foundMovement{false};
    if (mHaveLastImage)
    {
        DiffCurrentWithLastAndThreshold();
        ErrodeThreshold();
        foundMovement = FindMovementPixels();
        UpdateIgnoreMask();
    }
    CopyCurrentImageToLast();

    return foundMovement;
}