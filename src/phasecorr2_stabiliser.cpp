// =============================================
// phasecorr2_stabiliser.cpp
// =============================================

#include "phasecorr2_stabiliser.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

using cv::Mat;
using cv::Point;
using cv::Point2f;

// ---------- ctor ---------------------------------------------------------
PhaseCorrStabiliser2::PhaseCorrStabiliser2(int upsampleFactor, double referenceUpdate)
    : upsample_(std::max(1, upsampleFactor)),
      alpha_(std::clamp(referenceUpdate, 0.0, 1.0))
{
}

// ---------- misc helpers -------------------------------------------------
void PhaseCorrStabiliser2::complexMul(const Mat &a, const Mat &b, Mat &dst, bool conjB)
{
    cv::mulSpectrums(a, b, dst, /*flags=*/0, /* conjB = */ conjB);
}

inline float PhaseCorrStabiliser2::corrVal(int y, int x) const
{
    // periodic (modulo) addressing – correlation is circular
    int yy = (y + fftSize_.height) % fftSize_.height;
    int xx = (x + fftSize_.width) % fftSize_.width;
    return corr_.at<float>(yy, xx);
}

// quadratic 3×3 peak refinement (sub‑pixel)
Point2f PhaseCorrStabiliser2::subpixelRefine(const Point &pk) const
{
    // guard border – corr_ is circular so values always exist
    float c = corrVal(pk.y, pk.x);
    float cx = corrVal(pk.y, pk.x + 1);
    float cxm = corrVal(pk.y, pk.x - 1);
    float cy = corrVal(pk.y + 1, pk.x);
    float cym = corrVal(pk.y - 1, pk.x);

    float denomX = 2 * (2 * c - cx - cxm);
    float denomY = 2 * (2 * c - cy - cym);
    float dx = (denomX != 0.f) ? (cx - cxm) / denomX : 0.f;
    float dy = (denomY != 0.f) ? (cy - cym) / denomY : 0.f;

    return Point2f(dx, dy); // |dx|,|dy| ≤ 0.5
}

// ---------- public API ---------------------------------------------------
void PhaseCorrStabiliser2::initReference(const Mat &frameIn)
{
    CV_Assert(!frameIn.empty());

    // Convert to single‑channel float32
    Mat gray;
    if (frameIn.channels() == 3)
        cv::cvtColor(frameIn, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frameIn;

    gray.convertTo(refFloat_, CV_32F);
    fftSize_ = refFloat_.size();

    // Build (and cache) Hanning window – same size as frames
    cv::createHanningWindow(hannWin_, fftSize_, CV_32F);
    cv::multiply(refFloat_, hannWin_, refFloat_);

    // FFT once – cached for all subsequent calls
    cv::dft(refFloat_, refFFT_, cv::DFT_COMPLEX_OUTPUT);

    refSet_ = true;
}

// ------------------------------------------------------------------------
Point2f PhaseCorrStabiliser2::computeShift(const Mat &frameIn)
{
    CV_Assert(refSet_ && "initReference() must be called first");
    CV_Assert(!frameIn.empty());

    // --- 1. prepare current frame (gray → float32 → window) -------------
    if (frameIn.channels() == 3)
        cv::cvtColor(frameIn, curFloat_, cv::COLOR_BGR2GRAY);
    else
        frameIn.copyTo(curFloat_);

    curFloat_.convertTo(curFloat_, CV_32F);
    cv::multiply(curFloat_, hannWin_, curFloat_);

    // --- 2. FFT (only one per frame!) -----------------------------------
    cv::dft(curFloat_, curFFT_, cv::DFT_COMPLEX_OUTPUT);

    // --- 3. cross‑power spectrum ----------------------------------------
    complexMul(curFFT_, refFFT_, cps_, /* conjB = */ true);

    // normalise to unit magnitude (phase‑only)
    Mat planes[2];
    cv::split(cps_, planes);
    Mat mag;
    cv::magnitude(planes[0], planes[1], mag);
    mag += 1e-12f; // avoid /0
    planes[0] /= mag;
    planes[1] /= mag;
    cv::merge(planes, 2, cps_);

    // --- 4. inverse FFT → correlation map -------------------------------
    cv::dft(cps_, corr_, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    double maxVal;
    Point pk;
    cv::minMaxLoc(corr_, /*min*/ nullptr, &maxVal, /*minLoc*/ nullptr, &pk);

    // --- 5. coarse integer shift (wrap‑around aware) --------------------
    Point2f shift(pk.x, pk.y);
    if (shift.x > fftSize_.width / 2)
        shift.x -= fftSize_.width;
    if (shift.y > fftSize_.height / 2)
        shift.y -= fftSize_.height;

    // --- 6. sub‑pixel refinement (quadratic) ----------------------------
    Point2f frac = subpixelRefine(pk);
    shift += frac;

    // --- 7. reference exponential update --------------------------------
    if (alpha_ > 0.0)
    {
        cv::accumulateWeighted(curFloat_, refFloat_, alpha_);
        cv::dft(refFloat_, refFFT_, cv::DFT_COMPLEX_OUTPUT);
    }

    return shift; // (dx, dy)   positive = right/down
}

// ------------------------------------------------------------------------
void PhaseCorrStabiliser2::reset()
{
    refSet_ = false;
    hannWin_.release();
    refFloat_.release();
    refFFT_.release();
}
