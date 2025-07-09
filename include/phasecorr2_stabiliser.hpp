// =============================================
// phasecorr2_stabiliser.hpp
// Drop‑in replacement implementing Guizar‑Sicairos
// efficient sub‑pixel phase correlation with
//   * cached reference FFT (only one FFT per frame)
//   * 3×3 quadratic peak refinement (≈1⁄100 px)
//   * optional exponential rolling reference update
// Reference: M. Guizar-Sicairos, S. T. Thurman, and J. R. Fienup. "Efficient subpixel image registration algorithms." Optics Letters 33 (2), 156–158 (2008). DOI: 10.1364/OL.33.000156.
// =============================================

#ifndef PHASECORR2_STABILISER_HPP
#define PHASECORR2_STABILISER_HPP

#include <opencv2/core.hpp>

/*!
 * \brief  Lightweight translation‑only image stabiliser.
 *
 * Usage pattern (exactly matches the previous class):
 *     PhaseCorrStabiliser stab;
 *     stab.initReference(firstGrayFrame);
 *     for(each new frame) {
 *         cv::Point2f shift = stab.computeShift(frame);
 *     }
 *
 * The constructor lets you choose the sub‑pixel up‑sampling factor (≥1)
 * and the reference update weight (0 → fixed reference, 1 → fully rolling).
 */
class PhaseCorrStabiliser2
{
public:
    explicit PhaseCorrStabiliser2(int upsampleFactor = 8, double referenceUpdate = 0.02);

    /// sets the very first frame as the reference (grayscale 8‑bit or 32‑bit).
    void initReference(const cv::Mat &frame);

    /** \return (dx, dy) shift of <frame> w.r.t. the internal reference.
     *  Positive values mean the new frame is displaced right/down.
     *  After computation the reference is updated using an exponential
     *  moving average with weight <referenceUpdate> passed in constructor. */
    cv::Point2f computeShift(const cv::Mat &frame);

    void reset();

private:
    // helpers -------------------------------------------------------------
    static void complexMul(const cv::Mat &a, const cv::Mat &b, cv::Mat &dst, bool conjB);
    cv::Point2f subpixelRefine(const cv::Point &pk) const;
    inline float corrVal(int y, int x) const;

    // parameters ----------------------------------------------------------
    int upsample_;
    double alpha_; // reference exponential decay 0..1

    // cached data ---------------------------------------------------------
    bool refSet_{false};
    cv::Size fftSize_;
    cv::Mat hannWin_;  // CV_32F, same size as input frames
    cv::Mat refFloat_; // CV_32F windowed reference (spatial domain)
    cv::Mat refFFT_;   // CV_32FC2

    // scratch buffers to avoid re‑allocation ------------------------------
    mutable cv::Mat curFloat_, curFFT_, cps_, corr_;
};

#endif