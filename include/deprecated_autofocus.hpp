

  /**
   * @brief Computes the location of the best focus.
   *
   * Uses sharpness algorithms to determine the best focus position.
   * @param image Input image.
   * @param imgHeight Height of the image.
   * @param imgWidth Width of the image.
   * @return Best focus location.
   */
  int computeBestFocus(cv::Mat image, int imgHeight, int imgWidth);

  /**
   * @brief Computes the best focus at reduced resolution.
   *
   * Reduces the image size and applies sharpness algorithms to determine the
   * best focus.
   * @param image Input image.
   * @param imgHeight Height of the image.
   * @param imgWidth Width of the image.
   * @return Best focus location.
   */
  double computeBestFocusReduced(cv::Mat image, int imgHeight, int imgWidth);




  /**
   * @brief Computes the best focus at very reduced resolution.
   *
   * Further reduces the image size and applies sharpness algorithms to
   * determine the best focus.
   * @param image Input image.
   * @param imgHeight Height of the image.
   * @param imgWidth Width of the image.
   * @return Best focus location.
   */
  int computeBestFocusVeryReduced(cv::Mat image, int imgHeight, int imgWidth);


  std::vector<double> computeBestFocusReducedHorizontal(cv::Mat image, int imgHeight, int imgWidth);

  /**
   * @brief Fits a normal curve to the sharpness curve using brute force.
   *
   * @param sharpnesscurve Input sharpness curve.
   * @param amplitude Amplitude of the curve.
   * @param offset Offset of the curve.
   * @param std_dev_factor Standard deviation factor for the curve.
   * @return Fitted sharpness curve as a vector of doubles.
   */
  std::vector<double>
  fitnormalcurveBruteForce(std::vector<double> sharpnesscurve, double amplitude,
                           double offset, double std_dev_factor);


  // Switch methods at runtime
  void setPeakLocator(PeakLocator m);


  // Helper: truncated Gaussian peak estimator on the reduced-resolution curve
  // domain [0..N-1] Returns μ̂ (in curve index units). Optionally returns μ_a,
  // σ_a^2, and μ_[a,b].
  double estimatePeakTruncatedGaussian(const std::vector<double> &f,
                                       double sigmaPxReduced, int edgeAvgCount,
                                       double eps, double *out_mu_a = nullptr,
                                       double *out_sigma_a_sq = nullptr,
                                       double *out_mu_ab = nullptr);


  // Helper: Power-COM peak estimator
  double estimatePeakPowerCOM(const std::vector<double> &f, int power,
                              bool quadraticRefine,
                              double *out_plainCOM = nullptr);

