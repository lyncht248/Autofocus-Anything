




  cv::Mat reduced_roberts_kernelx;;
  cv::Mat reduced_roberts_kernely;


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


  double computeBestFocusRatioGaussian(cv::Mat image, int imgHeight, int imgWidth);

  // Power-COM tuning
  void setPowerCOMExponent(int p);          // integer >= 1, default 4
  void setPowerCOMQuadraticRefine(bool on); // sub-pixel parabola refine


    // Peak locator selection
  enum class PeakLocator {
    CenterOfMass,
    PowerCOM,
    TruncatedGaussian,
    GaussianFitBrute
  };

  PeakLocator m_peakLocator = PeakLocator::GaussianFitBrute;

  /**
   * @brief Calculates the center of mass for the given curve.
   *
   * @param curve Input curve.
   * @return Center of mass value.
   */
  double findCenterOfMass(const std::vector<double> &curve);

  // Power-COM parameters
  int m_powerExponent = 4;          // default ^4
  bool m_powerCOMQuadRefine = true; // parabolic refine on by default


    // TruncatedGaussian parameters
  double m_sigmaPxFullRes =
      -1.0;               // <=0 => auto-estimate σ from σ_a (apparent var)
  int m_edgeAvgCount = 5; // points to average at each edge

  // Optional output smoothing (shared)
  double m_peakEmaBeta = 1.0; // 1.0 => smoothing disabled
  double m_prevMuReduced = std::numeric_limits<double>::quiet_NaN();

  // Exponential smoothing of the *returned* peak (0<beta<=1). beta=1 disables
  // smoothing.
  void setPeakSmoothing(double beta);



  // Set the assumed Gaussian sigma in **full-resolution pixels**. If <= 0, we
  // auto-estimate from σ_a.
  void setTruncGaussSigmaFullRes(double sigmaPx);
  // How many points to average at each edge to estimate f(a), f(b). (>=1)
  void setTruncGaussEdgeAveraging(int count);

