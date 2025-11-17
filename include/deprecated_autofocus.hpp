

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
