/* This file contains deprecated autofocus methods for reference and testing

FLAGS

FITTING
- computeBestFocus:
    CLAHE, Gaussian Blur, Tenengrad, Column Means, 
    Hamming Window, fit normal curve (brute force)
- computeBestFocusReduced:
- computeBestFocusVeryReduced:
    Resize to 1/4 x 1/4, CLAHE, Gaussian Blur, Roberts Cross,
    Column Means, Hamming Window, fit normal curve (brute force)
- computeBestFocusReducedHorizontal:
    same as computeBestFocusReduced, but
    before taking Column Means, find sharpest point in each horizontal row and store
    (computes sharpest point for each row)
- computeBestFocusRatioGaussian:
    Resize to 1/2 x 1/2, Roberts Cross, Column Means,
    Reduced Roberts Cross, Reduced Column Means,
    Ratio of (standard RC column means) / (reduced RC column means) + 1,
    Least Square fitting of Gaussian to ratio curve (non-iterative)

HELPER FUNCTIONS



    */





  // Initialize reduced resolution benchmark CSV file with headers
  std::ofstream reducedBenchmarkFile("../output/focus_benchmark_reduced.csv");
  if (reducedBenchmarkFile.is_open()) {
    reducedBenchmarkFile
        << "timestamp,total_time_us,resize_time_us,clahe_time_us,blur_time_us,"
           "roberts_time_us,column_time_us,sliding_time_us,com_time_us"
        << std::endl;
    reducedBenchmarkFile.close();
  }
  std::ofstream leastSquaresBenchmarkFile(
      "../output/focus_benchmark_LeastSquares.csv");
  if (leastSquaresBenchmarkFile.is_open()) {
    leastSquaresBenchmarkFile
        << "timestamp,total_time_us,resize_time_us,"
           "roberts_time_us,column_time_us,reduced_roberts_us, reduced_column_us, ratio_us, fitting_time_us"
        << std::endl;
    leastSquaresBenchmarkFile.close();
  }

  std::ofstream iterativeBenchmarkFile(
      "../output/focus_benchmark_iterative.csv");
  if (iterativeBenchmarkFile.is_open()) {
    iterativeBenchmarkFile
        << "timestamp,total_time_us,resize_time_us,roberts_time_us,column_time_us,"
           "offset_time_us,rescale_time,gaussian_fit_time_us,"
           "mu_values"
        << std::endl;
    iterativeBenchmarkFile.close();
  }


  // Initialize very reduced resolution benchmark CSV file with headers
  std::ofstream veryReducedBenchmarkFile(
      "../output/focus_benchmark_very_reduced.csv");
  if (veryReducedBenchmarkFile.is_open()) {
    veryReducedBenchmarkFile
        << "timestamp,total_time_us,resize_time_us,clahe_time_us,blur_time_us,"
           "roberts_time_us,column_time_us,sliding_time_us,fitting_time_us"
        << std::endl;
    veryReducedBenchmarkFile.close();
  }

  // Define reduced Roberts Cross kernels
  reduced_roberts_kernelx = (cv::Mat_<double>(1, 2) << 1, -1);
  reduced_roberts_kernely = (cv::Mat_<double>(2, 1) << 1, -1);






int autofocus::computeBestFocus(cv::Mat image, int imgHeight, int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // CLAHE preprocessing with reduced noise amplification
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0); // Reduced from 2.0 to limit noise amplification
  clahe->setTilesGridSize(cv::Size(
      8, 8)); // Larger tiles (16x16) for less aggressive local adaptation
  cv::Mat clahe_enhanced;
  clahe->apply(image, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur: ~300μs (5.5%)
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::GaussianBlur(clahe_enhanced, blurred_preallocated, cv::Size(3, 3), 1, 1,
                   cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Tenengrad method: ~1,400μs (25.5%), occasionally spikes to ~4,000μs
  auto tenengradStart = std::chrono::high_resolution_clock::now();

  // Compute gradients using Sobel operators
  cv::Sobel(blurred_preallocated, img_x_preallocated, CV_16S, 1, 0,
            3); // 3x3 Sobel for X direction
  cv::Sobel(blurred_preallocated, img_y_preallocated, CV_16S, 0, 1,
            3); // 3x3 Sobel for Y direction

  // Square the gradients and sum them (Tenengrad formula: Gx² + Gy²)
  cv::multiply(img_x_preallocated, img_x_preallocated,
               img_x_squared_preallocated);
  cv::multiply(img_y_preallocated, img_y_preallocated,
               img_y_squared_preallocated);
  cv::add(img_x_squared_preallocated, img_y_squared_preallocated,
          sum_xy_preallocated);

  // Convert to float for better precision in subsequent calculations
  sum_xy_preallocated.convertTo(sharpness_float_preallocated, CV_32F);
  auto tenengradEnd = std::chrono::high_resolution_clock::now();

  // Column Means: ~2,400μs (43.6%), occasionally spikes to ~6,000μs
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float_preallocated, columnMeansMatrix, 0, cv::REDUCE_AVG,
             CV_64F);

  // Convert to vector for compatibility with existing code
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);

  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Window with Hamming: ~8μs (0.1%)
  auto slidingStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve;
  int kernel = 40; // must be an even number
  // 32*2*2*2*2 = 512 pixels
  // Pre-compute Hamming window coefficients
  std::vector<double> hammingWindow(kernel);
  for (int i = 0; i < kernel; i++) {
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));
  }

  for (int i = 0; i < imgWidth - kernel; i++) {
    double regionSharpnessScore = 0.0;
    double windowSum = 0.0;
    for (int k = 0; k < kernel; k++) {
      regionSharpnessScore += columnMeans[i + k] * hammingWindow[k];
      windowSum += hammingWindow[k];
    }
    regionSharpnessScore /=
        windowSum; // Normalize by sum of window coefficients
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  auto slidingEnd = std::chrono::high_resolution_clock::now();

  // Calculate and save the max amplitude of the sharpness curve
  double maxVal =
      *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double minVal =
      *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double amplitude = maxVal - minVal;
  double offset = minVal;
  // If the amplitude is too low, return the desiredLocBestFocus so no lens
  // movement occurs. TODO: add error message
  if (amplitude < 0.3) {
    std::cout << "Amplitude is too low, returning desiredLocBestFocus\n";
    return desiredLocBestFocus;
  }

  // Fitting a normal curve to the sharpness curve, to avoid local peaks in the
  // data around vessel edges (and deal with multiple real peaks) Curve Fitting:
  // ~1,400μs (25.5%), occasionally spikes to ~4,000μs
  auto fittingStart = std::chrono::high_resolution_clock::now();
  //  std::vector<double> sharpnesscurvenormalized =
  //  fitnormalcurve(sharpnesscurve, amplitude, offset, 0.2);
  std::vector<double> sharpnesscurvenormalized =
      fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2);
  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // printing to text files for testing
  if (bSaveSharpnessCurves) {
    std::string FileName = "TESTING" + std::to_string(increment);
    std::string TextFile =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_SharpnessCurve.txt";
    std::string TextFile2 =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_FittedNorm.txt";
    std::ofstream outputFile(TextFile);
    std::ofstream outputFile2(TextFile2);
    std::ostream_iterator<double> output_iterator(outputFile, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
    outputFile << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    std::copy(sharpnesscurvenormalized.begin(), sharpnesscurvenormalized.end(),
              output_iterator2);
    outputFile2 << "\n";
    outputFile.close();
    outputFile2.close();
    increment++;
  }

  int locBestFocus = distance(begin(sharpnesscurvenormalized),
                              max_element(begin(sharpnesscurvenormalized),
                                          end(sharpnesscurvenormalized)));

  if (bSaveImages) {
    // Store the curves for visualization
    lastSharpnessCurve = sharpnesscurve;
    lastFittedCurve = sharpnesscurvenormalized;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

  // Calculate timing benchmarks
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(
      claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(
      blurEnd - blurStart);
  auto tenengradTime = std::chrono::duration_cast<std::chrono::microseconds>(
      tenengradEnd - tenengradStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      slidingEnd - slidingStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      fittingEnd - fittingStart);
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);

  // Save timing information to CSV file with better error handling
  try {
    std::ofstream benchmarkFile("../output/focus_benchmark.csv", std::ios::app);
    if (benchmarkFile.is_open() && benchmarkFile.good()) {
      time_t now = time(0);
      benchmarkFile << now << "," << totalTime.count() << ","
                    << claheTime.count() << "," << blurTime.count() << ","
                    << tenengradTime.count() << "," << columnTime.count() << ","
                    << slidingTime.count() << "," << fittingTime.count()
                    << std::endl;

      if (benchmarkFile.fail()) {
        if (bAutofocusLogFlag) {
          logger->warn(
              "[autofocus::computeBestFocus] Failed to write benchmark data");
        }
      }
      benchmarkFile.close();
    }
  } catch (const std::exception &ex) {
    if (bAutofocusLogFlag) {
      logger->error("[autofocus::computeBestFocus] Benchmark file error: " +
                    std::string(ex.what()));
    }
  }

  return locBestFocus +
         kernel / 2; // Note: kernel/2 offset needed for sliding window
}



int autofocus::computeBestFocusVeryReduced(cv::Mat image, int imgHeight,
                                           int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/16 size (1/4 in each dimension)
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(imgWidth / 4, imgHeight / 4), 0, 0,
             cv::INTER_LINEAR);
  int reducedWidth = resized_image.cols;
  int reducedHeight = resized_image.rows;
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // CLAHE preprocessing with reduced noise amplification
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(
      cv::Size(4, 4)); // Increased from 2x2 to 4x4 to avoid artifacts
  cv::Mat clahe_enhanced;
  clahe->apply(resized_image, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred_very_reduced;
  cv::GaussianBlur(clahe_enhanced, blurred_very_reduced, cv::Size(3, 3), 1, 1,
                   cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross method instead of Tenengrad
  auto robertsStart = std::chrono::high_resolution_clock::now();

  // Apply Roberts Cross kernels
  cv::Mat img_x_roberts, img_y_roberts;
  cv::filter2D(blurred_very_reduced, img_x_roberts, CV_16S, roberts_kernelx);
  cv::filter2D(blurred_very_reduced, img_y_roberts, CV_16S, roberts_kernely);

  // Square the gradients and sum them (Roberts Cross formula: Gx² + Gy²)
  cv::Mat img_x_squared_roberts, img_y_squared_roberts, sum_xy_roberts;
  cv::multiply(img_x_roberts, img_x_roberts, img_x_squared_roberts);
  cv::multiply(img_y_roberts, img_y_roberts, img_y_squared_roberts);
  cv::add(img_x_squared_roberts, img_y_squared_roberts, sum_xy_roberts);

  // Convert to float for better precision in subsequent calculations
  cv::Mat sharpness_float_roberts;
  sum_xy_roberts.convertTo(sharpness_float_roberts, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column Means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float_roberts, columnMeansMatrix, 0, cv::REDUCE_AVG,
             CV_64F);

  // Convert to vector for compatibility with existing code
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Window with Hamming - scale kernel size for very reduced resolution
  auto slidingStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve;
  int kernel = 16; // Reduced from 40 to 10 for 1/4 resolution (40/4 = 10)

  // Pre-compute Hamming window coefficients
  std::vector<double> hammingWindow(kernel);
  for (int i = 0; i < kernel; i++) {
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));
  }

  for (int i = 0; i < reducedWidth - kernel; i++) {
    double regionSharpnessScore = 0.0;
    double windowSum = 0.0;
    for (int k = 0; k < kernel; k++) {
      regionSharpnessScore += columnMeans[i + k] * hammingWindow[k];
      windowSum += hammingWindow[k];
    }
    regionSharpnessScore /=
        windowSum; // Normalize by sum of window coefficients
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  auto slidingEnd = std::chrono::high_resolution_clock::now();

  // Calculate and save the max amplitude of the sharpness curve
  double maxVal =
      *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double minVal =
      *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double amplitude = maxVal - minVal;
  double offset = minVal;

  // If the amplitude is too low, return the scaled desiredLocBestFocus
  if (amplitude < 0.3) {
    std::cout << "Amplitude is too low, returning scaled desiredLocBestFocus\n";
    return desiredLocBestFocus / 4; // Scale down for very reduced resolution
  }

  // Fitting a normal curve - scale std_dev_factor for very reduced resolution
  auto fittingStart = std::chrono::high_resolution_clock::now();
  // std::vector<double> sharpnesscurvenormalized =
  // fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2); // 0.2 *
  // 4 = 0.8
  std::vector<double> sharpnesscurvenormalized = fitnormalcurveBruteForce(
      sharpnesscurve, amplitude, offset, 0.2); // 0.2 * 4 = 0.8

  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // Save sharpness curves if enabled
  if (bSaveSharpnessCurves) {
    std::string FileName = "TESTING_VERY_REDUCED" + std::to_string(increment);
    std::string TextFile =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_SharpnessCurve.txt";
    std::string TextFile2 =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_FittedNorm.txt";
    std::ofstream outputFile(TextFile);
    std::ofstream outputFile2(TextFile2);
    std::ostream_iterator<double> output_iterator(outputFile, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
    outputFile << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    std::copy(sharpnesscurvenormalized.begin(), sharpnesscurvenormalized.end(),
              output_iterator2);
    outputFile2 << "\n";
    outputFile.close();
    outputFile2.close();
  }

  int locBestFocus = distance(begin(sharpnesscurvenormalized),
                              max_element(begin(sharpnesscurvenormalized),
                                          end(sharpnesscurvenormalized)));

  if (bSaveImages) {
    // Scale the curves for visualization to match full resolution length
    // Original full resolution curve would be approximately (imgWidth - 40)
    // points Very reduced curve is approximately (imgWidth/4 - 10) points Scale
    // factor is roughly 4x

    int targetLength = imgWidth - 40; // Target length to match full resolution
    std::vector<double> scaledSharpnessCurve;
    std::vector<double> scaledFittedCurve;

    // Simple linear interpolation to scale up the curves
    for (int i = 0; i < targetLength; i++) {
      double sourceIndex =
          (double)i * (sharpnesscurve.size() - 1) / (targetLength - 1);
      int lowerIndex = (int)sourceIndex;
      int upperIndex = std::min(lowerIndex + 1, (int)sharpnesscurve.size() - 1);
      double fraction = sourceIndex - lowerIndex;

      // Interpolate sharpness curve
      double interpolatedSharpness =
          sharpnesscurve[lowerIndex] * (1.0 - fraction) +
          sharpnesscurve[upperIndex] * fraction;
      scaledSharpnessCurve.push_back(interpolatedSharpness);

      // Interpolate fitted curve
      double interpolatedFitted =
          sharpnesscurvenormalized[lowerIndex] * (1.0 - fraction) +
          sharpnesscurvenormalized[upperIndex] * fraction;
      scaledFittedCurve.push_back(interpolatedFitted);
    }

    // Store the scaled curves for visualization
    lastSharpnessCurve = scaledSharpnessCurve;
    lastFittedCurve = scaledFittedCurve;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

  // Calculate timing benchmarks
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      resizeEnd - resizeStart);
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(
      claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(
      blurEnd - blurStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      slidingEnd - slidingStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      fittingEnd - fittingStart);
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);

  // Save timing information to CSV file with different filename
  std::ofstream benchmarkFile("../output/focus_benchmark_very_reduced.csv",
                              std::ios::app);
  if (benchmarkFile.is_open()) {
    time_t now = time(0);
    benchmarkFile << now << "," << totalTime.count() << ","
                  << resizeTime.count() << "," << claheTime.count() << ","
                  << blurTime.count() << "," << robertsTime.count() << ","
                  << columnTime.count() << "," << slidingTime.count() << ","
                  << fittingTime.count() << std::endl;
    benchmarkFile.close();
  }

  // Scale the result back to full resolution and add kernel offset
  return (locBestFocus + kernel / 2) * 4; // Scale up by 4x for full resolution
}





double autofocus::computeBestFocusReduced(cv::Mat image, int imgHeight,
                                          int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/2 x 1/2
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // CLAHE
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(cv::Size(4, 4));
  cv::Mat clahe_enhanced;
  clahe->apply(resized, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred;
  cv::GaussianBlur(clahe_enhanced, blurred, cv::Size(3, 3), 1, 1,
                   cv::BORDER_DEFAULT);

  // try without CLAHE
  // cv::GaussianBlur(resized, blurred, cv::Size(7, 7), 1, 1,
  //                 cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross gradients -> sharpness image (float)
  auto robertsStart = std::chrono::high_resolution_clock::now();
  cv::Mat img_x, img_y;
  cv::filter2D(blurred, img_x, CV_16S, roberts_kernelx);
  cv::filter2D(blurred, img_y, CV_16S, roberts_kernely);
  cv::Mat img_x_squared, img_y_squared, sum_xy;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::add(img_x_squared, img_y_squared, sum_xy);
  cv::Mat sharpness_float;
  sum_xy.convertTo(sharpness_float, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Hamming
  auto slidingStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve;
  const int kernel = 20; // for 1/2 resolution

  // std::vector<double> hammingWindow(kernel);
  // for (int i = 0; i < kernel; ++i)
  //   hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));

  // sharpnesscurve.reserve(std::max(0, blurred.cols - kernel));
  // for (int i = 0; i < blurred.cols - kernel; ++i) {
  //   double acc = 0.0, wsum = 0.0;
  //   for (int k = 0; k < kernel; ++k) {
  //     const double w = hammingWindow[k];
  //     acc += columnMeans[i + k] * w;
  //     wsum += w;
  //   }
  //   sharpnesscurve.push_back(acc / wsum);
  // }

  // Use moving average instead of Hamming window
  for (int i = 0; i < blurred.cols - kernel; i++) {
    double regionSharpnessScore = 0.0;
    for (int k = 0; k < kernel; k++) {
      regionSharpnessScore += columnMeans[i + k];
    }
    regionSharpnessScore /= kernel; // Normalize by kernel size
    sharpnesscurve.push_back(regionSharpnessScore);
  }

  auto slidingEnd = std::chrono::high_resolution_clock::now();


  // Save range of values after each stage for debugging


  if (bSaveSharpnessCurves) {
    std::string FileName = "TESTING_" + std::to_string(increment);
    std::string TextFile1 = "../output/SharpnessCurves_steps/" + FileName + "_CLAHE_1.txt";
    std::string TextFile2 = "../output/SharpnessCurves_steps/" + FileName + "_blur_2.txt";
    std::string TextFile3 = "../output/SharpnessCurves_steps/" + FileName + "_robertscross_3.txt";
    std::string TextFile4 = "../output/SharpnessCurves_steps/" + FileName + "_column_means_4.txt";
    std::string TextFile5 = "../output/SharpnessCurves_steps/" + FileName + "_hamming_5.txt";
    std::ofstream outputFile1(TextFile1);
    std::ofstream outputFile2(TextFile2);
    std::ofstream outputFile3(TextFile3);
    std::ofstream outputFile4(TextFile4);
    std::ofstream outputFile5(TextFile5);

    std::ostream_iterator<double> output_iterator(outputFile1, ", ");
    for (int i = 0; i < clahe_enhanced.rows; ++i) {
      const uchar* row_ptr = clahe_enhanced.ptr<uchar>(i);
      for (int j = 0; j < clahe_enhanced.cols; ++j) {
        outputFile1 << static_cast<double>(row_ptr[j]) << ", ";
      }
    }
    outputFile1 << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    for (int i = 0; i < blurred.rows; ++i) {
      const uchar* row_ptr = blurred.ptr<uchar>(i);
      for (int j = 0; j < blurred.cols; ++j) {
        outputFile2 << static_cast<double>(row_ptr[j]) << ", ";
      }
    }
    outputFile2 << "\n";
    std::ostream_iterator<double> output_iterator3(outputFile3, ", ");
    for (int i = 0; i < sharpness_float.rows; ++i) {
        const float* row_ptr = sharpness_float.ptr<float>(i);
        for (int j = 0; j < sharpness_float.cols; ++j) {
            outputFile3 << static_cast<double>(row_ptr[j]) << ", ";
        }
    }
    outputFile3 << "\n";
    std::ostream_iterator<double> output_iterator4(outputFile4, ", ");
    std::copy(columnMeans.begin(), columnMeans.end(), output_iterator4);
    outputFile4 << "\n";

    // Add offset to sharpness curve
    std::vector<double> sharpnesscurve_with_offset(kernel / 2, 0.0); //
    sharpnesscurve_with_offset.insert(
        sharpnesscurve_with_offset.end(),
        sharpnesscurve.begin(),
        sharpnesscurve.end()
    );

    // Now save sharpnesscurve_with_offset instead of sharpnesscurve
    std::ostream_iterator<double> output_iterator5(outputFile5, ", ");
    std::copy(sharpnesscurve_with_offset.begin(), sharpnesscurve_with_offset.end(), output_iterator5);
    outputFile5 << "\n";

    outputFile1.close();
    outputFile2.close();
    outputFile3.close();
    outputFile4.close();
    outputFile5.close();
    
    // Save Roberts Cross as a png image for visual debugging
    // std::string sharpnessPngPath = "../output/SharpnessCurves_steps/" + FileName + "_RC.png";

    // // Normalize and convert to 8-bit for visualization
    // cv::Mat sharpness_norm, sharpness_8u;
    // cv::normalize(sharpness_float, sharpness_norm, 0, 255, cv::NORM_MINMAX);
    // sharpness_norm.convertTo(sharpness_8u, CV_8U);

    // cv::imwrite(sharpnessPngPath, sharpness_8u);

    increment++;
  }


  // Normalize baseline to zero for robust COM / TG moments
  if (!sharpnesscurve.empty()) {
    const double minVal =
        *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
    for (double &v : sharpnesscurve)
      v = std::max(0.0, v - minVal);
  }

  // Amplitude check (edge case / low SNR): behave like your current shortcut.
  const double maxVal =
      sharpnesscurve.empty()
          ? 0.0
          : *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  if (maxVal < 3.0) {
    // keep behavior: do nothing when almost no structure is visible
    return static_cast<double>(desiredLocBestFocus);
  }

  // --- Choose estimator on the curve domain [0..N-1] ---
  auto estStart = std::chrono::high_resolution_clock::now();

  double mu_curve = 0.0; // result in curve index units
  lastFittedCurve.clear();
  lastSharpnessCurve = sharpnesscurve;
  lastCenterOfMass = -1.0; // we'll fill it if available

  switch (m_peakLocator) {
  case PeakLocator::CenterOfMass: {
    // your plain COM (already baseline-subtracted)
    mu_curve = findCenterOfMass(sharpnesscurve);
    lastCenterOfMass = mu_curve;
  } break;

  case PeakLocator::PowerCOM: {
    double com_plain = -1.0;
    mu_curve = estimatePeakPowerCOM(sharpnesscurve, m_powerExponent,
                                    m_powerCOMQuadRefine, &com_plain);
    // Keep overlay meaning: cyan line = plain COM
    if (com_plain >= 0.0)
      lastCenterOfMass = com_plain;
  } break;

  case PeakLocator::TruncatedGaussian: {
    // Work in reduced coordinates. If user set σ at full-res, scale it by 0.5
    // If not set, use the same estimation as the brute force method
    double sigmaReduced;
    if (m_sigmaPxFullRes > 0.0) {
      sigmaReduced = 0.5 * m_sigmaPxFullRes;
    } else {
      // Use the same σ estimation as brute force Gaussian: 0.2 * curve_length
      sigmaReduced = 0.2 * sharpnesscurve.size();
    }
    double mu_a_dbg = -1.0;
    mu_curve = estimatePeakTruncatedGaussian(sharpnesscurve, sigmaReduced,
                                             m_edgeAvgCount, 1e-2, &mu_a_dbg,
                                             nullptr, nullptr);
    if (mu_a_dbg >= 0.0)
      lastCenterOfMass = mu_a_dbg; // overlay: COM
  } break;

  case PeakLocator::GaussianFitBrute: {
    const double amplitude = maxVal;
    const double offset = 0.0;
    std::vector<double> fitted =
        fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2);
    auto it = std::max_element(fitted.begin(), fitted.end());
    mu_curve = static_cast<double>(std::distance(fitted.begin(), it));
   // (optional) lastCenterOfMass = findCenterOfMass(sharpnesscurve);
  } break;
  }

  // Optional exponential smoothing of the peak (keeps continuity but still
  // fast)
  if (m_peakEmaBeta < 1.0) {
    if (std::isnan(m_prevMuReduced))
      m_prevMuReduced = mu_curve;
    mu_curve =
        (1.0 - m_peakEmaBeta) * m_prevMuReduced + m_peakEmaBeta * mu_curve;
    m_prevMuReduced = mu_curve;
  }

  // For your saved debug overlay: show COM (μ_a) if we have it
  // (mu_a_dbg is only available in TruncatedGaussian case, handled in switch
  // above)

  auto estEnd = std::chrono::high_resolution_clock::now();

  // --- Timing log unchanged footprint, writing "com_time_us" as estimator time
  // ---
  auto totalTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - startTime);
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      resizeEnd - resizeStart);
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(
      claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(
      blurEnd - blurStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      slidingEnd - slidingStart);
  auto estTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - estStart);

  try {
    std::ofstream reducedBenchmarkFile("../output/focus_benchmark_reduced.csv",
                                       std::ios::app);
    if (reducedBenchmarkFile.is_open() && reducedBenchmarkFile.good()) {
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      // keep CSV header compatibility: write estimator time in the
      // "com_time_us" column
      reducedBenchmarkFile << timestamp << "," << totalTime.count() << ","
                           << resizeTime.count() << "," << claheTime.count()
                           << "," << blurTime.count() << ","
                           << robertsTime.count() << "," << columnTime.count()
                           << "," << slidingTime.count() << ","
                           << estTime.count() << std::endl;
      reducedBenchmarkFile.close();
    }
  } catch (...) { /* ignore */
  }

  // Map curve index back to reduced-image x by adding the kernel/2 offset,
  // exactly like your original COM method (no scaling to full-res here).
  return mu_curve + kernel / 2.0;
}







// computeBestFocusReduced but instead of using Column Means, 
// find sharpest point in each horizontal row and store
std::vector<double> autofocus::computeBestFocusReducedHorizontal(cv::Mat image, int imgHeight,
                                          int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/2 x 1/2
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // CLAHE
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(cv::Size(4, 4));
  cv::Mat clahe_enhanced;
  clahe->apply(resized, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred;
  cv::GaussianBlur(clahe_enhanced, blurred, cv::Size(3, 3), 1, 1,
                   cv::BORDER_DEFAULT);


  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross gradients -> sharpness image (float)
  auto robertsStart = std::chrono::high_resolution_clock::now();
  cv::Mat img_x, img_y;
  cv::filter2D(blurred, img_x, CV_16S, roberts_kernelx);
  cv::filter2D(blurred, img_y, CV_16S, roberts_kernely);
  cv::Mat img_x_squared, img_y_squared, sum_xy;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::add(img_x_squared, img_y_squared, sum_xy);
  cv::Mat sharpness_float;
  sum_xy.convertTo(sharpness_float, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column means
  // auto columnStart = std::chrono::high_resolution_clock::now();
  // cv::Mat columnMeansMatrix;
  // cv::reduce(sharpness_float, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);
  // std::vector<double> columnMeans;
  // columnMeansMatrix.copyTo(columnMeans);
  // auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Hamming
  auto slidingStart = std::chrono::high_resolution_clock::now();


  std::vector<std::vector<double>> sharpnessCurves;
  const int kernel = 20; // or whatever kernel size you want
  std::vector<double> hammingWindow(kernel);
  for (int i = 0; i < kernel; ++i)
      hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));

  for (int row = 0; row < blurred.rows; ++row) {
      std::vector<double> rowSharpness;
      const float* sharpnessRow = sharpness_float.ptr<float>(row);
      for (int i = 0; i < blurred.cols - kernel; ++i) {
          double acc = 0.0, wsum = 0.0;
          for (int k = 0; k < kernel; ++k) {
              const double w = hammingWindow[k];
              acc += sharpnessRow[i + k] * w;
              wsum += w;
          }
          rowSharpness.push_back(acc / wsum);
      }
      sharpnessCurves.push_back(rowSharpness);
  }


  // Use moving average instead of Hamming window
  // for (int i = 0; i < blurred.cols - kernel; i++) {
  //   double regionSharpnessScore = 0.0;
  //   for (int k = 0; k < kernel; k++) {
  //     regionSharpnessScore += columnMeans[i + k];
  //   }
  //   regionSharpnessScore /= kernel; // Normalize by kernel size
  //   sharpnesscurve.push_back(regionSharpnessScore);
  // }

  auto slidingEnd = std::chrono::high_resolution_clock::now();





  // Compute sharpness for each row and store
  auto estStart = std::chrono::high_resolution_clock::now();
  std::vector<double> rowBestFocusLocations;
  for (auto& sharpnesscurve : sharpnessCurves) {

    // Normalize baseline to zero for robust COM / TG moments
    if (!sharpnesscurve.empty()) {
      const double minVal =
          *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
      for (double &v : sharpnesscurve)
        v = std::max(0.0, v - minVal);
    }

    // Amplitude check (edge case / low SNR): behave like your current shortcut.
    const double maxVal =
        sharpnesscurve.empty()
            ? 0.0
            : *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
    if (maxVal < 3.0) {
      // keep behavior: do nothing when almost no structure is visible
      return std::vector<double>(sharpnessCurves.size(), static_cast<double>(desiredLocBestFocus));
      //static_cast<double>(desiredLocBestFocus);
    }



    // --- Choose estimator on the curve domain [0..N-1] ---
    

    double mu_curve = 0.0; // result in curve index units
    lastFittedCurve.clear();
    lastSharpnessCurve = sharpnesscurve;
    lastCenterOfMass = -1.0; // we'll fill it if available

    switch (m_peakLocator) {

    case PeakLocator::TruncatedGaussian: {
      // Work in reduced coordinates. If user set σ at full-res, scale it by 0.5
      // If not set, use the same estimation as the brute force method
      double sigmaReduced;
      if (m_sigmaPxFullRes > 0.0) {
        sigmaReduced = 0.5 * m_sigmaPxFullRes;
      } else {
        // Use the same σ estimation as brute force Gaussian: 0.2 * curve_length
        sigmaReduced = 0.2 * sharpnesscurve.size();
      }
      double mu_a_dbg = -1.0;
      mu_curve = estimatePeakTruncatedGaussian(sharpnesscurve, sigmaReduced,
                                              m_edgeAvgCount, 1e-2, &mu_a_dbg,
                                              nullptr, nullptr);
      if (mu_a_dbg >= 0.0)
        lastCenterOfMass = mu_a_dbg; // overlay: COM
    } break;

    case PeakLocator::GaussianFitBrute: {
      const double amplitude = maxVal;
      const double offset = 0.0;
      std::vector<double> fitted =
          fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2);
      auto it = std::max_element(fitted.begin(), fitted.end());
      mu_curve = static_cast<double>(std::distance(fitted.begin(), it));
    // (optional) lastCenterOfMass = findCenterOfMass(sharpnesscurve);
    } break;
    }

    // Optional exponential smoothing of the peak (keeps continuity but still
    // fast)
    if (m_peakEmaBeta < 1.0) {
      if (std::isnan(m_prevMuReduced))
        m_prevMuReduced = mu_curve;
      mu_curve =
          (1.0 - m_peakEmaBeta) * m_prevMuReduced + m_peakEmaBeta * mu_curve;
      m_prevMuReduced = mu_curve;
    }

    // Store the best focus location for the current row
    rowBestFocusLocations.push_back(mu_curve + kernel / 2.0);

    // For your saved debug overlay: show COM (μ_a) if we have it
    // (mu_a_dbg is only available in TruncatedGaussian case, handled in switch
    // above)

    



  }

  // Save range of values after each stage for debugging
  if (bSaveSharpnessCurves2) {
  std::string FileName = "TESTING_" + std::to_string(increment2);
    // std::string TextFile1 = "../output/SharpnessCurves_steps_Horizontal/" + FileName + "_CLAHE_1.txt";
    // std::string TextFile2 = "../output/SharpnessCurves_steps_Horizontal/" + FileName + "_blur_2.txt";
    // std::string TextFile3 = "../output/SharpnessCurves_steps_Horizontal/" + FileName + "_robertscross_3.txt";
    //std::string TextFile4 = "../output/SharpnessCurves_steps_Horizontal/" + FileName + "_column_means_4.txt";
    //std::string TextFile5 = "../output/SharpnessCurves_steps_Horizontal/" + FileName + "_hamming_5.txt";
    // std::ofstream outputFile1(TextFile1);
    // std::ofstream outputFile2(TextFile2);
    // std::ofstream outputFile3(TextFile3);
   // std::ofstream outputFile4(TextFile4);
   // std::ofstream outputFile5(TextFile5);

    // std::ostream_iterator<double> output_iterator(outputFile1, ", ");
    // for (int i = 0; i < clahe_enhanced.rows; ++i) {
    //   const uchar* row_ptr = clahe_enhanced.ptr<uchar>(i);
    //   for (int j = 0; j < clahe_enhanced.cols; ++j) {
    //     outputFile1 << static_cast<double>(row_ptr[j]) << ", ";
    //   }
    // }
    // outputFile1 << "\n";
    // std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    // for (int i = 0; i < blurred.rows; ++i) {
    //   const uchar* row_ptr = blurred.ptr<uchar>(i);
    //   for (int j = 0; j < blurred.cols; ++j) {
    //     outputFile2 << static_cast<double>(row_ptr[j]) << ", ";
    //   }
    // }
    // outputFile2 << "\n";
    // std::ostream_iterator<double> output_iterator3(outputFile3, ", ");
    // for (int i = 0; i < sharpness_float.rows; ++i) {
    //     const float* row_ptr = sharpness_float.ptr<float>(i);
    //     for (int j = 0; j < sharpness_float.cols; ++j) {
    //         outputFile3 << static_cast<double>(row_ptr[j]) << ", ";
    //     }
    // }
    // outputFile3 << "\n";


    // outputFile1.close();
    // outputFile2.close();
    // outputFile3.close();
    


    // ROBERTS CROSS 
    // Normalize and convert to 8-bit for visualization
  if (!sharpness_float.empty()) {
    cv::Mat sharpness_norm, sharpness_8u;
    cv::normalize(sharpness_float, sharpness_norm, 0, 255, cv::NORM_MINMAX);
    sharpness_norm.convertTo(sharpness_8u, CV_8U);
    // 4. Save or display the overlay image
    std::string overlayPath = "../output/SharpnessCurves_steps_Horizontal/" + FileName + "_overlay.png";
    cv::imwrite(overlayPath, sharpness_8u);
  }
    // Raw tilted cam image
    // cv::Mat tiltedCam_norm, sharpness_8u, sharpness_color;
    // cv::normalize(resized, tiltedCam_norm, 0, 255, cv::NORM_MINMAX);
    // tiltedCam_norm.convertTo(sharpness_8u, CV_8U);


    // // 2. Convert to color for overlay
    // cv::cvtColor(sharpness_8u, sharpness_color, cv::COLOR_GRAY2BGR);

    // // 3. Overlay yellow points at best focus locations
    // for (int row = 0; row < sharpness_color.rows && row < rowBestFocusLocations.size(); ++row) {
    //     int col = static_cast<int>(std::round(rowBestFocusLocations[row]));
    //     if (col >= 0 && col < sharpness_color.cols) {
    //         cv::circle(sharpness_color, cv::Point(col, row), 2, cv::Scalar(0, 255, 255), -1); // Yellow (BGR)
    //     }
    // }





    increment2++;
  }

  auto estEnd = std::chrono::high_resolution_clock::now();

  // --- Timing log unchanged footprint, writing "com_time_us" as estimator time
  // ---
  auto totalTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - startTime);
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      resizeEnd - resizeStart);
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(
      claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(
      blurEnd - blurStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  // auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
  //     columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      slidingEnd - slidingStart);
  auto estTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - estStart);

  // try {
  //   std::ofstream reducedBenchmarkFile("../output/focus_benchmark_reduced.csv",
  //                                      std::ios::app);
  //   if (reducedBenchmarkFile.is_open() && reducedBenchmarkFile.good()) {
  //     auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
  //                          std::chrono::system_clock::now().time_since_epoch())
  //                          .count();
  //     // keep CSV header compatibility: write estimator time in the
  //     // "com_time_us" column
  //     reducedBenchmarkFile << timestamp << "," << totalTime.count() << ","
  //                          << resizeTime.count() << "," << claheTime.count()
  //                          << "," << blurTime.count() << ","
  //                          << robertsTime.count() << "," << columnTime.count()
  //                          << "," << slidingTime.count() << ","
  //                          << estTime.count() << std::endl;
  //     reducedBenchmarkFile.close();
  //   }
  // } catch (...) { /* ignore */
  // }

  // Map curve index back to reduced-image x by adding the kernel/2 offset,
  // exactly like your original COM method (no scaling to full-res here).
  return rowBestFocusLocations;
}




std::vector<double>
autofocus::fitnormalcurveBruteForce(std::vector<double> sharpnesscurve,
                                    double amplitude, double offset,
                                    double std_dev_factor) {
  // Fits a normal curve to the data. Should try to find a proper curve-fitting
  // library...
  std::vector<double> norm_curve;
  std::vector<double> sum_of_diffs;
  double std_dev =
      std_dev_factor * (sharpnesscurve.size()); // now uses the parameter

  // Calculates sum_of_elems, a vector showing how well each normal curve with
  // mean j fits grad
  for (int j = 0; j < sharpnesscurve.size(); j++) // Start from 0, not 1
  {
    // Generates normal curve for given mean j
    std::vector<double> norm_curve_temp;
    for (int i = 0; i < sharpnesscurve.size(); i++) {
      double normResult =
          normpdf(i, j, std_dev); // j is now the actual mean position
      norm_curve_temp.push_back(normResult);
    }
    // scales so the max value (when i=j) is equal to the desired amplitude
    double factor =
        1.0 / *max_element(begin(norm_curve_temp), end(norm_curve_temp));
    for (int i = 0; i < sharpnesscurve.size(); i++) {
      norm_curve_temp[i] = norm_curve_temp[i] * factor * amplitude + offset;
    }

    // Calculates difference between norm curve and sharpness curve
    std::vector<double> differences;
    for (int z = 0; z < sharpnesscurve.size(); z++) {
      double diff = abs(sharpnesscurve[z] - norm_curve_temp[z]);
      differences.push_back(diff);
    }
    double sum_of_diff = std::accumulate(differences.begin(), differences.end(),
                                         decltype(differences)::value_type(0));
    sum_of_diffs.push_back(sum_of_diff);
  }

  // Picks the mean that has the lowest overall difference
  int bestMeanIndex = distance(
      begin(sum_of_diffs), min_element(begin(sum_of_diffs), end(sum_of_diffs)));
  double mean = bestMeanIndex; // No +1 needed since j starts from 0

  // Generates final norm_curve using the correct mean
  for (int i = 0; i < sharpnesscurve.size(); i++) {
    double normResult = normpdf(i, mean, std_dev);
    norm_curve.push_back(normResult);
  }
  double factor = 1.0 / *max_element(begin(norm_curve), end(norm_curve));
  for (int i = 0; i < (sharpnesscurve.size()); i++) {
    norm_curve[i] = norm_curve[i] * factor * amplitude + offset;
  }

  std::vector<double> fittednormalcurve(norm_curve.begin(), norm_curve.end());
  return fittednormalcurve;
}


void autofocus::setPeakLocator(PeakLocator m) { m_peakLocator = m; }



// Discrete truncated-Gaussian correction on f[0..N-1] (non-negative).
// sigmaPxReduced: Gaussian sigma in the same pixel units as f's x-axis (reduced
// resolution). If sigmaPxReduced <= 0, we auto-estimate σ from apparent
// variance σ_a^2. eps: threshold for the |t| test (per note in the memo);
// default ~1e-2.
double autofocus::estimatePeakTruncatedGaussian(
    const std::vector<double> &f, double sigmaPxReduced, int edgeAvgCount,
    double eps, double *out_mu_a, double *out_sigma_a_sq, double *out_mu_ab) {
  const int N = static_cast<int>(f.size());
  if (N <= 1) {
    if (out_mu_a)
      *out_mu_a = 0.0;
    if (out_sigma_a_sq)
      *out_sigma_a_sq = 0.0;
    if (out_mu_ab)
      *out_mu_ab = 0.0;
    return 0.0;
  }

  // --- sums for μ_a and σ_a^2 (apparent, discrete) ---
  double S0 = 0.0, S1 = 0.0, S2 = 0.0;
  for (int i = 0; i < N; ++i) {
    const double w = std::max(0.0, f[i]); // ensure non-negative
    S0 += w;
    S1 += w * i;
    S2 += w * i * i;
  }

  if (S0 <= 1e-12) {
    // degenerate: return middle as a safe value
    const double mid = 0.5 * (N - 1);
    if (out_mu_a)
      *out_mu_a = mid;
    if (out_sigma_a_sq)
      *out_sigma_a_sq = 0.0;
    if (out_mu_ab)
      *out_mu_ab = mid;
    return mid;
  }

  const double mu_a = S1 / S0;
  const double sigma_a_sq = std::max(0.0, S2 / S0 - mu_a * mu_a);

  // --- edge averages for f(a), f(b) (robust to noise) ---
  const int k = std::min(
      edgeAvgCount, std::max(1, N / 10)); // don't overreach on short signals
  double fa = 0.0, fb = 0.0;
  for (int i = 0; i < k; ++i)
    fa += f[i];
  for (int i = 0; i < k; ++i)
    fb += f[N - 1 - i];
  fa /= k;
  fb /= k;

  // --- μ_[a,b] with a=0, b=N-1 (discrete indices) ---
  // μ_[a,b] = (b f(b) - a f(a)) / (f(b) - f(a))  with a=0 => μ_[a,b] = b f(b) /
  // (f(b) - f(a))
  double mu_ab;
  const double denom_fb_fa = (fb - fa);
  if (std::abs(denom_fb_fa) <= 1e-12) {
    // symmetric edges -> μ_ab → ∞ ; in that case μ = μ_a (per note)
    mu_ab = std::numeric_limits<double>::infinity();
  } else {
    mu_ab = (N - 1) * fb / denom_fb_fa;
  }

  // --- choose σ: user-provided (full-res) scaled to reduced, or auto from σ_a
  // ---
  double sigma_used = sigmaPxReduced;
  if (sigma_used <= 0.0) {
    // fallback: use sqrt(σ_a^2) but guard with a minimum to avoid near-zero
    sigma_used = std::sqrt(std::max(sigma_a_sq, 1.0));
  }

  // --- t test: t = 2*(f(b)-f(a))/(f(b)+f(a)) ; if |t| < eps => use μ_a ---
  const double denom_sum = fb + fa;
  const double t =
      (std::abs(denom_sum) <= 1e-12) ? 0.0 : (2.0 * (fb - fa) / denom_sum);

  double mu_hat;
  if (!std::isfinite(mu_ab) || std::abs(t) < eps) {
    mu_hat = mu_a; // safe case near symmetric edges
  } else {
    const double denom = (mu_a - mu_ab);
    if (std::abs(denom) <= 1e-9) {
      mu_hat = mu_a; // avoid blow-ups
    } else {
      // Core formula: μ = μ_a + (σ_a^2 - σ^2) / (μ_a - μ_[a,b])
      mu_hat = mu_a + (sigma_a_sq - sigma_used * sigma_used) / denom;
    }
  }

  // Bound to observed domain to keep downstream code happy (drawing, indexing).
  // This *doesn't* introduce discrete jumps; μ̂ approaches the bound smoothly.
  mu_hat = std::clamp(mu_hat, 0.0, static_cast<double>(N - 1));

  if (out_mu_a)
    *out_mu_a = mu_a;
  if (out_sigma_a_sq)
    *out_sigma_a_sq = sigma_a_sq;
  if (out_mu_ab)
    *out_mu_ab = mu_ab;

  return mu_hat;
}




double autofocus::estimatePeakPowerCOM(const std::vector<double> &v, int power,
                                       bool quadraticRefine,
                                       double *out_plainCOM) {
  const int N = static_cast<int>(v.size());
  if (N <= 0) {
    if (out_plainCOM)
      *out_plainCOM = 0.0;
    return 0.0;
  }

  // Baseline subtract to suppress center pull from background
  const double minv = *std::min_element(v.begin(), v.end());

  // Optional: also compute plain COM for overlay/debug
  if (out_plainCOM) {
    double S0 = 0.0, S1 = 0.0;
    for (int i = 0; i < N; ++i) {
      const double w = std::max(0.0, v[i] - minv);
      S0 += w;
      S1 += w * i;
    }
    *out_plainCOM = (S0 > 1e-12) ? (S1 / S0) : 0.5 * (N - 1);
  }

  double sumw = 0.0, sumiw = 0.0;
  for (int i = 0; i < N; ++i) {
    double w = v[i] - minv;
    if (w <= 0.0)
      continue;
    double wp = powi_pos(w, std::max(1, power)); // winner-take-most
    sumw += wp;
    sumiw += wp * i;
  }

  if (sumw <= 1e-12)
    return 0.5 * (N - 1); // degenerate fallback

  double idx = sumiw / sumw;

  // Optional sub-pixel parabolic refine around nearest integer bin
  if (quadraticRefine) {
    int k = std::clamp((int)std::round(idx), 0, N - 1);
    if (k > 0 && k < N - 1) {
      double y1 = v[k - 1], y2 = v[k], y3 = v[k + 1];
      double denom = 2.0 * y2 - y1 - y3;
      if (std::fabs(denom) > 1e-12) {
        double delta = 0.5 * (y1 - y3) / denom; // ~[-0.5, +0.5]
        idx = std::clamp(k + delta, 0.0, (double)(N - 1));
      }
    }
  }
  return idx;
}





double autofocus::computeBestFocusRatioGaussian(cv::Mat image, int imgHeight,
                                           int imgWidth) {

 auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/2 x 1/2
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross gradients -> sharpness image (float)
  auto robertsStart = std::chrono::high_resolution_clock::now();
  cv::Mat img_x, img_y;
  cv::filter2D(resized, img_x, CV_16S, roberts_kernelx);
  cv::filter2D(resized, img_y, CV_16S, roberts_kernely);
  cv::Mat img_x_abs, img_y_abs, sum_xy;
  img_x_abs = cv::abs(img_x);
  img_y_abs = cv::abs(img_y);
  cv::add(img_x_abs, img_y_abs, sum_xy);
  cv::Mat sharpness_float;
  sum_xy.convertTo(sharpness_float, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();


  // Column means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Reduced Roberts Cross (vertical/horizontal)
  auto reducedRobertsStart = std::chrono::high_resolution_clock::now();
  cv::Mat reduced_roberts_x, reduced_roberts_y;
  cv::filter2D(resized, reduced_roberts_x, CV_16S, reduced_roberts_kernelx);
  cv::filter2D(resized, reduced_roberts_y, CV_16S, reduced_roberts_kernely);
  cv::Mat sum_xy_reduced;
  cv::Mat reduced_x_abs = cv::abs(reduced_roberts_x);
  cv::Mat reduced_y_abs = cv::abs(reduced_roberts_y);
  cv::add(reduced_x_abs, reduced_y_abs, sum_xy_reduced);
  cv::Mat sharpness_float_reduced;
  sum_xy_reduced.convertTo(sharpness_float_reduced, CV_32F);
  auto reducedRobertsEnd = std::chrono::high_resolution_clock::now();

  // Column Means reduced
  auto columnReducedStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrixReduced;
  cv::reduce(sharpness_float_reduced, columnMeansMatrixReduced, 0, cv::REDUCE_AVG, CV_64F);
  std::vector<double> columnMeansReduced;
  columnMeansMatrixReduced.copyTo(columnMeansReduced);


  // Set the first value equal to the second
  if (columnMeansReduced.size() > 1) {
      columnMeansReduced[0] = columnMeansReduced[1];
  }

  auto columnReducedEnd = std::chrono::high_resolution_clock::now();

  


  // Compute ratio - 1
  auto ratioStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve_raw;
  sharpnesscurve_raw.reserve(columnMeans.size());
  for (size_t i = 0; i < columnMeans.size(); ++i) {
      if (columnMeansReduced[i] != 0.0)
          sharpnesscurve_raw.push_back(columnMeans[i] / columnMeansReduced[i] - 1.0);
      else
          sharpnesscurve_raw.push_back(0.0); // or handle divide-by-zero as needed
  }
  auto ratioEnd = std::chrono::high_resolution_clock::now();

  // Non-iterative Gaussian fit (weighted least squares)
  auto gaussianFitStart = std::chrono::high_resolution_clock::now();
  cv::Mat x_squared, y_squared, ln_y;

  // Only keep positive values of y (since you're taking log)
  std::vector<double> sharpnesscurve;
  for (const auto& val : sharpnesscurve_raw) {
      if (val > 0.0) {
          sharpnesscurve.push_back(val);
      }
  }


  double sum_11 = 0.0, sum_33 = 0.0;
  double sum_12 = 0.0, sum_13 = 0.0, sum_23 = 0.0;
  double b_11 = 0.0, b_12 = 0.0, b_13 = 0.0;

  for (int i = 0; i < sharpnesscurve.size(); ++i) {
      //y_squared = std::pow(sharpnesscurve[i], 2);
      double y = sharpnesscurve[i];
      double y2 = y * y;
      double x = static_cast<double>(i);
      double x2 = x * x;
      double x3 = x2 * x;
      double x4 = x2 * x2;
      double ln_y = std::log(y);

      sum_11 += y2;
      sum_12 += i * y2;
      sum_13 += x2 * y2;
      sum_23 += x3 * y2;
      sum_33 += x4 * y2;

      b_11 += y2 * ln_y;
      b_12 += i * y2 * ln_y;
      b_13 += x2 * y2 * ln_y;
  }

  double sum_21 = sum_12; // Symmetry
  double sum_31 = sum_13; // Symmetry
  double sum_22 = sum_13; 
  double sum_32 = sum_23; // Symmetry

  // Define A and b
  cv::Mat A = (cv::Mat_<double>(3, 3) << sum_11, sum_12, sum_13, 
                sum_21, sum_22, sum_23,
               sum_31, sum_32, sum_33);

  cv::Mat b = (cv::Mat_<double>(3, 1) << b_11, b_12, b_13);

  // Solve for parameters
  cv::Mat x;
  cv::solve(A, b, x, cv::DECOMP_NORMAL);
  double coeff_b = x.at<double>(1, 0);
  double c = x.at<double>(2, 0);
  double mu = -coeff_b / (2 * c);
  auto gaussianFitEnd = std::chrono::high_resolution_clock::now();
  //std::cout << "Estimated mu (Gaussian fit): " << mu << std::endl;
  // std::cout << "b: " << coeff_b << ", c: " << c << std::endl;
  // std::cout << "Range of sharpness curve: min = " << *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end())
  //           << ", max = " << *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end()) << std::endl;

  // Save range of values after each stage for debugging
  if (bSaveSharpnessCurves) {
    // std::cout << "Saving debug files" << std::endl;

    // UNCOMMENT TO SAVE TXT FILES
    // std::string FileName = "TESTING_GAUSSIAN_" + std::to_string(increment3);
    // std::string TextFile1 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_resized.txt";
    // std::string TextFile2 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_robertscross.txt";
    // std::string TextFile3 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_reduced_robertscross.txt";
    // std::string TextFile4 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_column_means.txt";
    // std::string TextFile5 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_reduced_column_means.txt";
    // std::string TextFile6 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_sharpnesscurve.txt";
    // std::ofstream outputFile1(TextFile1);
    // std::ofstream outputFile2(TextFile2);
    // std::ofstream outputFile3(TextFile3);
    // std::ofstream outputFile4(TextFile4);
    // std::ofstream outputFile5(TextFile5);
    // std::ofstream outputFile6(TextFile6);


    // std::ostream_iterator<double> output_iterator(outputFile1, ", ");
    // for (int i = 0; i < resized.rows; ++i) {
    //   const uchar* row_ptr = resized.ptr<uchar>(i);
    //   for (int j = 0; j < resized.cols; ++j) {
    //     outputFile1 << static_cast<double>(row_ptr[j]) << ", ";
    //   }
    // }
    // outputFile1 << "\n";

    // std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    // for (int i = 0; i < sharpness_float.rows; ++i) {
    //   const float* row_ptr = sharpness_float.ptr<float>(i);
    //   for (int j = 0; j < sharpness_float.cols; ++j) {
    //     outputFile2 << static_cast<double>(row_ptr[j]) << ", ";
    //   }
    // }
    // outputFile2 << "\n";

    // std::ostream_iterator<double> output_iterator3(outputFile3, ", ");
    // std::copy(columnMeans.begin(), columnMeans.end(), output_iterator3);
    // outputFile3 << "\n";

    // std::ostream_iterator<double> output_iterator4(outputFile4, ", ");
    // for (int i = 0; i < sharpness_float_reduced.rows; ++i) {
    //     const float* row_ptr = sharpness_float_reduced.ptr<float>(i);
    //     for (int j = 0; j < sharpness_float_reduced.cols; ++j) {
    //         outputFile4 << static_cast<double>(row_ptr[j]) << ", ";
    //     }
    // }
    // outputFile4 << "\n";

    // std::ostream_iterator<double> output_iterator5(outputFile5, ", ");
    // std::copy(columnMeansReduced.begin(), columnMeansReduced.end(), output_iterator5);
    // outputFile5 << "\n";

    // std::ostream_iterator<double> output_iterator6(outputFile6, ", ");
    // std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator6);
    // outputFile6 << "\n";

    // outputFile1.close();
    // outputFile2.close();
    // outputFile3.close();
    // outputFile4.close();
    // outputFile5.close();
    // outputFile6.close();






    // UNCOMMENT TO SAVE ROBERTS CROSS IMAGES
    // Plot Roberts Cross and reduced Roberts Cross
    // cv::Mat sharpness_norm, sharpness_8u, sharpness_color;
    // cv::normalize(sharpness_float, sharpness_norm, 0, 255, cv::NORM_MINMAX);
    // sharpness_norm.convertTo(sharpness_8u, CV_8U);


    // cv::Mat reduced_sharpness_norm, reduced_sharpness_8u, reduced_sharpness_color;
    // cv::normalize(sharpness_float_reduced, reduced_sharpness_norm, 0, 255, cv::NORM_MINMAX);
    // reduced_sharpness_norm.convertTo(reduced_sharpness_8u, CV_8U);

    // 4. Save or display the overlay image
    // std::string RC_Path = "../output/SharpnessCurves_Gaussian/" + std::to_string(increment3) + "_RC.png";
    // cv::imwrite(RC_Path, sharpness_8u);

    // std::string Reduced_RC_Path = "../output/SharpnessCurves_Gaussian/" + std::to_string(increment3) + "_Reduced_RC.png";
    // cv::imwrite(Reduced_RC_Path, reduced_sharpness_8u);


    // UNCOMMENT TO SAVE SHARPNESS CURVE WITH MEAN OVERLAY AND TILTED CAM IMAGE
    // Save sharpness curve as png with fitted Gaussian mean overlay
    lastSharpnessCurve = sharpnesscurve;

    // Convert processed_img to color if it's grayscale to match the graph image
    // type
    cv::Mat colorResized;
    if (resized.channels() == 1) {
      cv::cvtColor(resized, colorResized, cv::COLOR_GRAY2BGR);
    } else {
      colorResized = image.clone();
    }


    // draw vertical line for mu
    cv::line(colorResized, cv::Point(mu, 0),
              cv::Point(mu, colorResized.rows),
              cv::Scalar(255, 255, 0), 2);

    cv::Mat combined;

    // Draw graph if we have curve data
    if (!lastSharpnessCurve.empty()) {
      try {
        // Create graph image with same width as resized image
        int graphHeight = 200;
        int graphWidth = colorResized.cols;
        cv::Mat graphImage =
            cv::Mat::zeros(graphHeight, graphWidth, CV_8UC3);

        
        // The sharpness curve starts at kernel/2 and has (imageWidth -
        // kernel) points For reduced resolution: starts at 10, has (640/2
        // - 20) = 300 points
        // int curveStartX =
        //     10; // kernel/2 for reduced resolution (20/2 = 10)
        int curveWidth = lastSharpnessCurve.size();



        // // Find max value in sharpness curve for scaling
        double maxSharpness =
            lastSharpnessCurve.empty()
                ? 0.0
                : *std::max_element(lastSharpnessCurve.begin(),
                                    lastSharpnessCurve.end());
        double minSharpness =
            lastSharpnessCurve.empty()
                ? 0.0
                : *std::min_element(lastSharpnessCurve.begin(),
                                    lastSharpnessCurve.end());
        double range = maxSharpness - minSharpness;



        // Set fixed y-axis range
        double yAxisMin = 0.0;
        double yAxisMax = maxSharpness;
        double maxVal = yAxisMax;


        // if (maxVal <= 0)
        //   maxVal = 1.0;

        // Draw sharpness curve (blue) - map curve indices to correct x
        // positions
        for (size_t i = 1; i < lastSharpnessCurve.size(); i++) {
          int x1 = (i - 1);
          int x2 = i;

          // Only draw if within graph bounds
          if (x1 >= 0 && x2 < graphWidth) {
            int y1 =
                graphHeight -
                static_cast<int>((lastSharpnessCurve[i - 1] / maxVal) *
                                (graphHeight - 50));
            int y2 = graphHeight -
                    static_cast<int>((lastSharpnessCurve[i] / maxVal) *
                                      (graphHeight - 50));
            cv::line(graphImage, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(255, 0, 0), 1); // Blue
          }
        }
  // draw vertical line for locBestFocusDouble
        cv::line(graphImage, cv::Point(mu, 0),
                  cv::Point(mu, graphHeight),
                  cv::Scalar(255, 255, 0), 2);

  // Find amplitude of locBestFocus
        int sharpnessIdx = static_cast<int>(std::round(mu));
        double sharpnessAtBest = (sharpnessIdx >= 0 && sharpnessIdx < lastSharpnessCurve.size())
              ? lastSharpnessCurve[sharpnessIdx]
              : 0.0;

        // Display range, COM, amplitude of BestFocus position with double precision
        
        std::string rangeText =
            "Range: " + std::to_string(range).substr(0, 6);
        std::string comText =
            "COM: " + std::to_string(mu).substr(0, 8);
        std::string sharpnessText = "LoBF sharpness: " + std::to_string(sharpnessAtBest).substr(0, 8);
        

        // Add legend text and amplitude
        cv::putText(graphImage, "Sharpness Curve", cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0),
                    1);
        cv::putText(graphImage, "Center of Mass", cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 0), 1);

        // Place the values just below the legend
        cv::putText(graphImage, sharpnessText, cv::Point(200, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        cv::putText(graphImage, rangeText, cv::Point(450, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        cv::putText(graphImage, comText, cv::Point(200, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

        // Draw axes on the graphImage
        int tickLength = 6;
        int numXTicks = 6;
        int numYTicks = 4;

        // Draw x-axis (horizontal line at the bottom)
        cv::line(graphImage, cv::Point(0, graphHeight - 1), cv::Point(graphWidth - 1, graphHeight - 1), cv::Scalar(200, 200, 200), 1);

        // Draw y-axis (vertical line at the left)
        cv::line(graphImage, cv::Point(0, 0), cv::Point(0, graphHeight - 1), cv::Scalar(200, 200, 200), 1);

        // Draw x-axis ticks and optional labels
        for (int i = 0; i <= numXTicks; ++i) {
            int x = i * (graphWidth - 1) / numXTicks;
            cv::line(graphImage, cv::Point(x, graphHeight - 1), 
                    cv::Point(x, graphHeight - 1 - tickLength), 
                    cv::Scalar(200, 200, 200), 1);
            int labelValue = i * (curveWidth - 1) / numXTicks;
            cv::putText(graphImage, std::to_string(labelValue), 
                        cv::Point(x + 2, graphHeight - 1 - 5), 
                        cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(180,180,180), 1);
        }

        for (int i = 0; i <= numYTicks; ++i) {
            int y = graphHeight - static_cast<int>(i * (graphHeight - 50) / numYTicks);
            double labelValue = yAxisMin + (yAxisMax - yAxisMin) * i / numYTicks;
            cv::line(graphImage, cv::Point(0, y), cv::Point(tickLength, y), cv::Scalar(200, 200, 200), 1);
            cv::putText(graphImage, cv::format("%.1f", labelValue), cv::Point(2, y - 2),
                        cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(180,180,180), 1);
        }

  //axes



        // Combine image and graph
        cv::vconcat(colorResized, graphImage, combined);

        // Save with integer filename
        std::string FilePath = "../output/TiltedCam_Images/" +
                                std::to_string(increment3) + "_" +
                                std::to_string(static_cast<int>(
                                    std::round(mu))) +
                                ".png";
        cv::imwrite(FilePath, combined);
      } catch (const std::exception &e) {
        // Just save the original image if we can't combine
        std::string FilePath = "../output/TiltedCam_Images/" +
                                std::to_string(increment3) + "_" +
                                std::to_string(static_cast<int>(
                                    std::round(mu))) +
                                ".png";
        cv::imwrite(FilePath, colorResized);
        std::cout << "Error combining images: " << e.what() << std::endl;
      }
    } else {
      // Just save the original image if no curve data
      std::string FilePath = "../output/TiltedCam_Images/" +
                              std::to_string(increment3) + "_" +
                              std::to_string(static_cast<int>(
                                  std::round(mu))) +
                              ".png";
      cv::imwrite(FilePath, colorResized);
    }

    increment3++;
  
  }



  lastSharpnessCurve = sharpnesscurve;



  auto estEnd = std::chrono::high_resolution_clock::now();


  // --- Timing log unchanged footprint, writing "com_time_us" as estimator time
  // ---
  auto totalTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - startTime);
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      resizeEnd - resizeStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto reducedRobertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      reducedRobertsEnd - reducedRobertsStart);
  auto columnReducedTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnReducedEnd - columnReducedStart);
  auto ratioTime = std::chrono::duration_cast<std::chrono::microseconds>(
      ratioEnd - ratioStart);
  auto gaussianFitTime = std::chrono::duration_cast<std::chrono::microseconds>(
      gaussianFitEnd - gaussianFitStart);
  
  try {
    std::ofstream leastSquaresBenchmarkFile("../output/focus_benchmark_LeastSquares.csv",
                                       std::ios::app);
    if (leastSquaresBenchmarkFile.is_open() && leastSquaresBenchmarkFile.good()) {
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      // keep CSV header compatibility: write estimator time in the
      // "com_time_us" column
      leastSquaresBenchmarkFile << timestamp << "," << totalTime.count() << ","
                           << resizeTime.count() << "," << robertsTime.count()
                           << "," << columnTime.count() << ","
                           << reducedRobertsTime.count() << "," << columnReducedTime.count()
                           << "," << ratioTime.count() << ","
                           << gaussianFitTime.count() << std::endl;
      leastSquaresBenchmarkFile.close();
    }
  } catch (...) { /* ignore */
  }

  // Return mean from Gaussian fit
  return mu;
}