



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