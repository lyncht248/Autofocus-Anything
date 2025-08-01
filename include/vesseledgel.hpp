/**
 * @file vesseledgel.hpp
 * @brief Header file for the VesselEdgel class.
 * 
 * This file defines the VesselEdgel class, which represents an edge element
 * of a blood vessel in an image. It provides methods for initialization and
 * matching profiles to locate vessel edges.
 */

#ifndef VESSEL_EDGEL_H
#define VESSEL_EDGEL_H

#include <TooN/TooN.h>
#include <cvd/image.h>

/**
 * @class VesselEdgel
 * @brief Represents an edge element of a blood vessel in an image.
 * 
 * The VesselEdgel class provides functionality to initialize the edge element
 * with a transverse profile and find the best match for the profile in an image.
 */
class VesselEdgel {
 public:
  /**
   * @brief Length of the tracker profile in pixels on either side of the center.
   */
  static const int tracker_length = 10;

  /**
   * @brief Search range in pixels in each direction.
   */
  static const int search_length = 50;

  /**
   * @brief Value indicating that no match was found.
   */
  static const int not_found = 1000000;

  /**
   * @brief Initializes the VesselEdgel with position, direction, and image profile.
   * 
   * This method sets the position and direction of the edge element and extracts
   * the transverse profile of the blood vessel from the provided image.
   * 
   * @param pos Position of the edge element in the image.
   * @param dir Direction vector of the edge element.
   * @param im Image containing the blood vessel.
   */
  void init(CVD::ImageRef pos,
            TooN::Vector<2> dir,
            const CVD::BasicImage<double>& im);

  /**
   * @brief Finds the best match for the vessel profile in the image.
   * 
   * This method searches for the best match of the transverse profile in the
   * specified image, starting from the current position and applying an offset.
   * 
   * @param im Image to search for the profile match.
   * @param offset Offset to apply to the current position.
   * @return Best match position or `not_found` if no match is found.
   */
  double best_match(const CVD::BasicImage<unsigned char>& im, const CVD::ImageRef& offset);

  // private:

  /**
   * @brief Position of the edge element in the image.
   */
  CVD::ImageRef my_position;

  /**
   * @brief Compass direction vector for scanning the profile.
   */
  CVD::ImageRef my_compass;

  /**
   * @brief Direction vector of the edge element.
   */
  TooN::Vector<2> my_direction;

  /**
   * @brief Transverse profile of the blood vessel.
   * 
   * Array storing the pixel values of the vessel profile.
   */
  double my_profile[2 * tracker_length + 1];
};

#endif
