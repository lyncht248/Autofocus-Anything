// -*- c++ -*-

#ifndef STABILISER_H
#define STABILISER_H

#include <cvd/image.h>
#include <TooN/TooN.h>
#include <vector>
#include <list>
#include <set>

#include "cairomm/surface.h"
#include "vesseledgel.hpp"
#include "framefilter.hpp"


#define DEFAULT_NUM_TRACKERS 600

/**
 * @struct Chain
 * @brief Represents a chain of connected pixels in the vessel map.
 * 
 * Chains are used to track and analyze vessel structures in the image.
 */
struct Chain {
    /**
     * @brief Constructor for Chain.
     * 
     * Initializes the chain with a default level of 100.
     */
    Chain() {
        level = 100;
    }

    /**
     * @brief Draws the chain on a Cairo context.
     * 
     * @param cr Cairo context to draw on.
     * @param setcol Whether to set the color for the chain.
     */
    void draw(const ::Cairo::RefPtr< ::Cairo::Context>& cr, bool setcol);

    int level; ///< Level of the chain.
    std::vector<CVD::ImageRef> pixels; ///< Pixels in the chain.
};

/**
 * @struct compit
 * @brief Comparator for iterators to chains.
 * 
 * Used to compare iterators to chains based on their memory addresses.
 */
struct compit {
    /**
     * @brief Compares two iterators to chains.
     * 
     * @param it1 First iterator.
     * @param it2 Second iterator.
     * @return True if the first iterator points to a lower memory address than the second.
     */
    bool operator()(const std::list<Chain>::iterator& it1, const std::list<Chain>::iterator& it2) const {
        return (&(*it1) < &(*it2));
    }
};

/**
 * @struct AbsPixel
 * @brief Represents an abstract pixel with coordinates and color.
 */
struct AbsPixel {
    unsigned short int x; ///< X-coordinate of the pixel.
    unsigned short int y; ///< Y-coordinate of the pixel.
    uint32_t color; ///< Color of the pixel.
};

/**
 * @class Stabiliser
 * @brief Provides functionality for stabilizing frames and analyzing vessel structures.
 * 
 * The Stabiliser class includes methods for creating vessel maps, adjusting thresholds and scales,
 * stabilizing frames, and interacting with vessel structures.
 */
class Stabiliser : public FrameFilter {
public:
    /**
     * @brief Constructor for Stabiliser.
     * 
     * Initializes the stabilizer with default parameters.
     */
    Stabiliser() : patch_im(CVD::ImageRef(8,8)), compressed() {
        valid = false;
        hessian_thresh = 0.4;
        vessel_size = 2.0;
        current_edgel = -1;
        patch_im_valid = false;
    }

    /**
     * @brief Creates a vessel map from the input image.
     * 
     * @param im Input image.
     * @param num_trackers Number of trackers to use.
     */
    void make_map(const CVD::BasicImage<unsigned char>& im, int num_trackers);

    /**
     * @brief Recomputes chains in the vessel map.
     */
    void recompute_chains();

    /**
     * @brief Adjusts the threshold for vessel detection.
     * 
     * @param thresh New threshold value.
     */
    void adjust_thresh(double thresh);

    /**
     * @brief Adjusts the scale for vessel detection.
     * 
     * @param scale New scale value.
     */
    void adjust_scale(double scale);

    /**
     * @brief Stabilizes the input image based on vessel structures.
     * 
     * @param im Input image.
     * @param offset Offset vector.
     * @return Stabilization offset vector.
     */
    TooN::Vector<2> stabilise(CVD::BasicImage<unsigned char>& im, const TooN::Vector<2>& offset);

    /**
     * @brief Calculates the number of neighbors for a given position.
     * 
     * @param pos Position to check.
     * @return Number of neighbors.
     */
    int calc_num_neighbours(const CVD::ImageRef& pos);

    /**
     * @brief Gets the neighbors of a given position.
     * 
     * @param pos Position to check.
     * @return Vector of neighbor positions.
     */
    std::vector<CVD::ImageRef> get_neighbours(const CVD::ImageRef& pos);

    /**
     * @brief Gets the first neighbor of a given position.
     * 
     * @param pos Position to check.
     * @return First neighbor position.
     */
    CVD::ImageRef get_first_neighbour(const CVD::ImageRef& pos);

    /**
     * @brief Handles mouse click events.
     * 
     * @param x X-coordinate of the click.
     * @param y Y-coordinate of the click.
     * @param shiftdown Whether the shift key is pressed.
     */
    void click(int x, int y, bool shiftdown);

    /**
     * @brief Handles key press events.
     * 
     * @param keyval Key value.
     */
    void key(int keyval);

    /**
     * @brief Prepares the vessel map for drawing.
     */
    void predraw();

    /**
     * @brief Invalidates the current vessel map.
     */
    void invalidate();

    /**
     * @brief Checks whether the stabilizer is valid.
     * 
     * @return True if valid, false otherwise.
     */
    bool is_valid() const;

    /**
     * @brief Draws the vessel map on a Cairo surface.
     * 
     * @param cr Cairo surface to draw on.
     */
    virtual void draw(::Cairo::RefPtr< ::Cairo::ImageSurface> cr);

    /**
     * @brief Gets the level of the current chain.
     * 
     * @return Level of the current chain.
     */
    int get_level();

    /**
     * @brief Gets the length of the current chain.
     * 
     * @return Length of the current chain.
     */
    int get_length();

    /**
     * @brief Loads a vessel map from a file.
     * 
     * @param path Path to the file.
     */
    void load(const char* path);

    /**
     * @brief Saves the vessel map to a file.
     * 
     * @param path Path to the file.
     */
    void save(const char* path);

private:
    double hessian_thresh; ///< Threshold for vessel detection.
    bool valid; ///< Indicates whether the stabilizer is valid.

    double vessel_size; ///< Scale for vessel detection.

    int my_num_trackers; ///< Number of trackers used.

    std::set<std::list<Chain>::iterator, compit> current_chains; ///< Current chains being tracked.

    int current_edgel; ///< Current edgel being tracked.

    CVD::Image<double> dim; ///< Image dimensions.
    CVD::Image<double> lambda1; ///< Lambda values for vessel detection.
    CVD::Image<TooN::Vector<2>> direction; ///< Directions for vessel detection.
    CVD::ImageRef border; ///< Border for vessel detection.

    std::vector<CVD::ImageRef> vessels; ///< Vessel positions.
    std::vector<CVD::ImageRef> tracker_edgels; ///< Tracker edgels.
    std::vector<VesselEdgel> vessel_edgels; ///< Vessel edgels.
    std::vector<CVD::ImageRef> endpoints; ///< Endpoints of vessels.

    CVD::Image<int> vessel_im; ///< Vessel image.

    std::list<Chain> chains; ///< Chains of vessels.
    std::list<Chain> persistent_chains; ///< Persistent chains of vessels.

    CVD::Image<unsigned char> patch_im; ///< Patch image for tracking.
    bool patch_im_valid; ///< Indicates whether the patch image is valid.

    int track_frame_count; ///< Frame count for tracking.
    Cairo::RefPtr<Cairo::ImageSurface> surface; ///< Cairo surface for drawing.
    std::vector<AbsPixel> compressed; ///< Compressed pixel data.
    int frameWidth; ///< Width of the frame.
    int frameHeight; ///< Height of the frame.
};

#endif
