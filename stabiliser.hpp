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

struct Chain {
	Chain(){
		level=100;
	}

	void draw(const ::Cairo::RefPtr< ::Cairo::Context>& cr, bool setcol);

	int level;
	std::vector<CVD::ImageRef> pixels;
};

struct compit{
	bool operator() (const std::list<Chain>::iterator& it1, const std::list<Chain>::iterator& it2) const
	{
		return (&(*it1) < &(*it2));
	}
};

struct AbsPixel
{
	unsigned short int x, y;
	uint32_t color;
};

class Stabiliser : public FrameFilter
{
public:
  Stabiliser() : 
    patch_im(CVD::ImageRef(8,8)),
	compressed()
  {
    valid=false;
    hessian_thresh=0.4;
    vessel_size=2.0;
    current_edgel=-1;
    patch_im_valid=false;
  }

  void make_map(const CVD::BasicImage<unsigned char>& im, int num_trackers);

  void recompute_chains();
  void adjust_thresh(double thresh);
  void adjust_scale(double scale);

  TooN::Vector<2> stabilise(CVD::BasicImage<unsigned char>& im,
			    const TooN::Vector<2>& offset);

  int calc_num_neighbours(const CVD::ImageRef&  pos);
  std::vector<CVD::ImageRef> get_neighbours(const CVD::ImageRef&  pos);
  CVD::ImageRef get_first_neighbour(const CVD::ImageRef& pos);

  void click(int x, int y, bool shiftdown);
  void key(int keyval);

  void predraw();
  void invalidate();

  bool is_valid() const;

  virtual void draw(::Cairo::RefPtr< ::Cairo::ImageSurface> cr);

  int get_level();
  int get_length();

	void load(const char* path);
	void save(const char* path);


private:
  double hessian_thresh;
  bool valid;

  double vessel_size;
  
  int my_num_trackers;

	std::set<std::list<Chain>::iterator,compit> current_chains;

  int current_edgel;

  CVD::Image<double> dim;
  CVD::Image<double> lambda1;
  CVD::Image<TooN::Vector<2> > direction;
  CVD::ImageRef border;

  std::vector<CVD::ImageRef> vessels;
  std::vector<CVD::ImageRef> tracker_edgels;
  std::vector<VesselEdgel> vessel_edgels;
  std::vector<CVD::ImageRef> endpoints;
  // std::vector<CVD::ImageRef> junctions;
  CVD::Image<int> vessel_im;

	// std::vector<std::vector<CVD::ImageRef> > chains;

	std::list<Chain> chains;
	std::list<Chain> persistent_chains;

  CVD::Image<unsigned char> patch_im;
  bool patch_im_valid;

  int track_frame_count;
  Cairo::RefPtr<Cairo::ImageSurface> surface;
  std::vector<AbsPixel> compressed;
  int frameWidth, frameHeight;
};

#endif
