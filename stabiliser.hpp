// Initially written by Tom Drummond in 2014. 

#ifndef STABILISER_H
#define STABILISER_H

#include <cvd/image.h>
#include <cvd/gl_helpers.h>
#include <TooN/TooN.h>
#include <vector>
#include <list>
#include <set>

#include "vesseledge.hpp"

struct Chain {
	Chain(){
		level=100;
	}

	void draw(bool setcol);

	int level;
	std::vector<CVD::ImageRef> pixels;
};

// Added const here... 
struct compit{
	bool operator() (const std::list<Chain>::iterator& it1, const std::list<Chain>::iterator& it2) const{
		return (&(*it1) < &(*it2));
	}
};



class Stabiliser {
public:
  Stabiliser() : 
    patch_im(CVD::ImageRef(8,8))
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

  void click(int x, int y);
  void key(int keyval);

  void draw();

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
};

#endif
