// -*- c++ -*-

#ifndef VESSEL_EDGEL_H
#define VESSEL_EDGEL_H

#include <TooN/TooN.h>
#include <cvd/image.h>


class VesselEdgel {
 public:
  static const int tracker_length=10; // pixels either side of the centre
  static const int search_length=50;  // search range in each direction
  static const int not_found = 1000000;
  

  void init(CVD::ImageRef pos,
	    TooN::Vector<2> dir,
	    const CVD::BasicImage<double>& im);

  double best_match(const CVD::BasicImage<unsigned char>& im, const CVD::ImageRef& offset);


  // private:
  CVD::ImageRef my_position;
  CVD::ImageRef my_compass;
  TooN::Vector<2> my_direction;
  double my_profile[2*tracker_length+1];
};



#endif
