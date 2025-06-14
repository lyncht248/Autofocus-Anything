#include "vesseledgel.hpp"

#include <cvd/vector_image_ref.h>

#include "compass.hpp"
#include <iostream>

using namespace std;
using namespace TooN;
using namespace CVD;

void VesselEdgel::init(ImageRef pos,
		       Vector<2> dir,
		       const BasicImage<double>& im){
  
  my_position = pos;
  my_compass = direction_to_compass(dir);
  my_direction=dir;

  // grab a copy of the visual transverse profile
  // of this blood vessel from the supplied image
  ImageRef scan = pos-tracker_length*my_compass;
  double total=0;
  for(int i=0; i<2*tracker_length+1; i++){
    my_profile[i] = (unsigned char)(im[scan]);
    total+=my_profile[i];
    scan=scan+my_compass;
  }
  total/=2*tracker_length+1;
  for(int i=0; i<2*tracker_length+1; i++){
    my_profile[i]-=total;
  }
}


double VesselEdgel::best_match(const BasicImage<unsigned char>& im,
			       const ImageRef& offset){
  
  ImageRef start=my_position+offset-(tracker_length + search_length)*my_compass;
  ImageRef end = my_position+offset+(tracker_length + search_length)*my_compass;

  int best_pos = not_found;

  if(im.in_image(start) && im.in_image(end)){

   double  best_match = 0;

    for(int p = -search_length; p<=search_length; p++){
      ImageRef scan=start;
      double this_match = 0;
      for(int i=0; i<2*tracker_length+1; i++){
        this_match += im[scan]*my_profile[i];
        scan = scan+my_compass;
      }
      if (this_match > best_match){
        best_match = this_match;
        best_pos = p;
      }
      start = start+my_compass;
    }
  }
  if(best_pos==not_found) return not_found;

  // apply a correction factor to the best pos because
  // the compass approximation may not point exactly the same
  // way as my_direction
  // also compensate for direction like compass = (1,1) which has norm root two
  return best_pos * (vec(my_compass) * my_direction);
}

