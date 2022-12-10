// Used by vesseledge to determine the direction of each vessel

#ifndef COMPASS_H
#define COMPASS_H

#include <cvd/image.h>
#include <TooN/TooN.h>
#include <cmath>

inline CVD::ImageRef direction_to_compass(TooN::Vector<2> dir){
  int factor=1;
  if(dir[0] < 0){
    dir*=-1;
    factor=-1; // reverses the output
  }
  if(0.4142 * dir[0] > fabs(dir[1])){ // 0.4142 = tan(22.5 degrees)
    return CVD::ImageRef(factor,0);
  } else if (dir[0] > 0.4142 * fabs(dir[1])){ // 0.4142 = tan (22.5 degrees)
    if(dir[1]>0){
      return CVD::ImageRef(factor,factor);
    } else {
      return CVD::ImageRef(factor,-factor);
    }
  } else {
    if(dir[1]>0){
      return CVD::ImageRef(0,factor);
    } else {
      return CVD::ImageRef(0,-factor);
    }
  }
}

#endif
