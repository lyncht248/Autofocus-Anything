#include "stabiliser.hpp"
#include "cairomm/enums.h"
#include "cairomm/surface.h"

#include <cvd/image_ref.h>
#include <cvd/vector_image_ref.h>
#include <cvd/convolution.h>
#include <cvd/image_io.h>
#include <cvd/byte.h>
#include <TooN/helpers.h>
#include <TooN/irls.h>

#include <iostream>
#include <fstream>
#include <vector>
#include "main.hpp"
#include "sdlwindow.hpp"

using namespace CVD;
using namespace TooN;
using namespace std;

// non max suppression
template<class T>
void get_max(BasicImage<T>& im,
	     BasicImage<Vector<2> >& direction,
	     ImageRef border,
	     T threshold,
	     vector<ImageRef>& maximal);

// get the largest eigen value and eigen vector of the hessian of the image
void get_large_eig(BasicImage<double>& im,
		   ImageRef border,
		   BasicImage<double>& e_val,
		   BasicImage<Vector<2> >& e_vector);


// const double vessel_size=2.0; // around what size are the blood vessels

void Stabiliser::make_map(const CVD::BasicImage<unsigned char>& im, int num_trackers){
  my_num_trackers=num_trackers;
  ImageRef size = im.size();
  tracker_edgels.resize(num_trackers);
  vessel_edgels.resize(num_trackers);

  //std::cout << "vessel_edgels.size: "<< vessel_edgels.size() << std::endl;

  dim.resize(size);
  ImageRef scan;
  scan.home();
  do{
    ImageRef hscan(scan);
    dim[scan]=im[scan];
  } while(scan.next(size));

  //for predrawing
  frameWidth = size.x;
  frameHeight = size.y;

  recompute_chains();
}


void Stabiliser::adjust_scale(double scale){
  vessel_size=scale;
  // cerr << "new scale is " << scale << " and threshold is " << hessian_thresh << endl;
  recompute_chains();
}


void Stabiliser::adjust_thresh(double thresh){
  hessian_thresh=thresh;
  recompute_chains();
}


void Stabiliser::recompute_chains(){
  ImageRef size=dim.size();
  if (!size.x || !size.y) //no map created
  {
	  valid = false;
	  return;
  }

  Image<double> blurred(size);
  convolveGaussian(dim,blurred,vessel_size,3.0);
  
  // get the direction of the maximum second derivative 
  // at each point and its value
  lambda1.resize(size);
  direction.resize(size);
  border=ImageRef(VesselEdgel::tracker_length, VesselEdgel::tracker_length);
  get_large_eig(blurred, border, lambda1, direction);

  current_chains.clear();
  current_edgel=-1;
  patch_im_valid=false;

  vessels.clear();
  get_max(lambda1, direction, border, hessian_thresh, vessels);

  if(vessels.size()==0){
    valid=false;
    return;
  }


  // select a random subset
  for(int i=0; i<my_num_trackers; i++){
    tracker_edgels[i] = vessels[lrand48()%vessels.size()];
  }

 // get the compass directions for each tracker edgel and initialise the profiles
  for(int i=0; i<my_num_trackers; i++){
    vessel_edgels[i].init(tracker_edgels[i], direction[tracker_edgels[i]], dim);
  }

  vessel_im.resize(size);
  ImageRef scan;
  do{
    vessel_im[scan]=0;
  }while(scan.next(size));

  for(unsigned int i=0; i<vessels.size(); i++){
    vessel_im[vessels[i]]=1;
  }

  endpoints.clear();
  for(unsigned int i=0; i<vessels.size(); i++){
    ImageRef pos= vessels[i];
    int num_neighbours=calc_num_neighbours(pos);
    vessel_im[pos]=num_neighbours; // replace the number in the image with the number of neighbours
    // has side effect of removing pixels with no neighbours

    if(num_neighbours!=2){
      endpoints.push_back(pos);
    }
  }

  // construct chains
  chains.clear();
  for(unsigned int i=0; i<endpoints.size(); i++){
    ImageRef pos = endpoints[i];
    while(vessel_im[pos]>0){
	  chains.push_back(Chain());
      vector<ImageRef>& current = chains.back().pixels;
      current.push_back(pos);
      vessel_im[pos]--; // decrement the neighbour count on the beginning endpoint
      for(;;){
	pos=get_first_neighbour(pos);
	if(vessel_im[pos]==2){
	  vessel_im[pos]=0;
	} else {
	  break;
	}
	current.push_back(pos);
      }
      vessel_im[pos]--; // decrement the neighbour count on the end endpoint
    }
  }



  //cout << "got " << chains.size() << " chains" << endl;


  valid=true;
}

ImageRef compass[8]={
  ImageRef(1,1),
  ImageRef(1,0),
  ImageRef(1,-1),
  ImageRef(0,-1),
  ImageRef(-1,-1),
  ImageRef(-1,0),
  ImageRef(-1,1),
  ImageRef(0,1)
};

int Stabiliser::calc_num_neighbours(const ImageRef& pos){
  int num_neighbours=0;
  bool prev = vessel_im[pos+compass[7]];
  for(int i=1; i<8; i+=2){ // cardinal directions are odd numbers in compass list
    bool current;
    if((current=vessel_im[pos+compass[i]])){
      num_neighbours++;
    } else if ((!prev) && vessel_im[pos+compass[i-1]]) {
      num_neighbours++;
    }
    prev=current;
  }
  return num_neighbours;
}

vector<ImageRef> Stabiliser::get_neighbours(const ImageRef& pos){
  vector<ImageRef> result;
  bool prev = vessel_im[pos+compass[7]];
  for(int i=1; i<8; i+=2){ // cardinal directions are odd numbers in compass list
    bool current;
    if((current=vessel_im[pos+compass[i]])){
      result.push_back(pos+compass[i]);
    } else if ((!prev) && vessel_im[pos+compass[i-1]]) {
      result.push_back(pos+compass[i-1]);
    }
    prev=current;
  }
  return result;
}

ImageRef Stabiliser::get_first_neighbour(const ImageRef& pos){
  bool prev = vessel_im[pos+compass[7]];
  for(int i=1; i<8; i+=2){ // cardinal directions are odd numbers in compass list
    bool current;
    if((current=vessel_im[pos+compass[i]])){
      return pos+compass[i];
    } else if ((!prev) && vessel_im[pos+compass[i-1]]) {
      return pos+compass[i-1];
    }
    prev=current;
  }
  return pos; // no neighbours
}


int sq_diff(const BasicImage<unsigned char>& im,
	    const BasicImage<unsigned char>& patch,
	    const ImageRef offset) {
  int result=0;
  ImageRef scan;
  do{
    int diff = patch[scan]-im[offset+scan];
    result+=diff*diff;
  } while(scan.next(patch.size()));
  return result;
}


//int increment = 0;

Vector<2> Stabiliser::stabilise(CVD::BasicImage<unsigned char>& im,
				const Vector<2>& offset){
  if(!valid){
    return Zeros;
  }

  const int num_trackers=vessel_edgels.size();
    // match the trackers against the new frame

  IRLS<2,double, RobustI> irls;
  irls.sd_inlier=1.0;

  Vector<2> this_offset = Zeros;

  for(int iteration=0; iteration<6; iteration++){
    irls.clear();
    ImageRef pixel_offset=ir_rounded(this_offset+offset);
    Vector<2> offset_error = vec(pixel_offset) - (this_offset+offset);

    for(int i=0; i<num_trackers; i++){
      double movement = vessel_edgels[i].best_match(im,pixel_offset);
      if(movement != VesselEdgel::not_found){
	irls.add_mJ(movement,vessel_edgels[i].my_direction);
      }
      

    }
    irls.compute();
    this_offset+=irls.get_mu() + offset_error;
  }

  ImageRef pixel_offset=ir_rounded(this_offset+offset);  // add this amount to every pixel in the chain
  ImageRef patch_offset = patch_im.size()/2;

#if 0

  // track the patch into the current image

  if(patch_im_valid){
    if(track_frame_count < 5){
      track_frame_count++;
    } else {
      track_frame_count=0;
      ImageRef reference_pos=chains[current_chain].pixels[current_edgel]+pixel_offset-patch_offset;
      ImageRef best_motion(0,0);
      int best_error=sq_diff(im, patch_im, reference_pos);
      ImageRef min(-20,-20);
      ImageRef max(20,20);
      ImageRef scan(min);
      do{
	int error = sq_diff(im,patch_im, reference_pos+scan);
	if(error < best_error){
	  best_motion=scan;
	  best_error=error;
	}
      } while(scan.next(min,max));
      cerr << best_motion << endl;
    }
  }



  // grab a copy of the patch for tracking (if we can)
  if(current_chain!=-1){
    ImageRef scan;
    do{
      patch_im[scan] = im[scan+chains[current_chain].pixels[current_edgel]+pixel_offset-patch_offset];
      im[scan+chains[current_chain].pixels[current_edgel]+pixel_offset-patch_offset]=0;
    } while (scan.next(patch_im.size()));
    patch_im_valid=true;
  }

#endif

  return this_offset;
}


// get the largest eigen value and eigen vector of the hessian of the image
// the image should be pre-blurred to make this work well
void get_large_eig(BasicImage<double>& im,
		   ImageRef border,
		   BasicImage<double>& e_val,
		   BasicImage<Vector<2> >& e_vector){

  const ImageRef dx(1,0);
  const ImageRef dy(0,1);

  ImageRef size(im.size());
  ImageRef max(size-border);

  ImageRef scan=border;
  do{
    // Hessian of the (blurred) image of the form
    // [ a b ]
    // [ b c ]

    double a = im[scan-dx]+im[scan+dx]-2*im[scan];
    double c = im[scan-dy]+im[scan+dy]-2*im[scan];
    double b = 0.25*(im[scan-dx-dy] + im[scan+dx+dy] - im[scan-dx+dy] - im[scan+dx-dy]);
    double det = a*c - b*b;
    double trace2 = 0.5*(a+c); // half of the trace

    double large_e_val = trace2 + sqrt(trace2*trace2-det);
    e_val[scan]=large_e_val;

    if(fabs(a) > fabs(c)){
      e_vector[scan][0]=large_e_val-c;
      e_vector[scan][1]=b;
    } else {
      e_vector[scan][0]=b;
      e_vector[scan][1]=large_e_val-a;
    }
    e_vector[scan]*=1.0/norm(e_vector[scan]);  // L2 norm=1
  } while(scan.next(border,max));
}





// non max suppression
// compare the value of im to the bilinear interpolation between
// two of the eight neighbours determined by direction
// and compare to the opposite direction too
// retain if bigger than both *and* the supplied threshold
template<class T>
void get_max(BasicImage<T>& im,
	     BasicImage<Vector<2> >& direction,
	     ImageRef border,
	     T threshold,
	     vector<ImageRef>& maximal){

  const ImageRef dx(1,0);
  const ImageRef dy(0,1);

  ImageRef size(im.size());
  ImageRef max(size-border);

  ImageRef scan=border;
  do{
    if(im[scan] > threshold){
      double comp1,comp2;
      Vector<2> dir = direction[scan];
      if(fabs(dir[0]) > fabs(dir[1])){
	dir/=dir[0];
	if(dir[1]>0){
	  comp1=im[scan+dx]*(1-dir[1]) + im[scan+dx+dy]* dir[1];
	  comp2=im[scan-dx]*(1-dir[1]) + im[scan-dx-dy]* dir[1];
	} else {
	  comp1=im[scan+dx]*(1+dir[1]) - im[scan+dx-dy]* dir[1];
	  comp2=im[scan-dx]*(1+dir[1]) - im[scan-dx+dy]* dir[1];
	}
      } else {
	dir/=dir[1];
	if(dir[0]>0){
	  comp1=im[scan+dy]*(1-dir[0]) + im[scan+dx+dy]* dir[0];
	  comp2=im[scan-dy]*(1-dir[0]) + im[scan-dx-dy]* dir[0];
	} else {
	  comp1=im[scan+dy]*(1+dir[0]) - im[scan-dx+dy]* dir[0];
	  comp2=im[scan-dy]*(1+dir[0]) - im[scan+dx-dy]* dir[0];
	}
      }
      if((im[scan] > comp1) && (im[scan] > comp2)){
	maximal.push_back(scan);
      }
    }
  }while(scan.next(border,max));
}

int sqdist(ImageRef& p, int x, int y){
	int dx = p.x-x;
	int dy = p.y-y;
	return dx*dx + dy*dy;
}


void Stabiliser::click(int x, int y, bool shiftdown){
	int closest_dist;
	if(!chains.empty()){
		closest_dist = sqdist(chains.begin()->pixels[0],x,y);
	} else if (!persistent_chains.empty()){
		closest_dist = sqdist(persistent_chains.begin()->pixels[0],x,y);
	} else {
		return;
	}

	list<Chain>::iterator best_it;
	list<Chain>::iterator it;
	for(it=persistent_chains.begin(); it!=persistent_chains.end(); it++){
		for(unsigned int j=0; j<it->pixels.size(); j++){
			int dist = sqdist(it->pixels[j],x,y);
			if(dist < closest_dist){
				best_it=it;
				current_edgel=j;
				closest_dist=dist;
			}
		}
	}

	for(it=chains.begin(); it!=chains.end(); it++){
		for(unsigned int j=0; j<it->pixels.size(); j++){
			int dist = sqdist(it->pixels[j],x,y);
			if(dist < closest_dist){
				best_it=it;
				current_edgel=j;
				closest_dist=dist;
			}
		}
	}

	if(shiftdown)
	{
		set<list<Chain>::iterator>::iterator in_set=current_chains.find(best_it);
		if(in_set==current_chains.end()){
			current_chains.insert(best_it);
		} else {
			current_chains.erase(in_set);
		}
		current_edgel=-1;
	} else {
		current_chains.clear();
		current_chains.insert(best_it);
	}
	track_frame_count=0;
}

void Stabiliser::key(int keyval){
	if(chains.empty() || current_chains.empty()) return;

	cerr << "keyval is " << keyval << endl;

	int level=100;

	if(keyval >= 48 && keyval < 58){
		level=keyval-48;
	}
	if(keyval >=65456 && keyval < 65466){
		level=65456-keyval;
	}

	if(level!=100) {
		set<list<Chain>::iterator>::iterator it;
		for(it=current_chains.begin(); it!=current_chains.end();it++){
			if((*it)->level!=100){
				(*it)->level=level;
			} else {
				persistent_chains.push_back(Chain());
				Chain* new_chain=&(persistent_chains.back());
				new_chain->pixels.assign((*it)->pixels.begin(),(*it)->pixels.end());
				new_chain->level = level;
				current_chains.erase(it);
				list<Chain>::iterator nit=persistent_chains.end();
				nit--;
				current_chains.insert(nit);
			}
		}
	}

	if(keyval == 65288) { // delete
		set<list<Chain>::iterator>::iterator it;
		for(it=current_chains.begin(); it!=current_chains.end();it++){
			if((*it)->level!=100){
				persistent_chains.erase(*it);
			}
		}
		current_chains.clear();
		current_edgel=-1;
	}

	if(keyval == 's' && !current_chains.empty() && current_edgel!=-1){
		Chain& this_chain = **(current_chains.begin());
		Chain* new_chain;
		if(this_chain.level==100){
			chains.push_back(Chain());
			new_chain=&(chains.back());
		} else {
			persistent_chains.push_back(Chain());
			new_chain=&(persistent_chains.back());
		}

		vector<ImageRef>::iterator it0;
		vector<ImageRef>::iterator it1;
		vector<ImageRef>::iterator it2;
		it0=this_chain.pixels.begin();
		it1=this_chain.pixels.begin()+current_edgel;
		it2=this_chain.pixels.end();
		new_chain->pixels.assign(it1,it2);
		this_chain.pixels.assign(it0,it1);
		new_chain->level = this_chain.level;
		current_chains.clear();
		current_edgel=-1;
	}
	
	if(keyval == 'j'){
		chains.push_back(Chain());
		vector<ImageRef>& pixels = chains.back().pixels;
		set<list<Chain>::iterator>::iterator it;
		for(it = current_chains.begin(); it!=current_chains.end(); it++){
			pixels.insert(pixels.end(),(*it)->pixels.begin(), (*it)->pixels.end());
			if((*it)->level==100){
				chains.erase(*it);
			} else {
				persistent_chains.erase(*it);
			}
		}
		current_chains.clear();
		current_edgel=-1;
		list<Chain>::iterator nchain=chains.end();
		nchain--;
		current_chains.insert(nchain);
	}
}

static bool _crFirst = true;

static void crVertex(const ::Cairo::RefPtr< ::Cairo::Context>& cr, CVD::ImageRef &pixel)
{
	if (_crFirst)
	{
		cr->move_to(pixel.x, pixel.y);
		_crFirst = false;
	}
	else
	{
		cr->line_to(pixel.x, pixel.y);
	}
}

static void crVertex(const ::Cairo::RefPtr< ::Cairo::Context>& cr, std::vector<CVD::ImageRef> &pixels)
{
	for (CVD::ImageRef &pixel : pixels)
		cr->line_to(pixel.x, pixel.y);
}

static void crEnd(const ::Cairo::RefPtr< ::Cairo::Context>& cr)
{
	cr->stroke();
	_crFirst = true;
}

void Chain::draw(const ::Cairo::RefPtr< ::Cairo::Context>& cr, bool setcol)
{
	if(setcol){
		if(level==100){
			cr->set_source_rgba(0,1,0,1);
		} else {
			cr->set_source_rgba(((3.0+level)/6.0),0,((3.0-level)/6.0),1);
		}
	}
	crVertex(cr, pixels);
	crEnd(cr);
}

void Stabiliser::draw(::Cairo::RefPtr< ::Cairo::ImageSurface> cr)
{
}

static void compress(std::vector<AbsPixel> &pixels, ::Cairo::RefPtr<const ::Cairo::ImageSurface> surface)
{
	pixels.clear();
	const uint32_t *buf = (const uint32_t *) surface->get_data();
	int stride = surface->get_stride() / 4;
	int size = surface->get_width() * surface->get_height();
	for (int i = 0; i < size; i++)
	{
		if (buf[i] >> 24) //alpha value as per Cairo::FORMAT_ARGB32
		{
			pixels.push_back( (AbsPixel) { (unsigned short) (i % stride), (unsigned short) (i / stride), buf[i] | 0xFF000000} );
		}
	}
}

void Stabiliser::predraw()
{
    // if(valid){
		surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32, frameWidth, frameHeight);
		auto cr = Cairo::Context::create(surface);
		cr->set_source_rgba(0,0,0,0);
		cr->rectangle(0, 0, frameWidth, frameHeight);
		cr->fill();
		cr->set_line_width(1.5);
		list<Chain>::iterator it;
		for(it=chains.begin(); it!=chains.end(); it++){
			it->draw(cr, true);
		}

		for(it=persistent_chains.begin(); it!=persistent_chains.end(); it++){
			it->draw(cr, true);
		}

		set<list<Chain>::iterator>::iterator cit;
		for(cit=current_chains.begin(); cit!=current_chains.end();cit++){
			cr->set_source_rgba(1,1,1,1);
			cr->set_line_width(2.5);
			(*cit)->draw(cr, false);
			cr->set_line_width(1.5);
			(*cit)->draw(cr, true);
		}
		if(!current_chains.empty() && current_edgel!=-1){
			cr->set_source_rgba(0,0,0,1);
			cr->set_line_width(2.5);
			
			crVertex(cr, (*(current_chains.begin()))->pixels[current_edgel]);
			crEnd(cr);
		}
		SDLWindow::updateMap(childwin, surface->get_data(), surface->get_width(), surface->get_height(), surface->get_stride() );
		//compress(compressed, surface);
		// }
}

void Stabiliser::invalidate()
{
	current_chains.clear();
	current_edgel = -1;
	dim = CVD::Image<double>();
	lambda1 = CVD::Image<double>();
	direction = CVD::Image<TooN::Vector<2> >();
	border.x = 0;
	border.y = 0;
	vessels.clear();
	tracker_edgels.clear();
	vessel_edgels.clear();
	endpoints.clear();
	vessel_im = CVD::Image<int>();
	chains.clear();
	persistent_chains.clear();
	patch_im = CVD::Image<unsigned char>(CVD::ImageRef(8,8) );
	patch_im_valid = false;
	track_frame_count = 0;
	compressed.clear();
}

bool Stabiliser::is_valid() const
{
	return valid;
}

int Stabiliser::get_level(){
	if(!current_chains.empty() && current_edgel!=-1){
		int level=(*current_chains.begin())->level;
		if(level==100) level=0;
		return level;
	}
	return 0;
}

int Stabiliser::get_length(){
	if(!current_chains.empty() && current_edgel!=-1){
		return (*current_chains.begin())->pixels.size();
	}
	return 0;
}


void Stabiliser::load(const char* path){
	persistent_chains.clear();
	current_chains.clear();
	current_edgel=-1;

    char filename[1000];
	sprintf(filename, "%s/%s/vesselmap.txt",getenv("HOME"),path);
	std::ifstream ifs(filename);
	if(!ifs) return;
	
	list<Chain>::iterator it;
	for(;;) {
		int level;
		ifs >> level;
		if(!ifs){
			break;
		}
		int size;
		ifs >> size;
		persistent_chains.push_back(Chain());
		Chain& c = persistent_chains.back();
		c.level=level;
		c.pixels.resize(size);
		for(int i=0; i<size; i++){
			ifs >> c.pixels[i];
		}
	}
}

void Stabiliser::save(const char* path){
    char filename[1000];
	sprintf(filename, "%s/%s/vesselmap.txt",getenv("HOME"),path);
	std::ofstream ofs(filename);
	if(!ofs) return;
	
	list<Chain>::iterator it;
	for(it=persistent_chains.begin(); it!=persistent_chains.end(); it++){
		ofs << it->level << " " << it->pixels.size() << endl;
		for(int i=0; i<it->pixels.size(); i++){
			ofs << it->pixels[i] << endl;
		}
	}

	ofs.close();
	sprintf(filename, "%s/%s/vesselsummary.txt",getenv("HOME"),path);
	ofs.open(filename);
	if(!ofs) return;
	for(it=persistent_chains.begin(); it!=persistent_chains.end(); it++){
		ofs << it->level << " " << it->pixels.size() << endl;
	}
}
