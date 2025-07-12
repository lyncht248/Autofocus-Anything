<div align="center">
<h2>Autofocus Anything: Autofocus for Video-Microscopy using an Adjustable-Focus Air-Lens and Sharpness Analysis of an Inclined Image Plane
<img width="468" height="49" alt="image" src="https://github.com/user-attachments/assets/b8f242cf-cda1-4954-8de6-6a07ff4531b5" />
</h2>

[**Thomas Lynch**](https://github.com/lyncht248/)<sup>1</sup> · [**Paul A R Meyer**](https://scholar.google.com/citations?user=NmHgX-wAAAAJ)<sup>1</sup><sup>2</sup> · [**Thierry Savin**](http://savinlab.eng.cam.ac.uk/)<sup>1&dagger;</sup> 

<sup>1</sup>Department of Engineering, Cambridge University&emsp;&emsp;&emsp;&emsp;<sup>2</sup>Department of Ophthalmology, Addenbrooke's Hospital

&dagger;corresponding author

<a href="https://arxiv.org/abs/2401.10891"><img src='https://img.shields.io/badge/arXiv-Depth Anything-red' alt='Paper PDF'></a>
<a href='https://depth-anything.github.io'><img src='https://img.shields.io/badge/Project_Page-Depth Anything-green' alt='Project Page'></a>
<a href='https://huggingface.co/spaces/LiheYoung/Depth-Anything'><img src='https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-Spaces-blue'></a>
<a href='https://huggingface.co/papers/2401.10891'><img src='https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-Paper-yellow'></a>
</div>

This is the embedded software and desktop application used to operate the Autofocus Anything device, which is a modular XXX that can be attached to the camera port of any standard microscope or slit-lamp to allow imaging with real-time precise autofocus at any magnification. 

## Requirements

Package dependencies: cmake pkg-config libgtkmm-3.0-dev libtoon-dev libcvd-dev libsdl2-dev libusb-1.0-0-dev liblapack-dev

Other libraries 
Vimba: Copy the .so files in Vimba_6_0/VimbaCPP/DynamicLib/x86_64bit to your system's library path (e.g. /usr/lib/)

## Build Commands

```bash
mkdir build
cd build
cmake .. # add -DCMAKE_BUILD_TYPE=Release for production
sudo make
```
The resulting executable is placed in `build/`.


### Important

If moving the app to another location, ensure you have read-write privileges there and make sure the VimbaGigETL.cti file is present alongside the executable.


## Development

Most modules ship with dedicated tests under `test/`.

```bash
# Example: build & run autofocus tests
sudo make autofocus_test   # builds the specific test target
./autofocus_test
```


## File Structure

```
include/   ── Public headers
src/       ── Implementation (.cpp) files
lib/       ── Pre-compiled third-party libraries
Vimba_6_0/ ── Bundled Allied Vision SDK (optional, system-wide install preferred)
test/      ── GoogleTest / GoogleMock unit tests
```


## License

This project is released under the MIT License for academic use. For commercial use, please contact [lyncht248@gmail.com](mailto:lyncht248@gmail.com). See [LICENSE](LICENSE) for full details.

## Acknowledgements


* Paul Meyer MD, FRCP for optical concepts and designs of the autofocus-anything device
* Dr. Thierry Savin for supervising and providing engineering input
