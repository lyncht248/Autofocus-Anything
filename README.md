<div align="center">
<h2>Video Autofocus using an Adjustable-Focus Air-Lens and Sharpness Analysis of an Inclined Image Plane
</h2>

[**Thomas Lynch**](https://github.com/lyncht248/)<sup>1</sup> · [**Paul A R Meyer**](https://scholar.google.com/citations?user=NmHgX-wAAAAJ)<sup>1</sup><sup>, 2</sup> · [**Thierry Savin**](http://savinlab.eng.cam.ac.uk/)<sup>1, &dagger;</sup> 

<sup>1</sup>Dept. of Engineering, Cambridge University&emsp;&emsp;&emsp;&emsp;<sup>2</sup>Dept. of Ophthalmology, Addenbrooke's Hospital

&dagger;corresponding author

</div>

<p align="center">
    <img width=80% src="./assets/teaser-resize.gif">
</p>

This repository contains the embedded software and desktop application used to operate the _Autofocus-Anything (AA)_ modular device. This device can be mounted on the camera port of any standard microscope or slit-lamp, enabling high-precision autofocus at video rate. 

## Requirements

Package dependencies: cmake pkg-config libgtkmm-3.0-dev libtoon-dev libcvd-dev libsdl2-dev libusb-1.0-0-dev liblapack-dev

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

This project is released under the MIT License for academic use. For commercial use, please contact [tel32@cam.ac.uk](mailto:tel32@cam.ac.uk). See [LICENSE](LICENSE) for full details.

## Acknowledgements
* Paul Meyer MD, FRCP for optical designs of the autofocus device 
* Dr. Thierry Savin for supervising and providing engineering input
