# EASY BAXTER
A collection of useful scripts for the operation of the Baxter Research Robot.

## Dependencies
|             Dependency            | Version Tested On |
|:---------------------------------:|:-----------------:|
| [Ubuntu](https://www.ubuntu.com/) | 16.04             |
| [ROS](https://www.ros.org/)       | Kinetic Kame      |
| [Python](https://www.python.org/) | 2.7.15            |
| [pybullet](https://pybullet.org/) | 2.4.1             |
| [numpy](https://www.numpy.org/)   | 1.16.0            |
| [OpenCV](https://www.opencv.org/) | 4.0.0.21          |


## About
This repository is a ROS package that contains several scripts that:
- simplify the usage of main Baxter functions,
- allow the control of the Baxter end-effector using the keyboard,
- publish the feed of an external camera through ROS,
- help find the HSV values needed for simple color filtering,
- detect objects and their spatial relations to Baxter,
- simulate detected objects live in a 3D physics engine, and
- perform pick and place operations via visual servoing of Baxter's cameras.


## Usage
The scripts are meant to be modified for individual usage, whereas ```easy_baxter.py``` provides a high-level, simplified API for new scripts.

To pick and place from a flat surface:
- Identify the HSV values needed to filter your objects by ```rosrun```ning ```easy_baxter camera_calibration.py```
- Enter those values (and further modify as needed) ```image_processing.py``` and ```pick_and_place.py```
- ```rosrun``` ```easy_baxter image_processing.py``` and ```easy_baxter pick_and_place.py```.

To simulate in a physics engine objects on a flat surface:
- Identify the HSV values needed to filter your objects by ```rosrun```ning ```easy_baxter camera_calibration.py```
- Define the physical models in the ```data/models``` folder
- Enter those values (and further modify as needed) ```image_processing.py``` and ```simulation.py```
- ```rosrun``` ```easy_baxter image_processing.py``` and ```easy_baxter simulation.py```
- Type ```f``` into the terminal ```rosrun```ning the simulation to freeze the refreshing of the simulation as needed.


## Contributing
Contributions of all sorts including bug fixes, feature additions, etc. are welcome and encouraged.
First, raise an issue, then follow with a pull request.

## License
[GNU General Public License v3.0](https://github.com/ardabbour/easy-baxter/blob/master/LICENSE).
