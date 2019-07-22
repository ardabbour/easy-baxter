# EASY BAXTER
A collection of useful scripts for the operation of the Baxter Research Robot.

## Dependencies
|             Dependency            | Version Tested On |
|:---------------------------------:|:-----------------:|
| [Ubuntu](https://www.ubuntu.com/) | 16.04             |
| [ROS](https://www.ros.org/)       | Kinetic Kame      |
| [Python](https://www.python.org/) | 2.7.15            |
| [pybullet](https://pybullet.org/) | 2.5.1             |
| [numpy](https://www.numpy.org/)   | 1.16.4            |
| [OpenCV](https://www.opencv.org/) | 4.1.0.25          |

## About
This repository is a ROS package that contains several scripts that
- simplify the usage of main Baxter functions,
- allow the control of the Baxter end-effector using the keyboard,
- publish the feed of an external camera through ROS,
- help find the HSV values needed for simple color filtering, and
- perform pick and place.

## Usage
The scripts are meant to be modified for individual usage, whereas
```easy_baxter.py``` provides a high-level, simplified API for new scripts.

## Contributing
Contributions of all sorts including bug fixes, feature additions, etc. are
welcome and encouraged. First, raise an issue, then follow with a pull request.

## License
[GNU General Public License v3.0](https://github.com/ardabbour/easy-baxter/blob/master/LICENSE).
