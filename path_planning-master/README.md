| Gradescope Code Submission         | Wednesday, April 22 at 11:59 PM EST |
|------------------------------------|-------------------------------------|
| Presentation Due Date              | Wednesday, April 22 at 11:59 PM EST |
| Lab Report Due Date                | Friday, April 24 at 11:59PM EST     |

View instructions [here](https://docs.google.com/document/d/1FNYokYjDNTK_pMgk0SvqgELgh5R87fBVy8OmGSM8r0s/edit?usp=sharing).

## Installation:

```bash
sudo apt-get install libspatialindex-dev
pip install rtree
pip install scikit-image
```

## Installation for localization-solution

```bash
localization-solution/src$
export ROS_PACKAGE_PATH="~/racecar_ws/src:$ROS_PACKAGE_PATH"
sudo --preserve-env python setup.py install
```

## Example Usage

```bash
# catkin-make, etc.
roscore
roslaunch localization-solution localize_simulation.launch
roslaunch racecar_simulator simulate.launch
rviz
roslaunch lab6 plan_trajectory.launch # path_planning
roslaunch lab6 load_trajectory.launch
roslaunch lab6 follow_trajectory.launch
```
