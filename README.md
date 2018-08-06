# car_RRT_path_planning

dependencies:

sudo apt-get install libopencv-dev

kdTree was implemented for efficiency by darkthecross.

![kdTree](kdTree.gif)

The basic implementation was finished based on kdTree by darkthecross.

![RRT](rrt.gif)

Then consider a car which can only turn left with the dynamic model:

![car_model](car_model.png)

First, we would want to have a local planner which could move the car from \[ (x_1, y_1, \theta_1) \] to \[ (x_2, y_2, \theta_2) \].
It is not hard to derive a method to plan a path using the series of circles which lies to the left of the initial and terminal states.
