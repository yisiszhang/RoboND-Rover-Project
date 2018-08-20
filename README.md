## Project: Search and Sample Return

---
[//]: # (Image References)

[image1]: ./output/warped_example.jpg
[image2]: ./output/warped_threshed.jpg
[image3]: ./output/rock_threshed.png
[image4]: ./output/rover_navigation.png
[image5]: ./output/simulator_screen_shot.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This file includes the writeup of the project.

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
In the notebook, the following functions were tested:

1) The calibration image with grids was converted to top view using `perspect_transform()`.

![warped_example][image1]

2) To detect obstacles, `color_thresh()` function was used. RGB values above threshold was set to 1, corresponding to the navigable terrain. Pixels with zero values are obstacles.

![warped_threshed][image2]

3) Rocks were detected using the `rock_detector()` function. The RGB values were converted to HSV using `cv2.cvtColor()` function. Color of the golden rocks was determined in the range between (80,100,100) and (100,255,255). Using this criterion, rocks were detected reliably.

![rock_threshed][image3]

4) The perspect-transformed, thresholded images were translated to the rover's coordinate. Average direction was calculated using the mean of the non-zero coordinates.

![rover_navigation][image4]

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.

The `process_image()` function was filled with implementation of the above functions. In addition, the rover's navigation coordinate was populated to the world map. A test video ([test_mapping.mp4](./output/test_mapping.mp4)) was generated using the sample recordings.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

* Several additional functions/variables were added to `perception_step()` apart from its basic functions:

  1. In function `obstacle_detector()`, obstacle was identified as the complement of the color-thresholded image. The obstacle pixels were dilated using the `cv2.dilate()` function in order to reduce the chance for the rover to get stuck near the wall. Navigable areas were smoothed using `cv2.morphologyEx()` function.
  2. `worldmap` was only updated when pitch and roll were close to 0 degrees (`Rover.pitch` less  than 0.75 or greater than 359.25 degrees, `Rover.roll` less than 1 or greater than 359 degrees).
  3. In addition, rock location in the rover's coordinate system was recorded as global variables (`Rover.rock_dists` and `Rover.rock_angles`). A boolean variable `Rover.sample_in_view` was recorded to track whether a rock is in the view of the camera. The criterion was set by `Rover.sample_thresh` (set to 10), which is the minimal number of pixels of rock color.

* In `decision()`, several functions were implemented:

  1. Additional steps were added in the beginning to determine whether the rover is stuck. If the cumulative traveled distance over 10 steps (saved in `Rover.cumdist`) is less than 0.01, the rover is considered stuck. If the rover is stuck, `Rover.mode` is set to `stop` so it can rotate out of the stuck angle.
  2. Additionally, in case the rover orbits along the same trajectory over and over again with the same steering angle for 10 steps (if `Rover.cumsteer` is 0), the rover is considered spinning. If the rover is spinning, a random steering angle between -15 and 15 degrees is provided.
  3. If the rock is in the field of view, the rover steers towards the rock and slows down when it is 10 pixels away from the rock. In this case, the program skips the `Rover.mode` steps.
  4. For each step in the `forward` mode, explored terrain with respect to the rover's coordinate system was determined using function `pix_to_rover()`. Repetitively explored terrain was determined 200 above mean `worldmap` value. If the repetitively explored terrain is in the navigable view, the steering angle is biased against `Rover.avoid_angle` with a scale of `penal_strength`.
  5. In case the rover is still stuck at stop mode, seeing navigable terrain and accelerating, `Rover.throttle` and `Rover.brake` are reset to zero and `Rover.steer` is set to -15 degrees.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

The code was tested in the autonomous mode of the simulator. `640x480` screen resolution and `Fastest` graphics quality were selected. Frames per second (FPS) was `27`.

The code was able to achieve >70% map exploration within 10 minutes. The fidelity of the navigable pixels was above 60%. Rocks were picked up ~80% when detected.

There are some aspects yet to be improved. First, the exploration is relatively slow. Because the velocity was set to optimize the efficiency of braking upon rock detection, the maximal velocity was kept low at 2. In addition, terrain was still repetitively visited. The way to bias the navigation angle away from the visited areas was not efficient enough. However, when enough time is given, this method can explore >99% terrain. A boundary-closing method would be desirable. For example, areas behind the rover after it turned around can essentially be blocked.

Rock detection could be further improved by exploring the edge of the walls. The rock behind the boulders was the most difficult to pick, because the movement could be blocked by obstacles. The code could be further optimized to move the rover along the wall without getting stuck.

Here I included a screen shot of the simulator.

![simulator_screen_short][image5]
