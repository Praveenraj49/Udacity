
# SDC Term 2 PID Controller Project


[![PID Controller](PIDController.jpg)](https://youtu.be/aBCDsmM9ldE)

## Introduction
The goal of this project is to implement a PID controller and tune the hyper parameters Proportional ("P") , Integral ("I") and Differential("D")  to control the steering angle of car in order to drive the car autonomously in the simulator.
Implementation of the PID controller follows the  processing flow steps described in the course.
________________________________
## Rubric Points
Describe effects of each of the P , I and D components

### Proportional ("P"):
P is the most important and as the name states is the proportional component in the controller  and directly proportional to the error. It causes the car steer proportional to the car distance from the center lane. I.e if the car moves much towards right it will steer towards to left and vice versa.
Setting values to be high makes the car to steer constantly left and right and will create a really dizzy ride . Setting values anything more than 1 makes the car moves forward and backwards constantly as the steering rapidly  oscillating between right and left. On the other hand setting this value very low will make car to react very slowly in the simulator will make car jump the tracks during the turns . In real life this is catastrophic.

### Integral ("I"):
I or integral component counter the bias in the controller which prevents the car from steering towards the center lane. It prevents the car from driving either towards right or left.  If the integral co-efficient is too high car has oscillates quickly which affects the speed of the car and if the co-efficient is too low car will drive on right or left lane for a longer period of time.

### Derivative (D):
D or derivate component in the controller is the rate of change of CTE . This prevents the car from overshooting . If the derivative is quickly changing it will help the car to correct itself around corners and turns where steering needs to change quickly . If this co-efficient too high there will be constant steering change even though car moving in the middle of the lane.
If the co-efficient is too low there will be constant oscillations in the steering.

 ### Finding hyperparameters
 I initially started by manually tuning each of the parameters. I tried using twiddle to find the hyper parameters they did not result in good performance.
 Many of runs using twiddle made the car drive out of the track. I used twiddle to find some initial parameters and tried to manually tune.
 Eventually I settled to manually tune the parameters by tuning a single parameter and keep the other constant and finally tuning all the parameters in conjunction. Once zeroed on the parameters and tried to increase the throttle/speed performed well up to speeds to 60 miles/hr beyond that starts crashing. I also used a PID controller to control the throttle.


 ## Dependencies

 * cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
 * make >= 4.1(mac, linux), 3.81(Windows)
   * Linux: make is installed by default on most Linux distros
   * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
   * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
 * gcc/g++ >= 5.4
   * Linux: gcc / g++ is installed by default on most Linux distros
   * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
   * Windows: recommend using [MinGW](http://www.mingw.org/)
 * [uWebSockets](https://github.com/uWebSockets/uWebSockets)
   * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
   * If you install from source, checkout to commit `e94b6e1`, i.e.
     ```
     git clone https://github.com/uWebSockets/uWebSockets
     cd uWebSockets
     git checkout e94b6e1
     ```
     Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
 * Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

 There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

 ## Basic Build Instructions

 1. Clone this repo.
 2. Make a build directory: `mkdir build && cd build`
 3. Compile: `cmake .. && make`
 4. Run it: `./pid`.

 Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
