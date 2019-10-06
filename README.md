This is a project in Udacity Nanodegree for Self-driving car, forked from [CarND-PID-Control-Project](https://github.com/udacity/CarND-PID-Control-Project).


# PID Control
The goal of this project is to implement PID controller to steer a car in a simulation environment. In addition to the steering, the speed also needs to be set. For that, I have also used a PID controller. Following tasks have been completed and results can be found in this repository:

1. [./src/PID.cpp](./src/PID.cpp) is completed according to the instructions given in the file and steps as described in the lesson.
2. A new class, Twiddle, implemented in [./src/Twiddle.cpp](./src/Twiddle.cpp) and [./src/Twiddle.h](./src/Twiddle.h) is added. This class implements the Twiddle algorithm in such a way that it can be used in the simulation.
3. src/main.cpp is modified as follows:

    a. Two PID controllers are added, one for steering control and one for speed control. Variables for these two are steer_pid and speed_pid, both of type PID.  Line 39 [./src/main.cpp](./src/main.cpp).

    b. Two twiddle objects are added, steer_twiddle and speed_twiddle, to be used for optimization of the control coefficients.  Line 40 [./src/main.cpp].

    c. Both the controller and the Twiddle optimizers are added in the loop. To do that, references are added in the onMessage method call of microWebSocket hub object.  Lines 111 [./src/main.cpp](./src/main.cpp).

    d. References to the Twiddle object are added in the onConnection method call of microWebSocket hub object. Line 111 [./src/main.cpp](./src/main.cpp).

    e. The method sent to onConnection is modified such that references to the server are passed to the Twiddle objects such that it can restart the server. Lines 175 - 179 [./src/main.cpp](./src/main.cpp).

    f. Arguments are added to the invocation command (Lines 59 - 110 [./src/main.cpp](./src/main.cpp)), ./pid, such that one can run the controller in three different modes: 

        * running with default parameters, 
        
        * running with 6 parameters given as arguments
        
        * optimizing the parameters starting from the default values.

4. CMakeLists.txt is modified so that Twiddle class can be compiled and linked with the other object files.

# Build, compilation and installation on Ubuntu
After cloning the repository, create a new directory with the name build, change the directory to build and run `cmake` as shown below:

`mkdir build`

`cd build`

`cmake ..`

This will create a make file that can be used to compile and link the program using `make`. For that, while being in build directory, simply run:

`make`

Running `make` will compile the c++ files and try to link them. Assuming that all dependencies described in [Udacity's project repository](https://github.com/udacity/CarND-PID-Control-Project) are satisfied, and especially [micro WebSockets](https://github.com/uNetworking/uWebSockets) is installed, make will produce an executable file called `pid`. If you do not have [micro WebSockets](https://github.com/uNetworking/uWebSockets) installed, please follow [Udacity's instructions](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/1c4c9856-381c-41b8-bc40-2c2fd729d84e/lessons/3feb3671-6252-4c25-adf0-e963af4d9d4a/concepts/7dedf53a-324a-4998-aaf4-e30a3f2cef1d) to have it properly installed.

# Running the controller
Running the controller requires connection with [Udacity's simulator](https://github.com/udacity/self-driving-car-sim/releases). I had my simulator on Windows and the controller on Ubuntu running on [VirtualBox](https://www.virtualbox.org/wiki/Downloads).

The execution can start by executing `./pid` from the build directory, which runs the controller: 
<p align="center">
<img src="./images/normal_start.png" width="600"/>
</p>
The controller is configured with the parameters as given and waits for connection from the simulator. Here, `a_Kp`, `a_Ki` and `a_Kd` denote the coefficients for P, I and D part of PID controller for steering value nd equally, `v_Kp`, `v_Ki` and `v_Kd` denote the coefficients for P, I and D parts of PID controller for speed control. Thw simulation starts as soon as connection is established.

Alternatively, you can overwrite the parameters by giving them as arguments to the command `./pid` as shown below:
<p align="center">
<img src="./images/custom_parameters.png" width="600"/>
</p>
Note that the first three correspond to steering control and the last three to speed control.

In either of the two cases above, the simulation will start and you should be able to see the car driving close to centerline.

<p align="center">
<img src="./images/pid_controlled.png" width="600"/>
</p>


The default parameters can be optimized with Twiddle algorithm. To do that, you need to execute the command `./pid twiddle speed` for speed or `./pid twiddle steer` for steering as shown below:
<p align="center">
<img src="./images/twiddle_run.png" width="600"/>
</p>

If the Twiddle optimization is turned on, the simulation will be restarted a number of times according to the algorithm. The parameters are hard-coded in [./src/main.cpp](./src/main.cpp), lines 83 for steering and 87 for speed. 

# Twiddle implementation
Twiddle algorithm implemented in [./src/Twiddle.cpp](./src/Twiddle.cpp) is exactly as presented in the lessons. However, the `while` and `for` loops are replaced by named states so that the it can be run in a separate process. At each step, the error is sent to the Twiddle object by `twiddle_object.UpdateError(error)`. At each call, the error is accumulated if the initial steps are taken. Once the set limit on the number of steps to calculate error is reached, `UpdateError` makes a call to `Execute()`, which in turn makes a Twiddle step forward. The internal state of Twiddle object is respected by setting the private property `next_twiddle_step` to right value. For example, if adding the delta does not improve, we will have `next_twiddle_step = "CHECK_MINUS";`.

After each Twiddle step, the simulation should be restarted. The idea of restarting the process and implementation are taken from [Bruno Guisard's proposal](https://github.com/bguisard/CarND-PID-Control-Project). 

# Tuning parameters
PID is a simple linear controller and can be implemented easily. The difficulty in having a good controller is tuning parameters that can be somewhat cumbersome and might lead to problems if the degree of nonlinearity in the process is high. In the present case, there are two PID controllers that need to be tuned, one for speed and one for steering. These two processes are interconnected as changes in the steering angles has an influence on the speed and changing speed makes the process of deviating from the best driving line faster or slower. 

In the present case, I started with original Twiddle algorithm as presented in the lessons. However, I noticed that the process will be very slow and the algorithm would need a considerable amount of time to converge. To accelerate the process, I started with manual tuning, simply by visually following the changes and trying new values. 

Following steps are often recommended on internet: 

1. Set all parameters to zero.
2. Increase Kp until the response becomes steadily oscillating.
3. Increase Kd until the the oscillations go away.
4. Repeat steps 2 and 3 until increasing Kd cannot help.
5. Set Kp and Kd to the last stable values.
6. Increase Ki until the level of oscillations will be acceptable.

I followed the above steps, but not in very details. I stopped whern the control seemed to be acceptable visually.

To start with, I had the throttle at the value set, 0.3, so that I could find a first reasonable set of parameter values for controll of steering angle. Thereafter, I turned on the PID controller and tried to tune the speed controller and finally switched to the steering control and tuned until an acceptable level could be reached.

After reaching a visually acceptable level, I used Twiddle to optimize the values. I started with the manually found values and an updating values set to 20% of the obtained values. I started first with speed control and then steering control. This was repeated two times.

# Reflections

* PID is a commonly used control method to force a process signal follow desired values. As it is linear, it will be a very competitive method for linear system. For non-linear systems, usually a modeling step prior to generation of reference data is used to have PID to adjust for imperfections in the model, rather than the whole dynamic.

* The proportional term is basically working as a spring, trying to keep the mass in the equilibrium position. The proportionality constant, Kp, varies the "force", which enforces the response to stay close to the reference. Proportional tern cannot eliminate the errors alone. Low values of Kp will not reduce the error much enough and high values lead to oscillations that cannot be damnped with the proportional term alone.

* The derivative term, as the name suggests, acts on the (time) derivative of the error and acts as a damping term in the system. The higher value of Kd the more damping effect we will have. As this term acts on the derivative of the error it will be identically zero for a constant error, or very small when the changes in the error become small.

* With a proportional term and a derivative terrm, the system output, or system response, can converge to a constant, but non-zero error. The integral term is responsible to make sure that there will not be any constant error left. It accumulates the error and uses that as a correction to reduce the error. The constant Ki is basically deciding the time-scale over which this effect will work. Large Ki values makes the reactions fast, but they also increase the amplitude of oscillations.

* The manual tuning I made, followed the generally suggestions steps: Drive the proportional term until it reaches oscilations and then increase the damping until it becomes stable. Repeat until maximum values are reached for a stable system and then increase the integral term to reach the desired time for stabilization around the desired value. The work I did manually was simply not that accurate to be able to analyze this process quantitatively. Instead, I make a preliminary work myself and used Twiddle to optimize the parameters.

* Twiddle did certainly a better job than me, but the process was relatively slow. Both the initial simulation time before start of accumulation of error as well as the total time of accumulation were set without detailed justifications. I believe these parameters should be important both for accuracy and speed of the process. However, the final results seem to be acceptable.

* Although I was not able to quantify this observation, I got a feeling that the behavior of the car changed if I had print-outs on or off. My interpretation was that it was due to the impact on the socket communication and especially between the Windows machine and Ubuntu virrtual machine. To avoid such problems the sampling interval needs to be kept constant.

# Results
After completion of the implementation and tuning of parameter as explained above the car in the simulation was controlled with the two PID controllers, one for speed and one for steering. The controllers were capable to keep the car in the middle of the lane and at given speed of 20. A video recording of the simulated driving can be found in [./videos/PID_driver.mp4](./videos/PID_driver.mp4).

Judging from the video, one might suspect that the lane is modelled as linear segments. This can be seen as the car changes directions at some given points along the track and not in a continuous manner. These points are noticed frequently and periodically. In addition, at some points, like at entrance of the bridge and just before and after leaving the bridge, there are some disturbances. I believe these are also from the reference data.

The tuning was done at speed set to 20, but with the same parameters it can drive at 30 many laps, without problems. If you wish to do so, you need to set the `set_speed = 20;` at line 146 in [./src/main.cpp](./src/main.cpp).

At speed of 40, the car went off the track after some driving time.