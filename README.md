# quadx_firmware
 
### Made for self-calibration, system-identification and gust rejection demonstration. Tested on Teensy 4.0

A user-friendly software designed for testing advanced control and estimation algorithms without the headache of modifying PX4, Ardupilot, Betaflight or other firmwares. It uses C++ templates and matrix class, through which standard operators are overridden and used naturally in algebraic expressions, which results in simplistic Matlab-type syntaxes. Though, currently the code is dirty and more cleaning is needed. Microcontroller version of Eigen is also present in the libraries with UKF and sq-UKF implementations.

This software is in experimental release state and is currently being built and tested on a quadcopter / quadrotor. To use SE(3) controller, VIO data input through rosserial is required. More documentation on this is coming soon.

To use this software follow these 4 easy steps:
1.  Clone this repository in your Arduino folder.
2.  Clone the libraries from https://github.com/animeshshastry/quadx_firmware_libraries and place the library sub-folders in your Arduino/libraries.
3.  Open and compile the main file "quadx_firmware.ino".
4.  Upload to Teensy 4.0.

This software contains Implementations of

Control: SE(3) | Status: Complete<br/>
Control: SO(3) | Status: Complete<br/>
Estimator: UKF | Status: Complete<br/>
Estimator: sq-UKF | Status: Complete<br/>

This software was developed to satisfy the following objectives:

Primary Goal 1: Demonstration of self-calibration | Status: Partially Complete<br/>
Primary Goal 2: Demonstration of unsteady wind rejection | Status: Complete<br/>

Secondary Goal 1: Nonlinear control testing | Status: Complete<br/>
Secondary Goal 2: Dynamical Estimation testing | Status: Complete<br/>
Secondary Goal 3: Localization indoors using VIO | Status: Complete<br/>

For mathematical details and derivations, please read the following paper:<br/>
A. Shastry and D. A. Paley, "UAV State and Parameter Estimation in Wind Using Calibration Trajectories Optimized for Observability," in IEEE Control Systems Letters, vol. 5, no. 5, pp. 1801-1806, Nov. 2021, doi: [10.1109/LCSYS.2020.3044491](https://doi.org/10.1109/LCSYS.2020.3044491).

#### Acknowledgements:
Radio and LED functions of this software was built upon https://github.com/nickrehm/dRehmFlight
