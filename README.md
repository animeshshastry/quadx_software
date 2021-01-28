# quadx_firmware
 
Made for and tested on Teensy 4.0

To use this firmware follow these 2 easy steps:
1.  Clone this repository in your Arduino folder.
2.  Clone the libraries from https://github.com/animeshshastry/quadx_firmware_libraries and place the library sub-folders in your Arduino/libraries.
3.  Open and compile the main file "quadx_firmware.ino".
4.  Upload to teensy.

This firmware contains Implementations of

Control: SE(3) | Status: Complete

Estimator: UKF | Status: Complete

Estimator: sq-UKF | Status: Incomplete

This firmware was developed to satisfy the following objectives:

Primary Goal 1: Demonstration of self-calibration | Status: Not started

Primary Goal 2: Demonstration of unsteady wind rejection | Status: Not started

Secondary Goal 1: Nonlinear control testing | Status: In Progress

Secondary Goal 2: Dyanmical Estimation testing | Status: In Progress

Secondary Goal 3: Localization indoors using VIO | Status: Complete
