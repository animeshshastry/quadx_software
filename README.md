# quadx_firmware
 
#### Made for and tested on Teensy 4.0

To use this firmware follow these 2 easy steps:
1.  Clone this repository in your Arduino folder.
2.  Clone the libraries from https://github.com/animeshshastry/quadx_firmware_libraries and place the library sub-folders in your Arduino/libraries.
3.  Open and compile the main file "quadx_firmware.ino".
4.  Upload to teensy.

This firmware contains Implementations of

Control: SE(3) | Status: Complete<br/>
Estimator: UKF | Status: Complete<br/>
Estimator: sq-UKF | Status: Incomplete<br/>

This firmware was developed to satisfy the following objectives:

Primary Goal 1: Demonstration of self-calibration | Status: Not started<br/>
Primary Goal 2: Demonstration of unsteady wind rejection | Status: Not started<br/>

Secondary Goal 1: Nonlinear control testing | Status: In Progress<br/>
Secondary Goal 2: Dyanmical Estimation testing | Status: In Progress<br/>
Secondary Goal 3: Localization indoors using VIO | Status: Complete<br/>

For mathematical details and derivations, read the following paper:<br/>
A. Shastry and D. A. Paley, "UAV State and Parameter Estimation in Wind Using Calibration Trajectories Optimized for Observability," in IEEE Control Systems Letters, vol. 5, no. 5, pp. 1801-1806, Nov. 2021, doi: [10.1109/LCSYS.2020.3044491](https://doi.org/10.1109/LCSYS.2020.3044491).

#### Acknowledgements:
The capabilities/functionalities of this firmware was built upon https://github.com/nickrehm/dRehmFlight
