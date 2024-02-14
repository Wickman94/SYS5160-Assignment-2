# Water Tank Height Control System Using Fuzzy Logic

## Overview

This project implements a fuzzy logic controller to regulate the height of water in a tank. 
By adjusting control voltages based on the difference between the current and desired water levels, the system dynamically responds 
to varying conditions to maintain the target height. The controller uses fuzzy logic rules to determine the appropriate control actions, 
making it robust against uncertainties and nonlinearities inherent in the process.

## Features

**Fuzzy Logic Controller**: Utilizes skfuzzy to define and evaluate fuzzy logic rules for height control.

**Height Regulation**: Dynamically adjusts the control voltage to achieve and maintain a desired water height.

**Voltage Limitation**: Ensures that the control voltage does not exceed the maximum allowable limit.

**Simulation**: Models the water tank's dynamics and simulates the control process over time.

**Visualization**: Plots both the water height and control voltage over time to illustrate the system's performance.

## Components

Fuzzy Sets and Membership Functions: Defines linguistic variables (Low, Medium, High) for the error (difference between current and desired heights) and the control signal (voltage).<br>

Fuzzy Rules: Establishes rules that map the relationship between the error signal and the control action.<br>

Control System: Implements the fuzzy logic controller using defined rules.<br>

Water Tank Model: A differential equation models the dynamics of water height in the tank based on the input control voltage.<br>

Simulation Loop: Iteratively computes the control voltage, updates the tank height, and tracks these values over time.<br>

## Visualization
Generates plots for both the water height and control voltage throughout the simulation period.

# How It Works

Initialization: Defines the universe of discourse for the fuzzy logic controller, including the range of error signals and control signals.<br>
Membership Functions: Specifies triangular and trapezoidal membership functions for categorizing the error signal and control signal into fuzzy sets.<br>
Fuzzy Rules Definition: Creates rules that determine the control signal based on the current error signal's fuzzy classification.<br>
Control System Setup: Configures the fuzzy logic control system with the specified rules.<br>
Simulation: Simulates the water tank's response over a predefined time frame, using the fuzzy logic controller to adjust the control voltage based on the error between the actual and desired water heights.<br>
Voltage Adjustment and Tracking: Applies the control voltage, ensuring it stays within set limits, and records the voltage for visualization.<br>

# Result
## P1

![image](https://github.com/Wickman94/SYS5160-Assignment-2/assets/158237302/00ce47b2-69ea-4d9e-b2a2-45b69337349e)

## P2

![image](https://github.com/Wickman94/SYS5160-Assignment-2/assets/158237302/15ecff56-6a5a-4985-8ef4-b7447082df1f)



## Visualization:
Plots the simulation results, showing both the water height and control voltage over time to evaluate the system's performance.<br>
**Left Plot**: Shows the target and actual water heights over the simulation period, illustrating the effectiveness of the control strategy.<br>
**Right Plot**: Displays the variation of control voltage over time, highlighting how the controller responds to changes in the water height.<br>

## Conclusion
This project demonstrates the application of fuzzy logic in controlling a water tank's height, showcasing its potential in handling complex, nonlinear control tasks with uncertainty and variability. The use of fuzzy logic allows for a flexible, intuitive approach to designing control systems for a wide range of applications.
