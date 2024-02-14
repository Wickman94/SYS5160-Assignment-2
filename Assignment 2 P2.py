pip install scikit-fuzzy

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Initial parameters setup
initial_height = 1
rate_of_height_change = 0
target_height = 5
height_difference = target_height - initial_height
simulation_start = 0
simulation_end = 1999
time_resolution = 100
total_steps = (simulation_end + 1) * time_resolution
time_vector = np.linspace(simulation_start, simulation_end, total_steps)
desired_height_timeline = [5] * len(time_vector)
actual_heights = []
voltage_timeline = []

# Fuzzy logic setup for error and derivative of error, and the control action (voltage adjustment)
height_error = ctrl.Antecedent(np.arange(-4, 5, 1), 'height_error')
delta_height_error = ctrl.Antecedent(np.arange(-0.006, 0.007, 0.001), 'delta_height_error')
adjustment_voltage = ctrl.Consequent(np.arange(-3, 4, 1), 'adjustment_voltage')

# Membership functions for height error
height_error['very_low'] = fuzz.trapmf(height_error.universe, [-4, -4, -2, -1])
height_error['low'] = fuzz.trimf(height_error.universe, [-2, -1, 0])
height_error['medium'] = fuzz.trimf(height_error.universe, [-1, 0, 1])
height_error['high'] = fuzz.trimf(height_error.universe, [0, 1, 2])
height_error['very_high'] = fuzz.trapmf(height_error.universe, [1, 2, 4, 4])

# Membership functions for delta height error
delta_height_error['negative'] = fuzz.trapmf(delta_height_error.universe, [-0.006, -0.006, -0.003, 0])
delta_height_error['zero'] = fuzz.trimf(delta_height_error.universe, [-0.003, 0, 0.003])
delta_height_error['positive'] = fuzz.trapmf(delta_height_error.universe, [0, 0.003, 0.006, 0.006])

# Membership functions for voltage adjustment
adjustment_voltage['very_low'] = fuzz.trapmf(adjustment_voltage.universe, [-3, -3, -2, -1])
adjustment_voltage['low'] = fuzz.trimf(adjustment_voltage.universe, [-2, -1, 0])
adjustment_voltage['medium'] = fuzz.trimf(adjustment_voltage.universe, [-1, 0, 1])
adjustment_voltage['high'] = fuzz.trimf(adjustment_voltage.universe, [0, 1, 2])
adjustment_voltage['very_high'] = fuzz.trapmf(adjustment_voltage.universe, [1, 2, 3, 3])

# Defining rules for fuzzy logic control
rules = [
    ctrl.Rule(height_error['very_low'] & delta_height_error['negative'], adjustment_voltage['very_low']),
    ctrl.Rule(height_error['very_low'] & delta_height_error['zero'], adjustment_voltage['very_low']),
    ctrl.Rule(height_error['low'] & delta_height_error['negative'], adjustment_voltage['low']),
    ctrl.Rule(height_error['low'] & delta_height_error['zero'], adjustment_voltage['low']),
    ctrl.Rule(height_error['medium'] & delta_height_error['zero'], adjustment_voltage['medium']),
    ctrl.Rule(height_error['high'] & delta_height_error['zero'], adjustment_voltage['high']),
    ctrl.Rule(height_error['high'] & delta_height_error['positive'], adjustment_voltage['high']),
    ctrl.Rule(height_error['very_high'] & delta_height_error['zero'], adjustment_voltage['very_high']),
    ctrl.Rule(height_error['very_high'] & delta_height_error['positive'], adjustment_voltage['very_high'])
]

# Building the control system
height_control_system = ctrl.ControlSystem(rules)
height_controller = ctrl.ControlSystemSimulation(height_control_system)

# Function to calculate the voltage based on current height and rate of change
def calculate_voltage(current_error, rate_of_change):
    height_controller.input['height_error'] = current_error
    height_controller.input['delta_height_error'] = rate_of_change
    height_controller.compute()
    return height_controller.output['adjustment_voltage']

# Ensuring voltage stays within predefined limits
def enforce_voltage_limits(voltage):
    return max(0, min(voltage, 48))

# Initial voltage condition
initial_voltage = 0

# ODE model for height dynamics
def height_dynamics(current_height, time, applied_voltage):
    phase_shift = 0.1 if time <= 200 * time_resolution else 0.2  # Change in dynamics after 20 min
    constant_b = 0.01
    tank_radius = 5
    dheight_dt = (constant_b * applied_voltage - phase_shift * np.sqrt(current_height)) / (np.pi * tank_radius**2)
    return dheight_dt

# Simulation loop using ODE integration and fuzzy logic for voltage calculation
for step in range(1, total_steps):
    time_span = [time_vector[step-1], time_vector[step]]
    height_at_next_step = odeint(height_dynamics, initial_height, time_span, args=(initial_voltage,))
    rate_of_height_change = height_at_next_step[1][0] - initial_height
    initial_height = height_at_next_step[1][0]
    height_difference = target_height - initial_height
    initial_voltage += calculate_voltage(height_difference, rate_of_height_change)
    initial_voltage = enforce_voltage_limits(initial_voltage)

    actual_heights.append(initial_height)
    voltage_timeline.append(initial_voltage)

# Plotting the results
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.plot(time_vector[1:], desired_height_timeline[1:], '-b', label='Target Height')
plt.plot(time_vector[1:], actual_heights, '-r', label='Actual Height')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.legend(loc='best')

plt.subplot(1, 2, 2)
plt.plot(time_vector[1:], voltage_timeline, label='Control Voltage')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend(loc='best')
plt.tight_layout()
plt.show()

# Define the universe of discourse for the error signal and control signal
error_signal = ctrl.Antecedent(np.arange(-4, 4.1, 0.1), 'error')
control_signal = ctrl.Consequent(np.arange(0, 48.1, 0.1), 'control')

# Define the fuzzy sets and membership functions for error signal
error_signal['Low'] = fuzz.trimf(error_signal.universe, [-4, -4, -1])
error_signal['Medium'] = fuzz.trimf(error_signal.universe, [-2, 0, 2])
error_signal['High'] = fuzz.trimf(error_signal.universe, [1, 4, 4])

# Define the fuzzy sets and membership functions for control signal
control_signal['Low'] = fuzz.trimf(control_signal.universe, [0, 0, 24])
control_signal['Medium'] = fuzz.trimf(control_signal.universe, [16, 24, 32])
control_signal['High'] = fuzz.trimf(control_signal.universe, [24, 48, 48])

# Plot membership functions for error signal
error_signal.view()

# Plot membership functions for control signal
control_signal.view()
