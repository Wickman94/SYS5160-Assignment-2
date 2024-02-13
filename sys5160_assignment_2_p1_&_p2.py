pip install scikit-fuzzy
from cmath import e as euler
import numpy as np
from scipy.integrate import odeint
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# Establishing the universe for error signal e(t) and control signal V(t)
error_signal = ctrl.Antecedent(np.arange(-4, 4.1, 0.1), 'error')
control_signal = ctrl.Consequent(np.arange(0, 48.1, 0.1), 'control')

# Setting up the fuzzy sets and membership functions for both signals
error_signal['Low'] = fuzz.trimf(error_signal.universe, [-4, -4, -1])
error_signal['Medium'] = fuzz.trimf(error_signal.universe, [-2, 0, 2])
error_signal['High'] = fuzz.trimf(error_signal.universe, [1, 4, 4])
control_signal['Low'] = fuzz.trimf(control_signal.universe, [0, 0, 24])
control_signal['Medium'] = fuzz.trimf(control_signal.universe, [16, 24, 32])
control_signal['High'] = fuzz.trimf(control_signal.universe, [24, 48, 48])

# Establishing rules for the fuzzy logic controller
fuzzy_rule1 = ctrl.Rule(error_signal['Low'], control_signal['Low'])
fuzzy_rule2 = ctrl.Rule(error_signal['Medium'], control_signal['Medium'])
fuzzy_rule3 = ctrl.Rule(error_signal['High'], control_signal['High'])

# Assembling the controller with defined rules
fuzzy_controller = ctrl.ControlSystem([fuzzy_rule1, fuzzy_rule2, fuzzy_rule3])
fuzzy_simulation = ctrl.ControlSystemSimulation(fuzzy_controller)

# Parameters for the water tank model
TankRadius = 5
FlowRate = 0.01
LeakageCoeff = 0.1
MaxVoltage = 48
InitialHeight = 0
DesiredHeight = 4  # Target height

# Function modeling the water tank dynamics
def tank_model(height, time, voltage):
    dheight_dt = ((FlowRate * voltage) - LeakageCoeff * np.sqrt(height)) / (np.pi * TankRadius**2)
    return dheight_dt

# Simulation time points
simulation_time = np.linspace(0, 10000, 100)
heights = []

# Running the simulation with the fuzzy controller
current_height = InitialHeight
for time_index in range(len(simulation_time)-1):
    error = DesiredHeight - current_height
    fuzzy_simulation.input['error'] = error
    fuzzy_simulation.compute()
    voltage_control = fuzzy_simulation.output['control']
    voltage_control = min(voltage_control, MaxVoltage)  # Ensuring voltage does not exceed max
    current_height = odeint(tank_model, current_height, [simulation_time[time_index], simulation_time[time_index+1]], args=(voltage_control,))[1][0]
    if current_height > DesiredHeight:
        current_height = DesiredHeight
    heights.append(current_height)

# Visualizing the system's performance over time
plt.plot(simulation_time, np.ones_like(simulation_time) * DesiredHeight, 'k--', label='Target Height')
plt.plot(simulation_time[:-1], heights, 'r', linewidth=2, label='Tank Height')
plt.legend(loc='upper right')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.title('Water Tank Level Control Using Fuzzy Logic')
plt.show()

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Initial parameters setup
initial_height = 1                          # Starting height of the water
rate_of_height_change = 0                   # Initial rate of height change
target_height = 5                           # Target height to achieve
height_difference = target_height - initial_height  # Initial height difference
simulation_start = 0
simulation_end = 1999
time_resolution = 100
total_steps = (simulation_end + 1) * time_resolution
time_vector = np.linspace(simulation_start, simulation_end, total_steps)
desired_height_timeline = [5] * len(time_vector)  # Timeline for the desired height
actual_heights = []                         # To store the actual heights over time
voltage_timeline = []                       # To store the voltages applied over time

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
plt.plot(time_vector[1:], desired_height_timeline[1:], '-r', label='Target Height')
plt.plot(time_vector[1:], actual_heights, '-g', label='Actual Height')
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