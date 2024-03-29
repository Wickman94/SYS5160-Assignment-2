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
voltages = []  # To track voltage over time

# Running the simulation with the fuzzy controller
current_height = InitialHeight
for time_index in range(len(simulation_time)-1):
    error = DesiredHeight - current_height
    fuzzy_simulation.input['error'] = error
    fuzzy_simulation.compute()
    voltage_control = fuzzy_simulation.output['control']
    voltage_control = min(voltage_control, MaxVoltage)  # Ensuring voltage does not exceed max
    voltages.append(voltage_control)  # Track voltage
    current_height = odeint(tank_model, current_height, [simulation_time[time_index], simulation_time[time_index+1]], args=(voltage_control,))[1][0]
    if current_height > DesiredHeight:
        current_height = DesiredHeight
    heights.append(current_height)

# Visualizing the system's performance over time for height
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(simulation_time[:-1], np.ones_like(simulation_time[:-1]) * DesiredHeight, 'k--', label='Target Height')
plt.plot(simulation_time[:-1], heights, 'm', linewidth=2, label='Actual Height')
plt.legend(loc='upper right')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.title('Water Tank Height Control')

# Visualizing the voltage variation over time
plt.subplot(1, 2, 2)
plt.plot(simulation_time[:-1], voltages, 'b', linewidth=2, label='Control Voltage')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Control Voltage Over Time')
plt.legend(loc='upper right')

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

plt.tight_layout()
plt.show()
