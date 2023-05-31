## Simulation
This is a Python-based vehicle dynamics simulation tool that predicts the performance of a vehicle based on various 
physical parameters and driver inputs. The simulation helps in understanding the impact of these parameters on the 
vehicle's dynamics and control. The results are visualized in graph format for easy interpretation.

### Features
1. Calculation of vehicle's yaw rate, yaw moment, lateral velocity, and desired yaw rate based on various physical 
parameters and driver inputs.
2. Simulation of the vehicle's dynamics based on steering angle, torque applied to each wheel, and velocities.
3. Uses the magic formula from Pacejka's tire model for calculating lateral force based on wheel slippage.
4. Includes a function to convert steering wheel angle to steering angle.
5. Generation of Look-Up-Table (LUT) for throttle and steering inputs based on the simulation results for efficient 
real-time performance.
6. Visualization of vehicle's performance parameters over time using Matplotlib.
7. Set of unit tests ensuring the correctness of the implemented functions.

### Terms Used
- Yaw Rate: Angular speed around vertical axis of the vehicle in rad/s.
- Longitudinal Velocity: Velocity along the length of the vehicle (Vx) in m/s.
- Cornering stiffness at rear wheel (Cy_r) and front wheel (Cy_f) in N/rad.
- Inertia Moment around z axis (Izz) in Kg/m^2.
- Mass of the vehicle (m) in Kg.
- Front wheelbase (lf) and rear wheelbase (lr) in m.
- Steering Angle (St_a) in degrees (converts from radians to degrees internally).
- Yaw Moment (M_V) in Nm.
- Yaw Moment around vertical axis (Mz).
- Lateral Velocity (Vy) in m/s.
- Gear Ratio (Gr) dimensionless.
- Half the track of the car (Tr) in m.
- Radius of the wheel (Rw) in m.

### Look-Up-Table Generation
Needs to add some description about Look-Up Table.

### Unit Tests
In addition to the simulation functions, we have also provided a set of unit tests to validate the correctness of these 
functions. The unit tests are located in test_simulation.py. To run these tests, use the command pytest test_simulation.py.

### Dependencies
This simulator needs the following Python libraries:

math
random
array
numpy
matplotlib.pyplot
pytest (for testing the simulator)