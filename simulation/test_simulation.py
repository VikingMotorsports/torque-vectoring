from simulation import *
import pytest

random_steering_angle = random.uniform(-230, 230)
Vx = v_cg
St_a = 0.4 # last simulate argument (steering_a)
Trr = 5 # 4th simulate argument (rr_torqueWheel)
Trl = 1 # 3rd simulate argument (rl_torqueWheel)

def test_degrees_to_radians():
    degrees = random.uniform(0, 360)
    assert degrees_to_radians(degrees) == degrees * (math.pi / 180)
    assert degrees_to_radians(90) == 90 * (math.pi / 180)
    assert degrees_to_radians(90) == pytest.approx(1.5708, 0.0001)

def test_calculate_desired_yaw_rate():
    delta = random.uniform(-230, 230)
    desired_yaw_rate = (v_cg / (lf + lr) - 0.00168 * math.pow(v_cg, 2)) * delta
    assert calculate_desired_yaw_rate(v_cg, delta) == desired_yaw_rate

def test_calculate_yaw_rate():
    delta_torque = calculate_delta_torque(Trr, Trl)
    Ku = ((-lf * Cy_f + lr * Cy_r) / (Izz * Vx)) - ((lf**2 * Cy_f + lr**2 * Cy_r) / (Izz * Vx)) + ((lf * Cy_f) / Izz) * St_a + (1 / 0.05 * Izz) * delta_torque
    assert calculate_yaw_rate(Cy_f, Cy_r, Vx, St_a, Trr, Trl) == Ku