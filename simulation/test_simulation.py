from simulation import *
import pytest

random_steering_angle = random.uniform(-230, 230)
Vx = v_cg
St_a = 0.4 # last simulate argument (steering_a)
Trr = 5 # 4th simulate argument (rr_torqueWheel)
Trl = 1 # 3rd simulate argument (rl_torqueWheel)

def test_radians_to_ms():
    radians = random.uniform(0, 2 * math.pi)
    assert radians_to_ms(radians) == radians * wheelRadius
    assert radians_to_ms(math.pi) == math.pi * wheelRadius

def test_degrees_to_radians():
    degrees = random.uniform(0, 360)
    assert degrees_to_radians(degrees) == degrees * (math.pi / 180)
    assert degrees_to_radians(90) == 90 * (math.pi / 180)
    assert degrees_to_radians(90) == pytest.approx(1.5708, 0.0001)

def test_calculate_steer_gradient():
    Ku = ((lr * m) / (Cy_f * (lf + lr))) - ((lf * m) / (Cy_r * (lf + lr)))
    assert calculate_steer_gradient() == Ku

def test_calculate_desired_yaw_rate():
    delta = random.uniform(-230, 230)
    desired_yaw_rate = (v_cg / (lf + lr) - 0.00168 * math.pow(v_cg, 2)) * delta
    assert calculate_desired_yaw_rate(v_cg, delta) == desired_yaw_rate

def test_calculate_yaw_rate():
    delta_torque = calculate_delta_torque(Trr, Trl)
    Ku = ((-lf * Cy_f + lr * Cy_r) / (Izz * Vx)) - ((lf**2 * Cy_f + lr**2 * Cy_r) / (Izz * Vx)) + ((lf * Cy_f) / Izz) * St_a + (1 / 0.05 * Izz) * delta_torque
    assert calculate_yaw_rate(Cy_f, Cy_r, Vx, St_a, Trr, Trl) == Ku

def test_calculate_yaw_moment():
    M_V = (Trr - Trl) * Tr
    assert calculate_yaw_moment(Trr, Trl) == M_V

def test_calculate_lateral_velocity():
    Vy = ((-(Cy_f + Cy_r)/(m * Vx)) + (((-lf * Cy_f + lr * Cy_r)/(m * Vx)) - Vx)) + (Cy_f / (m * Vx)) * St_a
    assert calculate_lateral_velocity(Vx, St_a) == Vy

def test_calculate_delta_torque():
    yaw_moment = calculate_yaw_moment(Trr, Trl)
    delt_T = (Rw / ((2 * Tr) * Gr)) * yaw_moment
    assert calculate_delta_torque(Trr, Trl) == delt_T

def test_magic_formula(): 
    rl_slippage = rr_slippage = 1.0
    b, c, d, e = 10, 1.9, 1, 0.97
    w = weight / 2
    p = ((rl_slippage + rr_slippage) / 2) - 1
    F = w * d * math.sin(c * math.atan(b * p * (1 - e) + e * b * p - math.atan(b * p)))
    assert magic_formula(w, (rl_slippage + rr_slippage) / 2) == F