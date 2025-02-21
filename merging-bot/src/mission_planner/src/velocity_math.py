"""
This class holds functions for calculations of areas and time that are necessary for the different booking phases.

All functions in this class are static.
"""
from math import pow, sqrt


def area_rectangle(height, width):
    return height * width


def area_triangle(height, width):
    return (height * width) / 2


def area_trapezoid(height, width_1, width_2):
    return ((width_1 + width_2) * height) / 2


def time_from_vel_and_acc(vstart, vtarget, acc):
    if acc != 0:
        return abs((vstart - vtarget) / acc)
    return -1


# TODO assume vin is less than vout for now, rename
def calculate_acceleration_schema(tin, tout, vin, vout, vmax, amax, amin, s):
    v = find_cruise_velocity(tout - tin, vin, vout, vmax, amax, amin, s)
    if v >= vout:
        t2 = tout - time_from_vel_and_acc(v, vout, amin)
        schema = [(tin, v), (t2, vout)]
    elif vin < v < vout:
        t2 = tout - time_from_vel_and_acc(v, vout, amax)
        schema = [(tin, v), (t2, vout)]
    elif v <= vin:
        t2 = tout - time_from_vel_and_acc(v, vout, amax)
        schema = [(tin, v), (t2, vout)]
    else:  # Just stop, something is a bit wrong
        schema = [(tin, vout)]
    return schema


def find_cruise_velocity(tend, vstart, vend, vmax, amax, amin, s):
    area_l = calculate_minimum_area(vstart, amin)
    time_l = time_from_vel_and_acc(vstart, 0, amin)
    area_r = calculate_minimum_area(vend, amax)
    time_r = time_from_vel_and_acc(0, vend, amax)
    time_1_lower = tend - time_l - time_r
    time_1_upper = tend - time_from_vel_and_acc(vend, vstart, amax)
    area_1 = area_trapezoid(vstart, time_1_lower, time_1_upper)
    area_2 = area_rectangle(vend - vstart, time_1_upper)  # Actually a parallelogram
    time_l_high = time_from_vel_and_acc(vstart, vmax, amax)
    time_r_high = time_from_vel_and_acc(vmax, vend, amin)
    time_3_upper = tend - time_l_high - time_r_high
    time_3_lower = tend - time_from_vel_and_acc(vend, vstart, amax)
    area_3 = get_area_3(vend, vmax, time_3_lower, time_3_upper)

    if s >= area_l + area_r:
        a = (amax + amin) / (2 * amax * amin)
        b = time_1_lower
        c = -(s - area_l - area_r)
        return abc_formula(a, b, c)
    elif s >= area_l + area_r + area_1:
        a = s - area_l - area_r - area_1
        t = time_1_upper
        return (a / t) + vstart
    elif s >= area_l + area_3 + area_1 + area_2:
        a = (amax + amin) / (2 * amax * amin)
        b = time_3_upper
        c = s - area_l - area_r - area_1 - area_2 - area_3
        return abc_formula(a, b, c)
    return 0  # TODO handle this case better


# Calculate area from v = 0 to v = target_velocity with constant acceleration
def calculate_minimum_area(target_velocity, acceleration):
    if acceleration != 0:
        return pow(target_velocity, 2) / (2 * abs(acceleration))
    # Infinite area and time since no acceleration
    return -1


def get_area_3(vend, vmax, t_lower, t_upper):
    return area_trapezoid(vmax - vend, t_lower, t_upper)


# Returns the maximum assuming one might be negative, and we don't want negative velocity.
def abc_formula(a, b, c):
    return max(-((b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a)), -((b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a)))


def calculate_optimal_booking(t0, v0, v_kors, v_max, acc_min, acc_max, D):
    t1 = time_from_vel_and_acc(v0, v_max, acc_max)
    # self.t1 = (self.v_max - self.vo) / self.acc_max
    A1 = area_trapezoid(t0, t1, v0, v_max, acc_max)
    # self.A1 = ((self.v_max + self.vo) * (self.t1 - self.t0)) / (2 * self.acc_max)

    A3 = area_trapezoid(t0, t1, v_kors, v_max, acc_min)
    # self.A3 = math.abs((math.pow(self.v_max,2) - math.pow(self.v_kors,2)) / (2 * self.acc_min))

    t_in = None
    v_in = None
    schema = [(t0, 0)]

    if A1 + A3 <= D:
        A2 = D - A1 - A3
        t2 = t0 + (A2 / acc_max)
        # self.t2 = self.t0 + (self.A2/self.acc_max)
        t_end = time_from_vel_and_acc(v_max, v_kors, acc_min)
        # self.t_end = math.abs((self.v_max - self.v_kors) / self.acc_min)
        t_in = t0 + t1 + t2 + t_end
        v_in = v_kors
        # Acceleration schema (<t0,acc_max>, <t1,0>, <t2,acc_min)
        schema = [(t0, v_max), (t2, v_kors)]

    elif A1 + A3 > D:
        v_top = calculate_v_top(v0, v_kors, acc_min, acc_max, D)

        if (v_top >= v0) and (v_top >= v_kors):
            t4 = time_from_vel_and_acc(v0, v_top, acc_min)
            # self.t4 = math.pow((self.v_top - self.v0),2) / 2 * (self.acc_min)
            t_end = time_from_vel_and_acc(v_top, v_kors, acc_min)
            # self.t_end = math.abs((self.v_top - self.v_kors) / self.acc_min)
            t_in = t0 + t4 + t_end
            v_in = v_kors
            schema = [(t0, v_top), (t4, v_kors)]
            # Acceleration schema  (<t0,acc_max>, <t4,acc_min)

        elif v_top < v_kors:
            t_in = sqrt((2 * D) / acc_max) + t0
            v_in = acc_max * t_in - t0
            # Acceleration schema (<t0,acc_max>)
            schema = [(t0, v_kors)]

    # add some safty marigin
    # t_in += (t_in - t0) * 0,25
    return t_in, v_in, schema


def calculate_v_top(v0, v_kors, acc_min, acc_max, D):
    temp1 = acc_max * pow(v_kors, 2)
    temp2 = acc_min * pow(v0, 2)
    temp3 = 2 * acc_max * acc_min * D
    temp4 = acc_max - acc_min
    temp5 = (temp1 - temp2 - temp3) / temp4
    return sqrt(temp5)

