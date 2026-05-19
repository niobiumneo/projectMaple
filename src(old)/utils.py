def motor_angle2value(angle):
    value = (angle / 360) * 4095
    return value

def motor_value2angle(value):
    angle = (value / 4095) * 360
    return angle


