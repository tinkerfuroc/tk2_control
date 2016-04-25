from motor import Motor

class Chassis:
    def __init__(self):
        self.motors = [Motor(0), Motor(1), Motor(2), Motor(3)]

    def set_wheels_speed(self, speeds):
        for motor, vel in zip(self.motors, speeds):
            motor.set_vel(vel)
    
    def set_wheels_count(self, counts):
        for motor, count in zip(self.motors, counts):
            motor.set_count(count)

    def get_feedback(self):
        return [motor.get_feedback() for motor in self.motors]

    def stop(self):
        self.set_wheels_count([0, 0, 0, 0])

    def close(self):
        for motor in self.motors:
            motor.close()
