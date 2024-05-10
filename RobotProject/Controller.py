#Place Holder for Controller

class PID():

    def __init__(self, kp, ki, kd) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prevIntegral = 0
        self.prevError = 0

    def control(self, desired, actual, dt):
        error = desired - actual
    
        P = error * self.kp
        I = self.prevError + self.ki * error * dt
        D = (error - self.prevError) / dt

        self.prevError = error
        self.prevIntegral = I

        return P + I + D
