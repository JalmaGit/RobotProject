#Place Holder for Controller

class PID():

    def __init__(self, kp, ki, kd) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prevIntegral = 0
        self.prevError = 0

        self.windupGuard = 0
        self.enableWindup = False

    def control(self, desired, actual, dt):
        error = desired - actual
    
        P = error * self.kp
        I = self.prevError + self.ki * error * dt
        D = (error - self.prevError) / dt

        if self.enableWindup:
            I = self.windup(I)

        self.prevError = error
        self.prevIntegral = I

        return P + I + D
    
    def windup(self, I):
        
        if I >= self.windupGuard:
            I = self.windupGuard
        elif I <= -self.windupGuard:
            I = -self.windupGuard

        return I 
