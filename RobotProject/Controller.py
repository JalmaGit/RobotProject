#Place Holder for Controller

class PID():

    def __init__(self, kp, ki, kd) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prevIntegral = 0
        self.prevError = 0

        self.windupGuard = 0.5
        self.enableWindup = True
        self.maxAngle = 1.24

        self.P = 0
        self.I = 0
        self.D = 0

    def control(self, desired, actual, dt):
        error = (desired - actual)
    
        P = error * self.kp
        I = self.prevIntegral + self.ki * error * dt
        D = self.kd * ((error - self.prevError) / dt)

        if self.enableWindup:
            I = self.windup(I)

        #print(f"{P=}, {I=}, {D=}")

        self.prevError = error
        self.prevIntegral = I

        self.P = P
        self.I = I
        self.D = D

        return P + I + D
        

    def windup(self, I):
        
        if I >= self.windupGuard:
            I = -self.windupGuard
        elif I <= -self.windupGuard:
            I = self.windupGuard

        return I 
