# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 20,
                start_time = 0, alpha = 1., u_bounds = [float('-inf'), float('inf')]):
        
        
        # The PID controller can be initialized using specific kp, ki, and kd values.
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)

        # Set max wind up
        self.max_windup_ = float(max_windup)

        # Set alpha for the derivative filter
        self.alpha = float(alpha)

        # Set control effort saturation limits
        self.ce_saturation_min = u_bounds[0]
        self.ce_saturation_max = u_bounds[1]

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time
        self.error_sum_ = 0.0
        self.last_error_ = 0.0

        # Control effort history
        self.ce_p = [0]
        self.ce_i = [0]
        self.ce_d = [0]


    # Clear the class variables    
    def reset(self):
        self.set_point_ = 0.0
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0
        self.error_sum_ = 0.0
        self.last_timestamp_ = 0.0
        self.last_error_ = 0
        self.last_last_error_ = 0
        self.last_windup_ = 0.0
        

    def setTarget(self, target):
        self.set_point_ = float(target)
        

    def setKP(self, kp):
        self.kp_ = float(kp)
        

    def setKI(self, ki):
        self.ki_ = float(ki)
        

    def setKD(self, kd):
        self.kd_ = float(kd)
        

    def setMaxWindup(self, max_windup):
        self.max_windup_ = int(max_windup)
        

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        
        if delta_time == 0:
            return 0
        
        # Calculate the error
        error = self.set_point_ - measured_value

        # Set the last timestamp
        self.last_timestamp_ = timestamp

        # Sum the errors
        self.error_sum_ += error * delta_time

        # Find the delta error
        delta_error = error - self.last_error_

        # Update the past error
        self.last_error_ = error

        # Address max wind-up
        if self.error_sum_ > self.max_windup_:
            self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.max_windup_:
            self.error_sum_ = -self.max_windup_
        
        # Proportional error
        p = self.kp_ * error

        # Integral error
        i = self.ki_ * self.error_sum_

        # Recalculate the derivative error using derivative smoothing
        d = self.kd_ * (self.alpha * delta_error / delta_time + (1 - self.alpha) * self.last_error_)

        # Set the control effort
        ce = p + i + d

        # Enforce actuator saturation limits
        if ce > self.ce_saturation_max:
            ce = self.ce_saturation_max
        elif ce < self.ce_saturation_min:
            ce = self.ce_saturation_min

        # Store the control effort history for post-control observations
        self.ce_p.append(p)
        self.ce_i.append(i)
        self.ce_d.append(d)

        return ce
