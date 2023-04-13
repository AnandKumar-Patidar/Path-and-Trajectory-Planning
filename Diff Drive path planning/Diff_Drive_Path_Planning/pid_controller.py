class PIDController:
    def __init__(self, kp, ki, kd, setpoint, sample_time):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.previous_error = 0
        self.integral = 0
        self.output = 0
        self.last_time = None
    
    def update(self, measured_value):
        # Calculate error and delta_time
        error = self.setpoint - measured_value
        delta_time = self.sample_time
        
        # Calculate proportional term
        p = self.kp * error
        
        # Calculate integral term
        self.integral += error * delta_time
        i = self.ki * self.integral
        
        # Calculate derivative term
        derivative = (error - self.previous_error) / delta_time
        d = self.kd * derivative
        
        # Calculate output
        self.output = p + i + d
        
        # Update previous error and last_time
        self.previous_error = error
        self.last_time = time.monotonic()
        
        return self.output
    
    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.output = 0
        self.last_time = None
