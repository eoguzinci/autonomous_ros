# Drive-by-wire node

This document detail our implementation of drive-by-wire systed.

After the waypoints are published in [Waypoint Updater node](WaypointUpdater.md) we then have to steer the vehicle by passing throttle, steering and brake commands. These commands are published at 50Hz. If the commands are published significantly below the 50Hz level, he system interprets it as a computer failure and passes control to the driver.

### On\off dbw controller
First we check if the dbw controller is on or off. If we do not do this, we might accumulate integral part of PID controller while the car is controlled by the driver, which can lead to erratic behavior when the controll is passed back to the system.

```
if not dbw_enabled:
    self.throttle_controller.reset()
    return 0., 0. , 0. 
```

### PID controller

```
def step(self, error, sample_time):

    integral = self.int_val + error * sample_time;
    derivative = (error - self.last_error) / sample_time;

    val = self.kp * error + self.ki * integral + self.kd * derivative;

    if val > self.max:
        val = self.max
    elif val < self.min:
        val = self.min
    else:
        self.int_val = integral
    self.last_error = error

    return val
```

### Throttle 

### Steering 

### Breaking

We send the break commands in units of torque (Nm). To find the desired break power we need to multiply the desired acceleration by wheel radius and vehicles mass. 



```

```

We then take to cases for breaking into accont.

* Car is stopped. In that case we keep applying the brake at 400 Nm. 

```
if linear_vel == 0 and current_vel < 0.1:   
    throttle = 0
    brake = 400 # Nm - to hold the car in place if we are stopped at a light. Acceleration -1m/s^2
```

* Car is stopping. Then we apply the break according to the formula described above. To find the exact vehicle mass we take into account the current fuel level. 

```
elif throttle < .1 and vel_error < 0:
    throttle = 0 
    decel = max(vel_error, self.decel_limit)    
    brake = abs(decel)* (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)*self.wheel_radius # Torque Nm
```
