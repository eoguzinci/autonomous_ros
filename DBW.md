# Drive-by-wire node

This document detail our implementation of the drive-by-wire system (DBW).

After the waypoints are published in [Waypoint Updater node](WaypointUpdater.md) we then have to control the vehicle by passing throttle, brake and steering commands. These commands are published at 50Hz. If the commands are published significantly less frequent than 50Hz , he system interprets it as a computer failure and passes control to the driver.

### On\off DBW controller
First we check if the dbw controller is on or off. If we do not do this, we might accumulate integral part of PID controller while the car is controlled by the driver, which can lead to erratic behavior when the controll is passed back to the system.

```
if not dbw_enabled:
    self.throttle_controller.reset()
    return 0., 0. , 0. 
```

### Throttle

We use a standard PID controller with predetermined coefficients for proportional, integral and derivative parts.

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

A throttle is then a simple application of this PID controller.

### Breaking

If the PID velocity error is negative, we apply the breaks. We send the break commands in units of torque (Nm). To find the desired break power we need to multiply the desired acceleration by wheel radius and vehicles mass. 

We then take two cases into account:

* Car is not moving. In that case we keep applying the brake at 400 Nm. 

```
if linear_vel == 0 and current_vel < 0.1:   
    throttle = 0
    brake = 700 # Nm - to hold the car in place if we are stopped at a light. Acceleration -1m/s^2
```

* Car is moving. Then we apply the break according to the formula described above. To find the exact vehicle mass we take into account the current fuel level. 

```
elif throttle < .1 and vel_error < 0:
    throttle = 0 
    decel = max(vel_error, self.decel_limit)    
    brake = abs(decel)* (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)*self.wheel_radius # Torque Nm
```


### Steering 

We implement steering by first evaluating the turning radius:

```
def get_steering(self, linear_velocity, angular_velocity, current_velocity):
    angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

    if abs(current_velocity) > 0.1:
        max_yaw_rate = abs(self.max_lat_accel / current_velocity);
        angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

    return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
```

And then by converting turning radius to steering angle:

```
def get_angle(self, radius):
    angle = atan(self.wheel_base / radius) * self.steer_ratio
    return max(self.min_angle, min(self.max_angle, angle))
```
