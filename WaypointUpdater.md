# Waypoint Updater Node

The Waypoint Updater Node takes in a list of base waypoints and calculates the next 50 waypoints from our vehicle's current position. We can change the number of waypoints by changing the LOOKAHEAD_WPS constant. It also takes into account traffic lights and creates a waypoints list for when we need to decelerate the vehicle. 

### Calculating the list of the next waypoints 

First, we determine the closest point to our location. We use [KDTree](https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.spatial.KDTree.html) that allows us to perform the search in log(n) time.

We then determine whether or not the closest waypoint is behind us by analysing the dot product between two vectors. One is the vector from the previous car position to the closes waypoint and the other is the vector from closest waypoint to the current car position.

This figure illustrates the principle:

<img src="/imgs/waypoint_hyperplane.png" width="500">

If the dot product of these two vectors is positive then the closest waypoint is behind the car. If the dot product is negative, the closest waypoint is in front on the vehicle. If the closest waypoint is behind the car, we take the next closest waypoint from KDTree output.

```
def get_closest_waypoint_idx(self):
      x = self.pose.pose.position.x
      y = self.pose.pose.position.y
      closest_idx = self.waypoint_tree.query([x,y],1)[1]

      # Check if the closest point is ahead or behind the vehicle
      closest_coord = self.waypoint_2d[closest_idx]
      prev_coord = self.waypoint_2d[closest_idx-1]

      # Equation for hyperplane through closest coord
      cl_vec = np.array(closest_coord)
      prev_vec = np.array(prev_coord)
      pos_vec = np.array([x, y])

      val = np.dot(cl_vec - prev_vec, pos_vec - cl_vec)

      # If closest waypoint is behind the car, we take the next one
      if val > 0:
          closest_idx = (closest_idx+1) % len(self.waypoints_2d)
      return closest_idx
```

After determining the closest waypoint that is in front of the car, we take the next LOOKAHEAD_WPS points from the base_waypoints. If there is no red traffic light on the way, we just publish those as our final_waypoints.

### Taking traffic lights into account

However, if there is a red light on the way, we need to correct our line. 

We fierst limit our waypoint list to the index of stopline waypoint minus 2. That our the car won't go over the stop line. 

We then decelerate the vehicle to the target velocity of the square root of two times the maximum deceleration times the distance to the stop waypoint.

This way, the closer we are, the lower our speed and the higher the deceleration.

This figure illustrates how our target velocity changes depending on the distance to the stop waypoint:

<img src="/imgs/waypoint_stop.png" width="800">

This results in the following function for the generation of deceleration waypoints:

```
def decelerate_waypoints(self, waypoints, closest_idx):
  temp = []
  for i, wp in enumerate(waypoints):
      p = Waypoint()
      p.pose = wp.pose

      # Stop at two waypoints back from the traffic light
      stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
      dist = self.distance(waypoints,i,stop_idx)
      vel = math.sqrt(2 * MAX_DECEL * dist)
      if vel < 1.:
          vel = 0

      p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
      temp.append(p)

  return temp
```

To handle both cases we create the generate_lane function that outputs a list of base waypoints in case there is no red light and a list of deceleration waypoint in case there is.

```
def generate_lane(self):
  lane = Lane()

  closest_idx = self.get_closest_waypoint_idx()
  farthest_idx = closest_idx + LOOKAHEAD_WPS
  base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

  if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
      lane.waypoints = base_waypoints
  else:
      lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

  return lane
```

We then publish the final_waypoints:

```
def publish_waypoints(self):
  final_lane = self.generate_lane()
  self.final_waypoints_pub.publish(final_lane)
```
