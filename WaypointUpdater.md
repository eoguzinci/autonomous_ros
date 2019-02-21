# Waypoint Updater Node

### Determining the closest point

First, we determine the closest point by using [KDTree](https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.spatial.KDTree.html) that allows us to perform the search in log(n) time.

We then determine whether or not the closest waypoint is behind us by analysing the dot product between two vectors. One is the vector from the previous car position to the closes waypoint and the other is the vector from closest waypoint to the current car position.

This figure illustrates the principle:

<img src="/imgs/waypoint_hyperplane.png" width="500">

If the dot product of these two vectors is positive then the closest waypoint is behind the car. If the dot product is negative, the closest waypoint is in front on the vehicle. If the closest waypoint is behind the car, we take the next one.

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
