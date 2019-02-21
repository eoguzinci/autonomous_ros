# Traffic light detection and classification

This document describes the traffic light detection and classification modules that we have implemented.

### General approach
We use an [implementation](https://github.com/pierluigiferrari/ssd_keras) of the Single-Shot MultiBox Detector (SSD) that we finetune to our task.

### Network architecture

![alt text](https://github.com/AShpilman/autonomous_ros/blob/master/imgs/SSD.jpg)

SSD network needs an input image and ground truth boxes for each object for training. It evaluates default boxes of different aspect ratios at each location in several feature maps with different scales and predicts the shape offsets and the confidence for all classes in boxes.

![alt text](https://github.com/AShpilman/autonomous_ros/blob/master/imgs/SSD2.png)

SSD network model adds several feature layers to the end of a base network, which predict the offsets to default boxes of different scales and aspect ratios and their associated confidences.

*Images are from the [original paper](https://arxiv.org/pdf/1512.02325.pdf)*

### Implementation and training

First, we had to port SSD keras implementation from Python3 to Python2.

We then trained the network on Lara and Bosh traffic light datasets.

* [Lara Traffic Lights Recognition dataset](http://www.lara.prd.fr/benchmarks/trafficlightsrecognition) include 11 179 640x480 frames
* [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132) consists of 1342 images at a resolution 1280x720 pixels and contains about 24000 annotated traffic lights.

We trained the network to detect RED traffic lights only (otherwise returns UNKNOWN).

### ROS integration 

To integrate our traffic light detection pipeline we completed the following things:

* Publish upcoming red lights at camera frequency.

```
if self.state != state:
    self.state_count = 0
    self.state = state
elif self.state_count >= STATE_COUNT_THRESHOLD:
    self.last_state = self.state
    light_wp = light_wp if state == TrafficLight.RED else -1
    self.last_wp = light_wp
    self.upcoming_red_light_pub.publish(Int32(light_wp))
else:
    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
self.state_count += 1
```

STATE_COUNT_THRESHOLD is set to 3.

* Check the distance from the car to closest light.

```
if closest_light and closest_light_distance < LIGHT_DISTANCE_THRESHOLD:
    state = self.get_light_state(closest_light)  # approaching a light, try to determine its state
    rospy.loginfo("approaching %s traffic light %f ahead", state, closest_light_distance)
    return line_wp_idx, state
```

### Testing

We tested our module in both the simulator and on a real-world video data and found it to be highly accurate.
