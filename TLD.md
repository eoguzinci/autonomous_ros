# Traffic light detection and classification

This document describes the traffic light detection and classification modules that we have implemented.

### General approach
We use and implementation of Single-Shot MultiBox Detector (SSD) from https://github.com/pierluigiferrari/ssd_keras that we finetune to our task.

It is trained to detect the RED lights only... otherwise returns UNKNOWN
and is correct the vast majority of the time

possible next steps...

### Network architecture

![alt text](https://github.com/AShpilman/autonomous_ros/blob/master/imgs/SSD.jpg)

SSD network needs an input image and ground truth boxes for each object for training. It evaluates default boxes of different aspect ratios at each location in several feature maps with different scales and predicts the shape offsets and the confidence for all classes in boxes.

![alt text](https://github.com/AShpilman/autonomous_ros/blob/master/imgs/SSD2.png)

SSD network model adds several feature layers to the end of a base network, which predict the offsets to default boxes of different scales and aspect ratios and their associated confidences.

*Images are from the original paper* - https://arxiv.org/pdf/1512.02325.pdf

### Implementation

First, we had to port SSD keras implementation from Python3 to Python2.

### ROS integration 

To integrate our traffic light detection pipeline we completed the following things:

* Add a subscriber for /traffic_waypoing and /obstacle_waypoint

```
waypoint_updater 37
```

* Implement a callback for /traffic_waypoint message and /obstacle_waypoint message


```
waypoint_updater 136&140
```

* Add a function for detection of the closest visible traffic light (if one exists)

```
tl_detector 111
```

* Load classifier

```
tl_classifier 5
```

* Implement traffic light color prediction

```
tl_classifier 18
```


### Testing

We tested our module in both the simulator and on a real-world video data and found it to be highly accurate.
