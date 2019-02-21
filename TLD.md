# Traffic light detection and classification

This document describes the traffic light detection and classification modules that we have implemented.

### General approach
We use an [implementation](https://github.com/pierluigiferrari/ssd_keras) of the Single-Shot MultiBox Detector (SSD) that we finetune to our task.

### Network architecture

<img src="/imgs/SSD.jpg" width="800">

SSD network needs an input image and ground truth boxes for each object for training. It evaluates default boxes of different aspect ratios at each location in several feature maps with different scales and predicts the shape offsets and the confidence for all classes in boxes.

<img src="/imgs/SSD2.png" width="800">

SSD network model adds several feature layers to the end of a base network, which predict the offsets to default boxes of different scales and aspect ratios and their associated confidences.

*Images are from the [original paper](https://arxiv.org/pdf/1512.02325.pdf)*.

### Implementation and training

First, we had to port SSD keras implementation from Python3 to Python2.

We then trained the network on Lara and Bosh traffic light datasets.

* [Lara Traffic Lights Recognition dataset](http://www.lara.prd.fr/benchmarks/trafficlightsrecognition) includes 11 179 640x480 frames
* [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132) consists of 1342 images at a resolution 1280x720 pixels and contains about 24000 annotated traffic lights.

We downsampled all images to 600 by 800 pixels and trained the network for 14 epochs using Adam optimizer with learning rate of 0.001. 
Model is saved to 'ssd7_epoch-14_loss-1.0911_val_loss-0.5348.h5'.

Model trained quite well:

<img src="/imgs/ssd7_sim_3lights_20epochs.png" width="600">

This picture shows the output of the model on sample image:

<img src="/imgs/ssd7_low_conf_red_light_output.png" width="600">

We then filter bounding boxes by confidence threshold to detect green:

<img src="/imgs/ssd7_green_light_output.png" width="600">

or, more importantly, red lights:

<img src="/imgs/ssd7_red_light_output.png" width="600">

### ROS integration

To integrate our traffic light detection pipeline we perform the following steps:

* Load the model.

```
weights_path = 'ssd7_epoch-14_loss-1.0911_val_loss-0.5348.h5'
self.model.load_weights(weights_path, by_name=True)
```

* Use the model to classify red lights from the incoming images. 

```
with graph.as_default():
    y_pred = self.model.predict(input_images)

y_pred_decoded = decode_detections(y_pred,
                                   confidence_thresh=0.5,
                                   iou_threshold=0.1,
                                   top_k=200,
                                   normalize_coords=self.normalize_coords,
                                   img_height=self.img_height,
                                   img_width=self.img_width)
```

* Find the best average class and if it's red - return TrafficLight.RED state

```
top3_green_avg = np.average(np.sort(y_pred_decoded[0][list(y_pred_decoded[0][:, 0] == 1), 1])[-3:])
top3_red_avg = np.average(np.sort(y_pred_decoded[0][list(y_pred_decoded[0][:, 0] == 2), 1])[-3:])
top3_yellow_avg = np.average(np.sort(y_pred_decoded[0][list(y_pred_decoded[0][:, 0] == 3), 1])[-3:])
top3s = np.nan_to_num([top3_green_avg, top3_red_avg, top3_yellow_avg])
best_avg = np.argmax(top3s) + 1

if best_avg == 2:
    return TrafficLight.RED
```

* Publish upcoming red lights at camera frequency.

We publish upcoming red light if we observe it for 3 states (STATE_COUNT_THRESHOLD is set to 3)

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


* Check the distance from the car to closest light to see whether or not closer that LIGHT_DISTANCE_THRESHOLD (set to 60).

```
if closest_light and closest_light_distance < LIGHT_DISTANCE_THRESHOLD:
    state = self.get_light_state(closest_light)  # approaching a light, try to determine its state
    rospy.loginfo("approaching %s traffic light %f ahead", state, closest_light_distance)
    return line_wp_idx, state
```


### Testing

We tested our module in both the simulator and on a real-world video data and found it to be highly accurate.
