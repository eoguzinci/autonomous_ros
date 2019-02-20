from styx_msgs.msg import TrafficLight

import rospy

from keras import backend as K
from keras.models import load_model
from keras.preprocessing import image
from keras.optimizers import Adam
from imageio import imread
import numpy as np
from matplotlib import pyplot as plt
import tensorflow as tf
from models.keras_ssd7 import build_model
from keras_loss_function.keras_ssd_loss import SSDLoss
from ssd_encoder_decoder.ssd_output_decoder import decode_detections, decode_detections_fast

class TLClassifier(object):
    def __init__(self, on_sim):
        ## to force CPU mode
        # import os
        # os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
        from keras.backend.tensorflow_backend import set_session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        set_session(tf.Session(config=config))

        self.img_height = 600  # 396 #300 # Height of the input images
        self.img_width = 800  # 634 #480 # Width of the input images
        self.img_channels = 3  # Number of color channels of the input images
        intensity_mean = 127.5  # Set this to your preference (maybe `None`). The current settings transform the input pixel values to the interval `[-1,1]`.
        intensity_range = 127.5  # Set this to your preference (maybe `None`). The current settings transform the input pixel values to the interval `[-1,1]`.
        n_classes = 3  # 4 # Number of positive classes
        scales = [0.08, 0.16, 0.32, 0.64,
                  0.96]  # An explicit list of anchor box scaling factors. If this is passed, it will override `min_scale` and `max_scale`.
        aspect_ratios = [0.5, 1.0, 2.0]  # The list of aspect ratios for the anchor boxes
        two_boxes_for_ar1 = True  # Whether or not you want to generate two anchor boxes for aspect ratio 1
        steps = None  # In case you'd like to set the step sizes for the anchor box grids manually; not recommended
        offsets = None  # In case you'd like to set the offsets for the anchor box grids manually; not recommended
        clip_boxes = False  # Whether or not to clip the anchor boxes to lie entirely within the image boundaries
        variances = [1.0, 1.0, 1.0, 1.0]  # The list of variances by which the encoded target coordinates are scaled
        self.normalize_coords = True  # Whether or not the model is supposed to use coordinates relative to the image size

        K.clear_session()  # Clear previous
        self.model = build_model(image_size=(self.img_height, self.img_width, self.img_channels),
                                 n_classes=n_classes,
                                 mode='training',
                                 l2_regularization=0.0005,
                                 scales=scales,
                                 aspect_ratios_global=aspect_ratios,
                                 aspect_ratios_per_layer=None,
                                 two_boxes_for_ar1=two_boxes_for_ar1,
                                 steps=steps,
                                 offsets=offsets,
                                 clip_boxes=clip_boxes,
                                 variances=variances,
                                 normalize_coords=self.normalize_coords,
                                 subtract_mean=intensity_mean,
                                 divide_by_stddev=intensity_range)

        weights_path = 'ssd7_epoch-14_loss-1.0911_val_loss-0.5348.h5'
        if not on_sim:
            #TODO update this
            weights_path = 'ssd7_epoch-14_loss-1.0911_val_loss-0.5348.h5'

        self.model.load_weights(weights_path, by_name=True)

        global graph
        graph = tf.get_default_graph()

        adam = Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)
        ssd_loss = SSDLoss(neg_pos_ratio=3, alpha=1.0)
        self.model.compile(optimizer=adam, loss=ssd_loss.compute_loss)

        pass

    def get_classification(self, img):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        input_images = []
        input_images.append(img)
        input_images = np.array(input_images)

        with graph.as_default():
            y_pred = self.model.predict(input_images)

        y_pred_decoded = decode_detections(y_pred,
                                           confidence_thresh=0.5,
                                           iou_threshold=0.1,
                                           top_k=200,
                                           normalize_coords=self.normalize_coords,
                                           img_height=self.img_height,
                                           img_width=self.img_width)

        if len(y_pred_decoded[0]) == 0:
            return TrafficLight.UNKNOWN

        if len(y_pred_decoded[0][0]) == 0:
            return TrafficLight.UNKNOWN

        top3_green_avg = np.average(np.sort(y_pred_decoded[0][list(y_pred_decoded[0][:, 0] == 1), 1])[-3:])
        top3_red_avg = np.average(np.sort(y_pred_decoded[0][list(y_pred_decoded[0][:, 0] == 2), 1])[-3:])
        top3_yellow_avg = np.average(np.sort(y_pred_decoded[0][list(y_pred_decoded[0][:, 0] == 3), 1])[-3:])
        top3s = np.nan_to_num([top3_green_avg, top3_red_avg, top3_yellow_avg])
        best_avg = np.argmax(top3s) + 1

        if best_avg == 2:
            return TrafficLight.RED

        return TrafficLight.UNKNOWN
