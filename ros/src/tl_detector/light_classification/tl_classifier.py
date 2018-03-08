from __future__ import division
from styx_msgs.msg import TrafficLight
import rospy
import tensorflow as tf
import numpy as np
#from PIL import Image
#import cv2

FASTER_RCNN_GRAPH_FILE = 'light_classification/tld/frozen_inference_graph.pb'
BOX_CONFIDENCE = 0.8
RED_THRESHOLD = 150
GREEN_THRESHOLD = 150
CONF_TOP = 2.0
CONF_BOT = 0.5
TOP_5 = 5

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        
        self.detection_graph = self.load_graph(FASTER_RCNN_GRAPH_FILE)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
 
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        
        # The classification of the object (integer id).
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        self.count = 0
        
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        #norm_img = self.image_normalize(image)
        #image = (norm_img + 1.0)*255/2
        
        
        #cv2.imwrite("/media/sf_Shared/sim_{}_cv2.jpg".format(self.count), image)
        #self.count += 1
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
        with tf.Session(graph=self.detection_graph) as sess:                
            
            # Actual detection.
            (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                                feed_dict={self.image_tensor: image_np})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)

            #confidence_cutoff = BOX_CONFIDENCE
            top_x = TOP_5
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(top_x, boxes, scores, classes)

            # The current box coordinates are normalized to a range between 0 and 1.
            # This converts the coordinates actual location on the image.
            #width, height = image.size
            height, width, channel = image.shape
            box_coords = self.to_image_coords(boxes, height, width)

            # One pixel has one vote
            r_vote = 0
            g_vote = 0
            total_vote = 0
            if len(boxes) > 0:
                TL_Detected = True
            else:
                TL_Detected = False

    
            for i in range(len(boxes)):
                # Get the position of each box
                bot, left, top, right = box_coords[i, ...]
                # The class id of traffic light should be 1, but depend on the graph
                #class_id = int(classes[i])
                #tl_image = image.crop((int(left), int(bot), int(right), int(top)))
                tl_image = image[int(bot):int(top), int(left):int(right)]
                im = np.array(tl_image)
                total_vote += im.shape[0]*im.shape[1]
                # Create the histogram for each RGB channel
                rh, gh, bh = self.color_hist(im, nbins=32, bins_range=(0, 256))
                if rh is not None:           
                    for i in range(len(rh[0])):
                        if rh[1][i] > RED_THRESHOLD:
                            r_vote += rh[0][i] 
              
                if gh is not None:
                    for i in range(len(gh[0])):
                        if gh[1][i] > GREEN_THRESHOLD:
                            g_vote += gh[0][i]

            if TL_Detected:
                r_confidence = r_vote/total_vote
                g_confidence = g_vote/total_vote
                if g_confidence > 0.0:
                    conf_ratio = r_confidence/g_confidence
                    if conf_ratio > CONF_TOP:
                        return TrafficLight.RED
                    elif conf_ratio < CONF_BOT:
                        return TrafficLight.GREEN
                    else:
                        return TrafficLight.YELLOW
                else:
                    if r_confidence > 0.0:
                        return TrafficLight.RED
                    else:
                        rospy.loginfo('Check1...')
                        return TrafficLight.UNKNOWN
            else:
                rospy.loginfo('Check2...')
                return TrafficLight.UNKNOWN
            
        return TrafficLight.UNKNOWN

    def filter_boxes(self, top_x, boxes, scores, classes):
        """Return the top several scores boxes """
        idxs = []
        for i in range(top_x):
            #print("scores[{}] = {}, class = {}".format(i, scores[i], classes[i]))
            rospy.loginfo("scores[{}] = {}, class = {}".format(i, scores[i], classes[i]))
            idxs.append(i)
    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
    
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
    
        return box_coords

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def image_normalize(self, x):
        original_image = np.array(x, dtype=np.float32)
        for n in range(len(original_image)):
            #for the R, G, B channel respectively
            for i in range(3):
                maxPixel = np.amax(original_image[n][...,i])
                minPixel = np.amin(original_image[n][...,i])
                original_image[n][...,i] = -1.0 + (original_image[n][...,i] - minPixel)*2.0/(maxPixel-minPixel)
        return original_image
    
    def color_hist(self, img, nbins=32, bins_range=(0, 256)):
        # Compute the histogram of the RGB channels separately
        rhist = np.histogram(img[:,:,0], nbins, bins_range)
        ghist = np.histogram(img[:,:,1], nbins, bins_range)
        bhist = np.histogram(img[:,:,2], nbins, bins_range)
        return rhist, ghist, bhist
