#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  8 12:37:13 2018

@author: simonchu
"""

"""
How to use the file?
python test_classifier.py --image <your image path>
"""
from light_classification.tl_classifier import TLClassifier
import cv2
import tensorflow as tf

flags = tf.app.flags
FLAGS = flags.FLAGS

# Command line flags
flags.DEFINE_string('image', '', "The image path")

img_path = FLAGS.image

image = cv2.imread(img_path)
light_classifier = TLClassifier()
state = light_classifier.get_classification(image)
if state == 0:
    print("It's RED Light")
elif state == 1:
    print("It's YELLOW Light")
elif state == 2:
    print("It's GREEN Light")
elif state == 4:
    print("It's Unknown")