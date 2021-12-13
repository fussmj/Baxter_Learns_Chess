#!/usr/bin/python2
import cv2
import numpy as np
import os
path1 = r'C:/Users/bigmi/Desktop/Robotics Lab/Final Project/Baxter_Learns_Chess/baxter_chess/Images/saved_patches_previous/'
path2 = r'C:/Users/bigmi/Desktop/Robotics Lab/Final Project/Baxter_Learns_Chess/baxter_chess/Images/saved_patches/'

list = os.listdir('C:/Users/bigmi/Desktop/Robotics Lab/Final Project/Baxter_Learns_Chess/baxter_chess/Images/saved_patches_previous/')
for l in range(len(list)):
    before = cv2.imread(path1+str(list[l]))
    after = cv2.imread(path2+str(list[l]))
    err = np.sum((after.astype("float") - before.astype("float")) ** 2)
    if err > 10:
        string = list[l]
        output = string.replace('.png','')
        print(output)