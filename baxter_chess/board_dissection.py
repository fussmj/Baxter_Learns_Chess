#!/usr/bin/python2
import cv2
import numpy as np

img =  cv2.imread("/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/crop.png")
image_copy = img.copy() 
imgheight=img.shape[0]
imgwidth=img.shape[1]

M = imgheight/8
N = imgwidth/8
x1 = 0
y1 = 0

for y in range(0, imgheight, M):
    for x in range(0, imgwidth, N):
        if (imgheight - y) < M or (imgwidth - x) < N:
            break
            
        y1 = y + M
        x1 = x + N

        # check whether the patch width or height exceeds the image width or height
        if x1 >= imgwidth and y1 >= imgheight:
            x1 = imgwidth - 1
            y1 = imgheight - 1
            #Crop into patches of size MxN
            tiles = image_copy[y:y+M, x:x+N]
            #Save each patch into file directory
            cv2.imwrite('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/saved_patches/'+'tile'+str(x)+'_'+str(y)+'.png', tiles)
            cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)
        elif y1 >= imgheight: # when patch height exceeds the image height
            y1 = imgheight - 1
            #Crop into patches of size MxN
            tiles = image_copy[y:y+M, x:x+N]
            #Save each patch into file directory
            cv2.imwrite('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/saved_patches/'+'tile'+str(x)+'_'+str(y)+'.png', tiles)
            cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)
        elif x1 >= imgwidth: # when patch width exceeds the image width
            x1 = imgwidth - 1
            #Crop into patches of size MxN
            tiles = image_copy[y:y+M, x:x+N]
            #Save each patch into file directory
            cv2.imwrite('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/saved_patches/'+'tile'+str(x)+'_'+str(y)+'.png', tiles)
            cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)
        else:
            #Crop into patches of size MxN
            tiles = image_copy[y:y+M, x:x+N]
            #Save each patch into file directory
            cv2.imwrite('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/saved_patches/'+'tile'+str(x)+'_'+str(y)+'.png', tiles)
            cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)