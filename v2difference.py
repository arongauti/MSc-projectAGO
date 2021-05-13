import cv2
import os, os.path
import glob
import numpy as np
from datetime import date
import time
#from skimage.metrics import structural_similarity
from skimage.measure import compare_ssim
#from findDifference import*
#from get_difference import*
import matplotlib.pyplot as plt 
import argparse
import imutils
# Font for puttext funcion
font = cv2.FONT_HERSHEY_PLAIN
#path = "/Users/Aron Gauti/Documents/meis-myndgreining/frames"
#folder = "niveacleansingmilk100" #skip
#folder = "All_IMG" # skip 34 50 71 73 83 93
#folder = "elastic20"
#folder = "niveacleansingmilk300"
#folder = "niveatexture70"
item = ""

textpath = "/home/lab/Pictures/"
# for i in os
cv_img = []
images = []

YCUT=0#60
XCUT=0#180
HCUT=720#600
WCUT=1280#940

def findDifference(imageA, imageB):
    # convert the images to grayscale
    #imageA =cv_img[0]
    #imageB =cv_img[1]
    grayA = cv2.cvtColor(cv2.medianBlur(imageA,9), cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(cv2.medianBlur(imageB,9), cv2.COLOR_BGR2GRAY)
    #grayA = cv2.cvtColor(cv2.bilateralFilter(imageA,9,75,75),cv2.COLOR_BGR2GRAY)
    #grayB = cv2.cvtColor(cv2.bilateralFilter(imageB,9,75,75),cv2.COLOR_BGR2GRAY)
    
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    thresh = cv2.threshold(diff, 100, 255,
	cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours
    for c in cnts:
        # compute the bounding box of the contour and then draw the
        # bounding box on both input images to represent where the two
        # images differ
        area = cv2.contourArea(c)
        if area > 30000 and area < 80000:
            (x, y, w, h) = cv2.boundingRect(c)
            #cv2.rectangle(imageA, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.rectangle(imageB, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.imshow("Modified", imageB)
            diff = cv2.drawContours(imageB, cnts, -1, (0, 255, 0), 2)
            cv2.imshow("Diff", diff)
            
            return(x, y, w ,h)
    # show the output images
    #cv2.imshow("Original", imageA)
    #cv2.imshow("Modified", imageB)
    
    #cv2.imshow("Thresh", thresh)
    cv2.waitKey(0)
    return(0,0,0,0)

def is_contour_bad(c):
	# approximate the contour
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.02 * peri, True)
	# the contour is 'bad' if it is not a rectangle
	return not len(approx) == 4

def main():
    x, y, w, h = 0, 0 ,0 ,0
    #cv2.imshow("empty", empty)
    filenames = [img for img in glob.glob(path+"/*.png")]
    filenames.sort()
    #print(filenames)
    for img in filenames: #test3
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            #i = i + 1
            break
        #print(img)
        n = cv2.imread(img)
        cv_img.append(n)
    
    
    test = open(os.path.join(textpath, "listOfImagesTest.txt"),"a+")
    train = open(os.path.join(textpath, "listOfImagesTrain.txt"),"a+")
    for i in range(0,len(cv_img)-1):
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
    
        
        #Copy the images
        cropped = (cv_img[i].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]
        beforeImg = (cv_img[i].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]# empty.copy()[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]#(cv_img[i-1].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]
        afterImg = (cv_img[i+1].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]
        xd, yd, wd, hd = findDifference(beforeImg, afterImg)
        print(xd, yd, wd, hd)
        img_copy =  cv_img[i].copy()
        retImage = img_copy.copy()
        height, width, c = img_copy.shape
        #print("Height", height, "Width", width)
        #get_difference(cv_img[i-1], cv_img[i])
        gray = cv2.Canny(cv2.cvtColor(cv2.medianBlur(cropped,9), cv2.COLOR_BGR2GRAY), 127,127*2)
        #cv2.imshow("gray", gray)
        # create a binary thresholded image
        ret, thresh = cv2.threshold(gray, 127, 255, 0)
        # show it
        #cv2.imshow("Test", thresh)
        # find the contours from the thresholded image
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Write to a txt file
        imgname = folder +"_" + item + str(i).zfill(4)
        f= open(os.path.join(imgFolder, imgname+".txt"),"w+")
        
        x = xd
        y = yd
        h = hd
        w = wd       

        
        print("fer hingad")
        X = float(x+XCUT)
        Y = float(y+YCUT)
        XC = float(x+XCUT)+float(w/2)
        YC = float(y+YCUT)+float(h/2)
        W = float(w)
        H = float(h)
        center = (int(XC),int(YC))
        cv2.circle(img_copy, center, 5, (255, 0, 0), 2)
        cv2.rectangle(img_copy, (int(X), int(Y)), (int(X) + w, int(Y) + h), (36,255,12), 2) # to see the rectangle
        cv2.putText(img_copy,imgname,(10,675), font, 2,(255,255,255),2,cv2.LINE_4)
        cv2.imshow("test", img_copy)
        cv2.waitKey(0)
        #f.write("0 %.9f %.9f %.9f %.9f\r\n" % (float(X),float(Y),float(W),float(H)))
        f.write("0 %.9f %.9f %.9f %.9f\r\n" % (float(XC/width),float(YC/height),float(W/width),float(H/height)))
        #print("0 %.9f %.9f %.9f %.9f\r\n" % (float(X/width),float(Y/height),float(W/width),float(H/height)))
        #cv2.imwrite(os.path.join(imgFolder, imgname+"box.png"), img_copy)
        cv2.imwrite(os.path.join(imgFolder, imgname+".png"), retImage)
        cv2.imwrite(os.path.join(imgFolder, imgname+"marked.png"), img_copy)
        if((float(i)/float(len(cv_img)))<=0.75):
            train.write(imgFolder + "/" +imgname + ".png\r\n")
        elif((float(i)/float(len(cv_img)))>0.75):
            test.write(imgFolder + "/" + imgname + ".png\r\n")
        
        f.close() 
        #cv2.imwrite("{}/{}.png".format(dirName,i), img_copy)
        """ if cv2.waitKey(0) & 0xFF == ord('q'): 
            cv2.destroyAllWindows()
            break  """
    cv2.destroyAllWindows()

if __name__ == "__main__":
    today = date.today()
    start = time.time()
    folder = raw_input("Folder name\n")
    print(folder)
    path = "/home/lab/Pictures/data/"+ folder
    # Create director
    dirName = path #+ "/" + str(today) 
    imgFolder = dirName + "/imgs" 
    try:
        # Create target Directory
        #os.mkdir(dirName)
        os.mkdir(imgFolder)
        print("Directory created ") 
    except:
        print("Directory already exists") 
    print("Start")
    
    main()
    end = time.time()
    currTime=(end - start)
    print("Finished - Time: ", currTime)