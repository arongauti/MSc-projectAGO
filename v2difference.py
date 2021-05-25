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
import shutil
from scipy.ndimage.filters import gaussian_filter
# Font for puttext funcion
font = cv2.FONT_HERSHEY_PLAIN

item = ""
maxboxsize= 157500
textpath = "/home/lab/Pictures/"
# for i in os
cv_img = []
images = []

YCUT=0#60
XCUT=0#180
HCUT=720#600
WCUT=1280#940

def findDifference(imageA, imageB):
    global diff, testing1, testing2
    max_area = 0
    # convert the images to grayscale
    copybefore = imageA.copy()
    copyafter = imageB.copy()
    imageA = cv2.pyrMeanShiftFiltering(cv2.blur(imageA, (5,5)),3,3) #Filter to remove noise
    imageB = cv2.pyrMeanShiftFiltering(cv2.blur(imageB, (5,5)),3,3) #Filter to remove noise
    grayA = cv2.cvtColor(cv2.medianBlur(imageA,9), cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(cv2.medianBlur(imageB,9), cv2.COLOR_BGR2GRAY)
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    thresh = cv2.threshold(diff, 100, 255,cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    # loop over the contours
    (x, y, w, h) = 0,0,0,0
    for c in (cnts):
        # compute the bounding box of the contour and then draw the
        # bounding box on both input images to represent where the two
        # images differ
        area = cv2.contourArea(c)
        if area > 10000 and area < 100000:
            if area > max_area:
                cnt = c
                max_area = area
            
    
    if not (max_area == 0):
        perimeter = cv2.arcLength(cnt,True)
        epsilon = 0.005*cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        (x, y, w, h) = cv2.boundingRect(approx)
        hull = cv2.convexHull(cnt)
        
    if((not(x, y, w, h) == (0,0,0,0)) and ((h*w)<157500) and ((h*w)>30000) and cnt.size<2000 and cnt.size > 200 and not is_contour_bad(cnt)):
        print(cnt.size)
        testing2 = cv2.drawContours(copybefore.copy(), [approx], -1, (0, 0, 255), 3)
        testing1 = cv2.drawContours(copybefore.copy(), [hull], -1, (0, 0, 255), 3)
        diff = cv2.drawContours(copybefore.copy(), cnt, -1, (0, 0, 255), 3)

        return(x, y, w ,h)
    #print("contour bad")
    return(0,0,0,0)
def delete_spikes(cnt):
    extLeft = tuple(cnt[cnt[:, :, 0].argmin()][0])
    extRight = tuple(cnt[cnt[:, :, 0].argmax()][0])
    extTop = tuple(cnt[cnt[:, :, 1].argmin()][0])
    extBot = tuple(cnt[cnt[:, :, 1].argmax()][0])
    meanW = sum(tuple(cnt[:, :, 0])) / len(tuple(cnt[:, :, 0]))
    meanh = sum(tuple(cnt[:, :, 1])) / len(tuple(cnt[:, :, 1]))
    lenfromcenter1 = abs(meanW[0] - extLeft[0])
    lenfromcenter2 = abs(meanW[0] - extRight[0])
    if ((float(extLeft[0])/float(meanW[0])) < 0.8):
        idx = 0
        idx = np.where(cnt[:, :, 0] == extLeft[0])[0]
        print(idx)
        if not idx[0]==0:
            print(extLeft[0])
            while idx.size > 0: 
                cnt = np.delete(cnt, idx[0], 0)
                idx = np.delete(idx, 0, 0)
                if not idx.size == 0:
                    idx[0] = idx[0]-1
    if (float(extRight[0])/float(meanW[0]))> 1.2:
        idx = np.where(cnt[:, :, 0] == extRight[0])[0]
        while idx.size > 0: 
                cnt = np.delete(cnt, idx[0], 0)
                idx = np.delete(idx, 0, 0)
                if not idx.size == 0:
                    idx[0] = idx[0]-1
    if ((float(extTop[0])/float(meanh[0])) < 0.8):
        idx = np.where(cnt[:, :, 1] == extTop[0])[0]
        while idx.size > 0: 
                cnt = np.delete(cnt, idx[0], 0)
                idx = np.delete(idx, 0, 0)
                if not idx.size == 0:
                    idx[0] = idx[0]-1
    if (float(extBot[0])/float(meanh[0]))> 1.2:
        idx = np.where(cnt[:, :, 1] == extBot[0])[0]
        while idx.size > 0: 
                cnt = np.delete(cnt, idx[0], 0)
                idx = np.delete(idx, 0, 0)
                if not idx.size == 0:
                    idx[0] = idx[0]-1
    return cnt

def is_contour_bad(c):
	# approximate the contour
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.02 * peri, True)
	# the contour is 'bad' if it is not a rectangl
	return not (len(approx) >= 3 and len(approx) < 10)

def main():
    j = 0
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
    
    traincounter = 0
    test = open(os.path.join(path, "listOfImagesTest.txt"),"a+")
    train = open(os.path.join(path, "listOfImagesTrain.txt"),"a+")
    for i in range(0,len(cv_img)-1):
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
        #Copy the images
        cropped = (cv_img[i].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]
        beforeImg = (cv_img[i].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]# empty.copy()[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]#(cv_img[i-1].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]
        afterImg = (cv_img[i+1].copy())[YCUT:YCUT+HCUT, XCUT:XCUT+WCUT]
        x, y, w, h = findDifference(beforeImg, afterImg)
        
        img_copy =  cv_img[i].copy()
        retImage = img_copy.copy()
        height, width, c = img_copy.shape
        gray = cv2.Canny(cv2.cvtColor(cv2.medianBlur(cropped,9), cv2.COLOR_BGR2GRAY), 127,127*2)
        # create a binary thresholded image
        ret, thresh = cv2.threshold(gray, 127, 255, 0)
        # show it
        #cv2.imshow("Test", thresh)
        # find the contours from the thresholded image
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Write to a txt file
        if not (x == 0 and y == 0 and h == 0 and w ==0) and ((w*h)<maxboxsize): 
            # Write to a txt file
            imgname = folder+ "-" + str(j).zfill(4)
            j = j + 1
            f= open(os.path.join(imgFolder, imgname+".txt"),"w+")
            X = float(x+XCUT)
            Y = float(y+YCUT)
            XC = float(x+XCUT)+float(w/2)
            YC = float(y+YCUT)+float(h/2)
            W = float(w)
            H = float(h)
            center = (int(XC),int(YC))

            cv2.circle(img_copy, center, 5, (255, 0, 0), 2)
            cv2.rectangle(img_copy, (int(X), int(Y)), (int(X) + w, int(Y) + h), (36,255,12), 2) # to see the rectangle

            f.write("0 %.9f %.9f %.9f %.9f\r\n" % (float(XC/width),float(YC/height),float(W/width),float(H/height)))
            cv2.imwrite(os.path.join(imgFolder, imgname+".png"), retImage)
            cv2.imwrite(os.path.join(imgFolder2, imgname+"marked.png"), img_copy)
            cv2.imwrite(os.path.join(imgFolder2, imgname+"diff.png"), diff)
            cv2.imwrite(os.path.join(imgFolder2, imgname+"img1.png"), testing1)
            cv2.imwrite(os.path.join(imgFolder2, imgname+"img2.png"), testing2)
            if(traincounter<=3):
                traincounter = traincounter+1
                train.write(imgFolder + "/" +imgname + ".png\r\n")
            else:
                traincounter = 0
                test.write(imgFolder + "/" + imgname + ".png\r\n")
            f.close() 
        
        
    cv2.destroyAllWindows()

if __name__ == "__main__":
    today = date.today()
    start = time.time()
    folder = raw_input("Folder name\n")
    path = "/home/lab/Pictures/data/"+ folder
    # Create director
    dirName = path #+ "/" + str(today) 
    imgFolder = dirName + "/imgs"
    
    if os.path.exists(imgFolder):
        shutil.rmtree(imgFolder)
        print("File deleted")
    else:
        print("The file does not exist")
    try:
        os.mkdir(imgFolder)
    except:
        print("Directory already exists") 
    
    imgFolder2 = dirName + "/masked" 
    if os.path.exists(imgFolder2):
        shutil.rmtree(imgFolder2)
        print("File deleted")
    else:
        print("The file does not exist")
    try: 
        os.mkdir(imgFolder2)
    except:
        print("Directory already exists") 
    if os.path.exists(path+"/listOfImagesTest.txt"):
        os.remove(path+"/listOfImagesTest.txt")
        print("File deleted")
    else:
        print("The file does not exist")
    if os.path.exists(path+"/listOfImagesTrain.txt"):
        os.remove(path+"/listOfImagesTrain.txt")
        print("File deleted")
    else:
        print("The file does not exist")
    main()
    end = time.time()
    currTime=(end - start)
   #print("Finished - Time: ", currTime)