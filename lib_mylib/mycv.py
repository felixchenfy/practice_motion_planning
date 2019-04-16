
'''
These are just some common functions
    for displaying and saving/writing images.
I put them here in case I forget their function's names.

'''
import numpy as np
import matplotlib.pyplot as plt
import math
from math import floor, ceil
import cv2
import sys


class mycv(object):
    def __init__(self):
        return
    @staticmethod
    def imread(filename):
        img=cv2.imread(filename)
        if img is None:
            print("Fail to read in image:",filename)
            sys.exit(0)
        return img

    @staticmethod
    def imwrite(filename, img):
        cv2.imwrite(filename,img)

    @staticmethod    
    def savefig(filename, border_size=10):
        plt.savefig(filename)
        img=mycv.imread(filename)
        img_out = mycv.cropimage(img, border_size)
        mycv.imwrite(filename, img_out)

    @staticmethod
    def imshow(img):
        Xmax =img.shape[0]
        Ymax =img.shape[1]
        if len(img.shape)==3:
            plt.imshow(img, cmap=plt.cm.binary, interpolation='nearest', origin='lower',
                extent=[0,Xmax,0,Ymax])
        else:
            plt.imshow(np.invert(img), cmap=plt.cm.binary, interpolation='nearest', origin='lower',
                extent=[0,Xmax,0,Ymax])
        # plt.imshow(img, origin='lower',
        #           extent=[0,Xmax,0,Ymax])
    
    @staticmethod
    def imshow2(img): # convert opencv image to plt image
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    @staticmethod
    def rgb2gray(img):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        return img

    @staticmethod
    def gray2bw(img):
        (thresh, img) = cv2.threshold(img, 128, 255, \
            cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        return img

    @staticmethod
    def gray2rgb(img):
        # return cv2.merge((img,img,img))
        return cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)

    @staticmethod
    def rgb2bw(img):
        img=mycv.rgb2gray(img)
        img=mycv.gray2bw(img)
        return img

    @staticmethod
    def AddCircle(img, num=8,radius=5, std=3, rand_range=None):
        if rand_range is None:
            circles = generate_circles(num, radius, std,
                                       x_y_range=[img.shape[1], img.shape[0]])
        else:
            rr=rand_range # [x0, x1, y0, y1]
            circles = generate_circles(num, radius, std,
                                       x_y_range=[rr[1]-rr[0],rr[3]-rr[2]])
            circles[:,1]+=rr[0]
            circles[:,2]+=rr[2]
            
        # print circles onto img
        rows=img.shape[0]
        cols=img.shape[1]
        for k in range(num):
            r=circles[k][0]+1
            x=circles[k][1]
            y=circles[k][2]
            row0=int(max(0, floor(y-r)-1))
            row1=int(min(rows, ceil(y+r)+1))
            col0=int(max(0, floor(x-r))-1)
            col1=int(min(cols, ceil(x+r)+1))
            for i in range(row0,row1): # row
                for j in range(col0,col1): # col
                    if (i-y)**2+(j-x)**2<r**2:
                        img[i,j]=0
        return img

    @staticmethod        
    def cropimage(img, border_size=10):
        gray=mycv.rgb2gray(img)
        [r,c]=gray.shape
        ref=float(gray[1,1])
        for i in range(c):
            left=i
            same_color=abs(gray[:,left]-ref)<10
            if sum(same_color)!=len(same_color):
                break
        for i in range(c):
            right=c-i-1
            same_color=abs(gray[:,right]-ref)<10
            if sum(same_color)!=len(same_color):
                break
        for i in range(r):
            up=i
            same_color=abs(gray[up,:]-ref)<10
            if sum(same_color)!=len(same_color):
                break
        for i in range(r):
            down=r-i-1
            same_color=abs(gray[down,:]-ref)<10
            if sum(same_color)!=len(same_color):
                break
        left=max(0, left-border_size)
        right=min(c-1, right+border_size)
        up=max(0,up-border_size)
        down=min(r-1, down+border_size)

        box=np.array([ up, down, left, right,])
        Iout=img[box[0]:box[1]+1, box[2]:box[3]+1,:]
        
        # print(box)
        # plt.figure(2)
        # mycv.imshow(img)
        # plt.figure(3)
        # mycv.imshow(Iout)
        # plt.show()
        
        return Iout

def generate_circles(num, radius, std, x_y_range):
    circles = np.zeros((num,3))
    circles[:,1:2] = np.random.uniform(radius, x_y_range[0]-radius, size=(num,1)) # x
    circles[:,2:3] = np.random.uniform(radius, x_y_range[1]-radius, size=(num,1)) # y
    circles[:,0] = np.random.normal(radius, std, size=(num,))
    return circles

if __name__=='__main__':
    I = mycv.imread('map.png')
    img = mycv.rgb2bw(I)
    img=mycv.AddCircle(img)
    mycv.imshow(img)
    plt.show()
    # print(img[1:10][2:10])