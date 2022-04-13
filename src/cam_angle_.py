
import numpy as np
import cv2,itertools
import matplotlib.pyplot as plt
import math,os
from numpy import dot
from numpy.linalg import norm
import time
import serial

os.chdir('F:\\캡스톤\\corner1')
file_dir = 'F:\\캡스톤\\corner1'
file_names = os.listdir(file_dir)

# original_img = cv2.imread(file_names[300])``
original_img = cv2.imread('j2355.jpg')
def cos_sim(A, B):   #유사도 검사하는 함수
       return dot(A, B)/(norm(A)*norm(B))

def filter_red(img):   #filter_red
    #범위 내의 픽셀들은 흰색, 나머지는 검은색
    lower_mask = cv2.inRange(img, lower1, upper1)
    upper_mask = cv2.inRange(img, lower2, upper2)
    
    full_mask = lower_mask+upper_mask;
    # plt.subplot(339),plt.imshow(full_mask),plt.title('RED')
    hsv_img = cv2.bitwise_and(original_img, original_img, mask = full_mask)
    
    img2 = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
    
    return img2


lower1 = np.array([0, 70, 20])
upper1 = np.array([15, 255, 255])
     
# upper boundary RED color range values; Hue (160 - 180)
lower2 = np.array([155,40,20])
upper2 = np.array([180,255,255])

H = 90        #      <<<<<<<------ 맨 밑바닥 까지 보니까 아크릴판 살짝 나온거때매 이상함! 그래서 아크릴판 나오는거 자름 !
W = 320

pts1 = np.float32([[10,0],[10,H],[W-10,0],[W-10,H]])
pts2 = np.float32([[10,0],[45,H],[W-10,0],[279-10,H]])
M = cv2.getPerspectiveTransform(pts1,pts2)
bin_ = [12,25,37,50,62,75,87,100,112,125,137,150,162,175,187,200,212,225]
s=1

blue_lower = np.array([100,80,80])
blue_upper = np.array([140,255,255])

# cap1 = cv2.VideoCapture(gstreamer_pipeline(),cv2.CAP_GSTREAMER)
count = 0


original_img = cv2.imread('j2355.jpg')
original_img = cv2.resize(original_img, dsize=(320, 240), interpolation=cv2.INTER_AREA)
hsv = cv2.cvtColor(original_img, cv2.COLOR_BGR2HSV)
blue_img  = cv2.inRange(hsv,blue_lower,blue_upper)
slide_blue = blue_img[:,:]
blue_average = slide_blue.sum()/600
red_img = filter_red(hsv)
roi_area = red_img[140:140+H,10:W-10]
_,thresh = cv2.threshold(roi_area,50,255,cv2.THRESH_BINARY)
warped_img = cv2.warpPerspective(thresh, M, (W,H))           # threshold 시킨 이미지를 warp시킴.
blur = cv2.GaussianBlur(warped_img[:,35:260],(3,3),5)
blur2 = cv2.cvtColor(blur,cv2.COLOR_GRAY2RGB)
edge_img2 = cv2.Canny(blur,150,200)          #blur한게 더 깔끔함 ! 
contours, hierachy = cv2.findContours(edge_img2, cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
h,w = edge_img2.shape
img_point = np.zeros((h,w,3))

lane_degree = {}
our_state =[]
real_contours = []

if len(contours) != 0:	    
 	    for i in contours:
             if len(i) > 50:
                 real_contours.append(i)
        if len(real_contours) == 0:
            print('Here is no lane!')
        else:
		    for count,i in enumerate(real_contours):
 			    x= []
 			    y= []
 			    x_idx=[]
 			    x_idx2=[]
				#contour 의 좌표 값 얻은 후 pixel좌표를 uclidian좌표로 바꿈 그래서 최소 최대 점 빼오고 기울기 구해    
 			    for m in i:
 			        x.append(m[0][0])
 			        y.append(m[0][1])
				    
 			    for i in range(len(x)):
 			        if x[i] == max(x):
 			            x_idx.append(i)
 			        if x[i] == min(x):
 			            x_idx2.append(i)
 			    x = np.array(x)
 			    y = -np.array(y)
 			    y_max = []
 			    y_min = []
 			    for k in x_idx:
 			        y_max.append(y[k])
 			    for k in x_idx2:
 			        y_min.append(y[k])   

 			    pointx_2 = max(x)
 			    pointy_2 = max(y_max)
 			    pointx_1 = min(x)
 			    pointy_1 = min(y_min)

 			    lane_degree[count] = [pointx_1,pointx_2,pointy_1,pointy_2]

		    total_line = []
		    combination = list(itertools.combinations(lane_degree,2))
 			# 유사도가 0.6 이상인 라인은 중복된 것이라 판단하고 하나만 받아옴.
		    filtered_lane = {}
		    #print(lane_degree)
		    if combination == []:
 		    	filtered_lane[0] = lane_degree[0]
		    else:
 		    	for i in combination:
		    		rate = cos_sim(lane_degree[i[0]],lane_degree[i[1]])
		    		if rate > 0.6:
 		    			filtered_lane[i[0]] = lane_degree[i[0]]
 			
		    #print(lane_degree)
		    degree=[]
 			
		    for _,i in enumerate(filtered_lane):
 		    	X = [filtered_lane[i][0],filtered_lane[i][1]]
 		    	Y = [filtered_lane[i][2],filtered_lane[i][3]]
 		    	radian = np.arctan2(filtered_lane[i][3]-filtered_lane[i][2],filtered_lane[i][1]-filtered_lane[i][0])
 		    	if abs(math.degrees(radian)) > 35:
		    		degree.append(math.degrees(radian))
	    			cv2.line(blur2,(X[0],abs(Y[0])),(X[1],abs(Y[1])),(0,255,0),5,lineType= cv2.LINE_AA)

		    if degree:
 		    	a= list(np.abs(degree))
 		    	idx = a.index(max(a))
 		    	max_degree = degree[idx]
 		    	print('degree of line is : ' ,max_degree)

		    if degree == []:
 		    	adjust = '1'  #adjust
		    elif max_degree > 0 and max_degree<82:
 		    	adjust = '6'  #adjust right
		    elif max_degree<0 and max_degree>-82:
 		    	adjust = '5'   #adjust left
		    else: 
 		    	adjust = '1'
 			
		    #print('state :' ,adjust)
 			
		    fast = cv2.FastFeatureDetector_create(threshold=28,type = cv2.FastFeatureDetector_TYPE_7_12)
		    kp = fast.detect(blur,None)
 			
		    if kp:
 		    	for _,i in enumerate(kp):
				    if (i.pt[0] <20 or i.pt[0]>210) or i.pt[1]<60:
 					    kp.pop(_)
				
 				       
		    blur2 = cv2.drawKeypoints(blur2, kp, None, color=(255,0,0))
 			
		    roi2 = np.uint8(blur[48:,:]/255)
		    hist = []
		    for i in bin_:
 		    	f = roi2[:,i-12:i]
 		    	hist.append(f.sum()/60)
		    #print(hist)
		    hist2 = hist.copy()
 			
		    hist_ = []
		    for i in hist:
 		    	if i > 1.8:
 		    	    hist_.append(hist.index(i))
		    if len(hist_)==2:
 		    	X= np.array(hist[min(hist_):max(hist_)]) > 0.4
 		    	if False not in X:
 		    	    our_state.append('0')   #finish

		    for i in kp:
 		    	local = math.ceil(i.pt[0]/12)
 		    	A = hist[:local]
 		    	C = [False for i in A if i < 0.4]
				
 		    	A = hist[local-1:]
 		    	D = [False for i in A if i<0.4]
 		    	if len(C) == 0:
 		    	    our_state.append('2')# left
 		    	if len(D) == 0:
 		    	    our_state.append('3') # right

		    #dst = cv2.addWeighted(img_point/255.,0.3,np.float64(blur2)/255.,0.7,0)
		    #print(our_state)
else:
 	    adjust = '7' #stop
 	    #print(adjust)
#print(len(real_contours))
cv2.imshow('Capture_gray',blur2)
#cv2.imshow('Capture_gray',original_img)
#cv2.imshow('Capture_gray',slide_blue)
if blue_average > 300:
    if max_degree > 0 and max_degree<80:
 	        ser.write('6'.encode())  #adjust right
    elif max_degree<0 and max_degree>-80:
 	        ser.write('5'.encode())   #adjust left
    else: 
            ser.write('4'.encode())
            print('find_blue_one')
elif len(our_state)!=0:
    ser.write(our_state[0].encode())
    print('jetson to rio (turn L or R or F):' ,our_state)
else:
    ser.write(adjust.encode())
    print('jetson to rio (go or adjust or stop) :' ,adjust)
#cv2.imwrite('/home/juhan/code_test/IMAGE_FOLDER/j{}.jpg'.format(count),img)
#count += 1
if cv2.waitKey(1) ==27:
    s=0

cap1.release()
cv2.destroyAllWindows()


