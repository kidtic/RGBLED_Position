import cv2
import numpy as np
from sklearn.metrics import silhouette_score
from sklearn.cluster import DBSCAN 
import time

#差分



cap=cv2.VideoCapture('res/test_rgb3.mp4')
lower_blue = np.array([100-5, 60, 90])
upper_blue = np.array([125+5, 255,255])
lower_green = np.array([35-5, 60,90])
upper_green = np.array([77+5, 255,255])
lower_red0 = np.array([156, 60,90])
upper_red0 = np.array([180, 255,255])
lower_red1 = np.array([0, 60,90])
upper_red1 = np.array([10, 255,255])



ret,frame=cap.read()
last_frame=frame.copy()
last_frame_c=frame.copy()
rg_centers=np.array([[0,0],[0,0]])#检测到r变到g的信号位置

cv2.namedWindow("diff_mask_rg",0)
#cv2.namedWindow("mask_g",0)
#cv2.namedWindow("last_mask_r",0)
cv2.namedWindow("src",0)

while(ret):
    ret,frame=cap.read()

    starttime=time.time()
    img_hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    img_v=img_hsv[...,2]
    mask_g = cv2.inRange(img_hsv, lower_green, upper_green) #绿色掩模
    #last
    last_img_hsv=cv2.cvtColor(last_frame,cv2.COLOR_BGR2HSV)
    last_img_v=last_img_hsv[...,2]
    last_mask_r0 = cv2.inRange(last_img_hsv,lower_red0,upper_red0)#红色掩膜
    last_mask_r1 = cv2.inRange(last_img_hsv,lower_red1,upper_red1)#红色掩膜
    last_mask_r = cv2.bitwise_or(last_mask_r0,last_mask_r1)

    #diff
    diff_mask_rg=cv2.bitwise_and(mask_g,last_mask_r)
    #图像膨胀
    #retval=cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))
    #diff_mask_rg=cv2.dilate(diff_mask_rg,retval)

    
    #提取所有非0点，准备聚类
    
    listpoint_c=np.nonzero(diff_mask_rg)
    if len(listpoint_c[0])>1:
        

        listpoint=np.vstack(listpoint_c)
        listpoint=np.transpose(listpoint) 
        listpoint=np.float32(listpoint)
        #print("listpoint:",listpoint) 
        #print("type:",type(listpoint)) 
        # 使用DBSCAN进行聚类分析，设置终止条件为执行10次迭代或者精确度epsilon=1.0
        cluster_model = DBSCAN(min_samples=1,eps=10 ).fit(listpoint)
        
        #计算中心点
        rg_centers=np.zeros((max(cluster_model.labels_)+1,2))
        ct=np.zeros((max(cluster_model.labels_)+1,1))
        for i in range(0,len(listpoint)):
            clss=cluster_model.labels_[i]
            rg_centers[clss]=rg_centers[clss]+listpoint[i]
            ct[clss]=ct[clss]+1
        for i in range(0,len(ct)):
            rg_centers[i]=rg_centers[i]/ct[i]
        #print("中心点计算耗时：",time.time()-starttime)
        #print("rg_centers",rg_centers)
       
      
    


    print("总花费时间：",time.time()-starttime)
    
    cv2.imshow("diff_mask_rg",diff_mask_rg)
    #cv2.imshow("mask_g",mask_g)                      
    #cv2.imshow("last_mask_r",last_mask_r)

    #在原图上标记
    #print("len(rg_centers)",len(rg_centers))
    for i in range(0,len(rg_centers)):
        cv2.circle(frame,(int(rg_centers[i][1]),int(rg_centers[i][0])),15,(255,255,0),3)
    cv2.imshow("src",frame)
    if cv2.waitKey(1) & 0xff ==ord('q'):
        break
    last_frame=last_frame_c
    last_frame_c=frame.copy()
    
cap.release()

