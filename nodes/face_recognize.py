#!/usr/bin/env python
import cv2
import serial #导入模块
import threading
STRGLO="" #读取的数据
BOOL=True  #读取标志位

#读数代码本体实现
def ReadData(ser):
    global STRGLO,BOOL
    # 循环接收数据，此为死循环，可用线程实现
        if ser.in_waiting:
            STRGLO = ser.read(ser.in_waiting).decode("ascii")
            print(STRGLO)


#打开串口
# 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
# 波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
# 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
def DOpenPort(portx,bps,timeout):
    ret=False
    try:
        # 打开串口，并得到串口对象
        ser = serial.Serial(portx, bps, timeout=timeout)
        #判断是否打开成功
        if(ser.is_open):
           ret=True
           threading.Thread(target=ReadData, args=(ser,)).start()
    except Exception as e:
        print("---异常---：", e)
    return ser,ret



#关闭串口
def DColsePort(ser):
    global BOOL
    BOOL=False
    ser.close()



#写数据
def DWritePort(ser,text):
    result = ser.write(text.encode("utf8"))  # 写数据
    return result




#读数据
def DReadPort():
    global STRGLO
    str=STRGLO
    STRGLO=""#清空当次读取
    return str



if __name__=="__main__":
    ser,ret=DOpenPort("/dev/ttyACM0",115200,None)
    if(ret==True):#判断串口是否成功打开
         count=DWritePort(ser,"i'm ll")
         print("写入字节数：",count)
         
         DReadPort() #读串口数据
         #DColsePort(ser)  #关闭串口

# create a new cam object
cap = cv2.VideoCapture(0)
# initialize the face recognizer (default face haar cascade)
face_cascade = cv2.CascadeClassifier("/usr/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml")
while True:
    # read the image from the cam
    _, image = cap.read()
    # converting to grayscale
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # detect all the faces in the image
    faces = face_cascade.detectMultiScale(image_gray, 1.3, 5)
    # for every face, draw a blue rectangle
    img_width=size(image_gray)
    for x, y, width, height in faces:
        cv2.rectangle(image, (x, y), (x + width, y + height), color=(255, 0, 0), thickness=2)
    cv2.imshow("image", image)
    if cv2.waitKey(1) == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()