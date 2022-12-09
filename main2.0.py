import time
import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import serial

'''定义了一个目标检测函数'''
def ShapeDetection():
    Area = []
    XYWH = []
    flag = []

    white_low = 160
    while_high = 256

    i = 0

    # 摄像头实时加载图像，图像初始化
    cap = cv.VideoCapture(0)
    frameWidth = 640
    frameHeight = 480
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)

    while i < 10:
        print("开始识别")
        success, imgBegin = cap.read()
        imgGray = cv.cvtColor(imgBegin, cv.COLOR_BGR2GRAY)
        imgRange = cv.inRange(imgGray, white_low, while_high)
        imgErode = cv.erode(imgRange, np.ones((10, 10), np.uint8))
        imgDilate = cv.dilate(imgErode, np.ones((10, 10), np.uint8))
        imgDilate2 = cv.dilate(imgDilate, np.ones((5, 5), np.uint8))
        imgColor = cv.cvtColor(imgDilate2, cv.COLOR_GRAY2BGR)

        contours, hierarchy = cv.findContours(imgDilate2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)  # 寻找轮廓点
        for obj in contours:
            area = cv.contourArea(obj)
            cv.drawContours(imgColor, obj, -1, (255, 0, 0), 4)  # 绘制轮廓线
            perimeter = cv.arcLength(obj, True)  # 计算轮廓周长
            approx = cv.approxPolyDP(obj, 0.02 * perimeter, True)  # 获取轮廓角点坐标
            CornerNum = len(approx)  # 轮廓角点的数量
            x, y, w, h = cv.boundingRect(approx)  # 获取坐标值和宽度、高度
            if CornerNum == 3:
                objType = "triangle"
            elif CornerNum == 4:
                if w == h:
                    objType = "Square"
                else:
                    objType = "MilkBox"
            elif CornerNum > 4:
                objType = "MilkBox"
            else:
                objType = "N"
            if w * h > 10000:
                Area.append(area)
                XYWH.append((x, y, w, h))
                cv.rectangle(imgColor, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 绘制边界框
                cv.putText(imgColor, objType, (x + (w // 2), y + (h // 2)), cv.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1)  # 绘制文字
                # cv.imshow("res", imgColor)
                if w > h:
                    flag.append('h')
                elif w < h:
                    flag.append('s')
                else:
                    flag.append('n')
        time.sleep(0.05)
        i = i + 1
    print("识别结束")
    H = 0
    S = 0
    N = 0
    print("开始判断")
    for j in range(len(flag)):
        if flag[j] == 'h':
            H = H + 1
        elif flag[j] == 's':
            S = S + 1
        else:
            N = N + 1

    if H > S and H > N:
        FLAG = 'h'
    elif S > H and S > N:
        FLAG = 's'
    else:
        FLAG = 'n'
    print("判断结束")
    return FLAG

'''
定义了一个超声波测距函数
'''
def measure():
    # 设置警告信息为不输出
    GPIO.setwarnings(False)
    # 使用BCM针脚编号方式
    GPIO.setmode(GPIO.BCM)
    # 控制引脚GPIO22
    trig = 22
    # 接收引脚GPIO17
    echo = 17
    # 设置trig引脚为输出模式，初始化输出为低电平
    GPIO.setup(trig, GPIO.OUT, initial=GPIO.LOW)
    # 设置echo引脚为输入模式
    GPIO.setup(echo, GPIO.IN)
    HIGH = 1
    LOW = 0

    ms = []
    i = 0
    while i < 10:
        # 树莓派向trig引脚发送信号，一个持续10us的方波脉冲
        GPIO.output(trig, HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, LOW)

        # HC - SR04接收到脉冲信号，开始发送超声波并将Echo引脚置为高电平
        # echo引脚之前一直接收低电平信号，一旦收到高电平信号就开始记录时间
        while GPIO.input(echo) == LOW:
            pass
        start = time.time()
        # 当 HC-SR04 接收到返回的超声波 时，把Echo引脚置为低电平
        # 也就是说echo引脚接收到的高电平结束，终止计时
        while GPIO.input(echo) == HIGH:
            pass
        end = time.time()

        # 计算距离，单位厘米，这里的340m/s是超声波在空气中的传播速度
        distance = round((end - start) * 340 / 2 * 100, 2)
        # print("distance:{0}cm".format(distance))

        ms.append(distance)
        i = i + 1
        time.sleep(0.02)

    ms.remove(max(ms))
    ms.remove(min(ms))
    LEN = len(ms)
    SUM = sum(ms)
    average = SUM/LEN
    if average < 32:
        flag = '2'
    elif 32 < average < 50:
        flag = '1'
    elif 50 < average:
        flag = '0'
    else:
        flag = '3'

    return flag

'''
接收到指定串口信号时
'''

ser = serial.Serial("/dev/ttyAMA0", 9600)
while True:
    print("已经开始运行")
    Read = ser.read()
    if Read.decode('ISO-8859-1', 'strict') == 'g':
        print("进行判断")
        nx = ShapeDetection()
        cs = measure()

        if nx == 'h' and cs == '1':
            ser.write("1".encode('utf-8'))
            print('1')
        elif nx == 's' and cs == '1':
            ser.write("2".encode('utf-8'))
            print('2')
        elif nx == 'h' and cs == '2':
            ser.write("3".encode('utf-8'))
            print('3')
        elif nx == 's' and cs == '2':
            ser.write("4".encode('utf-8'))
            print('4')
        else:
            ser.write("0".encode('utf-8'))
            print('0')
        print("全部结束")