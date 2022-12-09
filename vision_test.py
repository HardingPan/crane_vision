import time
import cv2 as cv
import numpy as np

# 摄像头实时加载图像，图像初始化
cap = cv.VideoCapture(0)
frameWidth = 640
frameHeight = 480
cap.set(3, frameWidth)
cap.set(4, frameHeight)

# 定义检测函数
def ShapeDetection(imgColor):
    Area = []
    XYWH = []
    flag = []

    img = cv.cvtColor(imgColor, cv.COLOR_BGR2GRAY)
    contours, hierarchy = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)  # 寻找轮廓点
    for obj in contours:
        area = cv.contourArea(obj)
        cv.drawContours(img, obj, -1, (255, 0, 0), 4)  # 绘制轮廓线
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
            if w > h:
                flag.append('h')
            elif w < h:
                flag.append('s')
            else:
                flag.append('n')
    return [Area, XYWH, flag]

HS = []
H = 0
S = 0

while True:
    white_low = 160
    while_high = 256
    success, img = cap.read()
    imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    imgRange = cv.inRange(imgGray, white_low, while_high)
    imgErode = cv.erode(imgRange, np.ones((10, 10), np.uint8))
    imgDilate = cv.dilate(imgErode, np.ones((10, 10), np.uint8))
    imgDilate2 = cv.dilate(imgDilate, np.ones((5, 5), np.uint8))
    imgColor = cv.cvtColor(imgDilate2, cv.COLOR_GRAY2BGR)
    SD_res = ShapeDetection(imgColor)

    Area = SD_res[0]
    XYWH = SD_res[1]
    Flag = SD_res[2]
    HS.append(Flag)
    while len(HS) > 20:
        for i in range(len(HS)):
            if HS[i] == ['h']:
                H = H + 1
            elif HS[i] == ['s']:
                S = S + 1
        if H > 15:
            flag = 'h'
        elif S > 15:
            flag = 's'
        else:
            flag = 'n'
        print(flag)
        HS = []
        H = 0
        S = 0

    time.sleep(0.05)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
