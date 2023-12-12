import math

def distance_cal(x1,y1,x2,y2):

	distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	return distance

while True:
    x1 = float(input("첫 번째 점의 x 좌표를 입력하세요: "))
    y1 = float(input("첫 번째 점의 y 좌표를 입력하세요: "))
    x2 = float(input("두 번째 점의 x 좌표를 입력하세요: "))
    y2 = float(input("두 번째 점의 y 좌표를 입력하세요: "))

    distance = distance_cal(x1, y1, x2, y2)
    print("두 점 사이의 거리는:", distance)
