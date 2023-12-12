import math

# 두 점 A와 B의 좌표
x1, y1 = 1.0, 2.0  # 점 A의 좌표
x2, y2 = 3.0, 4.0  # 점 B의 좌표

# 두 점을 지나는 직선의 방정식 계산
# 기울기 (m) 계산
if x1 != x2:
    m = (y2 - y1) / (x2 - x1)
else:
    m = math.inf  # 수직선의 경우 기울기가 무한대

# y 절편 (b) 계산
b = y1 - m * x1

# 다른 점 C의 좌표
x3, y3 = 2.0, 3.0  # 점 C의 좌표

# 직선과 다른 점 C와의 거리 계산
distance = abs(m * x3 - y3 + b) / math.sqrt(m**2 + 1)

print("직선과 점 C의 거리:", distance)
