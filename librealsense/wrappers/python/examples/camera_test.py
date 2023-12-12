import cv2
import numpy as np

# 카메라 연결
cap = cv2.VideoCapture(0)  # 0은 기본 카메라를 나타냅니다. 다른 카메라를 사용하려면 카메라 인덱스를 변경하세요.

# 체스보드 가로 및 세로 내부 코너 수
board_width = 6
board_height = 9


# 체스보드 내부 코너 좌표를 저장할 리스트
chessboard_corners = []

# 체스보드의 물리적 크기와 코너 사이의 거리 설정
square_size = 0.0253  # 체스보드의 각 사각형의 한 변의 길이 (예: cm)

while True:
    ret, frame = cap.read()  # 비디오 프레임 읽기

    if not ret:
        break

    # 체스보드 코너 검출
    ret, corners = cv2.findChessboardCorners(frame, (board_width, board_height), None)

    if ret:
        # 코너 좌표를 추가
        chessboard_corners.append(corners)

        # 코너 표시
        cv2.drawChessboardCorners(frame, (board_width, board_height), corners, ret)

    # 프레임 표시
    cv2.imshow("Chessboard", frame)

    # 'q' 키를 누르면 루프 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 저장할 이미지 생성
output_image = frame.copy()

# 모든 프레임에서 검출된 코너를 출력 이미지에 추가
for corners in chessboard_corners:
    cv2.drawChessboardCorners(output_image, (board_width, board_height), corners, True)

# 이미지 저장
cv2.imwrite("chessboard_corners.png", output_image)

print("이미지 저장 완료.")
