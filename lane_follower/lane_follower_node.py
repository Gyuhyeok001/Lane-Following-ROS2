import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

# ─── 헬퍼: 점선 그리기 ──────────────────────────────────────────────
def draw_dashed_line(img, pt1, pt2, color, thickness=2, dash_length=10, gap_length=10):
    x1, y1 = pt1; x2, y2 = pt2
    dist = int(math.hypot(x2 - x1, y2 - y1))
    if dist == 0:
        return
    dx = (x2 - x1) / dist
    dy = (y2 - y1) / dist
    i = 0
    while i < dist:
        sx = int(x1 + dx * i)
        sy = int(y1 + dy * i)
        ei = min(i + dash_length, dist)
        ex = int(x1 + dx * ei)
        ey = int(y1 + dy * ei)
        cv2.line(img, (sx, sy), (ex, ey), color, thickness)
        i += dash_length + gap_length

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 비디오 파일 열기
        pkg_share = get_package_share_directory('lane_follower')
        video_path = os.path.join(pkg_share, 'test', 'lanevideo.mp4')
        self.get_logger().info(f'Opening video: {video_path}')
        self.cap = cv2.VideoCapture(video_path)

        # ── VideoWriter 설정 ─────────────────────────────────────────
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps    = self.cap.get(cv2.CAP_PROP_FPS) or 30
        w      = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h      = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.out = cv2.VideoWriter(
            'lane_following_output.mp4', fourcc, fps, (w, h)
        )
        self.get_logger().info(
            f'Recording to lane_following_output.mp4 @ {fps:.1f}FPS, {w}x{h}'
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # 녹화 파일 닫기
            self.out.release()
            return

        # 전처리
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50,150)
        frame_h, frame_w = frame.shape[:2]

        # ROI를 아래 2/3으로 늘림
        roi_top = frame_h // 3
        roi = edges[roi_top:, :]

        # 허프 직선 검출
        lines = cv2.HoughLinesP(
            roi, 1, np.pi/180,
            threshold=100, minLineLength=150, maxLineGap=100
        )

        if lines is not None:
            for x1, y1_, x2, y2_ in lines.reshape(-1,4):
                y1o = y1_ + roi_top
                y2o = y2_ + roi_top

                # 기울기 계산
                s = (y2o - y1o) / float(x2 - x1 + 1e-6)
                # 수평 또는 너무 가파른 선 제거
                if abs(s) < 0.5 or abs(s) > 5:
                    continue

                # 단편 선분을 점선으로 그리기
                draw_dashed_line(
                    frame,
                    (x1, y1o),
                    (x2, y2o),
                    color=(0,255,0),
                    thickness=2,
                    dash_length=15,
                    gap_length=10
                )

        # ── 녹화: 프레임 저장 ─────────────────────────────────────────
        self.out.write(frame)

        # 화면에 표시
        cv2.imshow('Lane', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # 노드 종료 시 VideoWriter 해제
        if hasattr(self, 'out') and self.out.isOpened():
            self.out.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
