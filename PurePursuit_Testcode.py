import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import *
import math

# Pure Pursuit 알고리즘 구현
class PurePursuit:
    def __init__(self, path, look_ahead):
        self.path = path
        self.look_ahead = look_ahead

    def get_steering_angle(self, vehicle_position, vehicle_angle):
        # 차량 위치에서 가장 가까운 경로 포인트를 찾는다.
        dist = [math.hypot(vp[0]-vehicle_position[0], vp[1]-vehicle_position[1]) for vp in self.path]
        nearest_index = dist.index(min(dist))

        # Look ahead distance 내에 있는 경로 포인트를 찾는다.
        look_ahead_point = None
        for i in range(nearest_index, len(self.path)):
            if math.hypot(self.path[i][0]-vehicle_position[0], self.path[i][1]-vehicle_position[1]) > self.look_ahead:
                look_ahead_point = self.path[i]
                break

        # 차량 위치와 Look ahead point 사이의 각도를 계산한다.
        if look_ahead_point is not None:
            angle = math.atan2(look_ahead_point[1]-vehicle_position[1], look_ahead_point[0]-vehicle_position[0])
            steering_angle = angle - vehicle_angle
        else:
            steering_angle = 0

        return steering_angle

# GUI 구현
class App:
    def __init__(self, master):
        # Figure와 Axis 설정
        self.fig, self.ax = plt.subplots()

        # Tkinter Canvas 생성 및 Figure 추가
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)

        # Pure Pursuit 알고리즘 초기화
        #self.pure_pursuit = PurePursuit([(i, 0) for i in range(11)] + [(10, i) for i in range(1, 11)], 5)
        self.pure_pursuit = PurePursuit([(i, 0) for i in range(11)] + [(10, i/2) for i in range(1, 21)], 5)


        # 차량 초기 위치 및 각도 설정
        self.vehicle_position = [0, 0]
        self.vehicle_angle = 0

        # 차량 속도 설정
        self.vehicle_speed = 0.3

        # 애니메이션 설정
        self.animation = self.fig.canvas.new_timer(interval=100)
        self.animation.add_callback(self.callback)
        self.animation.start()

    def callback(self):
        # 차량이 목표 지점에 도달했는지 확인한다.
        if math.hypot(self.vehicle_position[0] - 10, self.vehicle_position[1] - 10) < 0.5:
            self.animation.stop()

        # Pure Pursuit 알고리즘을 이용해 조향 각도를 계산한다.
        steering_angle = self.pure_pursuit.get_steering_angle(self.vehicle_position, self.vehicle_angle)

        # 차량 각도를 업데이트한다.
        self.vehicle_angle += steering_angle

        # 차량 위치를 업데이트한다.
        self.vehicle_position[0] += self.vehicle_speed * math.cos(self.vehicle_angle)
        self.vehicle_position[1] += self.vehicle_speed * math.sin(self.vehicle_angle)

        # 그래프를 업데이트한다.
        self.ax.clear()
        self.ax.plot(*zip(*self.pure_pursuit.path), 'b')
        self.ax.plot(*self.vehicle_position, 'ro')

        self.canvas.draw()

root = Tk()
app = App(root)
root.mainloop()
