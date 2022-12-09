import numpy as np
import random
import time
from numpy import sqrt, sin, cos
from numpy.linalg import norm
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSlider, QPushButton,
                               QSizePolicy, QLabel)
from PyQt5.QtCore import QSize
from PyQt5.QtCore import Qt
import pyqtgraph.opengl as gl
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
import pyqtgraph as pg
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.widgets import Slider

from robot_arm import RobotArm

# from .transforms import *
# from .kinematics import SerialArm

from time import perf_counter, sleep

red = np.array([0.7, 0, 0, 1])
green = np.array([0, 0.7, 0, 1])
blue = np.array([0, 0, 0.7, 1])
dark_red = np.array([0.3, 0, 0, 1])
dark_green = np.array([0, 0.3, 0, 1])
dark_blue = np.array([0, 0, 0.3, 1])
white = np.array([1, 1, 1, 1])
grey = np.array([0.3, 0.3, 0.3, 1])

class TransformMPL:

    def __init__(self, A, ax=None):
        self.A = A

    def update(self, A):
        self.A = A

    def show(self):
        plt.show()


class PlanarMPL:

    def __init__(self, arm, q0=None, trace=False, ax=None):
        self.arm = arm
        if ax is not None:
            fig = ax.figure
            flag = True
        else:
            fig, ax = plt.subplots()
            flag = False
        self.fig = fig
        self.ax = ax
        self.n = arm.n
        self.reach = arm.reach
        if not flag:
            plt.axis('equal')
            plt.xlim([-arm.reach * 1.5, arm.reach * 1.5])
            plt.ylim([-arm.reach * 1.5, arm.reach * 1.5])

        self.joints = []
        self.links = []
        self.joints.append(self.ax.add_patch(Circle([0, 0], 0.15, color=[0,0,0,1])))

        if q0 is None:
            q0 = [0] * self.n
        self.q0 = q0
        for i in range(self.n):
            A = self.arm.fk(q0, index=i)
            A_next = self.arm.fk(q0, index=i+1)

            if i != 0:
                joint = Circle(A[0:2, 3], 0.1, color=[0,0,0,1])
                self.joints.append(self.ax.add_patch(joint))
            link, = self.ax.plot([A[0,3], A_next[0,3]], [A[1,3], A_next[1,3]], lw=3, color=[0,0,0,1])
            self.links.append(link)

        end_effector = Circle(A_next[0:2,3], 0.1, color=[0.5,0,0,1])
        self.joints.append(self.ax.add_patch(end_effector))

        self.do_trace = trace
        if trace:
            self.start_trace(q0)

    def start_trace(self, q0):
        self.xs = []
        self.ys = []
        self.traces = []
        for i in range(self.n):
            pos = self.arm.fk(q0, index=i + 1)[0:2, 3]
            C = [0, 0, 0, 0.4]
            C[i] = 0.5
            line, = self.ax.plot(pos[0], pos[1], lw=1, color=C)
            self.xs.append([pos[0]])
            self.ys.append([pos[1]])
            self.traces.append(line)

    def update(self, q):

        for i in range(self.n):
            A = self.arm.fk(q, index=i)
            A_next = self.arm.fk(q, index=i+1)

            if i != 0:
                self.joints[i].set_center(A[0:2, 3])

            self.links[i].set_xdata([A[0,3], A_next[0,3]])
            self.links[i].set_ydata([A[1,3], A_next[1,3]])
        self.joints[-1].set_center(A_next[0:2, 3])

        if self.do_trace:
            for i in range(self.n):
                pos = self.arm.fk(q, index=i+1)[0:2,3]
                self.xs[i].append(pos[0])
                self.ys[i].append(pos[1])
                self.traces[i].set_xdata(self.xs[i])
                self.traces[i].set_ydata(self.ys[i])

        plt.pause(0.02)

    def show(self):
        plt.show()

    def set_bounds(self, xbound=None, ybound=None):
        print('finish me')

    def play(self):
        # clear all the line plots
        if self.do_trace:
            self.do_trace = False
            for i in range(self.n):
                self.ax.lines.remove(self.traces[i])

        # move to the default position
        self.update(self.q0)

        # resize and create slider bars
        self.ax.set_position([0, 0, 0.75, 1])
        plt.axis('equal')
        plt.xlim([-self.arm.reach * 1.5, self.arm.reach * 1.5])
        plt.ylim([-self.arm.reach * 1.5, self.arm.reach * 1.5])

        max_h = 0.2
        min_h = 0.8 / self.n
        h = min(max_h, min_h)
        self.sliders = []
        self.gui_axes = []

        def get_text_from_A(A):
            pos = np.around(A[0:2, 3], decimals=2)
            theta = np.around(np.arctan2(A[1, 0], A[0, 0]), decimals=2)

            s = 'Pos: [' + str(pos[0]) + ', ' + str(pos[1]) + ']\n'
            s += 'Angle: [' + str(theta) + ']\n'

            return s
        text_pos = [-self.reach * 1.35, self.reach * 1.15]
        self.text_box = self.ax.text(text_pos[0], text_pos[1], get_text_from_A(self.arm.fk(self.q0)))

        def slider_update(val):
            q = self.q0
            for i in range(self.n):
                q[i] = self.sliders[i].val
            self.update(q)
            self.text_box.set_text(get_text_from_A(self.arm.fk(q)))
            plt.draw()

        for i in range(self.n):
            self.gui_axes.append(self.fig.add_subplot())
            self.gui_axes[i].set_position([0.775, 0.8 - h * i, 0.15, h - 0.05])
            self.sliders.append(Slider(ax=self.gui_axes[i],
                                        label=str(i),
                                        valmin=-np.pi,
                                        valmax=np.pi,
                                        valinit=0,
                                        orientation='horizontal'))
            self.sliders[i].on_changed(slider_update)


class VizScene:
    """The viz scene holds all the 3d objects to be plotted. This includes arms (which are GLMeshObjects), transforms
    (which are plots or quiver type things), scatter points, and lines."""
    def __init__(self):
        # self.arm = arm
        # self.arm_object = ArmMeshObject(arm, draw_frames=draw_frames)
        self.arms = []
        self.frames = []
        self.markers = []
        self.range = 5

        if QApplication.instance() is None:
            self.app = pg.QtGui.QApplication([])
        else:
            self.app = QApplication.instance()
        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Robot Visualization 2: The Sequel')
        self.window.setGeometry(200, 100, 1200, 900)
        self.grid = gl.GLGridItem()
        self.grid.scale(1, 1, 1)
        self.window.addItem(self.grid)
        self.window.setCameraPosition(distance=self.range)
        self.window.setBackgroundColor('k')
        self.window.show()
        self.window.raise_()
        self.window.opts['center'] = pg.Vector(0, 0, 0)

        self.app.processEvents()

    def add_arm(self, arm, draw_frames=False, joint_colors=None):
        self.arms.append(ArmMeshObject(arm, draw_frames=draw_frames, joint_colors=joint_colors))
        self.arms[-1].update()
        self.window.addItem(self.arms[-1].mesh_object)

        if 2 * arm.reach > self.range:
            self.range = 2 * arm.reach
            self.window.setCameraPosition(distance=self.range)

        self.app.processEvents()

    def remove_arm(self, arm=None):
        if arm is None:
            for arm in self.arms:
                self.window.removeItem(arm.mesh_object)
            self.arms = []
        elif isinstance(arm, (int)):
            self.window.removeItem(self.arms[arm].mesh_object)
            self.arms.pop(arm)
        else:
            print("Warning: invalid index entered!")
            return None
        self.app.processEvents()

    def add_frame(self, A, label=None):
        self.frames.append(FrameViz(label=label))
        self.frames[-1].update(A)
        self.window.addItem(self.frames[-1].mesh_object)
        if label is not None:
            self.window.addItem(self.frames[-1].label)
            self.frames[-1].label.setGLViewWidget(self.window)

        if 2 * norm(A[0:3, 3]) > self.range:
            self.range = 2 * norm(A[0:3, 3])
            self.window.setCameraPosition(distance=self.range)

        self.app.processEvents()

    def remove_frame(self, ind=None):
        if ind is None:
            for frame in self.frames:
                self.window.removeItem(frame.mesh_object)
            self.frames = []
        elif isinstance(ind, (int)):
            self.window.removeItem(self.frames[ind].mesh_object)
            self.frames.pop(ind)
        else:
            print("Warning: invalid index entered!")
            return None
        self.app.processEvents()

    def add_marker(self, pos, color=green, size=10):
        if not isinstance(pos, (np.ndarray)):
            pos = np.array(pos)

        self.markers.append(gl.GLScatterPlotItem(pos=pos, color=color, size=size))
        self.window.addItem(self.markers[-1])

        if 2 * norm(pos) > self.range:
            self.range = 2 * norm(pos)
            self.window.setCameraPosition(distance=self.range)

        self.app.processEvents()

    def remove_marker(self, ind=None):
        if ind is None:
            for marker in self.markers:
                self.window.removeItem(marker.mesh_object)
            self.markers = []
        elif isinstance(ind, (int)):
            self.window.removeItem(self.markers[ind])
            self.markers.pop(ind)
        else:
            print("Warning: invalid index entered!")
            return None
        self.app.processEvents()

    def update(self, qs=None, As=None, poss=None):
        if qs is not None:
            if isinstance(qs[0], (list, tuple, np.ndarray)):
                for i in range(len(self.arms)):
                    self.arms[i].update(qs[i])
            else:
                self.arms[0].update(qs)

        if As is not None:
            if isinstance(As, (list, tuple)):
                for i in range(len(self.frames)):
                    self.frames[i].update(As[i])
            elif len(As.shape) == 3:
                for i in range(len(self.frames)):
                    self.frames[i].update(As[i])
            else:
                self.frames[0].update(As)

        if poss is not None:
            if isinstance(poss, (list, tuple)):
                for i in range(len(self.markers)):
                    if not isinstance(poss[i], (np.ndarray)):
                        pos = np.array(poss[i])
                    else:
                        pos = poss[i]
                    self.markers[i].setData(pos=pos)
            else:
                if not isinstance(poss, (np.ndarray)):
                    pos = np.array(poss)
                else:
                    pos = poss
                self.markers[0].setData(pos=pos)

        self.app.processEvents()
        sleep(0.00001)

    def hold(self):
        while self.window.isVisible():
            self.app.processEvents()

    def wander(self, index=None, q0=None, speed=1e-1, duration=np.inf, accel=5e-4):
        if index is None:
            index = range(len(self.arms))

        tstart = perf_counter()
        t = tstart
        flag = True
        qs = []
        dqs = []

        while t < tstart + duration and self.window.isVisible():
            for i, ind in enumerate(index):
                n = self.arms[ind].n
                if flag:
                    if q0 is None:
                        qs.append(np.zeros((n,)))
                    else:
                        qs.append(q0[i])
                    # dqs.append(np.random.random_sample((n,)) * speed - speed / 2)
                    dqs.append(np.zeros((n,)))

                dqq = np.zeros((n,))
                for j in range(n):
                    s = dqs[i][j] / speed
                    dqq[j] = dqq[j] + np.random.random_sample((1,)) * accel - accel / 2 - accel * s**3
                dqs[i] = dqs[i] + dqq
                qs[i] = qs[i] + dqs[i]
                self.arms[ind].update(qs[i])

            if flag:
                flag = False
            t = perf_counter()
            self.app.processEvents()

    def close_viz(self):
        self.app.closeAllWindows()


class ArmPlayer:
    def __init__(self, arm:RobotArm, target, obstacles, obstacle_radii):
        if QApplication.instance() is None:
            self.app = pg.QtGui.QApplication([])
        else:
            self.app = QApplication.instance()
        self.window = QMainWindow()
        self.window.setGeometry(200, 300, 1000, 700)
        self.window.setWindowTitle("Robot Arm Control")

        self.target = target # IK targets
        self.obstacles = obstacles # IK Obstacles
        self.obstacle_radii = obstacle_radii

        self.main_layout = QHBoxLayout()
        w1 = gl.GLViewWidget()
        grid = gl.GLGridItem()
        grid.scale(1, 1, 1)
        w1.addItem(grid)
        self.arm_hardware = arm
        self.arm_mesh = ArmMeshObject(arm)
        self.n = arm.n
        w1.addItem(self.arm_mesh.mesh_object)
        w1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        w2 = QVBoxLayout()
        self.slider_list = []
        self.slider_label_list = []
        for i in range(arm.n):
            t = QLabel()
            t.setText(f"Joint {i + 1}: 0 degrees")
            t.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            s = QSlider(Qt.Horizontal)
            s.setMinimum(self.arm_hardware.joint_limits[i][0])
            s.setMaximum(self.arm_hardware.joint_limits[i][1])
            s.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            s.sliderMoved.connect(self.update_sliders)
            self.slider_list.append(s)
            self.slider_label_list.append(t)
            w2.addWidget(t, stretch=1)
            w2.addWidget(s, stretch=3)

        # RANDOMIZE BUTTON PROPERTIES
        randomize_button = QPushButton()
        randomize_button.setText("Random Position")
        randomize_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        randomize_button.pressed.connect(self.randomize_slider_values)
        self.randomize_button = randomize_button
        w2.addWidget(randomize_button)

        # RESET BUTTON PROPERTIES
        reset_button = QPushButton()
        reset_button.setText("Reset Joints")
        reset_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        reset_button.pressed.connect(self.reset_sliders)
        self.reset_button = reset_button
        w2.addWidget(reset_button)

        # SEND COMMAND BUTTON PROPERTIES
        send_command_button = QPushButton()
        send_command_button.setText("Send Position to Hardware")
        send_command_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        send_command_button.pressed.connect(self.send_sliders_command)
        self.send_command_button = send_command_button
        w2.addWidget(send_command_button)

        # "VIEW LAST COMMAND" BUTTON PROPERTIES
        self.last_command = [0 for _ in range(len(self.slider_list))]
        view_last_command_button = QPushButton()
        view_last_command_button.setText("View Last Command")
        view_last_command_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        view_last_command_button.pressed.connect(self.view_last_command)
        self.view_last_command_button = view_last_command_button
        w2.addWidget(view_last_command_button)

        # GENERATE PATH BUTTON PROPERTIES
        path_to_target_button = QPushButton()
        path_to_target_button.setText("Generate Path to Target")
        path_to_target_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        path_to_target_button.pressed.connect(self.generate_path_to_target)
        self.path_to_target_button = path_to_target_button
        w2.addWidget(path_to_target_button)

        self.main_layout.addWidget(w1, stretch=3)
        self.main_layout.addLayout(w2, stretch=1)

        w = QWidget()
        w.setLayout(self.main_layout)
        self.window.setCentralWidget(w)
        self.window.show()
        self.window.raise_()

        self.app.processEvents()

        self.app.exec_()

    def generate_path_to_target(self):
        print(f"Calculating path to target...")
        # Grab current state of arm
        q_current = [0, 0, 0, np.pi/2] # This will change depending on what our initial position is. 0 is just a magic number
        q_list = self.arm_hardware.compute_robot_path(q_init=q_current, goal=self.target, obst_location=self.obstacles, obst_radius=self.obstacle_radii, joint_limits=self.arm_hardware.joint_limits)
        print(f"PATH TO TARGET CALCULATED! Number of steps: {len(q_list)}")
        # print(f"q_list: \n{q_list}")
        # print(f"Target position sent to ArmPlayer: {q_list[-1]}")

        # Update slider values
        for i, z in enumerate(zip(self.slider_list, q_list[-1])):
            s, q1 = z[0], z[1]
            q = int(q1 * 180/np.pi)
            s.setValue(q)
            self.slider_label_list[i].setText(f"Joint {i + 1}: {q} degrees")

        # Update ArmMesh object
        self.arm_mesh.update(q_list[-1])

        # Update hardware with path from qs in the trajectory computed
        for i, q_step in enumerate(q_list):
            if i%10 == 0:
                # print(f"unconverted q for trajectory: {q_step}")
                q_step_converted = [converted_joint_step * (180/np.pi) for converted_joint_step in q_step]
                for i in range(len(q_step_converted)):
                    if i==0:
                        q_step_converted[i] += 90 # Compensate for the base joint having -180 range
                    q_step_converted[i] += 90 # Compensate for -90 range available in the slider but not available as a servo command
                # print(f"converted q for trajectory: {q_step_converted}")
                self.arm_hardware.update_hardware_q(q_step_converted, verbose=True)
                time.sleep(0.3)


    def update_sliders(self):
        qs = np.zeros((self.n,))
        for i, s in enumerate(self.slider_list):
            q = s.value()
            # print(f"slider value: {q}")
            qs[i] = q * np.pi / 180
            self.slider_label_list[i].setText(f"Joint {i + 1}: {q} degrees")
        self.arm_mesh.update(qs)

    def reset_sliders(self):
        qs = np.zeros((self.n,))
        for i, z in enumerate(zip(self.slider_list, qs)):
            s, q1 = z[0], z[1]
            q = q1
            s.setValue(q)
            self.slider_label_list[i].setText(f"Joint {i + 1}: {q} degrees")
        self.arm_mesh.update(qs)

    def send_sliders_command(self):
        q = list()
        last_command = list()
        for i, slider in enumerate(self.slider_list):
            value = slider.value()
            last_command.append(value)
            if i==0:
                value += 90 # Compensate for the base joint having -180 range
            value += 90 # Compensate for -90 range available in the slider but not available as a servo command
            q.append(value)
        self.last_command = last_command
        self.arm_hardware.update_hardware_q(q, verbose=True)

    def view_last_command(self):
        for i, z in enumerate(zip(self.slider_list, self.last_command)):
            s = z[0]
            s.setValue(z[1])
            self.slider_label_list[i].setText(f"Joint {i + 1}: {z[1]} degrees")
        self.arm_mesh.update([self.last_command[i]*np.pi/180 for i in range(len(self.last_command))])

    def randomize_slider_values(self):
        # qs = np.random.random_sample((self.n,)) * np.pi
        qs = list()
        for i in range(self.n):
            random_angle = random.randint(self.arm_hardware.joint_limits[i][0], self.arm_hardware.joint_limits[i][1]) * np.pi/180
            qs.append(random_angle)
        for i, z in enumerate(zip(self.slider_list, qs)):
            s, q1 = z[0], z[1]
            q = int(180 / np.pi * q1)
            s.setValue(q)
            self.slider_label_list[i].setText(f"Joint {i + 1}: {q} degrees")
        self.arm_mesh.update(qs)


class ArmMeshObject:
    def __init__(self, arm, link_colors=None, joint_colors=None, ee_color=None, q0=None, draw_frames=False, frame_names=None):
        self.arm = arm
        self.n = arm.n
        self.dh = arm.dh
        self.draw_frames = draw_frames

        if q0 is None:
            q0 = np.zeros((self.n,))
        self.q0 = q0

        self.link_objects = []
        self.frame_objects = []

        if link_colors is None:
            link_colors = [dark_blue] * self.n

        if joint_colors is None:
            joint_colors = [dark_red] * self.n

        dh_array = np.array(self.dh)
        arm_scale = np.max([np.max(dh_array[0:,1:3]), 0.3])
        frame_scale = 1.5
        ee_scale = 2.0

        self.frame_objects.append(FrameMeshObject(scale=arm_scale*frame_scale))

        link_width = np.max([arm_scale, 0.10])*0.20 #* 0.5
        joint_width = np.max([arm_scale, 0.10])*0.30 * 0.5
        joint_height = np.max([arm_scale, 0.10])*0.70 * 0.5

        for i in range(self.n):
            if arm.jt[i] == 'p':
                joint_colors[i] = np.array([0.0, 0.6, 0.0, 1.0])
            self.link_objects.append(LinkMeshObject(self.dh[i],
                                                    link_width=link_width,
                                                    joint_width=joint_width,
                                                    joint_height=joint_height,
                                                    link_color=link_colors[i],
                                                    joint_color=joint_colors[i]))
            self.frame_objects.append(FrameMeshObject(scale=arm_scale*frame_scale))


        self.ee_object = EEMeshObject(scale=arm_scale*ee_scale)
        self.frame_objects.append(FrameMeshObject(scale=arm_scale*frame_scale))

        self.mesh = np.zeros((0, 3, 3))
        self.colors = np.zeros((0, 3, 4))

        self.set_mesh(q0)

        self.mesh_object = gl.GLMeshItem(vertexes=self.mesh,
                                         vertexColors=self.colors,
                                         drawEdges=True,
                                         computeNormals=False,
                                         edgeColor=np.array([0, 0, 0, 1]))

    def update(self, q=None):
        if q is None:
            q = self.q0
        self.set_mesh(q)
        self.mesh_object.setMeshData(vertexes=self.mesh,
                                    vertexColors=self.colors)
    def set_mesh(self, q):
        meshes = []
        colors = []
        if self.draw_frames:
            A = self.arm.fk(q, 0, base=True, tip=False)
            R = A[0:3, 0:3]
            p = A[0:3, 3]
            meshes.append(self.frame_objects[0].get_mesh(R, p))
            colors.append(self.frame_objects[0].get_colors())
        for i in range(self.n):
            A = self.arm.fk(q, i+1, base=True, tip=False)
            R = A[0:3, 0:3]
            p = A[0:3, 3]
            meshes.append(self.link_objects[i].get_mesh(R, p))
            colors.append(self.link_objects[i].get_colors())
            if self.draw_frames:
                meshes.append(self.frame_objects[i+1].get_mesh(R, p))
                colors.append(self.frame_objects[i + 1].get_colors())
        A = self.arm.fk(q, i + 1, base=True, tip=True)
        R = A[0:3, 0:3]
        p = A[0:3, 3]
        if self.draw_frames:
            meshes.append(self.frame_objects[-1].get_mesh(R, p))
            colors.append(self.frame_objects[-1].get_colors())

        meshes.append(self.ee_object.get_mesh(R, p))
        colors.append(self.ee_object.get_colors())


        self.mesh = np.vstack(meshes)
        self.colors = np.vstack(colors)


class LinkMeshObject:
    def __init__(self, dh, jt='r',
                 link_width=0.1,
                 joint_width=0.15,
                 joint_height=0.25,
                 link_color=None,
                 joint_color=None):

        theta = dh[0]
        d = dh[1]
        a = dh[2]
        alpha = dh[3]

        lw = link_width

        w = joint_width
        h = joint_height

        le = sqrt(d**2 + a**2)

        self.link_points = np.array([[0, lw/2, 0],
                                     [0, 0, lw/2],
                                     [0, -lw/2, 0],
                                     [0, 0, -lw/2],
                                     [-le, lw / 2, 0],
                                     [-le, 0, lw / 2],
                                     [-le, -lw / 2, 0],
                                     [-le, 0, -lw / 2]])

        v1 = np.array([-le, 0, 0])
        v2 = np.array([-a, d * sin(-alpha), -d * cos(-alpha)])

        axis = np.cross(v1, v2)
        n_axis = np.linalg.norm(axis)
        if np.abs(n_axis) < 1e-12:
            R = np.eye(3)
        else:
            axis = axis / np.linalg.norm(axis)
            ang = np.arccos(v1 @ v2 / (norm(v1) * norm(v2)))
            v = axis / norm(axis)
            V = np.array([[0, -v[2], v[1]],
                          [v[2], 0, -v[0]],
                          [-v[1], v[0], 0]])
            R = np.eye(3) + sin(ang) * V + (1 - cos(ang)) * V @ V
            # R = axis2R(ang, axis)

        self.link_points = self.link_points @ R.T

        self.joint_points = np.array([[0.5 * w, -0.5 * w, -0.5 * h],
                                [-0.5 * w, -0.5 * w, -0.5 * h],
                                [-0.5 * w, -0.5 * w, 0.5 * h],
                                [0.5 * w, -0.5 * w, 0.5 * h],
                                [0.5 * w, 0.5 * w, -0.5 * h],
                                [-0.5 * w, 0.5 * w, -0.5 * h],
                                [-0.5 * w, 0.5 * w, 0.5 * h],
                                [0.5 * w, 0.5 * w, 0.5 * h]])

        Rz = np.array([[cos(theta), -sin(theta), 0],
                       [sin(theta), cos(theta), 0],
                       [0, 0, 1]])

        Rx = np.array([[1, 0, 0],
                       [0, cos(alpha), -sin(alpha)],
                       [0, sin(alpha), cos(alpha)]])

        self.joint_points = self.joint_points @ Rz @ Rx + v2

        if link_color is None:
            link_color = np.array([0, 0, 0.35, 1])
        elif not isinstance(link_color, (np.ndarray)):
            link_color = np.array(link_color)
        self.link_colors = np.zeros((12, 3, 4)) + link_color

        if joint_color is None:
            joint_color = np.array([0.35, 0, 0., 1])
        elif not isinstance(joint_color, (np.ndarray)):
            joint_color = np.array(joint_color)
        self.joint_colors = np.zeros((12, 3, 4)) + joint_color
        self.joint_colors[6:8, :, :] = np.zeros((2, 3, 4)) + grey

    @staticmethod
    def points_to_mesh(link_points, joint_points):

        link_mesh = np.array([[link_points[0], link_points[4], link_points[5]],
                              [link_points[0], link_points[1], link_points[5]],
                              [link_points[1], link_points[5], link_points[6]],
                              [link_points[1], link_points[2], link_points[6]],
                              [link_points[2], link_points[6], link_points[7]],
                              [link_points[2], link_points[7], link_points[3]],
                              [link_points[3], link_points[7], link_points[4]],
                              [link_points[3], link_points[0], link_points[4]],
                              [link_points[0], link_points[2], link_points[3]],
                              [link_points[0], link_points[1], link_points[2]],
                              [link_points[5], link_points[4], link_points[7]],
                              [link_points[5], link_points[6], link_points[7]]])

        joint_mesh = np.array([[joint_points[0], joint_points[1], joint_points[2]],
                         [joint_points[0], joint_points[2], joint_points[3]],
                         [joint_points[0], joint_points[3], joint_points[4]],
                         [joint_points[3], joint_points[4], joint_points[7]],
                         [joint_points[2], joint_points[3], joint_points[6]],
                         [joint_points[3], joint_points[6], joint_points[7]],
                         [joint_points[0], joint_points[1], joint_points[5]],
                         [joint_points[0], joint_points[4], joint_points[5]],
                         [joint_points[1], joint_points[2], joint_points[6]],
                         [joint_points[1], joint_points[5], joint_points[6]],
                         [joint_points[4], joint_points[5], joint_points[6]],
                         [joint_points[4], joint_points[6], joint_points[7]]])

        return np.vstack((link_mesh, joint_mesh))

    def get_colors(self):
        return np.vstack((self.link_colors, self.joint_colors))

    def get_mesh(self, R, p):
        lp = self.link_points @ R.T + p
        jp = self.joint_points @ R.T + p

        return self.points_to_mesh(lp, jp)


class FrameViz:
    def __init__(self, A=np.eye(4), scale=1, colors=[red, green, blue], label=None):
        self.frame_object = FrameMeshObject(scale, colors)
        self.mesh = self.frame_object.get_mesh(A[0:3, 0:3], A[0:3, 3])
        self.colors = self.frame_object.get_colors()
        self.mesh_object = gl.GLMeshItem(vertexes=self.mesh,
                                         vertexColors=self.colors,
                                         computeNormals=False,
                                         drawEdges=False)
        if label is not None:
            self.label = GLTextItem(A[0:3, 3], text=label)
        else:
            self.label = None

    def update(self, A):
        self.mesh = self.frame_object.get_mesh(A[0:3, 0:3], A[0:3, 3])
        self.colors = self.frame_object.get_colors()
        self.mesh_object.setMeshData(vertexes=self.mesh,
                                     vertexColors=self.colors)
        if self.label is not None:
            p = A[0:3, 3] + A[0:3, 0:3] @ np.array([0.35, 0, 0])
            self.label.setData(pos=p)


class FrameMeshObject:
    def __init__(self, scale=1.0, colors=[red, green, blue]):
        a = 0.1 * scale
        b = 0.35 * scale
        self.points = np.array([[0, 0, 0],
                                [a, 0, 0],
                                [0, a, 0],
                                [0, 0, a],
                                [b, 0, 0],
                                [0, b, 0],
                                [0, 0, b]])
        c = [np.zeros((4,3,4)) + red, np.zeros((4,3,4)) + green, np.zeros((4,3,4)) + blue]
        self.colors = np.vstack(c)

    @staticmethod
    def points_to_mesh(p):
        mesh = np.array([[p[0], p[2], p[3]],
                         [p[0], p[3], p[4]],
                         [p[3], p[2], p[4]],
                         [p[2], p[0], p[4]],
                         [p[0], p[1], p[3]],
                         [p[0], p[1], p[5]],
                         [p[1], p[3], p[5]],
                         [p[3], p[5], p[5]],
                         [p[0], p[1], p[2]],
                         [p[0], p[1], p[6]],
                         [p[1], p[2], p[6]],
                         [p[2], p[0], p[6]]
                         ])
        return mesh

    def get_mesh(self, R, p):
        points = self.points @ R.T + p
        mesh = self.points_to_mesh(points)
        return mesh

    def get_colors(self):
        return self.colors


class EEMeshObject:
    def __init__(self, scale=1, w=0.05, o1=0.05, o2=0.15, o3=0.3, o4=0.2, o5=0.1):
        w = w * scale
        o1 = o1 * scale
        o2 = o2 * scale
        o3 = o3 * scale
        o4 = o4 * scale
        o5 = o5 * scale

        self.points = np.array([[-o1, o2, w/2],
                                [-o1, -o2, w/2],
                                [o3, 0, w/2],
                                [-o1, o2, -w/2],
                                [-o1, -o2, -w/2],
                                [o3, 0, -w/2]
                                ])
        self.colors = np.zeros((8,3,4)) + dark_red
        self.colors[1,:,:] = np.zeros((3,4)) + dark_blue

    @staticmethod
    def points_to_mesh(p):
        mesh = np.array([[p[0], p[1], p[2]],
                         [p[3], p[4], p[5]],
                         [p[0], p[1], p[3]],
                         [p[1], p[4], p[3]],
                         [p[1], p[2], p[5]],
                         [p[1], p[4], p[5]],
                         [p[0], p[2], p[5]],
                         [p[0], p[3], p[5]]
                         ])
        return mesh

    def get_mesh(self, R, p):
        points = self.points @ R.T + p
        mesh = self.points_to_mesh(points)
        return mesh

    def get_colors(self):
        return self.colors


class GLTextItem(GLGraphicsItem):
    """
    Class for plotting text on a GLWidget
    """

    def __init__(self, pos=None, text=None):
        GLGraphicsItem.__init__(self)
        self.setGLOptions('translucent')
        self.text = text
        self.pos = pos

    def setGLViewWidget(self, GLViewWidget):
        self.GLViewWidget = GLViewWidget

    def setText(self, text):
        self.text = text
        self.update()

    def setPos(self, pos):
        self.pos = pos
        self.update()

    def setData(self, pos=None, text=None):
        if pos is not None:
            self.pos = pos

        if text is not None:
            self.text = text

        self.update()

    def paint(self):
        self.GLViewWidget.qglColor(Qt.white)
        self.GLViewWidget.renderText(self.pos[0], self.pos[1], self.pos[2], self.text)
