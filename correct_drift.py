import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import matplotlib.patches as patches
from matplotlib.widgets import Button, CheckButtons
import math
from networktables import NetworkTables, NetworkTablesInstance
import time
import atexit
import json

field_width_in = 360
field_height_in = 180

arrow_len = 20
marker_points_in = {}

select_distance = 10
for x in range(11):
    for y in range(5):
        xx = (x + 1) * 30
        xn = str(x + 1)

        yy = (y + 1) * 30
        yn = ["E", "D", "C", "B", "A"][y]
        marker_points_in[yn + xn] = (xx, yy)


def m_to_in(m):
    return m * 39.3701


def in_to_m(i):
    return i * 0.0254


class CorrectDrift:
    def __init__(self):
        self.points = []
        self.transformlater = False
        self.drag_pt = -1
        self.drag_arrow = False
        self.last_mousepos = np.array([0, 0])

        # NetworkTables.startClientTeam(2877)
        NetworkTables.startClient("localhost")

        while not NetworkTables.isConnected():
            print("waiting for NT connection...")
            time.sleep(1)

        smartdashboard = NetworkTables.getTable("SmartDashboard")
        self.run_nt = smartdashboard.getEntry("correctdrift/run")
        self.path_name_nt = smartdashboard.getEntry("correctdrift/path_name")

        # points: [init_x, init_y, init_dir, waypoint0_x, waypoint0_y, ..., final_x, final_y, final_dir]
        self.original_points_nt = smartdashboard.getEntry("correctdrift/original_points")
        self.modified_points_nt = smartdashboard.getEntry("correctdrift/modified_points")

        self.original_points_nt.setDefaultDoubleArray([])
        self.modified_points_nt.setDefaultDoubleArray([])

        self.field_elements = []
        self.init_plot()

        '''
        self.original_points_nt.addListener(lambda entry, key, value, param: self.update_original_points(),
                                            NetworkTablesInstance.NotifyFlags.UPDATE)

        self.modified_points_nt.addListener(lambda entry, key, value, param: self.update_modified_points(),
                                            NetworkTablesInstance.NotifyFlags.UPDATE)

        self.path_name_nt.addListener(lambda entry, key, value, param: self.update_field(value),
                                      NetworkTablesInstance.NotifyFlags.UPDATE)
        '''
        self.original_points_nt.addListener(lambda entry: print("???",entry),
                                            NetworkTablesInstance.NotifyFlags.UPDATE)

        self.update_original_points(True)
        self.update_field(self.path_name_nt.getString(""))

        self.update_plot()
        atexit.register(self.save_modified_points)
        plt.show()

    def save_modified_points(self):
        with open('path.json', 'w') as f:
            json.dump(self.combine_pts_and_rot(self.points), f)
        for i in range(len(self.points)):
            pt = self.points[i]
            print("Point " + str(i) + ":")
            print("position: " + pt["position"])
            if "angle" in pt:
                print("angle: " + pt["angle"])
            print()

    def update_field(self, path_name):
        self.draw_field(path_name)
        self.update_plot()

    @staticmethod
    def extract_pts(arr):
        if len(arr) < 6:
            return np.zeros((0, 2))
        else:
            arr_np = np.array(arr)
            start = arr_np[:2]
            center = arr_np[3:-3]
            end = arr_np[-3:-1]
            combined = np.concatenate((start, center, end))
            return m_to_in(combined.reshape((-1, 2)))

    @staticmethod
    def extract_pts_and_rot(arr):
        if len(arr) < 6:
            return []
        else:
            arr_np = np.array(arr)
            start = arr_np[:3]
            end = arr_np[-3:]
            center = arr_np[3:-3]
            pts = [
                {
                    "position": m_to_in(np.array(start[:2], dtype=np.float32)),
                    "angle": start[2]
                }
            ]

            for i in range(len(center) // 2):
                pts.append(
                    {
                        "position": m_to_in(np.array(center[2 * i:2 * i + 2], dtype=np.float32)),
                    }
                )
            pts.append(

                {
                    "position": m_to_in(np.array(end[:2], dtype=np.float32)),
                    "angle": end[2]
                }

            )
            return pts

    @staticmethod
    def combine_pts_and_rot(dat):
        res = []
        for x in dat:
            res.extend(in_to_m(x["position"]))
            if "angle" in x:
                res.append(x["angle"])
        return res

    def update_original_points(self, first=False):
        print("a")
        self.orig_line.set_data(*self.extract_pts(self.original_points_nt.getDoubleArray([])).T)
        if first:
            try:
                with open("path.json") as f:
                    self.points = self.extract_pts_and_rot(json.load(f))
            except FileNotFoundError:
                print("path.json not found")

            except json.decoder.JSONDecodeError:
                print("bad json")

        else:
            self.points = self.extract_pts_and_rot(self.original_points_nt.getDoubleArray([]))
        self.push_modified_points()
        self.update_plot()
        print("b")
    def update_modified_points(self):
        print("c")
        self.points = self.extract_pts_and_rot(self.modified_points_nt.getDoubleArray([]))
        self.update_plot()
        print("d")
    def push_modified_points(self):
        print("e")
        self.modified_points_nt.setDoubleArray(self.combine_pts_and_rot(self.points))
        print("f")
    def init_plot(self):
        self.figure = plt.figure("Correct drift")
        ax = plt.subplot(1, 1, 1)
        plt.subplots_adjust(top=.95, bottom=0.15, left=.05, right=.95)

        ax.spines['right'].set_position(('axes', 0))
        ax.spines['top'].set_position(('axes', 0))

        ax.spines['left'].set_color('none')
        ax.spines['bottom'].set_color('none')

        ax.xaxis.set_ticks_position('top')
        ax.yaxis.set_ticks_position('right')
        ax.grid(linestyle='-', linewidth='0.5')
        plt.axis([0, field_width_in, 0, field_height_in])

        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_xticks(np.arange(1, field_width_in / 30 + 1, 1) * 30)
        ax.set_yticks(np.arange(1, field_height_in / 30 + 1, 1) * 30)

        plt.gca().set_aspect('equal')

        self.figure.canvas.mpl_connect('button_press_event', self.mousedown)
        self.figure.canvas.mpl_connect('button_release_event', self.mouseup)
        self.figure.canvas.mpl_connect('motion_notify_event', self.mousemove)

        self.axtransformlater = plt.axes([.05, 0.05 - .1, 0.2, 0.075 + .2], zorder=-1)
        self.axtransformlater.axis('off')

        # self.axgenerate =plt.axes([.55, 0.05, 0.15, 0.075])
        self.axrun = plt.axes([.8, 0.05, 0.15, 0.075])
        self.btransformlater = CheckButtons(self.axtransformlater, ['Transform Later'])
        self.btransformlater.on_clicked(self.transformlater_pressed)

        # self.bgenerate = Button(self.axgenerate, 'Generate')
        self.brun = Button(self.axrun, 'Run')
        self.brun.on_clicked(self.runpressed)

        self.ax = ax

        self.orig_line, = ax.plot([], [], "red", marker="o", markersize=5)

        self.mod_line, = ax.plot([], [], "green", marker="o", markersize=5)
        self.mod_arrow = ax.quiver([], [], [], [])

    def draw_field(self, path_name):
        print("DRAW FIELD", path_name)
        for x in self.field_elements:
            x.remove()
        self.field_elements.clear()

        ax = self.ax
        print("ax", ax)

        def addpt(x, col):
            self.field_elements.extend(ax.plot(*x, "o", color=col))

        def addptname(name, col):
            addpt(marker_points_in[name], col)
            self.field_elements.append(ax.annotate(name, marker_points_in[name]))

        def addrect(a, b, color):
            rect = patches.Rectangle(a, b[0] - a[0], b[1] - a[1], linewidth=1, edgecolor=color, facecolor=color)
            self.field_elements.append(rect)
            ax.add_patch(rect)

        def add_gal_search_field(path):
            addrect((0, 0), (30, 180), "#bcd4bc")
            addrect((330, 0), (360, 180), "#d4c4c4")
            if path == "pathA":
                addptname("C3", "r")
                addptname("D5", "r")
                addptname("A6", "r")
                addptname("E6", "b")
                addptname("B7", "b")
                addptname("C9", "b")
            elif path == "pathB":
                addptname("B3", "r")
                addptname("D5", "r")
                addptname("B7", "r")
                addptname("D6", "b")
                addptname("B8", "b")
                addptname("D10", "b")

        def add_autonav_field(path):
            if path == "barrel":
                addrect((0, 60), (60, 120), "#bcd4bc")
                addptname("D5", "c")
                addptname("B8", "c")
                addptname("D10", "c")
            elif path == "slalom":
                addrect((0, 0), (60, 60), "#bcd4bc")
                addrect((0, 60), (60, 120), "#d4c4c4")
                addptname("D4", "c")
                addptname("D5", "c")
                addptname("D6", "c")
                addptname("D7", "c")
                addptname("D8", "c")
                addptname("D10", "c")
            elif path == "bounce":
                addrect((0, 60), (60, 120), "#bcd4bc")
                addrect((300, 60), (360, 120), "#d4c4c4")
                addptname("D3", "c")
                addptname("E3", "c")
                addptname("B4", "c")
                addptname("B5", "c")
                addptname("D5", "c")
                addptname("B7", "c")
                addptname("D7", "c")
                addptname("B8", "c")
                addptname("D8", "c")
                addptname("A3", "g")
                addptname("A6", "g")
                addptname("A9", "g")

        if path_name == "pathA" or path_name == "pathB":
            add_gal_search_field(path_name)
        elif path_name == "barrel" or path_name == "slalom" or path_name == "bounce":
            add_autonav_field(path_name)

    def runpressed(self, event):
        print("Run")
        self.run_nt.setBoolean(True)

    def update_plot(self):
        xs = []
        ys = []
        axs = []
        ays = []
        dxs = []
        dys = []
        for i in range(len(self.points)):
            pt = self.points[i]
            xs.append(pt["position"][0])
            ys.append(pt["position"][1])
            if "angle" in pt:
                axs.append(pt["position"][0])
                ays.append(pt["position"][1])
                dxs.append(math.cos(pt["angle"]) * arrow_len)
                dys.append(math.sin(pt["angle"]) * arrow_len)

            elif self.transformlater:
                dx = self.points[i + 1]["position"] - pt["position"]
                dx /= np.linalg.norm(dx)
                dx *= arrow_len

                axs.append(pt["position"][0])
                ays.append(pt["position"][1])
                dxs.append(dx[0])
                dys.append(dx[1])

        self.mod_line.set_data(xs, ys)
        self.mod_arrow.remove()
        self.mod_arrow = self.ax.quiver(axs, ays, dxs, dys, zorder=3, angles='xy', scale_units='xy', scale=1,
                                        width=.007, color="green")
        self.figure.canvas.draw_idle()

    def transformlater_pressed(self, event):
        self.transformlater = self.btransformlater.get_status()[0]
        self.update_plot()

    def getpoint(self, mpos):
        for i in range(len(self.points)):
            pt = self.points[i]
            d = np.linalg.norm(pt["position"] - mpos)
            if d < select_distance:
                return i
        return -1

    def getarrow(self, mpos):
        for i in range(len(self.points)):
            pt = self.points[i]
            apos = None
            if "angle" in pt:
                apos = pt["position"] + np.array([math.cos(pt["angle"]) * arrow_len, math.sin(pt["angle"]) * arrow_len])
            elif self.transformlater:
                dx = self.points[i + 1]["position"] - pt["position"]
                dx /= np.linalg.norm(dx)
                dx *= arrow_len
                apos = pt["position"] + dx

            if apos is not None:
                d = np.linalg.norm(mpos - apos)
                if d < select_distance:
                    return i
        return -1

    def mousedown(self, event):
        if event.xdata is not None and event.ydata is not None:
            mpos = np.array([event.xdata, event.ydata])
            self.drag_pt = self.getpoint(mpos)
            if self.drag_pt != -1:
                self.drag_arrow = False
            else:
                self.drag_pt = self.getarrow(mpos)
                if self.drag_pt != -1:
                    self.drag_arrow = True
            self.last_mousepos = mpos

        pass

    def mouseup(self, event):
        self.drag_pt = -1
        self.push_modified_points()

    def mousemove(self, event):
        if self.drag_pt != -1:
            if event.xdata is not None and event.ydata is not None:
                mpos = np.array([event.xdata, event.ydata])
                if self.drag_arrow:
                    if self.transformlater:
                        pt = self.points[self.drag_pt]["position"]
                        prev_ang = math.atan2(self.last_mousepos[1] - pt[1], self.last_mousepos[0] - pt[0])
                        curr_ang = math.atan2(mpos[1] - pt[1], mpos[0] - pt[0])
                        delta = curr_ang - prev_ang
                        for i in range(self.drag_pt, len(self.points)):
                            if "angle" in self.points[i]:
                                self.points[i]["angle"] += delta
                            diff = self.points[i]["position"] - pt
                            this_d = math.sqrt(diff[0] ** 2 + diff[1] ** 2)
                            this_ang = math.atan2(diff[1], diff[0])
                            this_ang += delta
                            self.points[i]["position"] = np.array(
                                [this_d * math.cos(this_ang), this_d * math.sin(this_ang)]) + pt

                        self.last_mousepos = mpos
                    else:
                        pt = self.points[self.drag_pt]["position"]
                        prev_ang = math.atan2(self.last_mousepos[1] - pt[1], self.last_mousepos[0] - pt[0])
                        curr_ang = math.atan2(mpos[1] - pt[1], mpos[0] - pt[0])
                        delta = curr_ang - prev_ang
                        self.points[self.drag_pt]["angle"] += delta
                        self.last_mousepos = mpos
                    self.update_plot()
                else:
                    delta = mpos - self.last_mousepos
                    if self.transformlater:
                        for i in range(self.drag_pt, len(self.points)):
                            self.points[i]["position"] += delta
                    else:
                        self.points[self.drag_pt]["position"] += delta
                    self.last_mousepos = mpos
                self.update_plot()


CorrectDrift()
