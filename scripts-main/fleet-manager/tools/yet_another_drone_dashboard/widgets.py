import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gio, Gdk
import rclpy
from threading import Thread
from datetime import datetime
from comm import CommNode
import comm
import os
import sys
import json

class DronesBox(Gtk.Box):
    def __init__(self, **kwargs):
        Gtk.Box.__init__(self, **kwargs)
        node = CommNode(self.update_telem)
        self.drones = []
        self.telem_frame = TelemetryFrame()
        self.cmd_frame = CommandsFrame(node)
        self.status_frame = StatusFrame()
        self.mission_frame = MissionFrame()

        telem_and_status_col = Gtk.VBox()
        telem_and_status_col.pack_start(self.telem_frame, True, True, 5)
        # TODO add status when implemented
        #telem_and_status_col.pack_start(self.status_frame, True, True, 5)

        cmd_and_mission_col = Gtk.VBox()
        cmd_and_mission_col.pack_start(self.cmd_frame, True, True, 5)
        cmd_and_mission_col.pack_start(self.mission_frame, True, True, 5)

        self.pack_start(telem_and_status_col, True, True, 5)
        self.pack_start(cmd_and_mission_col, True, True, 5)
        thread = Thread(target=rclpy.spin, args=(node,))
        thread.start()

    def update_telem(self, telem):
        Gdk.threads_enter()
        if telem.get('droneId') in self.drones:
            self.telem_frame.update_telemetry(telem)
        else:
            self.drones.append(telem.get('droneId'))
            self.cmd_frame.add_drone_to_list(telem.get('droneId'))
            self.telem_frame.add_new_drone(telem)
        self.show_all()
        Gdk.threads_leave()


class TelemetryFrame(Gtk.Frame):
    def __init__(self, **kwargs):
        Gtk.Frame.__init__(self, label="Telemetry", border_width=10)
        self.telem_views = dict()
        nb = Gtk.Notebook(scrollable=True, margin_top=10, margin_bottom=10, margin_start=10, margin_end=10)
        self.add(nb)

    def add_new_drone(self, telem):
        telem_view = TelemView(telem)
        nb = self.get_child()
        nb.append_page(telem_view)
        nb.set_tab_label_text(telem_view, telem.get('droneId'))
        self.telem_views[telem.get('droneId')] = telem_view

    def update_telemetry(self, telem):
        self.telem_views[telem.get('droneId')].update_view(telem)


class TelemView(Gtk.TreeView):
    def __init__(self, telem, **kwargs):
        kwargs.update({'margin_top': 10, 'margin_bottom': 10, 'margin_start': 10, 'margin_end': 10})
        Gtk.TreeView.__init__(self, **kwargs)
        telem_store = Gtk.ListStore(str, str)
        for k, v in telem_to_text(telem).items():
            telem_store.append([k, v])
        self.set_model(telem_store)
        renderer = Gtk.CellRendererText()
        properties = Gtk.TreeViewColumn("Property", renderer, text=0)
        values = Gtk.TreeViewColumn("Current value", renderer, text=1)
        self.append_column(properties)
        self.append_column(values)

    def update_view(self, telem):
        telem_store = self.get_model()
        i = 0
        for v in telem_to_text(telem).values():
            path = Gtk.TreePath(i)
            treeiter = telem_store.get_iter(path)
            telem_store.set_value(treeiter, 1, v)
            i += 1


# TODO
class StatusFrame(Gtk.Frame):
    def __init__(self, **kwargs):
        Gtk.Frame.__init__(self, label="Status", border_width=10)
        status_window = Gtk.ScrolledWindow()


class MissionFrame(Gtk.Frame):
    def __init__(self, **kwargs):
        Gtk.Frame.__init__(self, label="Missions", border_width=10)
        self.mission_id = None
        demo1_btn = ActionButton(self.demo_1, label="Demo 1", )
        demo2_btn = ActionButton(self.demo_2, label="Demo 2", )
        demo_row = ButtonRow([demo1_btn, demo2_btn])
        cancel_btn = ActionButton(self.cancel, label="Cancel last mission", )
        mission_box = Gtk.VBox()
        mission_box.set_margin_start(10)
        mission_box.set_margin_end(10)
        mission_box.set_margin_top(10)
        mission_box.set_margin_bottom(10)
        mission_box.pack_start(demo_row, False, True, 10)
        # TODO selection box from dir
        mission_box.pack_start(cancel_btn, False, True, 10)
        self.add(mission_box)

    def demo_1(self, button):
        with open(os.path.join(sys.path[0], "demos/up_and_down.groovy"), "r") as f:
            response = comm.submit_mission(f.read())
            if response.status_code == 201:
                self.mission_id = json.loads(response.text)['missionId']

    def demo_2(self, button):
        with open(os.path.join(sys.path[0], "demos/square.groovy"), "r") as f:
            response = comm.submit_mission(f.read())
            if response.status_code == 201:
                self.mission_id = json.loads(response.text)['missionId']

    def cancel(self, button):
        if self.mission_id is not None:
            comm.cancel_mission(self.mission_id)
            self.mission_id = None

class CommandsFrame(Gtk.Frame):
    def __init__(self, node, **kwargs):
        Gtk.Frame.__init__(self, label="Commands", border_width=10)
        self.node = node
        self.drone_selection = DroneComboBox()
        arm_disarm_row = ButtonRow(
            [ActionButton(self.on_arm, label="Arm", ), ActionButton(self.on_disarm, label="Disarm")])
        takeoff_land_row = ButtonRow(
            [ActionButton(self.on_takeoff, label="Takeoff", ), ActionButton(self.on_land, label="Land")])
        self.move_row = ButtonEntriesRow(
            [{"label": "Forward:", "field": "forward"}, {"label": "Right:    ", "field": "right"},
             {"label": "Up:       ", "field": "up"}], "Move", self.on_move)
        self.turn_row = ButtonEntriesRow([{"label": "degrees", "field": "degrees"}], "Turn", self.on_turn, True)
        home_cancel_row = ButtonRow(
            [ActionButton(self.on_home, label="Home", ), ActionButton(self.on_cancel, label="Cancel")])

        cmd_box = Gtk.VBox()
        cmd_box.pack_start(self.drone_selection, False, True, 10)
        cmd_box.pack_start(Gtk.Separator(), False, True, 10)
        cmd_box.pack_start(arm_disarm_row, False, True, 10)
        cmd_box.pack_start(takeoff_land_row, False, True, 10)
        cmd_box.pack_start(Gtk.Separator(), False, True, 10)
        cmd_box.pack_start(self.move_row, False, True, 10)
        cmd_box.pack_start(Gtk.Separator(), False, True, 10)
        cmd_box.pack_start(self.turn_row, False, True, 10)
        cmd_box.pack_start(Gtk.Separator(), False, True, 10)
        cmd_box.pack_end(home_cancel_row, False, True, 10)
        self.add(cmd_box)

    def add_drone_to_list(self, id):
        self.drone_selection.drone_store.append([id])

    def get_drone_id(self):
        return self.drone_selection.get_selected_drone()

    def on_arm(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_action_cmd(self.get_drone_id(), "arm")

    def on_disarm(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_action_cmd(self.get_drone_id(), "disarm")

    def on_takeoff(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_action_cmd(self.get_drone_id(), "takeoff")

    def on_land(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_action_cmd(self.get_drone_id(), "land")

    def on_move(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_move_cmd(self.get_drone_id(), self.move_row.get_value("forward"),
                                   self.move_row.get_value("right"), self.move_row.get_value("up"))

    def on_turn(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_turn_cmd(self.get_drone_id(), self.turn_row.get_value("degrees"))

    def on_home(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_action_cmd(self.get_drone_id(), "return")

    def on_cancel(self, button):
        if self.get_drone_id() is not None:
            self.node.pub_cancel_cmd(self.get_drone_id())


class DroneComboBox(Gtk.ComboBox):
    def __init__(self, **kwargs):
        Gtk.ComboBox.__init__(self, **kwargs)
        self.set_margin_start(10)
        self.set_margin_end(10)
        self.drone_id = None
        self.drone_store = Gtk.ListStore(str)
        self.drone_store.append(["Select a drone"])
        self.set_model(self.drone_store)
        self.connect("changed", self.on_drone_changed)
        renderer_text = Gtk.CellRendererText()
        self.pack_start(renderer_text, True)
        self.add_attribute(renderer_text, "text", 0)
        self.set_active(0)

    def on_drone_changed(self, combo):
        tree_iter = self.get_active_iter()
        if tree_iter is not None:
            model = self.get_model()
            if self.get_active() == 0:
                self.drone_id = None
            else:
                self.drone_id = model[tree_iter][0]

    def get_selected_drone(self):
        return self.drone_id


class LabeledNumberEntry(Gtk.Box):
    def __init__(self, label, align_right=False, **kwargs):
        Gtk.Box.__init__(self, **kwargs)
        entry_label = Gtk.Label(label=label, xalign=0)
        self.entry = NumberEntry(text=0)
        if align_right:
            self.pack_start(self.entry, True, True, 5)
            self.pack_start(entry_label, True, True, 5)
        else:
            self.pack_start(entry_label, True, True, 5)
            self.pack_start(self.entry, True, True, 5)

    def get_value(self):
        if self.entry.get_text() is '':
            return 0
        return float(self.entry.get_text())


class NumberEntry(Gtk.Entry):
    def __init__(self, **kwargs):
        Gtk.Entry.__init__(self, **kwargs)
        self.connect('changed', self.on_changed)
        self.dot = 0
        self.char_count = 0

    def on_changed(self, *args):
        text = self.get_text().strip()
        self.dot = 0
        self.char_count = 0
        self.set_text(''.join([i for i in text if self.validate_minus() or i in '0123456789' or (i is '.' and self.validate_dot())]))

    def validate_minus(self):
        self.char_count += 1
        if self.char_count == 1:
            return True
        return False

    def validate_dot(self):
        self.dot += 1
        return self.dot < 2


class ButtonEntriesRow(Gtk.Box):
    def __init__(self, entries, btn_label, btn_action, align_right=False, **kwargs):
        Gtk.Box.__init__(self, **kwargs)
        self.entries = dict()
        button = ActionButton(btn_action, label=btn_label)
        entries_box = Gtk.VBox()
        for entry_data in entries:
            entry = LabeledNumberEntry(entry_data["label"], align_right)
            entries_box.pack_start(entry, True, True, 5)
            self.entries[entry_data["field"]] = entry
        self.pack_start(entries_box, True, True, 10)
        self.pack_end(button, True, True, 10)

    def get_value(self, field):
        return self.entries[field].get_value()


class ActionButton(Gtk.Button):
    def __init__(self, action, **kwargs):
        Gtk.Button.__init__(self, **kwargs)
        self.connect("clicked", action)


class ButtonRow(Gtk.Box):
    def __init__(self, buttons, **kwargs):
        Gtk.Box.__init__(self, **kwargs)
        for btn in buttons:
            self.pack_start(btn, True, True, 10)


def telem_to_text(telem):
    return {"Flight mode": telem.get("flight_mode"),
            "Landed state": telem.get("landed_state"),
            "Armed": str(telem.get("armed")),
            "Current position": format_number_5(telem.get("position").get("lat")) + ", " + format_number_5(telem.get(
                "position").get("lon")) + ", " + format_number_2(telem.get("position").get("alt")) + "m",
            "Heading": format_number_2(telem.get("heading")) + "°N",
            "Height": format_height(telem.get("height")),
            "Speed": format_number_2(telem.get("speed")) + "m/s",
            "Remaining battery": format_number_2((telem.get("battery").get("remaining_percent") * 100)) + "%",
            "Voltage": format_number_2(telem.get("battery").get("voltage")) + "V",
            "Home position": format_number_5(telem.get("home").get("lat")) + ", " + format_number_5(
                telem.get("home").get(
                    "lon")) + ", " + format_number_2(telem.get("home").get("alt")) + "m",
            "Health Failures": str(telem.get("healthFail")),
            "GPS fix": telem.get("gpsInfo").get("fixType"),
            "Satellites": str(telem.get("gpsInfo").get("satellites")),
            "Timestamp": datetime.fromtimestamp(telem.get("timestamp") / 1000.0).strftime("%d/%m/%Y, %H:%M:%S")}


def format_number_5(num, ):
    if num is None:
        return ''
    return "%0.5f" % num


def format_number_2(num, ):
    if num is None:
        return ''
    return "%0.2f" % num

def format_height(num):
    if num is None:
        return '--'
    return "%0.2f mm" % num