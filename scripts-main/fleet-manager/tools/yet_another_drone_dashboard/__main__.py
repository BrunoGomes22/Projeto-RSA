import rclpy
import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gio, Gdk
from widgets import DronesBox


class DashboardWindow(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="Yet Another Drone Dashboard")
        self.set_wmclass ("yaDD", "yaDD")
        self.set_border_width(10)
        self.set_default_size(940, 600)
        self.connect("destroy", Gtk.main_quit)
        self.add(DronesBox())
        self.show_all()


def on_destroy(widget=None, *data):
    rclpy.shutdown()
    exit()


def main():
    rclpy.init()
    gi.require_version("Gtk", "3.0")
    win = DashboardWindow()
    win.connect("destroy", Gtk.main_quit)
    win.connect('delete-event', on_destroy)
    win.show_all()
    Gdk.threads_init()
    Gtk.main()


if __name__ == '__main__':
    main()
