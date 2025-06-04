#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from tkinter import filedialog, messagebox, simpledialog

import re
import subprocess
import json
import copy
import threading


def parse_ansi_and_display(output, text_box=None, clear=True):
    if clear:
        text_box.delete(1.0, tk.END)

    color_map = {
        "\033[0;32m": "green",
        "\033[0;31m": "red",
        "\033[0;33m": "red",
        "\033[0m": "default"
    }

    pattern = re.compile(r'(\033\[[0-9;]*m)') # Match ANSI escape codes
    parts = pattern.split(output)

    current_color = "default"

    for part in parts:
        if part in color_map:
            current_color = color_map[part]
        else:
            text_box.insert(tk.END, part, current_color)
            text_box.see(tk.END)

    text_box.tag_config("green", foreground="green")
    text_box.tag_config("red", foreground="red")
    text_box.tag_config("yellow", foreground="yellow")    

def run_command(command, sudo=False):
    print(f"Running command: {command}")
    global command_dump_text
    parse_ansi_and_display(f'{command}\n', command_dump_text, False)

    if sudo:
        password = simpledialog.askstring("Authentication", "Enter sudo password:", show="*")
        if password:
            command_with_sudo = f"echo {password} | sudo -S {command}"
        else:
            return
    else:
        command_with_sudo = command

    def execute():
        global output_text
        try:
            result = subprocess.run(command_with_sudo, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output = result.stdout.decode()
            parse_ansi_and_display(output, output_text)
        except subprocess.CalledProcessError as e:
            error_output = e.stderr.decode()
            parse_ansi_and_display(error_output, output_text)

    # Run in a separate thread
    thread = threading.Thread(target=execute, daemon=True)
    thread.start()

def make_section(root, title, padding=20):
    # Frame
    frame = tk.Frame(root)
    frame.pack(padx=20, anchor="n")

    # Title with a fixed width to align all titles
    title_label = tk.Label(frame, text=title, font=("Arial", 14), width=padding, anchor="w")
    title_label.pack(side=tk.LEFT, pady=10)

    return frame

def add_buttons(frame, buttons, padding=(3, 10)):
    for b in buttons:
        event_type = "action" if len(b) == 3 and b[2] == "action" else "command"

        button = None
        if event_type == "action":
            button = tk.Button(frame, text=b[0], command=b[1])
        else:
            if isinstance(b[1], str):
                button = tk.Button(frame, text=b[0], command=lambda cmd=b[1], is_sudo=(len(b) == 5 and b[4] == "sudo"): run_command(cmd, is_sudo))
            else:
                button = tk.Button(frame, text=b[0], command=lambda cmd=b[1]: run_command(cmd()))

        if len(b) == 4:
            if b[3] == "disabled":
                disable_button(button)
            
        button.pack(side=tk.LEFT, pady=padding[1], padx=padding[0])


def add_combobox(frame, values, default=None, change_callback=None, padding=((0, 20), 10)):
    combobox = ttk.Combobox(frame,
                            values=values,
                            state="readonly",
                            width=5,
                            justify="center")
    
    combobox.set(default or values[0])
    combobox.pack(side=tk.LEFT, pady=padding[1], padx=padding[0])
    if change_callback:
        combobox.bind("<<ComboboxSelected>>", lambda event: change_callback(combobox))
    return combobox


def add_checkbox(frame, text, default=False, change_callback=None, padding=(0, 10)):
    var = tk.BooleanVar()
    var.set(default)
    checkbox = tk.Checkbutton(frame, text=text, variable=var, command=lambda: change_callback(var))
    checkbox.pack(side=tk.LEFT, pady=padding[1], padx=padding[0])
    return var


def add_entry(frame, default=None, padding=(0, 10), width=20, change_callback=None):
    entry = tk.Entry(frame, width=width)
    entry.insert(0, default or "")
    entry.pack(side=tk.LEFT, pady=padding[1], padx=padding[0], ipady=3)
    entry.xview_moveto(1)

    if change_callback:
        entry.bind("<Return>", lambda event: change_callback(entry))

    return entry


def update_num_drones(combobox):
    global num_drones
    num_drones = int(combobox.get())
    
    sim_config_file = num_drones == 3 and "three_drones_aveiro.yml" or num_drones == 2 and "two_drones_aveiro.yml" or "single_drone_aveiro.yml"
    comboboxes[1].set(num_drones)
    config["services"]["simulator"]["file"] = sim_config_file
    global config_file
    update_config(config_file, config)

def update_mode(combobox, service):
    value = combobox.get()
    if service == "groundstation":
        global groundstation_mode
        groundstation_mode = value
        comboboxes[0].set(value)
    elif service == "backend":
        global dashboard_backend_mode
        dashboard_backend_mode = value
        comboboxes[2].set(value)
    elif service == "frontend":
        global dashboard_frontend_mode
        dashboard_frontend_mode = value
        comboboxes[3].set(value)

    config["services"][service]["mode"] = value
    global config_file
    update_config(config_file, config)

def update_enabled(checkbox, service):
    value = checkbox.get()
    config["services"][service]["enabled"] = value
    global config_file
    update_config(config_file, config)

    # uncheck the "all" checkbox
    checkbox_all.set(False)

def update_drone_id(entry):
    print("Updating drone ID")
    value = entry.get()
    config["services"]["drone"]["id"] = value
    global config_file
    update_config(config_file, config)


def toggle_all(checkbox, checkboxes):
    value = checkbox.get()
    for cb in checkboxes:
        cb.set(value)
    
    for i, service in enumerate(["simulator", "groundstation", "backend", "frontend", "drone"]):
        config["services"][service]["enabled"] = value

    global config_file
    update_config(config_file, config)

def disable_button(button):
    button.config(state=tk.DISABLED)
    button.bind("<Button-1>", lambda event: "break")  # Prevents the button from being clicked again

def enable_button(button):
    button.config(state=tk.NORMAL)
    button.unbind("<Button-1>")  # Re-enables the button click


def update_config(file, data):
    try:
        with open(file, "w") as f:
            json.dump(data, f, indent=4)
    except FileNotFoundError:
        print(f"File {file} not found.")


def load_config(file):
    try:
        with open(file) as f:
            data = json.load(f)
            return data
    except FileNotFoundError:
        print(f"File {file} not found.")
    except json.JSONDecodeError:
        print(f"File {file} is not a valid JSON file.")


def select_config_file():
    file = filedialog.askopenfilename()
    if file:
        global config_file
        config_file = file
        file_entry.delete(0, tk.END)
        file_entry.insert(0, file)
        file_entry.xview_moveto(1)
        
        set_defaults(file)

        return file
    

def set_values(data, file):
    services = data["services"]
    
    groundstation_mode = services["groundstation"]["mode"]
    comboboxes[0].set(groundstation_mode)
    sim_conf_file = services["simulator"]["file"]
    num_drones = 3 if sim_conf_file == "three_drones_aveiro.yml" else 2 if sim_conf_file == "two_drones_aveiro.yml" else 1
    comboboxes[1].set(num_drones)
    dashboard_backend_mode = services["backend"]["mode"]
    comboboxes[2].set(dashboard_backend_mode)
    dashboard_frontend_mode = services["frontend"]["mode"]
    comboboxes[3].set(dashboard_frontend_mode)
    drone_id = services["drone"]["id"]
    drone_entry.delete(0, tk.END)
    drone_entry.insert(0, drone_id)
    drone_entry.xview_moveto(1)

    enabled = [
        services["groundstation"]["enabled"],
        services["simulator"]["enabled"],
        services["backend"]["enabled"],
        services["frontend"]["enabled"],
        services["drone"]["enabled"]
    ]
    for i, cb in enumerate(checkboxes):
        cb.set(enabled[i])
    
    checkbox_all.set(all(enabled))
    update_config(file, data)


def set_defaults(file):
    global config
    global initial_config
    config = load_config(file)
    initial_config = copy.deepcopy(config)

    global num_drones
    global groundstation_mode
    global dashboard_backend_mode
    global dashboard_frontend_mode

    set_values(config, file)


config_file = "config.json"
initial_config = {}
config = {}

section_padding = 22

num_drones = 2
groundstation_mode = "prod"
dashboard_backend_mode = "prod"
dashboard_frontend_mode = "prod"
drone_id = "drone01"
drone_entry = None

comboboxes = []
checkboxes = []
checkbox_all = None
file_entry = None

def main():
    # Set up the main window
    root = tk.Tk()
    root.title("Fleetman GUI")

    # config file section
    global file_entry
    config_frame = make_section(root, "Config File", padding=0)
    file_entry = add_entry(config_frame, config_file, padding=(20, 10), width=40)
    add_buttons(config_frame, [
        ("Select", select_config_file, "action"),
    ])

    # Groundstation Section
    groundstation_frame = make_section(root, "Ground Station", padding=section_padding)
    comboboxes.append(add_combobox(groundstation_frame, ["prod", "dev"], "prod", lambda combobox: update_mode(combobox, "groundstation")))
    add_buttons(groundstation_frame, [
        ("Run", lambda: f"./run.sh -t groundstation {groundstation_mode}"),
        ("Restart", lambda: f"./stop.sh groundstation {groundstation_mode} && ./run.sh -t groundstation {groundstation_mode}"),
        ("Stop", lambda: f"./stop.sh groundstation {groundstation_mode}")
    ])
    checkboxes.append(add_checkbox(groundstation_frame, "", change_callback=lambda checkbox: update_enabled(checkbox, "groundstation")))

    # Simulator Section
    simulator_frame = make_section(root, "Simulator", padding=section_padding)
    comboboxes.append(add_combobox(simulator_frame, ["1", "2", "3"], "2", update_num_drones))
    add_buttons(simulator_frame, [
        ("Run", lambda: f"./run.sh -t simulator {num_drones}"),
        ("Restart", lambda: f"./stop.sh simulator && ./run.sh -t simulator {num_drones}"),
        ("Stop", "./stop.sh simulator")
    ])
    checkboxes.append(add_checkbox(simulator_frame, "", change_callback=lambda checkbox: update_enabled(checkbox, "simulator")))

    # Dashboard Backend Section
    dashboard_backend_frame = make_section(root, "Dash Backend", padding=section_padding)
    comboboxes.append(add_combobox(dashboard_backend_frame, ["prod", "dev"], "prod", lambda combobox: update_mode(combobox, "backend")))
    add_buttons(dashboard_backend_frame, [
        ("Run", f"./run.sh -t backend {dashboard_backend_mode}"),
        ("Restart", f"./stop.sh backend {dashboard_backend_mode} && ./run.sh -t backend {dashboard_backend_mode}"),
        ("Stop", f"./stop.sh backend {dashboard_backend_mode}")
    ])
    checkboxes.append(add_checkbox(dashboard_backend_frame, "", change_callback=lambda checkbox: update_enabled(checkbox, "backend")))

    # Dashboard Frontend Section
    dashboard_frontend_frame = make_section(root, "Dash Frontend", padding=section_padding)
    comboboxes.append(add_combobox(dashboard_frontend_frame, ["prod", "dev"], "prod", lambda combobox: update_mode(combobox, "frontend")))
    add_buttons(dashboard_frontend_frame, [
        ("Run", f"./run.sh -t frontend {dashboard_frontend_mode}"),
        ("Restart", f"./stop.sh frontend {dashboard_frontend_mode} && ./run.sh -t frontend {dashboard_frontend_mode}"),
        ("Stop", f"./stop.sh frontend {dashboard_frontend_mode}")
    ])
    checkboxes.append(add_checkbox(dashboard_frontend_frame, "", change_callback=lambda checkbox: update_enabled(checkbox, "frontend")))

    # Drone Section
    drone_frame = make_section(root, "Drone", padding=section_padding-9)
    global drone_entry
    drone_entry = add_entry(drone_frame, drone_id, padding=(12, 10), width=10, change_callback=update_drone_id)
    drone_entry.config(justify="right")
    add_buttons(drone_frame, [
        ("Set", lambda: update_drone_id(drone_entry), "action"),
        ("Run", "./run.sh -t drone", "command", "enabled", "sudo"),
        ("Restart", "./stop.sh drone && ./run.sh -t drone", "command", "disabled"),
        ("Stop", "./stop.sh drone", "command", "disabled")
    ])
    checkboxes.append(add_checkbox(drone_frame, "", change_callback=lambda checkbox: update_enabled(checkbox, "drone")))

    # Selected Section
    selected_frame = tk.Frame(root)
    selected_frame.pack(padx=20, anchor="n")
    add_buttons(selected_frame, [
        ("Reset Config", lambda: set_values(initial_config, config_file), "action"),
        ("Run Selected", "./run.sh -t"),
        ("Restart Selected", "./stop.sh && ./run.sh -t"),
        ("Stop Selected", "./stop.sh")
    ])

    global checkbox_all
    checkbox_all = add_checkbox(selected_frame, "", change_callback=lambda checkbox: toggle_all(checkbox, checkboxes), padding=((2, 0), 0))

    # Load the configuration
    set_defaults(file=config_file)

    # Output Text Box
    global output_text
    output_text = tk.Text(root, height=5, width=63, padx=10, pady=10)
    output_text.pack(pady=10)

    # Command Dump Text Box
    global command_dump_text
    command_dump_text = tk.Text(root, height=5, width=63, padx=10, pady=10)
    command_dump_text.pack(pady=5)

    # Start the GUI event loop
    root.mainloop()


if __name__ == "__main__":
    main()
