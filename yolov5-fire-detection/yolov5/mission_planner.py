import requests
import textwrap
import math
import argparse
import subprocess
import os
class MissionPlanner:
    def __init__(self):
        self.gs_url = "http://localhost:8001"
        self.fire_coords = []
        self.debug = True
        self.video_source = None

    def log(self, message):
        if self.debug:
            print(f"[DEBUG] {message}")
    
    def detect_fire(self, video_source):
        self.log(f"Starting fire detection on: {video_source}")
        
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        # Convert to absolute paths
        weights_path = os.path.abspath(os.path.join(base_dir, "model", "yolov5s_best.pt"))
        video_path = os.path.abspath(os.path.join(base_dir, video_source))
        
        self.log(f"Using weights from: {weights_path}")
        self.log(f"Using video source: {video_path}")
        
        if not os.path.exists(weights_path):
            self.log(f"Error: Weights file not found at {weights_path}")
            return False
            
        if not os.path.exists(video_path):
            self.log(f"Error: Video file not found at {video_path}")
            return False

        try:
            result = subprocess.run(
                [
                    "python", "detect.py",
                    "--weights", weights_path,
                    "--source", video_path,
                    "--img", "640",
                    "--conf", "0.4"
                ],
                capture_output=True,
                text=True,
                cwd=os.path.dirname(os.path.abspath(__file__))  # Run from yolov5 directory
            )
            
            self.log(f"Return code: {result.returncode}")
            self.log(f"stdout: {result.stdout}")
            self.log(f"stderr: {result.stderr}")
            
            if "FIRE_DETECTED" in result.stdout.upper():
                self.log("Fire detected in the video")
                return True
            return False
                
        except Exception as e:
            self.log(f"Error during detection: {str(e)}")
            return False
    
    def generate_mission_1(self):
        if not self.detect_fire(self.video_source):
            print("No fire detected. Exiting mission generation.")
            return None
        self.fire_coords = (40.63242, -8.660162) 
        detected_lat, detected_lon = self.fire_coords

        #Grid generation
        grid_size = 3 # 3x3 grid
        spacing_m = 20 # meters between waypoints
        delta_lat = spacing_m / 111320
        delta_lon = spacing_m / (111320 * math.cos(math.radians(detected_lat)))

        waypoints = []
        offset = grid_size // 2
        for z in range(-offset, offset + 1):
            for m in range(-offset, offset + 1):
                if z == 0 and m == 0: # skip the center waypoint (directly above the fire)
                    continue
                wp_lat = detected_lat + z * delta_lat
                wp_lon = detected_lon + m * delta_lon
                waypoints.append((wp_lat, wp_lon))

        #Generate mission script
        waypoints_groovy = ",\n    ".join(
            [f"[lat: {lat:.6f}, lon: {lon:.6f}]" for lat, lon in waypoints]
        )

        mission_script = textwrap.dedent(f"""\
        /*
        * multi_waypoint_scout.groovy
        * Sends one drone to multiple waypoints and returns home.
        */

        drone = assign 'drone01'
        arm drone
        takeoff drone, 5.meters
        takeoff_alt = drone.position.alt

        waypoints = [
            {waypoints_groovy}
        ]

        for (wp in waypoints) {{
            move drone, lat: wp.lat, lon: wp.lon, alt: takeoff_alt
        }}
        home drone
        """)
        return mission_script
    
    def generate_mission_2(self):
        if not self.detect_fire(self.video_source):
            print("No fire detected. Exiting mission generation.")
            return None
        fire_lat, fire_lon = (40.63242, -8.660162)
        
        side_m = 40
        half_side = side_m / 2
        delta_lat = half_side / 111320
        delta_lon = half_side / (111320 * math.cos(math.radians(fire_lat)))

        corners = [
            (fire_lat + delta_lat, fire_lon - delta_lon),  # NW
            (fire_lat + delta_lat, fire_lon + delta_lon),  # NE
            (fire_lat - delta_lat, fire_lon + delta_lon),  # SE
            (fire_lat - delta_lat, fire_lon - delta_lon),  # SW
            (fire_lat + delta_lat, fire_lon - delta_lon)  # NW again
        ]

        drones = ["drone01", "drone03"]
        drone_waypoints = {drone: [] for drone in drones}
        for i, corner in enumerate(corners):
            drone = drones[i % len(drones)]
            drone_waypoints[drone].append(corner)

        mission_lines = []
        for drone in drones:
            mission_lines.append(f"{drone} = assign '{drone}'")
            mission_lines.append(f"arm {drone}")
            mission_lines.append(f"takeoff {drone}, 5.meters")
            mission_lines.append(f"takeoff_alt_{drone} = {drone}.position.alt")
            mission_lines.append(f"waypoints_{drone} = [")
            mission_lines.extend(
                [f"    [lat: {lat:.6f}, lon: {lon:.6f}]," for lat, lon in drone_waypoints[drone]]
            )
            mission_lines.append("]")
            mission_lines.append(f"for (wp in waypoints_{drone}) {{")
            mission_lines.append(f"    move {drone}, lat: wp.lat, lon: wp.lon, alt: takeoff_alt_{drone}")
            mission_lines.append("}")
            mission_lines.append(f"home {drone}\n")

        mission_script = textwrap.dedent(f"""\
        /*
        * perimeter_patrol.groovy
        * Two drones patrol a square perimeter around a detected fire.
        */
        """) + "\n".join(mission_lines)


        return mission_script
    
    def generate_mission_3(self):
        if not self.detect_fire(self.video_source):
            print("No fire detected. Exiting mission generation.")
            return None
        detected_lat, detected_lon = (40.63242, -8.660162)
        grid_size = 3
        spacing_m = 20

        delta_lat = spacing_m / 111320
        delta_lon = spacing_m / (111320 * math.cos(math.radians(detected_lat)))

        waypoints_grid = []
        offset = grid_size // 2
        for z in range(-offset, offset + 1):
            row = []
            for m in range(-offset, offset + 1):
                if z == 0 and m == 0:  # skip the center
                    continue
                wp_lat = detected_lat + z * delta_lat
                wp_lon = detected_lon + m * delta_lon
                row.append((wp_lat, wp_lon))
            waypoints_grid.append(row)
        
        # Assign each row to a drone (no overlap)
        drone_waypoints = {
            "drone01": waypoints_grid[0],
            "drone02": waypoints_grid[1],
            "drone03": waypoints_grid[2],
        }

        # Build Groovy mission script for all drones
        mission_script = textwrap.dedent("""\
        /*
        * multi_drone_multi_waypoint_scout.groovy
        * Sends three drones to different waypoints and returns them home.
        */
        """)

        for drone, waypoints in drone_waypoints.items():
            waypoints_groovy = ",\n    ".join(
                [f"[lat: {lat:.6f}, lon: {lon:.6f}]" for lat, lon in waypoints]
            )
            mission_script += textwrap.dedent(f"""
            {drone} = assign '{drone}'
            arm {drone}
            takeoff {drone}, 5.meters
            takeoff_alt_{drone} = {drone}.position.alt

            waypoints_{drone} = [
                {waypoints_groovy}
            ]

            for (wp in waypoints_{drone}) {{
                move {drone}, lat: wp.lat, lon: wp.lon, alt: takeoff_alt_{drone}
            }}
            home {drone}
            """)
        return mission_script

    def upload_mission(self, mission_script, mission_type):
        mission_path = f"mission{mission_type}_temp.groovy"

        with open(mission_path, "w") as f:
            f.write(mission_script)
        with open(mission_path, "rb") as f:
            file_content = f.read()
            headers = {"Accept": "application/json"}
            mission_response = requests.post(f"{self.gs_url}/mission", data=file_content, headers=headers)
            if mission_response.ok:
                print("Mission launched:", mission_response.text)
            else:
                print(f"Mission launch failed: {mission_response.status_code} {mission_response.text}")


    def execute_mission(self, mission_type, video_source):
        self.video_source = video_source
        if mission_type == 1:
            mission_script = self.generate_mission_1()
            if mission_script:
                self.upload_mission(mission_script, mission_type)
        elif mission_type == 2:
            mission_script = self.generate_mission_2()
            if mission_script:
                self.upload_mission(mission_script, mission_type)
        elif mission_type == 3:
            mission_script = self.generate_mission_3()
            if mission_script:
                self.upload_mission(mission_script, mission_type)
        else :
            raise ValueError("Unsupported mission type")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", type=int,choices=[1,2,3], default=1, help="Mission type to execute")
    parser.add_argument("--video_source", type=str, default="input.mp4", help="Path to video source for fire detection")
    args = parser.parse_args()

    planner = MissionPlanner()
    planner.execute_mission(args.mission, args.video_source)

if __name__ == "__main__":
    main()