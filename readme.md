# How to run the mission planner
First of all build the containers:
```bash
cd scripts-main/
./build.sh simulator
./build.sh groundstation
```

## Mission 1 (only 1 drone required)
Initialize the drone and the groundstation:
```bash
cd scripts-main/
./run.sh -t simulator 1
./run.sh -t groundstation
```
Execute the mission planner: 
```bash
cd yolov5-fire-detection/yolov5/
python3 mission_planner.py --mission 1 --video_source input.mp4
```

## Mission 2 (1 or 2 drones required)
Initialize either 1 or more drones and the groundstation:
```bash
cd scripts-main/
./run.sh -t simulator <1-2>
./run.sh -t groundstation
```

Execute the mission planner:
```bash
cd yolov5-fire-detection/yolov5/
python3 mission_planner.py --mission 2 --video_source input.mp4
```

## Mission 3 (3 drones required)
Initialize the drones and the groundstation:
```bash
cd scripts-main/
./run.sh -t simulator 3
./run.sh -t groundstation
```
Execute the mission planner: 
```bash
cd yolov5-fire-detection/yolov5/
python3 mission_planner.py --mission 3 --video_source input.mp4
```

## How to stop the simulation
```bash
cd scripts-main/
./stop.sh
```

## Software to visualize drone movement
(QGroundControl)
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#download-and-install
