# How to run the mission planner

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


## Mission 3 (3 drones required)
Initialize the drone and the groundstation:
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