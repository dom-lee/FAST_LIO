## Setup Instructions
1. Clone the Repository
```bash
mkdir -p FAST_LIO_ws/src
cd FAST_LIO_ws/src
git clone --recurse-submodules git@github.com:dom-lee/FAST_LIO.git
```

2. Build the Docker Image
```bash
./scripts/build_docker.sh
```

3. Run the Docker Container
```bash
./scripts/run_docker.sh
```

3. Build FAST-LIO
```bash
catkin_make
source devel/setup.bash
```

## Running FAST-LIO
To launch FAST-LIO with a sample configuration (e.g., Ouster 64):
```bash
roslaunch fast_lio mapping_ouster64.launch
```

## Record output
1. Set proper `T_IMU_BASE` in `scripts/odom_logger.py`

2. process FAST_LIO and start odom_logger
```bash
./scripts/process_record.sh <bag_dir> <launch_file>
```