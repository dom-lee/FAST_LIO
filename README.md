## Setup Instructions
1. Clone the Repository
```bash
git clone git@github.com:dom-lee/FAST_LIO.git
git submodule update --init --recursive
```

2. Build the Docker Image
```bash
docker build -t fastlio2 .
```

3. Run the Docker Container
```bash
./run_docker.sh
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