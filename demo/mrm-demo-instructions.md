## Preparation
### Docker Install
ref: https://docs.docker.com/engine/install/ubuntu/

```
#### Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

#### Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

#### Install docker
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

#### Add user group
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

If it's your first time to install docker, you might have to reboot the system once to apply the user group settings.
```
sudo reboot
```

## Getting scenarios
Download scenario file and map files from https://drive.google.com/file/d/1httYKgKpVUCIpKuX3Vgk_5wfIgr9dv8s/view?usp=sharing and extract the zip file.

Download a custom launch file for running with actuation modules.
```
wget https://gist.githubusercontent.com/mitsudome-r/8525fdf8a9dac7ee901178d8ee571619/raw/817df9660f104084e1f9187eb763bad2cc19047d/control.launch.xml
```

## Terminal 1 (Visualization Container)

```
docker run --name visualizer --rm --net=host -p 6080:6080 \
-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
-e ROS_DOMAIN_ID=2 \
-e USE_SIM_TIME=true \
ghcr.io/mitsudome-r/mrm-demo:visualizer-amd64
```

Open browser and access 
http://127.0.0.1:6080/vnc.html?resize=scale&password=openadkit&autoconnect=true

## Terminal 2 (Planning-Control Container) Before Scenario
Make sure that the path given after `-v` is the path to the fault injection folder that you downloaded in preparation. The instruction assumes that you downloaded the fault injection folder under $HOME/Downloads/fault_injection and the launch file under $HOME/Downloads/control.launch.xml.

If you want to run without the actuation module, remove `-v  $HOME/Downloads/control.launch.xml:/opt/autoware/share/tier4_control_launch/launch/control.launch.xml` from the command.

```
docker run -it --name planning --rm --net=host \
  -e ROS_DOMAIN_ID=2 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v $HOME/Downloads/fault_injection_v2:/autoware/scenario-sim \
  -v $HOME/Downloads/control.launch.xml:/opt/autoware/share/tier4_control_launch/launch/control.launch.xml \
  ghcr.io/mitsudome-r/mrm-demo:universe-before-arm64 \
  bash -c 'ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/autoware/scenario-sim/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit scenario_simulation:=true rviz:=false use_sim_time:=false'
```


## Terminal 1 (Planning-Control Container) After Scenario
Make sure that the path given after `-v` is the path to the fault injection folder that you downloaded in preparation. The instruction assumes that you downloaded the fault injection folder under $HOME/Downloads/fault_injection and the launch file under $HOME/Downloads/control.launch.xml.

If you want to run without the actuation module, remove `-v  $HOME/Downloads/control.launch.xml:/opt/autoware/share/tier4_control_launch/launch/control.launch.xml` from the command.

```
docker run -it --name planning --rm --net=host \
-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
-e ROS_DOMAIN_ID=2 \
-v $HOME/Downloads/fault_injection_v2:/autoware/scenario-sim \
-v  $HOME/Downloads/control.launch.xml:/opt/autoware/share/tier4_control_launch/launch/control.launch.xml \
ghcr.io/mitsudome-r/mrm-demo:universe-after-arm64 \
bash -c 'ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/autoware/scenario-sim/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit scenario_simulation:=true rviz:=false use_sim_time:=false'
```

## Terminal 3 (Simulator Container)
```
docker run -it --name simulator --rm --net=host \
-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
-e ROS_DOMAIN_ID=2 \
-e SCENARIO=/autoware/scenario-sim/mrm-scenario.yaml \
-v $HOME/Downloads/fault_injection_v2:/autoware/scenario-sim \
ghcr.io/mitsudome-r/mrm-demo:scenario-simulator-arm64
```


# Troubleshooting
## Cannot stop planning container
If you cannot stop the planning container with CTRL+C, run the following commands to stop the container: 
```
# find the id for the planning container
docker ps

# stop the container (modify "ID" with the actual string of ID you find from the above command)
docker container stop ID 

# you can also stop all containers with the following command
docker stop $(docker ps -aq)

```

# Creating the images locally
You can create your own image by the following commands:

## Before scenario
```
git clone github.com:mitsudome-r/autoware.git -b mrm-demo-before

cd autoware

./docker/build.sh
```

## After Scenario
```
git clone github.com:mitsudome-r/autoware.git -b mrm-demo

cd autoware

./docker/build.sh
```
