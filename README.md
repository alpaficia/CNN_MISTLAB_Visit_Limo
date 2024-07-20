# Simple 'cooking recipe' for our limos.


### docker install and setup

### Git clone the ros2 repo for limo

```
# Create local workspace
mkdir -p ~/agx_workspace
cd ~/agx_workspace
# Clone the project
git clone https://github.com/agilexrobotics/limo_ros2.git src

mv ~/agx_workspace/src/.devcontainer ~/agx_workspace
```

### pull the docker image online:

```
docker pull lagrangeluo/limo_ros2:v1
```

Please check if the docker image is available by:
```
docker image list
```

If you see:
```
agilex@agilex-desktop:~$ docker image list
REPOSITORY                                                         TAG        IMAGE ID       CREATED          SIZE
lagrangeluo/limo_ros2                                              v1         224540b5b168   11 minutes ago   7.57GB
```
Or something similar, that means we can take next step.

### Run the docker container
Before you run the container, you need to check out XAUTH value available, by doing:
```
echo $XAUTH
```
That is the key to transfer the video steam though X11 from the docker image to your local machine. If you see nothing, 
you need to export the link to your XAUTH value by:
```
export XAUTH=~/.Xauthority
```
Meanwhile, you also need to allow the user has the permissions to access the X11 by:
```
```
After these steps, you can run:
```
docker run --network=host \
      -d \
      -v=/dev:/dev \
      --privileged \
      --device-cgroup-rule="a *:* rmw" \
      --volume=/tmp/.X11-unix:/tmp/.X11-unix \
      -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
      --runtime nvidia \
      --gpus=all \
      -w=/workspace \
      --name limo_dev \
      -e LIBGL_ALWAYS_SOFTWARE="1" \
      -e DISPLAY=${DISPLAY} \
      --restart=always \
      -v ~/agx_workspace:/home/limo_ros2_ws \
      lagrangeluo/limo_ros2:v1
```
**--network=host:** This option allows the container to share the network namespace with the host, meaning the container can directly access network interfaces on the host.

**-d:** Detached mode, which means the container runs in the background.

**-v=/dev:/dev:** This mounts the host's /dev directory into the container's /dev directory, giving the container direct access to device files on the host.

**--privileged:** This option gives the container full access to the host system, effectively disabling containerization in terms of security isolation.

**--device-cgroup-rule="a *:* rmw":** This allows the container to access devices based on certain rules within the cgroups.

**--volume=/tmp/.X11-unix:/tmp/.X11-unix:** This mounts the X11 Unix socket from the host into the container, allowing GUI applications running in the container to connect to the host's X server.

**-v $XAUTH:$XAUTH** -e XAUTHORITY=$XAUTH: This mounts the X authentication file into the container and sets the XAUTHORITY environment variable to point to it.

**--runtime nvidia --gpus=all:** This option is specific to NVIDIA Docker runtime, allowing the container to utilize NVIDIA GPUs. It specifies that all available GPUs should be accessible to the container.

**-w=/workspace:** This sets the working directory inside the container to /workspace.

**--name limo_dev:** This sets the name of the container to "limo_dev".

**-e LIBGL_ALWAYS_SOFTWARE="1" -e DISPLAY=${DISPLAY}:** These set environment variables inside the container, LIBGL_ALWAYS_SOFTWARE to "1" and DISPLAY to the value of the DISPLAY environment variable on the host, likely to enable software rendering and set up X11 display forwarding, respectively.

**--restart=always:** This instructs Docker to always restart the container automatically if it stops.

**-v ~/agx_workspace:/home/limo_ros1_ws:** This mounts the host's "~/agx_workspace" directory into the container's "/home/limo_ros1_ws" directory.

After you successfully create the container, you can use ros2 foxy inside it. Meanwhile, if you build your image 
locally, please change your docker image name at the end.

## First Init
You need to get the ID of your container by:
```
docker ps
```
Once you got the ID, you can get into the docker container by:
```
docker exec -it YOUR_ID_HERE /bin/bash
```

Change the `agx_workspace` folder to ours:

`git clone https://github.com/alpaficia/CNN_MISTLAB_Visit_Limo.git`

You can find a `agx_workspace` folder inside, use that folder please.

For the first init, you need to do the colcon build:
```
cd /home/limo_ros2_ws
colcon build
source install/setup.bash
```
Then we can use the optitrack:

`ros2 run limo_optitrack limo_pos_publisher.py`


If there is any new troubles, please feel free to ask Dong Wang (dong.wang950305@gmail.com)

Have fun :)
