# Kalibr 

This is a guide to get started with Kalibr. This particular guide will focus on using Kalibr with Intel Realsense D435i.

## 1. Docker Setup

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh

sudo groupadd docker
sudo usermod -aG docker $USER

sudo systemctl enable docker.service
sudo systemctl enable containerd.service

newgrp docker
```

Once this is done, please test if you can run `docker ps -a`. If it returns an error, run this once again:

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## 2. Building the Docker image

### a. Clone the Github repository

```bash
git clone https://github.com/SlyPredator/kalibr
cd kalibr
```

### b. Build the Docker container

```{tip}
Please adjust `-j6` in the Dockerfile, based on your `echo $(nproc)` output.
For example, if it is 8, set it to 5 or 6, if it is 20, set it to 12 or 16.

This will help your system to not crash while building the Docker container.
```

```bash
docker compose up -d
```

This will build the Docker container with:
- A local bind mount of `data` folder to `data` inside the container
- Intel Realsense SDK for ROS1
- Kalibr binaries built for ROS1

This may take a while to complete, so be patient.

If the command fails with an error, if you see `403 Forbidden` in the error trace, add the following line after the first `RUN` command in the Dockerfile:

```docker
RUN echo 'Acquire::http::User-Agent "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:90.0) Gecko/20100101 Firefox/90.0";' > /etc/apt/apt.conf.d/99useragent
```

Now run `docker compose up -d` again, this should fix your issue.


## 3. Running the container

Firstly run `xhost +local:docker` to grant screen display permissions to Docker.

Also connect your Intel Realsense camera to a USB port on your system, make sure it is a USB 3.2 port, verify with `lsusb -t` where you should see an entry under Bus 1 or 2 with several `5000M`s. This verifies that the camera is connected to a high-speed port and can transfer data without latency.

Make sure you're still inside the `kalibr` directory. Run `docker start -ai kalibr` to start the container.

Open up few other terminals and type `docker exec -it kalibr bash` to open multiple terminals inside the container.

```{note}
No need to run `docker start -ai kalibr` multiple times, if one terminal has started it, the other terminals can use `docker exec -it kalibr bash` to start a new terminal inside the already running container.
```

### Verify Intel Realsense is detected inside the container

Run this command to verify the Realsense camera: `realsense-viewer`
Once it loads up, toggle to the 2D section (button is on top right) and toggle the 3 buttons one by one on the left side to verify each of the modules are working.

### Verify ROS1 and Kalibr binaries work

Once in the container, run `ls` to find out the directories in your current folder. If you are not in `catkin_ws`, please do navigate there and source the binaries built:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

This sources ROS1 binaries and the Kalibr binaries that you have built during the process of building the container.

```{tip}
For your own sanity, it is recommended to add the sourcing lines to your `.bashrc` to avoid sourcing them repeatedly.

`echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
```

## 4. Recording the camera and IMU topics for calibration

Once all the preliminary checks from above has been done, you can proceed to calibration.

### Starting up the Realsense publisher

```bash
roslaunch realsense2_camera rs_camera.launch     enable_gyro:=true     enable_accel:=true     unite_imu_method:="linear_interpolation"     enable_sync:=true
```

In a separate terminal within the container, verify that the following topics are being published:
- *_/camera/imu_* - `rostopic echo /camera/imu` or `rostopic hz /camera/imu` 
- */camera/color/image_raw_* - `rostopic echo /camera/color/image_raw` or `rostopic hz /camera/color/image_raw` 

### Starting the rosbag recording

You will be using ROS1's internal `rosbag` utility to record a selection of topics for later use.

In another terminal that you have opened inside the container, start the rosbag to record the incoming topics for camera data (`/camera/color/image_raw`) and IMU data (`/camera/imu`):

```bash
cd ~/data
rosbag record -o my_test_bag /camera/imu /camera/color/image_raw
```

This will ensure the topics mentioned are captured as long as they are publishing valid values.

This will create a `my_test_bag.bag` file which has the recorded data.


## 5. Running Kalibr to obtain calibration parameters

Provided you have a valid bag file, you will be calibrating
- Camera
- Camera + IMU

in that order.

For the below steps, we are using an [Aprilgrid](https://drive.google.com/file/d/14dY7z8pDb2iEBdveTviDXsoi5H9AaQP1/view?usp=sharing) printed on an A4 sheet.

### Camera calibration

You do not need the camera connected for this step. The bag file you have recorded earlier is enough.

To calibrate the camera, you need only one file:
- The `.yaml` file containing the details of the AprilGrid you recorded the bag on

This file should be in the same directory as the bag file, i.e., `data` folder inside the container

`april_6x6.yaml`
```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.025          # MEASURE THIS: Edge length of one black tag in meters
tagSpacing: 0.3        # Ratio: (gap between tags) / tagSize
```

Then run Kalibr on the bag file:

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --target april_6x6.yaml \
  --bag my_test_bag.bag \
  --models pinhole-radtan \
  --topics /camera/color/image_raw
```

This will provide you several files:
- **report-cam-%BAGNAME%.pdf**: Report in PDF format. Contains all plots for documentation.
- **results-cam-%BAGNAME%.txt**: Result summary as a text file.
- **camchain-%BAGNAME%.yaml**: Results in YAML format.

### Camera + IMU calibration

You do not need the camera connected for this step. The bag file you have recorded earlier is enough.

To calibrate the camera, you need only one thing:
- The `.yaml` file containing the details of the AprilGrid you recorded the bag on
- The `.yaml` file containing the details (intrinsics) of the IMU

These files should be in the same directory as the bag file, i.e., `data` folder inside the container

`april_6x6.yaml`

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.025          # MEASURE THIS: Edge length of one black tag in meters
tagSpacing: 0.3        # Ratio: (gap between tags) / tagSize
```

`imu.yaml`

```yaml
rostopic: /camera/imu
update_rate: 200.0 # Hz
accelerometer_noise_density: 0.289
accelerometer_random_walk: 0.000455
gyroscope_noise_density: 0.00302
gyroscope_random_walk: 0.0000229
```

Then run Kalibr on the bag file:

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag my_test_bag.bag \
  --cam day1-camchain.yaml \
  --imu imu.yaml \
  --target april_6x6.yaml \
```

This will provide you several files:

- **report-imucam-%BAGNAME%.pdf**: Report in PDF format. Contains all plots for documentation.
- **results-imucam-%BAGNAME%.txt**: Result summary as a text file.
- **camchain-imucam-%BAGNAME%.yaml**: Results in YAML format. This file is based on the input camchain.yaml with added transformations (and optionally time shifts) for all cameras with respect to the IMU. 

## 6. Results

You have successfully obtained the calibration parameters for your camera and IMU.

Make sure that:
- Reprojection error in camera results (ideal values of error: under 0.5)
- Accelerometer, gyroscope error (as close to 0 as possible)

are below the ranges mentioned in the brackets above. This means your calibration process went well and you can use the obtained parameters for reliable use in other applications.
