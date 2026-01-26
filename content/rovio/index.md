# ROVIO

This is a guide to get started with ROVIO. This particular guide will focus on using ROVIO with Intel Realsense D435i.

```{warning}
This article is still in an experimental and live stage, further information or corrections will be added as and when verified to be true.
```

## 1. Prerequisites

- Docker (follow Step 1 from the Kalibr guide and verify it is working)

## 2. Building the Docker image

### a. Clone the required repositories

We need to clone the ROVIO and Kindr repositories locally first, so we can build it inside the container later.

```bash
mkdir -p ~/isro_ws/src/isro_rovio
cd ~/isro_ws/src/isro_rovio
rm -rf rovio lightweight_filtering kindr
git clone --recursive https://github.com/ethz-asl/rovio.git
git clone https://github.com/ethz-asl/kindr.git
```

### b. Build the Docker container

Create a Dockerfile in the same directory as `rovio` folder:

```{tip}
Please adjust `-j6` in the Dockerfile, based on your `echo $(nproc)` output.
For example, if it is 8, set it to 5 or 6, if it is 20, set it to 12 or 16.

This will help your system to not crash while building the binaries.
```

`Dockerfile`

```{dropdown} Click to see Dockerfile
:animate: fade-in
```docker
FROM osrf/ros:noetic-desktop-full

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    git wget autoconf automake nano \
    python3-dev python3-pip python3-scipy python3-matplotlib \
    ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    doxygen cmake libfreeimage-dev libglew-dev freeglut3-dev\
    curl gnupg2 lsb-release libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
    python3-catkin-tools python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/

# Install Intel RealSense SDK
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y \
    librealsense2-utils librealsense2-dev ros-noetic-realsense2-camera \
    && rm -rf /var/lib/apt/lists/

ENV WORKSPACE /catkin_ws
RUN mkdir -p $WORKSPACE/src

RUN cd $WORKSPACE && \
    catkin init && \
    catkin config --extend /opt/ros/noetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY . $WORKSPACE/src/isro_rovio

RUN sed -i 's/CV_GRAY2RGB/cv::COLOR_GRAY2RGB/g' $WORKSPACE/src/isro_rovio/rovio/include/rovio/ImgUpdate.hpp
RUN sed -i 's/${catkin_LIBRARIES}/${catkin_LIBRARIES} GLEW GL/g' $WORKSPACE/src/isro_rovio/rovio/CMakeLists.txt
RUN sed -i 's/set(ROVIO_CHECK_JACOBIANS 1)/set(ROVIO_CHECK_JACOBIANS 0)/g' $WORKSPACE/src/isro_rovio/rovio/CMakeLists.txt

RUN cd $WORKSPACE && \
    catkin build rovio kindr -j6 --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE/devel/setup.bash" >> ~/.bashrc

WORKDIR $WORKSPACE
ENTRYPOINT ["/bin/bash", "-c", "source $WORKSPACE/devel/setup.bash && /bin/bash"]
```

```{note}
The first two `sed` commands are to replace two references inside the libraries that are broken and causes the build to fail otherwise.
```


Similarly, a `docker-compose.yml`:

```{dropdown} Click to see docker-compose.yml
:animate: fade-in

```docker
version: '3.8'

services:
  isro_rovio:
    build: .
    container_name: isro_rovio
    network_mode: host
    privileged: true
    volumes:
      - .:/catkin_ws/src/isro_rovio
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /dev:/dev
      - /dev/dri:/dev/dri
      - /run/udev:/run/udev:ro
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    devices:
      - "/dev/bus/usb:/dev/bus/usb"
    stdin_open: true
    tty: true
    command: /bin/bash
```

Finally, build the container:

```bash
docker compose up -d
```

This will build the Docker container with:
- A local bind mount of your `isro_rovio` folder to `catkin_ws/src/isro_rovio` inside the container
- Intel Realsense SDK for ROS1
- ROVIO binaries built for ROS1

This may take a while to complete, so be patient.

If the command fails with an error, if you see `403 Forbidden` in the error trace, add the following line after the second `RUN` command in the Dockerfile:

```docker
RUN echo 'Acquire::http::User-Agent "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:90.0) Gecko/20100101 Firefox/90.0";' > /etc/apt/apt.conf.d/99useragent
```

Now run `docker compose up -d` again, this should fix your issue.

## 3. Running the container

Firstly run `xhost +local:docker` to grant screen display permissions to Docker.

Also connect your Intel Realsense camera to a USB port on your system, make sure it is a USB 3.2 port, verify with `lsusb -t` where you should see an entry under Bus 1 or 2 with several `5000M`s. This verifies that the camera is connected to a high-speed port and can transfer data without latency.

Make sure you're still inside the `isro_ws/src/isro_rovio` directory. Run `docker start -ai isro_rovio` to start the container.

Open up few other terminals and type `docker exec -it isro_rovio bash` to open multiple terminals inside the container.

```{note}
No need to run `docker start -ai isro_rovio` multiple times, if one terminal has started it, the other terminals can use `docker exec -it isro_rovio bash` to start a new terminal inside the already running container.
```

### Verify Intel Realsense is detected inside the container

Run this command to verify the Realsense camera: `realsense-viewer`
Once it loads up, toggle to the 2D section (button is on top right) and toggle the 3 buttons one by one on the left side to verify each of the modules are working.

Once that is done, source the install files:
```bash
source devel/setup.bash 
```

## 4. Running ROVIO

Once all of the above checks are done, we can now proceed to use ROVIO itself.

### Starting up the Realsense publisher

```bash
roslaunch realsense2_camera rs_camera.launch \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true \
    enable_accel:=true \
    enable_infra1:=true \
    enable_infra2:=true \
    initial_reset:=true
```

### Making the necessary files for ROVIO to run

Navigate inside the `catkin_ws/src/isro_rovio/rovio/cfg` folder and create the below two files:

```bash
cd ~/catkin_ws/src/isro_rovio/rovio/cfg
touch rovio.info d435i.yaml
```

```{warning}
These files are not finalized. This serves as a self-reminder to update them.
```

Edit the files using `nano <file_name>`: (Shortcuts to save and exit are: `Ctrl+O` -> `Enter` -> `Ctrl+X`)

`d435i.yaml`

```yaml
image_width: 640
image_height: 480
camera_name: cam0
camera_matrix:
  rows: 3
  cols: 3
  data: [604.212768554688, 0.0, 339.295349121094,
         0.0, 603.471130371094, 257.16796875,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
```

`rovio.info` 

```{dropdown} Click to see rovio.info
:animate: fade-in

(Click the copy button on top right)

```text
; You can partially override parameter set in this file by creating your own subset of parameter in a separate info-file and include it with:
; #include "/home/user/workspace/rovio/cfg/rovio_custom.info"
Common
{
	doVECalibration true;		Should the camera-IMU extrinsics be calibrated online
	depthType 1;				Type of depth parametrization (0: normal, 1: inverse depth, 2: log, 3: hyperbolic)
	verbose false;				Is the verbose active
}

Camera0
{
    CalibrationFile ;

    qCM_x  0.0005546;
    qCM_y  0.0037831;
    qCM_z  -0.0000678;
    qCM_w  0.9999927;

    MrMC_x  0.020400632172823;
    MrMC_y -0.00495732482522726;
    MrMC_z -0.0115533517673612;
}


Init
{
    State
    {
        pos_0 0;			X-entry of initial position (world to IMU in world) [m]
        pos_1 0;			Y-entry of initial position (world to IMU in world) [m]
        pos_2 0;			Z-entry of initial position (world to IMU in world) [m]
        vel_0 0;			X-entry of initial velocity (robocentric, IMU) [m/s]
        vel_1 0;			Y-entry of initial velocity (robocentric, IMU) [m/s]
        vel_2 0;			Z-entry of initial velocity (robocentric, IMU) [m/s]
        acb_0 0;			X-entry of accelerometer bias [m/s^2]
        acb_1 0;			Y-entry of accelerometer bias [m/s^2]
        acb_2 0;			Z-entry of accelerometer bias [m/s^2]
        gyb_0 0;			X-entry of gyroscope bias [rad/s]
        gyb_1 0;			Y-entry of gyroscope bias [rad/s]
        gyb_2 0;			Z-entry of gyroscope bias [rad/s]
        att_x 0;			X-entry of initial attitude (IMU to world, Hamilton)
        att_y 0;			Y-entry of initial attitude (IMU to world, Hamilton)
        att_z 0;			Z-entry of initial attitude (IMU to world, Hamilton)
        att_w 1;			W-entry of initial attitude (IMU to world, Hamilton)
    }
    Covariance
    {
        pos_0 0.0001;		X-Covariance of initial position [m^2]
        pos_1 0.0001;		Y-Covariance of initial position [m^2]
        pos_2 0.0001;		Z-Covariance of initial position [m^2]
        vel_0 1.0;			X-Covariance of initial velocity [m^2/s^2]
        vel_1 1.0;			Y-Covariance of initial velocity [m^2/s^2]
        vel_2 1.0;			Z-Covariance of initial velocity [m^2/s^2]
        acb_0 4e-4;			X-Covariance of initial accelerometer bias [m^2/s^4]
        acb_1 4e-4;			Y-Covariance of initial accelerometer bias [m^2/s^4]
        acb_2 4e-4;			Z-Covariance of initial accelerometer bias [m^2/s^4]
        gyb_0 3e-4;			X-Covariance of initial gyroscope bias [rad^2/s^2]
        gyb_1 3e-4;			Y-Covariance of initial gyroscope bias [rad^2/s^2]
        gyb_2 3e-4;			Z-Covariance of initial gyroscope bias [rad^2/s^2]
        vep 0.0001;			Covariance of initial linear camera-IMU extrinsics, same for all entries [m^2]
        att_0 0.1;			X-Covariance of initial attitude [rad^2]
        att_1 0.1;			Y-Covariance of initial attitude [rad^2]
        att_2 0.1;			Z-Covariance of initial attitude [rad^2]
        vea 0.01;			Covariance of initial rotational camera-IMU extrinsics, same for all entries [rad^2]
    }
}
ImgUpdate
{
	updateVecNormTermination 1e-4;
	maxNumIteration 20;
    doPatchWarping false;										Should the patches be warped
    doFrameVisualisation false;									Should the frames be visualized
    visualizePatches true;										Should the patches be visualized
    useDirectMethod true;										Should the EKF-innovation be based on direct intensity error (o.w. reprojection error)
    startLevel 2;												Highest patch level which is being employed (must be smaller than the hardcoded template parameter)
    endLevel 1;													Lowest patch level which is being employed
    nDetectionBuckets 100;										Number of discretization buckets used during the candidates selection
    MahalanobisTh 9.21;											Mahalanobis treshold for the update, 5.8858356
    UpdateNoise
    {
        pix 2;													Covariance used for the reprojection error, 1/focal_length is roughly 1 pixel std [rad] (~1/f ~ 1/400^2 = 1/160000)
        int 400;												Covariance used for the photometric error [intensity^2]
    }
    initCovFeature_0 0.5;										Covariance for the initial distance (Relative to initialization depth [m^2/m^2])
    initCovFeature_1 1e-5;										Covariance for the initial bearing vector, x-component [rad^2]
    initCovFeature_2 1e-5;										Covariance for the initial bearing vector, y-component [rad^2]
    initDepth 0.5;												Initial value for the initial distance parameter
    startDetectionTh 0.8;										Threshold for detecting new features (min: 0, max: 1)
    scoreDetectionExponent 0.25;								Exponent used for weighting the distance between candidates
    penaltyDistance 100;										Maximal distance which gets penalized during bucketing [pix]
    zeroDistancePenalty 100;									Penalty for zero distance (max: nDetectionBuckets)
    statLocalQualityRange 10;									Number of frames for local quality evaluation
    statLocalVisibilityRange 100;								Number of frames for local visibility evaluation
    statMinGlobalQualityRange 100;								Minimum number of frames for obtaining maximal global quality
    trackingUpperBound 0.9;										Threshold for local quality for min overall global quality
    trackingLowerBound 0.8;										Threshold for local quality for max overall global quality
    minTrackedAndFreeFeatures 0.75;								Minimum of amount of feature which are either tracked or free
    removalFactor 1.1;											Factor for enforcing feature removal if not enough free
    minRelativeSTScore 0.75;									Minimum relative ST-score for extracting new feature patch
    minAbsoluteSTScore 5.0;										Minimum absolute ST-score for extracting new feature patch
    minTimeBetweenPatchUpdate 1.0;								Minimum time between new multilevel patch extrection [s]
    fastDetectionThreshold 5;									Fast corner detector treshold while adding new feature
    alignConvergencePixelRange 10;								Assumed convergence range for image alignment (gets scaled with the level) [pixels]
    alignCoverageRatio 2;										How many sigma of the uncertainty should be covered in the adaptive alignement
    alignMaxUniSample 1;										Maximal number of alignment seeds on one side -> total number of sample = 2n+1. Carefull can get very costly if diverging!
    patchRejectionTh 50.0;										If the average itensity error is larger than this the feauture is rejected [intensity], if smaller 0 the no check is performed
    alignmentHuberNormThreshold 10;								Intensity error threshold for Huber norm (enabled if > 0)
    alignmentGaussianWeightingSigma -1;							Width of Gaussian which is used for pixel error weighting (enabled if > 0)
    alignmentGradientExponent 0.0;								Exponent used for gradient based weighting of residuals
    useIntensityOffsetForAlignment true;						Should an intensity offset between the patches be considered
    useIntensitySqewForAlignment true;							Should an intensity sqew between the patches be considered
    removeNegativeFeatureAfterUpdate true;						Should feature with negative distance get removed
    maxUncertaintyToDepthRatioForDepthInitialization 0.3;		If set to 0.0 the depth is initialized with the standard value provided above, otherwise ROVIO attempts to figure out a median depth in each frame
    useCrossCameraMeasurements true;							Should cross measurements between frame be used. Might be turned of in calibration phase.
    doStereoInitialization true;								Should a stereo match be used for feature initialization.
    noiseGainForOffCamera 10.0;									Factor added on update noise if not main camera
    discriminativeSamplingDistance 0.02;						Sampling distance for checking discriminativity of patch (if <= 0.0 no check is performed).
    discriminativeSamplingGain 1.1;								Gain for threshold above which the samples must lie (if <= 1.0 the patchRejectionTh is used).
    MotionDetection
    {
    	isEnabled 0;											Is the motion detection enabled
	    rateOfMovingFeaturesTh 0.5;								Amount of feature with motion for overall motion detection
	    pixelCoordinateMotionTh 1.0;							Threshold for motion detection for patched [pixels]
	    minFeatureCountForNoMotionDetection 5;					Min feature count in frame for motion detection
	}
    ZeroVelocityUpdate
    {
        UpdateNoise
        {
            vel_0 0.01;											X-Covariance of zero velocity update [m^2/s^2]
            vel_1 0.01;											Y-Covariance of zero velocity update [m^2/s^2]
            vel_2 0.01;											Z-Covariance of zero velocity update [m^2/s^2]
        }
        MahalanobisTh0 7.689997599999999;						Mahalanobid distance for zero velocity updates
        minNoMotionTime 1.0;									Min duration of no-motion
        isEnabled 0;											Should zero velocity update be applied, only works if MotionDetection.isEnabled is true
    }
}
Prediction
{
    PredictionNoise
    {
        pos_0 1e-4;								X-covariance parameter of position prediction [m^2/s]
        pos_1 1e-4;								Y-covariance parameter of position prediction [m^2/s]
        pos_2 1e-4;								Z-covariance parameter of position prediction [m^2/s]
        vel_0 4e-6;								X-covariance parameter of velocity prediction [m^2/s^3]
        vel_1 4e-6;								Y-covariance parameter of velocity prediction [m^2/s^3]
        vel_2 4e-6;								Z-covariance parameter of velocity prediction [m^2/s^3]
        acb_0 1e-8;								X-covariance parameter of accelerometer bias prediction [m^2/s^5]
        acb_1 1e-8;								Y-covariance parameter of accelerometer bias prediction [m^2/s^5]
        acb_2 1e-8;								Z-covariance parameter of accelerometer bias prediction [m^2/s^5]
        gyb_0 3.8e-7;							X-covariance parameter of gyroscope bias prediction [rad^2/s^3]
        gyb_1 3.8e-7;							Y-covariance parameter of gyroscope bias prediction [rad^2/s^3]
        gyb_2 3.8e-7;							Z-covariance parameter of gyroscope bias prediction [rad^2/s^3]
        vep 1e-8;								Covariance parameter of linear extrinsics prediction [m^2/s]
        att_0 7.6e-7;							X-Covariance parameter of attitude prediction [rad^2/s]
        att_1 7.6e-7;							Y-Covariance parameter of attitude prediction [rad^2/s]
        att_2 7.6e-7;							Z-Covariance parameter of attitude prediction [rad^2/s]
        vea 1e-8;								Covariance parameter of rotational extrinsics prediction [rad^2/s]
        dep 0.0001;							Covariance parameter of distance prediction [m^2/s]
        nor 0.00001;								Covariance parameter of bearing vector prediction [rad^2/s]
    }
    MotionDetection
    {
	    inertialMotionRorTh 0.1;				Treshold on rotational rate for motion detection [rad/s]
	    inertialMotionAccTh 0.5;				Treshold on acceleration for motion detection [m/s^2]
	}
}
PoseUpdate
{
    UpdateNoise
    {
        pos_0 0.01;							X-Covariance of linear pose measurement update [m^2]
        pos_1 0.01;							Y-Covariance of linear pose measurement update [m^2]
        pos_2 0.01;							Z-Covariance of linear pose measurement update [m^2]
        att_0 0.01;							X-Covariance of rotational pose measurement update [rad^2]
        att_1 0.01;							Y-Covariance of rotational pose measurement update [rad^2]
        att_2 0.01;							Z-Covariance of rotational pose measurement update [rad^2]
    }
	init_cov_IrIW 1;							Covariance of initial pose between inertial frames, linear part [m^2]
	init_cov_qWI 1;								Covariance of initial pose between inertial frames, rotational part [rad^2]
	init_cov_MrMV 1;							Covariance of initial pose between inertial frames, linear part [m^2]
	init_cov_qVM 1;								Covariance of initial pose between inertial frames, rotational part [rad^2]
    pre_cov_IrIW 1e-4;							Covariance parameter of pose between inertial frames, linear part [m^2/s]
    pre_cov_qWI 1e-4;							Covariance parameter of pose between inertial frames, rotational part [rad^2/s]
    pre_cov_MrMV 1e-4;							Covariance parameter of pose between inertial frames, linear part [m^2/s]
    pre_cov_qVM 1e-4;							Covariance parameter of pose between inertial frames, rotational part [rad^2/s]
    MahalanobisTh0 12.6511204;					Mahalonobis distance treshold of pose measurement
	doVisualization true;
	enablePosition true;						Should the linear part be used during the update
	enableAttitude true;						Should the rotation part be used during the update (e.g. set to fals eif feeding GPS-measurement)
	noFeedbackToRovio true;						By default the pose update is only used for aligning coordinate frame. Set to false if ROVIO should benefit frome the posed measurements.
	doInertialAlignmentAtStart true;			Should the transformation between I and W be explicitly computed and set with the first pose measurement.
	timeOffset 0.0;								Time offset added to the pose measurement timestamps
    useOdometryCov false;                    Should the UpdateNoise position covariance be scaled using the covariance in the Odometry message
    qVM_x 0;									X-entry of quaterion representing IMU to reference body coordinate frame of pose measurement (Hamilton)
    qVM_y 0;									Y-entry of quaterion representing IMU to reference body coordinate frame of pose measurement (Hamilton)
    qVM_z 0;									Z-entry of quaterion representing IMU to reference body coordinate frame of pose measurement (Hamilton)
    qVM_w 1;									W-entry of quaterion representing IMU to reference body coordinate frame of pose measurement (Hamilton)
    MrMV_x 0;									X-entry of vector representing IMU to reference body coordinate frame of pose measurement
    MrMV_y 0;									Y-entry of vector representing IMU to reference body coordinate frame of pose measurement
    MrMV_z 0;									Z-entry of vector representing IMU to reference body coordinate frame of pose measurement
    qWI_x 0;									X-entry of quaterion representing World to reference inertial coordinate frame of pose measurement (Hamilton)
    qWI_y 0;									Y-entry of quaterion representing World to reference inertial coordinate frame of pose measurement (Hamilton)
    qWI_z 0;									Z-entry of quaterion representing World to reference inertial coordinate frame of pose measurement (Hamilton)
    qWI_w 1;									W-entry of quaterion representing World to reference inertial coordinate frame of pose measurement (Hamilton)
    IrIW_x 0;									X-entry of vector representing World to reference inertial coordinate frame of pose measurement
    IrIW_y 0;									Y-entry of vector representing World to reference inertial coordinate frame of pose measurement
    IrIW_z 0;									Z-entry of vector representing World to reference inertial coordinate frame of pose measurement
}
VelocityUpdate
{
    UpdateNoise
    {
        vel_0 0.0001
        vel_1 0.0001
        vel_2 0.0001
    }
    MahalanobisTh0 7.689997599999999
    qAM_x 0
    qAM_y 0
    qAM_z 0
    qAM_w 1
}
```

### Running the ROVIO node

Finally, if all of the above have been done correctly, we should now be able get estimated poses from ROVIO. The below command starts up the ROVIO node:

```bash
roslaunch rovio rovio_node.launch \
    cam0_topic:=/camera/infra1/image_rect_raw \
    imu0_topic:=/camera/imu
```

You should now see a (yellow) Scene GUI window open up showing you the _pose updates_ from the camera.