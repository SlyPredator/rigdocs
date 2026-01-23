# ROVIO
This project uses a Pixhawk and a Jetson Nano. weee

## 1. Prerequisites

- Docker (follow Step 1 from the Kalibr guide and verify it is working)

## 2. Building the Docker image

### a. step a

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
- ROVIO binaries built for ROS1

This may take a while to complete, so be patient.

If the command fails with an error, if you see `403 Forbidden` in the error trace, do so and so.
