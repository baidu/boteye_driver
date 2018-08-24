![boteye](https://image.ibb.co/b1N40z/page_1_1.png)

## boteye_sensor
This is Baidu's boteye driver for multiple types of camera modules.

## Required dependencies (built from source): ##
 You can git clone the XP_3rdparty repo, which has all the source code of needed 3rdparty dependencies by other repos, but that's an overkill. You can just download the Opencv.

- Opencv
  ```
  cd ~/Development/tracking-docker/3rdparty/opencv-3.0.0
  mkdir build; cd build
  cmake ..
  make -j4
  sudo make install
  ```

## Optional dependencies (built from source): ##
 To run an application of the driver, glog and gflags are needed. You can get them from the XP_3rdparty repo, or use apt-get:
  ```
  apt-get install -y --allow-unauthenticated \
    libgflags-dev \
    libgoogle-glog-dev
  ```

## Setup steps: ##
  ```
  mkdir ~/Development; cd ~/Development
  git clone [the git repo of driver]
  cd driver
  mkdir build; cd build;
  cmake ..
  make -j4
  ```

  Currently, we only enable pre-commit hook for cpplint.To enable the git hook, you need to do the following (You only need to config git hook once per fresh clone):
  ```
  cd ~/Development/driver/utils
  source config_githook.sh
  ```

## Optional setup steps: ##
 Build a simple binary that depends on the driver library. The binary can show and record the raw image and imu data from the sensor.
 ```
 cd app/
 mkdir build; cd build;
 cmake ..
 make -j4
 ```
