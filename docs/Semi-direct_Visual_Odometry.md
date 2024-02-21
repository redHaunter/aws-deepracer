# Semi-Direct Visual Odometry (SVO)

## SLAM Methods

## Sparse Image Alignment, Relaxation and Refinement


# Device Flow Instructions

First, you have to setup the ros1_bridge from [ROS2to1-bridge](https://github.com/redHaunter/ROS2to1-bridge/tree/main).

Clone the package from [rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open). Then, in dependencies.yaml file replace "git@" with "https://", the previous one is deprecated. Now you can build the package following the instructions provided in the package's repository.

Then you have to calibrate your sensors, in our case, stereo cameras and stereo cameras with the IMU, so you can use the SVO pro porperly. Find require steps for camera and IMU calibration [kalibr](https://github.com/ethz-asl/kalibr). Here is our calibration output:

```
cameras:
- T_B_C:
    cols: 4
    data: [-0.9997154797074667, 0.018309448041398897, 0.015288026219584892, 0.041699373266834094,
      0.009752163869159555, -0.2711739283805381, 0.962480958703359, 0.2591315238872347,
      0.021768209231364438, 0.9623562046763597, 0.27091821715745407, -0.03590864692908663,
      0.0, 0.0, 0.0, 1.0]
    rows: 4
  calib_date: 0
  camera:
    distortion:
      parameters:
        cols: 1
        data: [-0.392262, 0.129396, -0.005213, 0.006695]
        rows: 4
      type: radial-tangential
    image_height: 480
    image_width: 640
    intrinsics:
      cols: 1
      data: [485.829096, 483.165143, 306.863116, 240.282215]
      rows: 4
    label: cam0
    line-delay-nanoseconds: 0
    type: pinhole
  description: cam0/image_raw
  serial_no: 0
- T_B_C:
    cols: 4
    data: [-0.9995027379528209, 0.022608198561762657, 0.021980586493689076, 0.10154729340333707,
      0.015106770500020126, -0.26854484614247576, 0.9631487170190145, 0.2588359171349027,
      0.027677830656941946, 0.9630018353918677, 0.26806977211543204, -0.03514434116260867,
      0.0, 0.0, 0.0, 1.0]
    rows: 4
  calib_date: 0
  camera:
    distortion:
      parameters:
        cols: 1
        data: [-0.400533, 0.140885, -0.005469, 0.006470]
        rows: 4
      type: radial-tangential
    image_height: 480
    image_width: 640
    intrinsics:
      cols: 1
      data: [489.782852, 486.803095, 306.292638, 238.961187]
      rows: 4
    label: cam1
    line-delay-nanoseconds: 0
    type: pinhole
  description: cam1/image_raw
  serial_no: 0
label: deepracer_stereo.yaml

imu_params:
  acc_max: 176.0
  delay_imu_cam: 0.0
  g: 9.8082
  imu_rate: 60.0
  max_imu_delta_t: 0.01
  omega_max: 7.8
  sigma_acc_bias_c: 0.0001718984099214701
  sigma_acc_c: 0.0025723124515807405
  sigma_integration: 0.0
  sigma_omega_bias_c: 5.555781590328033e-06
  sigma_omega_c: 0.0005395057151492824

imu_initialization:
  acc_bias: [0.0, 0.0, 0.0]
  acc_bias_sigma: 0.0025723124515807405
  omega_bias: [0.0, 0.0, 0.0]
  omega_bias_sigma: 0.0005395057151492824
  velocity: [0.0, 0.0, 0.0]
  velocity_sigma: 1.0
```


Finally you can use this calibaraion file in yaml format similar to the one used in the package's launch files.

> just be sure that bridged topics from ROS2 to ROS1 are working properly