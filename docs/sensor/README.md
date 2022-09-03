# Sersor

---

## Types

* Camera: https://cv.cgabc.xyz/cam/

* Radar
    - LiDAR

* IMU

* Sonar

* GNSS


## Pros. vs Cons.

Todo


## Capture Data

### Save Images and IMUs in ASL format from Phone

Init

```cpp
std::string root_dir = "/sdcard/cg/data_asl/";
std::string cam0_dir = root_dir + "cam0/";
std::string imu0_path = root_dir + "imu0.csv";

std::ofstream file_imu0;
file_imu0.open(imu0_path);
```

Save Images

```cpp
// Image
long long ts_cam0 = round(timestamp_cam0);
char buffer[250];
sprintf(buffer, "%lld", (ts_cam0));
cv::imwrite(cam0_dir+buffer+".png", image);
```

Normalize Depth Image for Display

```cpp
cv::Mat mat_depth8;
{
    double min, max;
    cv::minMaxLoc(imgD, &min, &max);
    imgD.convertTo(mat_depth8, CV_8UC1, 255.0/(max-min), -255.0*min/(max-min));
}
```

Save IMUs

```cpp
// IMU
long long ts_imu0 = round(timestamp_imu0);
char buffer[250];
sprintf(buffer, "%lld", (ts_imu0));
file_imu0 << ts_imu0 << ",";
file_imu0 << std::setprecision(16) 
          << gyrX << "," << gyrY << "," << gyrZ << "," 
          << accX << "," << accY << "," << accZ << endl;
```


## Calibration

* Spatial Calibration

* Temporal Calibration
