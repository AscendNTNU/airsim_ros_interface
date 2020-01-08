# AirSim ros interface

This is a simple interface for retrieving sensor data from AirSim for mission 9. The sensors extracted as per now vision, lidar and imu. There are two versions of this interface: one for C++ and one for Python. Through tests the Python interface tends to freeze the Unreal editor far less and in general be more stable and also faster somehow. This might be an underlying issue with RPC/the AirSim plugin/Unreal Editor.

There exists a ros plugin given from Microsoft, but it is quite complicated for simple use and doesn't comply well with Ascend's TF tree. It also, being written in C++, occasionally have the same symptoms as the C++-implementation in this package: UE freezes suddenly. From my experience it is really hard to work with it and interfacing with the API directly has been far easier. That is why this package exists.  

*Note that threading is not used per now, this should be considered in the future.*

## Installation

You have to specify where the AirSim directory is located in the CMakeLists.txt.

## How to launch 

Simply do: 

```bash
roslaunch airsim_ros_interface main.launch
```

There are some paramteres in the launch file that can be modified. 


## Quirks with the AirSim plugin

Through the development of this plugin there has been made several interesting remarks:

- The use of multiple clients to receive multiple types of data is preferred instead of using one client to retrieve e.g. imu, lidar and vision data (have not found any documentation on why this is the case as per january 2020). This causes UE to freeze far less.
- The lidar data is given in world frame, not local frame.
- Requesting images from UE seems to happen on the main rendering thread in UE, thus making it hard to achieve high frequencies for vision data. There is also use of a for loop in the request code which can be replaced by a memcpy. This does improve performance some. The underlying code should be replaced for these ![lines](https://github.com/microsoft/AirSim/blob/0d7cf52517b72a50a1d94e426d0ba06dfa744017/Unreal/Plugins/AirSim/Source/RenderRequest.cpp#L109).

```cpp
results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height * 4, false);
if (params[i]->compress)
    UAirBlueprintLib::CompressImageArray(results[i]->width, results[i]->height, results[i]->bmp, results[i]->image_data_uint8);
else {
    uint8* ptr = results[i]->image_data_uint8.GetData();
    std::memcpy(ptr, results[i]->bmp.GetData(), result[i]->bmp.GetTypeSize() * result[i]->bmp.Num());
}
```

Be aware that this enables the alpha channel as well, because it copies all of the data in the bitmap from result. If this is done, we have to make changes in the image processing from the ros interface. We have to enable the alpha in the encoding and declare that the step is the width times four.

```python
msg = Image() 
msg.encoding = "bgra8"
msg.height = image_height 
msg.width = image_width
msg.data = img_rgb_string
msg.is_bigendian = 0
msg.step = msg.width * 4
```