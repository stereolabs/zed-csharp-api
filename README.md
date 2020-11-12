# Stereolabs.Net - C#/.NET API 

This repository shows how to use the ZED SDK functionalities in C#.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/release/).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/docs/api/index.html).

## Prerequisites

- **Windows** 10 64bits or later
- Visual Studio 2017 with C# extensions
- Cmake 3.8 at least (Support of C#)
- [ZED SDK **3.3**](https://www.stereolabs.com/developers/release/) and its dependency ([CUDA](https://developer.nvidia.com/cuda-downloads))

## NuGet

The C# API uses two packages that are published to Nuget.org.

| Package | Description | Link |
|---------|-------------|------|
|**sl_zed_interface**| C interface of the ZED SDK | [![NuGet version](https://badge.fury.io/nu/sl_zed_interface.svg)](https://badge.fury.io/nu/sl_zed_interface) |
|**Stereolabs.zed**| .NET Wrapper that imports the functions from the interface | [![NuGet version](https://badge.fury.io/nu/Stereolabs.zed.svg)](https://badge.fury.io/nu/Stereolabs.zed) |

### Build the tutorials

1. Enter the tutorials folder and generate the Tutorials solution using cmake.
**Use Visual studio 2017** (at least) compiler in **x64** mode.

![Cmake](./Documentation/img/cmake_settings.jpg)

2. Press configure and generate.
3. Open Tutorials.zed.sln solution and build the solution.

### Usage

Here is an example of the .NET API. 
```C#
using System.Numerics;

namespace sl
{
    class Program
    {
        static void Main(string[] args)
        {
            // Set configuration parameters
            InitParameters init_params = new InitParameters();
            init_params.resolution = RESOLUTION.HD1080;
            init_params.cameraFPS = 30;
            Camera zedCamera = new Camera(0);
            // Open the camera
            ERROR_CODE err = zedCamera.Open(ref init_params);
            if (err != ERROR_CODE.SUCCESS)
                Environment.Exit(-1);

            // Get camera information (serial number)
            int zed_serial = zedCamera.GetZEDSerialNumber();
            Console.WriteLine("Hello! This is my serial number: " + zed_serial);
            Console.ReadLine();

            zedCamera.Close();
        }
    }
}
```

### Deployment

When the program is built, *Stereolabs.zed.dll* and *sl_zed_interface.dll* will be automatically downloaded into the build folder.

When deploying the application, make sure that *sl_zed_interface.dll* and *Stereolabs.zed.dll* are packaged with the executable files, and shipped on a target PC that has the proper ZED SDK version installed.
