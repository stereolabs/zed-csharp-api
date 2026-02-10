# Stereolabs.Net - C#/.NET API

This package lets you use the ZED stereo camera in C#. The C# API is a wrapper around the ZED SDK which is written in C++ optimized code. We make the ZED SDK accessible from external C# code.


## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers)
- For more information, read the ZED [API documentation](https://www.stereolabs.com/docs/api/python/) or our [Community page](https://community.stereolabs.com)

## Prerequisites

- **Windows** 10 64bits or later
- Visual Studio 2017 with C# extensions
- Cmake 3.23 at least
- [C wrapper](https://github.com/stereolabs/zed-c-api) of the ZED SDK
- [ZED SDK **5.2**](https://www.stereolabs.com/developers/release/) and its dependency ([CUDA](https://developer.nvidia.com/cuda-downloads))

## From NuGet

The C# API is available as a Nuget Package on Nuget.org.

| Package | Description | Link |
|---------|-------------|------|
|**Stereolabs.zed**| .NET Wrapper that imports the functions from the interface | [![NuGet version](https://badge.fury.io/nu/Stereolabs.zed.svg)](https://badge.fury.io/nu/Stereolabs.zed) |

## Add the package to your Project

To add a NuGet pacakge to your project, please follow this [documentation](https://learn.microsoft.com/en-us/nuget/quickstart/install-and-use-a-package-in-visual-studio).

# From Sources

It is also possible to build the wrapper from the sources. The C# wrapper made of two libraries :

- The C# wrapper called Stereolabs.zed.dll. It defines all the functions of the ZED SDK in C#.
- The C wrapper called zed-c-api. This native dll binds all the C# functions to the ZED SDK (written in C++). This wrapper is available here : https://github.com/stereolabs/zed-c-api

### Build the C# wrapper

- Open cmake-gui.
- In “Where is the source code“, enter the path of the project folder where the CMakeLists.txt is located.
- In “Where to build the binaries“, enter the previous path and add: /build.
- Click on [*Configure*].
- A dialog window asks you if CMake can create the “build” folder. Say yes.
- Another dialog window will ask you to specify a generator for your project. Choose [Visual Studio](https://visualstudio.microsoft.com/downloads/)  in **Win64** and click on [*Finish*].
- CMake may take a few seconds to configure the project.
- Click on [*Generate*] to build the Visual Studio project files.

- CMake has now generated your project in the build directory.
You can directly open the solution by clicking on [*Open Project*] or by closing the cmake-gui window and opening the **build** folder.

- A Visual Studio solution has been generated. Open **Stereolabs.ZED.sln** and set it in `Release` mode.

#### Build the C wrapper

The Csharp wrapper is using the [C wrapper](https://github.com/stereolabs/zed-c-api) to interface with the ZED SDK.
Therefore, you also need to build it following the same step as before.

## Add the wrapper to your Project

- Open yourproject in Visual studio
- Right click on your project in the Solution Window and select `Add ` ->  `Project Reference`.
- Click on  `Browse ` and select the Stereolabs.ZED.dll you just created before. Add it in your project.
- For the moment you also need to manually copy the c wrapper (sl_zed_c.dll) at the same location, in yourproject.

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
            // Create the camera
            Camera zedCamera = new Camera(0);
            // Create default configuration parameters
            InitParameters init_params = new InitParameters();
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

## Support
If you need assistance go to our Community site at https://community.stereolabs.com/
