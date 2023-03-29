# Stereolabs ZED - Csharp API

These packages let you use the ZED stereo camera in C#.

## Getting Started

To start using the ZED SDK in C#, you need to install the following dependencies on your system:  

  - Visual Studio 2017 with C# extensions
  - Cmake 3.8 at least (Support of C#)
  - [ZED SDK](https://www.stereolabs.com/developers/release/) (see Installation section)

The C# API uses two libraries that is published to Nuget.org into a single package.

  | Package | Description | Link |
  |---------|-------------|------|
  |**Stereolabs.zed**| Contains the C wrapper of the ZED SDK and a .NET interface that imports the functions from the wrapper in C#     | [![NuGet version](https://badge.fury.io/nu/Stereolabs.zed.svg)](https://badge.fury.io/nu/Stereolabs.zed) |

In our [tutorials](https://github.com/stereolabs/zed-examples/tree/master/tutorials) and [samples](https://github.com/stereolabs/zed-examples), these packages are automatically downloaded when building the program.
However, it can be manually added to any C# project within Visual Studio. See the [Nuget documentation](https://docs.microsoft.com/en-us/nuget/consume-packages/install-use-packages-visual-studio).


## Building a sample

We will build a simple tutorial application **Hello ZED** using the ZED SDK and CMake. [CMake](https://cmake.org) is a cross-platform project generation tool. It provides an easy way to build project files that can be used in the compiler environment of your choice. For example, a CMake script can produce Visual Studio project and solution files.

- Download the [ZED Examples](https://github.com/stereolabs/zed-examples/archive/master.zip) sample code available on our [GitHub](https://github.com/stereolabs/zed-examples) page. You can also browse our GitHub for additional plugins and sample codes.

- Open cmake-gui.
- In “Where is the source code“, enter the path of the project folder where the CMakeLists.txt is located.
- In “Where to build the binaries“, enter the previous path and add: /build.
- Click on [*Configure*].

![](../../images/csharp/cmake_configure.png)

- A dialog window asks you if CMake can create the “build” folder. Say yes.
- Another dialog window will ask you to specify a generator for your project. Choose [Visual Studio](https://visualstudio.microsoft.com/downloads/)  in **Win64** and click on [*Finish*].

![](../../images/csharp/cmake_settings.jpg)

- CMake may take a few seconds to configure the project.
- Click on [*Generate*] to build the Visual Studio project files.

![](../../images/csharp/cmake_generate.PNG)

- CMake has now generated your project in the build directory.
You can directly open the solution by clicking on [*Open Project*] or by closing the cmake-gui window and opening the **build** folder.

- A Visual Studio solution has been generated. Open **Hello_ZED.sln** and set it in `Release` mode.

![](../../images/csharp/set_release.png)

- To run the builds from the *Build* menu or from keyboard shortcuts, set the `Hello_ZED` target as the startup project. In the Solution Explorer,
right-click the Hello_ZED solution and click **Set As Startup Project**

![](../../images/csharp/startup_project.png)

- You can now edit and compile your program in the Visual Studio IDE. Hit the `Ctrl+F5` key to launch the program.

- When the program is built, the C# interface (*Stereolabs.zed.dll*) and the C wrapper (*sl_zed_c.dll*) are automatically added into the build folder, next to the executable file.

![](../../images/csharp/nugget_install.png)

- When deploying the application, make sure that *sl_zed_c.dll* and *Stereolabs.zed.dll* are packaged with the executable files, and shipped on a target PC that has the proper ZED SDK version installed.
