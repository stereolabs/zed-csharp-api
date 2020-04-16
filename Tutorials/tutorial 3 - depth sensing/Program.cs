//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============
using System;
using System.Runtime.InteropServices;
using System.Numerics;

namespace sl
{
    class Program
    {
        static void Main(string[] args)
        {
            // Set Initialization parameters
            InitParameters init_params = new InitParameters();
            init_params.resolution = RESOLUTION.HD720;
            init_params.cameraFPS = 60;
            init_params.coordinateUnit = UNIT.METER;
            init_params.coordinateSystem = COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP;
            init_params.depthMode = DEPTH_MODE.PERFORMANCE;

            ZEDCamera zedCamera = new ZEDCamera();
            // Open the camera
            ERROR_CODE err = zedCamera.Init(ref init_params);
            if (err != ERROR_CODE.SUCCESS)
                Environment.Exit(-1);

            // Initialize runtime parameters and frame counter
            int i = 0;
            RuntimeParameters runtimeParameters = new RuntimeParameters();

            // Get resolution of camera
            uint mWidth = (uint)zedCamera.ImageWidth;
            uint mHeight = (uint)zedCamera.ImageHeight;

            // Initialize the Mat that will contain the Point Cloud
            ZEDMat depth_map = new ZEDMat();
            depth_map.Create(mWidth, mHeight,MAT_TYPE.MAT_32F_C1, MEM.MEM_CPU); // Mat need to be created before use.

            // To avoid Nan Values, set to FILL to remove holes.
            runtimeParameters.sensingMode = SENSING_MODE.FILL;

            while (i < 1000)
            {
                if (zedCamera.Grab(ref runtimeParameters) == ERROR_CODE.SUCCESS)
                {
                    // Get the pose of the left eye of the camera with reference to the world frame
                    zedCamera.RetrieveMeasure(depth_map, MEASURE.XYZRGBA);

                    // Display the X, Y , Z at the center of the image
                    if (i % 10 == 0)
                    {
                        float4 xyz_value;
                        depth_map.GetValue((int)mWidth / 2, (int)mHeight / 2, out xyz_value, MEM.MEM_CPU);
                        Console.WriteLine("Depth At Image Center : (" + xyz_value.x + "," + xyz_value.y + "," + xyz_value.z + ")");
                    }

                    // increment frame count
                    i++;
                }
            }

            // Disable positional tracking and close the camera
            zedCamera.DisableTracking("");
            zedCamera.Close();
        }
    }
}