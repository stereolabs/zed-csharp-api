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
            // Set configuration parameters
            InitParameters init_params = new InitParameters();
            init_params.resolution = RESOLUTION.HD1080;
            init_params.cameraFPS = 30;
            ZEDCamera zedCamera = new ZEDCamera(0);
            // Open the camera
            ERROR_CODE err = zedCamera.Init(ref init_params);
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