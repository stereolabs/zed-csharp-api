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
            init_params.resolution = RESOLUTION.HD2K;
            init_params.coordinateUnit = UNIT.METER;
            init_params.coordinateSystem = COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP;
            init_params.depthMode = DEPTH_MODE.PERFORMANCE;

            ZEDCamera zedCamera = new ZEDCamera(0);
            // Open the camera
            ERROR_CODE err = zedCamera.Init(ref init_params);
            if (err != ERROR_CODE.SUCCESS)
                Environment.Exit(-1);

            // Enable positional tracking
            Quaternion quat = Quaternion.Identity;
            Vector3 vec = Vector3.Zero;
            err = zedCamera.EnableTracking(ref quat, ref vec);
            if (err != ERROR_CODE.SUCCESS)
                Environment.Exit(-1);

            // Enable Object Detection
            dll_ObjectDetectionParameters object_detection_parameters = new dll_ObjectDetectionParameters();
            object_detection_parameters.detectionModel = sl.DETECTION_MODEL.MULTI_CLASS_BOX_ACCURATE;
            object_detection_parameters.enableObjectTracking = true;
            err = zedCamera.EnableObjectsDetection(ref object_detection_parameters);
            if (err != ERROR_CODE.SUCCESS)
                Environment.Exit(-1);

            // Create Runtime parameters
            RuntimeParameters runtimeParameters = new RuntimeParameters();

            // Create Object Detection frame handle (contains all the objects data)
            sl.ObjectsFrameSDK object_frame = new sl.ObjectsFrameSDK();

            // Create object detection runtime parameters (confidence, ...)
            dll_ObjectDetectionRuntimeParameters obj_runtime_parameters = new dll_ObjectDetectionRuntimeParameters();
            // To select a set of specific object classes:
            /*obj_runtime_parameters.objectClassFilter = new int[(int)OBJECT_CLASS.LAST];
            obj_runtime_parameters.objectClassFilter[(int)sl.OBJECT_CLASS.PERSON] = Convert.ToInt32(true);
            obj_runtime_parameters.objectClassFilter[(int)sl.OBJECT_CLASS.VEHICLE] = Convert.ToInt32(true);*/

            // default detection threshold, apply to all object class
            obj_runtime_parameters.detectionConfidenceThreshold = 20;
            // To set a specific threshold
            obj_runtime_parameters.object_confidence_threshold = new int[(int)OBJECT_CLASS.LAST];
            obj_runtime_parameters.object_confidence_threshold[(int)sl.OBJECT_CLASS.PERSON] = 35;
            obj_runtime_parameters.object_confidence_threshold[(int)sl.OBJECT_CLASS.VEHICLE] = 35;


            int i = 0;
            while (i < 1000)
            {
                if (zedCamera.Grab(ref runtimeParameters) == ERROR_CODE.SUCCESS)
                {
                     // Retrieve Objects from Object detection
                     err  = zedCamera.RetrieveObjectsDetectionData(ref obj_runtime_parameters, ref object_frame);

                     // Display the data each 10 frames
                     if (i % 10 == 0)
                     {
                         Console.WriteLine("Nb Objects Detection : " + object_frame.numObject);
                         for (int p = 0; p < object_frame.numObject; p++)
                         {
                             Console.WriteLine("Position of object " + p + " : " + object_frame.objectData[p].rootWorldPosition + "Tracked? : " + object_frame.objectData[p].objectTrackingState);
                         }
                     }
                    i++;
                }
            }

            // Disable object detection, positional tracking and close the camera
            zedCamera.DisableObjectsDetection();
            zedCamera.DisableTracking("");
            zedCamera.Close();
        }
    }
}