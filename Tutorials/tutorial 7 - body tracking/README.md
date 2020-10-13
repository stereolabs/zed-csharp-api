# Tutorial 7: Body tracking

This tutorial shows how to use the body tracking feature.
It will draw skeletons on detected people and display the image on a OpenGL.net window.

We assume that you have read the tutorial 1 and successfully opened your ZED.

# Code overview
## Create a camera

As with previous tutorial, we create, configure and open the ZED. Here we show how to set a resolution and a framerate. 

If you want to use the SVO input, you need to specify both the Input type and the SVO file path  : 

```
 init_params.inputType = sl.INPUT_TYPE.INPUT_TYPE_SVO;
 init_params.pathSVO =  "D:/mySVOfile.svo";
 ```

 ## Enable Body tracking

 Once the camera is opened, we must enable body tracking feature to retrieve the detected persons and their 3D skeleton.
 If we want body tracking, we also need to enable position tracking in order to be able to track objects with a moving camera.

 ```
Quaternion quat = Quaternion.Identity;
Vector3 vec = Vector3.Zero;

// Enable Tracking
err = zedCamera.EnableTracking(ref quat, ref vec);
if (err != ERROR_CODE.SUCCESS)
    Environment.Exit(-1);


// Enable Object Detection
dll_ObjectDetectionParameters object_detection_parameters = new dll_ObjectDetectionParameters();
object_detection_parameters.detectionModel = sl.DETECTION_MODEL.HUMAN_BODY_ACCURATE;
object_detection_parameters.enableObjectTracking = true;
err = zedCamera.EnableObjectsDetection(ref object_detection_parameters);
if (err != ERROR_CODE.SUCCESS)
    Environment.Exit(-1);
```

The only difference with using the Object detection feature is the **Detection Model** parameter. It has to be set to either HUMAN_BODY_FAST or HUMAN_BODY_ACCURATE in order to enable body tracking.