//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

using System.Runtime.InteropServices;
using System.Numerics;
using System;
using System.Collections.Generic;

/// \defgroup Video_group Video Module
/// \defgroup Depth_group Depth Sensing Module
/// \defgroup Core_group Core Module
/// \defgroup SpatialMapping_group Spatial Mapping Module
/// \defgroup PositionalTracking_group Positional Tracking Module
/// \defgroup Object_group Object Detection Module
/// \defgroup Body_group Body Tracking Module
/// \defgroup Sensors_group Sensors Module
/// \defgroup Fusion_group Fusion Module

namespace sl
{
    public class ZEDCommon
    {
        public const string NameDLL = "sl_zed_c.dll";
    }

    /// <summary>
    /// Constant for plugin. Should not be changed
    /// </summary>
    public enum Constant
    {
        MAX_OBJECTS = 75,
        /// <summary>
        /// Maximum number of chunks. It's best to get relatively few chunks and to update them quickly.
        /// </summary>
        MAX_SUBMESH = 1000,
        /// <summary>
        /// Max size of trajectory data (number of frames stored)
        /// </summary>
        MAX_BATCH_SIZE = 200,
        /// <summary>
        /// Maximum number of camera can that be instancied at the same time. Used to initialized arrays of cameras (ex: GetDeviceList())
        /// </summary>
        MAX_CAMERA_PLUGIN = 20,
        /// <summary>
        /// Maximum number of camera that can be fused by the Fusion API.
        /// </summary>
        MAX_FUSED_CAMERAS = 20
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Core  ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Core Module

    /// \ingroup Core_group
    /// <summary>
    /// Structure representing a generic 3*3 matrix.
    /// </summary>
    public struct Matrix3x3
    {
        /// <summary>
        /// Array containing the values fo the 3*3 matrix.
        /// </summary>
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
        public float[] m; //3x3 matrix.

        /// <summary>
        /// Gives the result of the multiplication between a sl.Matrix3x3 and a specified scalar value.
        /// </summary>
        /// <param name="a">Scalar value to multiple the sl.Matrix3x3 with.</param>
        /// <returns>The result of the multiplication with the scalar given as input.</returns>
        public float3 multiply(float3 a)
        {
            float3 result = new float3();

            result.x = m[0] * a.x + m[1] * a.y + m[2] * a.z;
            result.y = m[3] * a.x + m[4] * a.y + m[5] * a.z;
            result.z = m[6] * a.x + m[7] * a.y + m[8] * a.z;

            return result;
        }
    };

    /// \ingroup Fusion_group
    /// <summary>
    /// Lists the types of possible position outputs.
    /// </summary>
    public enum POSITION_TYPE
    {
        /// <summary>
        /// The output position will be the raw position data.
        /// </summary>
        RAW = 0,
        /// <summary>
        /// The output position will be the fused position projected into the requested camera repository.
        /// </summary>
        FUSION,
        ///@cond SHOWHIDDEN 
        SL_POSITION_TYPE_LAST
        ///@endcond
    };

    /// <summary>
    /// \ingroup Core_group
    /// Structure containing the width and height of an image.
    /// </summary>
    public struct Resolution
    {
        /// <summary>
        /// Default constructor.
        /// </summary>
        /// <param name="width">Width of the image in pixels.</param>
        /// <param name="height">Height of the image in pixels.</param>
        public Resolution(uint width = 0, uint height = 0)
        {
            this.width = (System.UIntPtr)width;
            this.height = (System.UIntPtr)height;
        }
        /// <summary>
        /// Width of the image in pixels.
        /// </summary>
        public System.UIntPtr width;
        /// <summary>
        /// Height of the image in pixels.
        /// </summary>
        public System.UIntPtr height;
    };

    /// <summary>
    /// Structure defining a 2D rectangle with top-left corner coordinates and width/height in pixels.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Rect
    {
        /// <summary>
        /// x coordinates of top-left corner.
        /// </summary>
        public int x;
        /// <summary>
        /// y coordinates of top-left corner.
        /// </summary>
        public int y;
        /// <summary>
        /// Width of the rectangle in pixels.
        /// </summary>
        public int width;
        /// <summary>
        /// Height of the rectangle in pixels.
        /// </summary>
        public int height;
    };

    ///\ingroup  Core_group
    /// <summary>
    /// List of error codes in the ZED SDK.
    /// </summary>
    /// \note For more info, read about the ZED SDK C++ enum it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/group__Core__group.html#ga4db9ee29f2ff83c71567c12f6bfbf28c">ERROR_CODE</a>
    public enum ERROR_CODE
    {
        /// <summary>
        ///  The image could be corrupted, Enabled with the parameter InitParameters.enable_image_validity_check
        /// </summary>
        CORRUPTED_FRAME = -2,
        /// <summary>
        /// The camera is currently rebooting.
        /// </summary>
        CAMERA_REBOOTING = -1,
        /// <summary>
        /// Standard code for successful behavior.
        /// </summary>
        SUCCESS,
        /// <summary>
        /// Standard code for unsuccessful behavior.
        /// </summary>
        FAILURE,
        /// <summary>
        /// No GPU found or CUDA capability of the device is not supported.
        /// </summary>
        NO_GPU_COMPATIBLE,
        /// <summary>
        /// Not enough GPU memory for this depth mode. Try a different mode (such as \ref DEPTH_MODE "PERFORMANCE"), or increase the minimum depth value (see \ref InitParameters.depthMinimumDistance).
        /// </summary>
        NOT_ENOUGH_GPUMEM,
        /// <summary>
        /// No camera was detected.
        /// </summary>
        CAMERA_NOT_DETECTED,
        /// <summary>
        /// The MCU that controls the sensors module has an invalid serial number. You can try to recover it by launching the <b>ZED Diagnostic</b> tool from the command line with the option <code>-r</code>.
        /// </summary>
        SENSORS_NOT_INITIALIZED,
        /// <summary>
        /// A camera with sensor is detected but the sensors (IMU, barometer, ...) cannot be opened. Only the \ref MODEL "MODEL.ZED" does not has sensors. Unplug/replug is required.
        /// </summary>
        SENSOR_NOT_DETECTED,
        /// <summary>
        /// In case of invalid resolution parameter, such as an upsize beyond the original image size in Camera.RetrieveImage.
        /// </summary>
        INVALID_RESOLUTION,
        /// <summary>
        /// Insufficient bandwidth for the correct use of the camera. This issue can occur when you use multiple cameras or a USB 2.0 port.
        /// </summary>
        LOW_USB_BANDWIDTH,
        /// <summary>
        /// The calibration file of the camera is not found on the host machine. Use <b>ZED Explorer</b> or <b>ZED Calibration</b> to download the factory calibration file.
        /// </summary>
        CALIBRATION_FILE_NOT_AVAILABLE,
        /// <summary>
        /// The calibration file is not valid. Try to download the factory calibration file or recalibrate your camera using <b>ZED Calibration</b>.
        /// </summary>
        INVALID_CALIBRATION_FILE,
        /// <summary>
        /// The provided SVO file is not valid.
        /// </summary>
        INVALID_SVO_FILE,
        /// <summary>
        /// An error occurred while trying to record an SVO (not enough free storage, invalid file, ...).
        /// </summary>
        SVO_RECORDING_ERROR,
        /// <summary>
        /// An SVO related error, occurs when NVIDIA based compression cannot be loaded.
        /// </summary>
        SVO_UNSUPPORTED_COMPRESSION,
        /// <summary>
        /// SVO end of file has been reached.
        /// \n No frame will be available until the SVO position is reset.
        /// </summary>
        END_OF_SVO_FILE_REACHED,
        /// <summary>
        /// The requested coordinate system is not available.
        /// </summary>
        INVALID_COORDINATE_SYSTEM,
        /// <summary>
        /// The firmware of the camera is out of date. Update to the latest version.
        /// </summary>
        INVALID_FIRMWARE,
        /// <summary>
        ///  Invalid parameters have been given for the function.
        /// </summary>
        INVALID_FUNCTION_PARAMETERS,
        /// <summary>
        /// A CUDA error has been detected in the process, in Camera.Grab() or Camera.RetrieveXXX() only. Activate wrapperVerbose in ZEDManager.cs for more info.
        /// </summary>
        CUDA_ERROR,
        /// <summary>
        /// The ZED SDK is not initialized. Probably a missing call to Camera.Open().
        /// </summary>
        CAMERA_NOT_INITIALIZED,
        /// <summary>
        /// Your NVIDIA driver is too old and not compatible with your current CUDA version.
        /// </summary>
        NVIDIA_DRIVER_OUT_OF_DATE,
        /// <summary>
        /// The call of the function is not valid in the current context. Could be a missing call of Camera.Open().
        /// </summary>
        INVALID_FUNCTION_CALL,
        /// <summary>
        ///  The ZED SDK was not able to load its dependencies or some assets are missing. Reinstall the ZED SDK or check for missing dependencies (cuDNN, TensorRT).
        /// </summary>
        CORRUPTED_SDK_INSTALLATION,
        /// <summary>
        /// The installed ZED SDK is incompatible with the one used to compile the program.
        /// </summary>
        INCOMPATIBLE_SDK_VERSION,
        /// <summary>
        /// The given area file does not exist. Check the path.
        /// </summary>
        INVALID_AREA_FILE,
        /// <summary>
        /// The area file does not contain enough data to be used or the \ref DEPTH_MODE used during the creation of the area file is different from the one currently set.
        /// </summary>
        INCOMPATIBLE_AREA_FILE,
        /// <summary>
        /// Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.
        /// </summary>
        CAMERA_FAILED_TO_SETUP,
        /// <summary>
        /// Your camera can not be opened. Try replugging it to another port or flipping the USB-C connector (if there is one).
        /// </summary>
        CAMERA_DETECTION_ISSUE,
        /// <summary>
        /// Cannot start the camera stream. Make sure your camera is not already used by another process or blocked by firewall or antivirus.
        /// </summary>
        CAMERA_ALREADY_IN_USE,
        /// <summary>
        ///  No GPU found. CUDA is unable to list it. Can be a driver/reboot issue.
        /// </summary>
        NO_GPU_DETECTED,
        /// <summary>
        /// Plane not found. Either no plane is detected in the scene, at the location or corresponding to the floor,
        /// or the floor plane doesn't match the prior given.
        /// </summary>
        PLANE_NOT_FOUND,
        /// <summary>
        /// The module you try to use is not compatible with your camera \ref MODEL. \note \ref MODEL "MODEL.ZED" does not has an IMU and does not support the AI modules.
        /// </summary>
        MODULE_NOT_COMPATIBLE_WITH_CAMERA,
        /// <summary>
        /// The module needs the sensors to be enabled (see InitParameters.sensorsRequired).
        /// </summary>
        MOTION_SENSORS_REQUIRED,
        /// <summary>
        /// The module needs a newer version of CUDA.
        /// </summary>
        MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION,
        /// @cond SHOWHIDDEN 
        ERROR_CODE_LAST
        /// @endcond
    };

    ///\ingroup  Core_group
    /// <summary>
    /// Lists available coordinates systems for positional tracking and 3D measures.
    /// \image html CoordinateSystem.png
    /// </summary>
    public enum COORDINATE_SYSTEM
    {
        /// <summary>
        /// Standard coordinates system in computer vision.
        /// \n Used in OpenCV: see <a href="http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html">here</a>.
        /// </summary>
        IMAGE,
        /// <summary>
        /// Left-handed with Y up and Z forward.
        /// \n Used in Unity3D with DirectX
        /// </summary>
        LEFT_HANDED_Y_UP,
        /// <summary>
        ///  Right-handed with Y pointing up and Z backward.
        /// \n Used in OpenGL.
        /// </summary>
        RIGHT_HANDED_Y_UP,
        /// <summary>
        /// Right-handed with Z pointing up and Y forward.
        /// \n Used in 3DSMax.
        /// </summary>
        RIGHT_HANDED_Z_UP,
        /// <summary>
        /// Left-handed with Z axis pointing up and X forward
        /// \n Used in Unreal Engine.
        /// </summary>
        LEFT_HANDED_Z_UP,
        /// <summary>
        /// Right-handed with Z pointing up and X forward.
        /// \n Used in ROS (REP 103)
        /// </summary>
        RIGHT_HANDED_Z_UP_X_FWD
    }

    /// \ingroup Core_groupCameraInformation
    /// <summary>
    /// Structure containing information about the camera sensor. 
    /// </summary>
    /// \note This object is meant to be used as a read-only container, editing any of its field won't impact the ZED SDK.
    /// \warning sl.CalibrationParameters are returned in sl.COORDINATE_SYSTEM.IMAGE, they are not impacted by the sl.InitParameters.coordinateSystem.
    [StructLayout(LayoutKind.Sequential)]
    public struct CameraConfiguration
    {
        /// <summary>
        /// Intrinsics and extrinsic stereo parameters for rectified/undistorted images.
        /// </summary>
        public CalibrationParameters calibrationParameters;
        /// <summary>
        /// Intrinsics and extrinsic stereo parameters for unrectified/distorted images.
        /// </summary>
        public CalibrationParameters calibrationParametersRaw;
        /// <summary>
        /// Internal firmware version of the camera.
        /// </summary>
        public uint firmwareVersion;
        /// <summary>
        /// FPS of the camera.
        /// </summary>
        public float fps;
        /// <summary>
        /// Resolution of the camera.
        /// </summary>
        public Resolution resolution;
    };

    /// \ingroup Core_group
    /// <summary>
    /// Structure containing information of a single camera (serial number, model, input type, etc.)
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CameraInformation
    {
        /// <summary>
        /// Serial number of the camera.
        /// </summary>
        public uint serialNumber;
        /// <summary>
        /// Model of the camera (see sl.MODEL).
        /// </summary>
        public MODEL cameraModel;
        /// <summary>
        /// Input type used in the ZED SDK.
        /// </summary>
        public INPUT_TYPE inputType;
        /// <summary>
        /// Camera configuration parameters stored in a sl.CameraConfiguration.
        /// </summary>
        public CameraConfiguration cameraConfiguration;
        /// <summary>
        /// Sensors configuration parameters stored in a sl.SensorsConfiguration.
        /// </summary>
        public SensorsConfiguration sensorsConfiguration;
    };

    /// \ingroup Video_group
    /// <summary>
    /// Structure defining the input type used in the ZED SDK.
    /// </summary>
    /// It can be used to select a specific camera with an id or serial number, or from a SVO file.
    [StructLayout(LayoutKind.Sequential)]
    public struct InputType
    {
        /// <summary>
        /// Current input type.
        /// </summary>
        public INPUT_TYPE inputType;

        /// <summary>
        /// Serial number of the camera.
        /// </summary>
	    uint serialNumber;

        /// <summary>
        /// Id of the camera.
        /// </summary>
        uint id;

        /// <summary>
        /// Path to the SVO file.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
        char[] svoInputFilename;

        /// <summary>
        /// IP address of the streaming camera.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
        char[] streamInputIp;

        /// <summary>
        /// Port of the streaming camera.
        /// </summary>
        ushort streamInputPort;
    };

    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Tracking  ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Tracking Module

    /// \ingroup PositionalTracking_group
    /// <summary>
    /// Class containing a set of parameters for the positional tracking module initialization.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be user adjusted.
    [StructLayout(LayoutKind.Sequential)]
    public class PositionalTrackingParameters
    {
        /// <summary>
        /// Rotation of the camera in the world frame when the camera is started.
        /// </summary>
        public Quaternion initialWorldRotation = Quaternion.Identity;
        /// <summary>
        /// Position of the camera in the world frame when the camera is started.
        /// </summary>
        public Vector3 initialWorldPosition = Vector3.Zero;
        /// <summary>
        /// Whether the camera can remember its surroundings.
        /// </summary>
        /// This helps correct positional tracking drift and can be helpful for positioning different cameras relative to one other in space.
        /// \warning This mode requires more resources to run, but greatly improves tracking accuracy.
        /// \warning We recommend leaving it on by default.
        public bool enableAreaMemory = true;
        /// <summary>
        /// Whether to enable smooth pose correction for small drift correction.
        /// </summary>
        public bool enablePoseSmothing = false;
        /// <summary>
        /// Initializes the tracking to be aligned with the floor plane to better position the camera in space.
        /// </summary>
        /// \note This launches floor plane detection in the background until a suitable floor plane is found.
        /// \note The tracking will start in sl.POSITIONAL_TRACKING_STATE.SEARCHING state.
        /// \warning This features does not work with sl.MODEL.ZED since it needs an IMU to classify the floor.
        /// \warning The camera needs to look at the floor during initialization for optimum results.
        public bool setFloorAsOrigin = false;
        /// <summary>
        /// Whether to define the camera as static.
        /// </summary>
        /// If true, it will not move in the environment. This allows you to set its position using \ref initialWorldPosition and \ref initialWorldRotation.
        /// \n All ZED SDK functionalities requiring positional tracking will be enabled without additional computation.
        /// \n sl.Camera.GetPosition() will return the values set as \ref initialWorldPosition and \ref initialWorldRotation.
        public bool setAsStatic = false;
        /// <summary>
        /// Whether to enable the IMU fusion.
        /// </summary>
        /// When set to false, only the optical odometry will be used.
        /// \note This setting has no impact on the tracking of a camera.
        /// \note sl.MODEL.ZED does not have an IMU.
        public bool enableIMUFusion = true;
        /// <summary>
        /// Minimum depth used by the ZED SDK for positional tracking.
        /// </summary>
        /// It may be useful for example if any steady objects are in front of the camera and may perturb the positional tracking algorithm.
        /// \n Default: -1 (no minimum depth)
        public float depthMinRange = -1f;
        /// <summary>
        /// Whether to override 2 of the 3 components from \ref initialWorldRotation using the IMU gravity.
        /// </summary>
        /// \note This parameter does nothing on sl.ZED.MODEL since it does not have an IMU.
        public bool setGravityAsOrigin = true;
        /// <summary>
        /// Path of an area localization file that describes the surroundings (saved from a previous tracking session).
        /// </summary>
        /// \note Loading an area file will start a search phase, during which the camera will try to position itself in the previously learned area.
        /// \warning The area file describes a specific location. If you are using an area file describing a different location, the tracking function will continuously search for a position and may not find a correct one.
        /// \warning The '.area' file can only be used with the same depth mode (sl.DEPTH_MODE) as the one used during area recording.
        public string areaFilePath = "";
        /// <summary>
        /// Positional tracking mode used.
        /// </summary>
        /// Can be used to improve accuracy in some types of scene at the cost of longer runtime.
        public sl.POSITIONAL_TRACKING_MODE mode = sl.POSITIONAL_TRACKING_MODE.STANDARD;
    }
    /// \ingroup PositionalTracking_group
    /// <summary>
    /// Structure containing positional tracking data giving the position and orientation of the camera in 3D space.
    /// </summary>
    /// Different representations of position and orientation can be retrieved, along with timestamp and pose confidence.
    [StructLayout(LayoutKind.Sequential)]
    public struct Pose
    {
        /// <summary>
        /// Whether the tracking is activated or not.
        /// </summary>
        /// \note You should check that first if something is wrong.
        public bool valid;
        /// <summary>
        /// Timestamp of the sl.Pose.
        /// </summary>
        /// This timestamp should be compared with the camera timestamp for synchronization.
        public ulong timestamp;
        /// <summary>
        /// Orientation component of the sl.Pose.
        /// </summary>
        public Quaternion rotation;
        /// <summary>
        /// Translation component of the sl.Pose.
        /// </summary>
        public Vector3 translation;
        /// <summary>
        /// Confidence/quality of the pose estimation for the target frame.
        /// </summary>
        /// A confidence metric of the tracking [0-100] with:
        /// - 0: tracking is lost
        /// - 100: tracking can be fully trusted
        public int pose_confidence;
        /// <summary>
        /// 6x6 pose covariance matrix of translation (the first 3 values) and rotation in so3 (the last 3 values).
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
        public float[] pose_covariance;
        /// <summary>
        /// Twist of the camera available in reference camera.
        /// </summary>
        /// This expresses velocity in free space, broken into its linear and angular parts.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] twist;
        /// <summary>
        /// Row-major representation of the 6x6 twist covariance matrix of the camera.
        /// </summary>
        /// This expresses the uncertainty of the twist.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
        public float[] twist_covariance;
    };

    /// \ingroup PositionalTracking_group
    /// <summary>
    /// Structure containing a set of parameters for the region of interest.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RegionOfInterestParameters
    {
        /// <summary>
        /// Filtering how far object in the ROI should be considered, this is useful for a vehicle for instance
        /// Default is 2.5meters
        /// </summary>
        public float depthFarThresholdMeters;
        /// <summary>
        /// By default consider only the lower half of the image, can be useful to filter out the sky
        /// Default is 0.5, corresponding to the lower half of the image.
        /// </summary>
        public float imageHeightRatioCutoff;
        /// <summary>
        /// Once computed the ROI computed will be automatically applied.
        /// </summary>
        public bool autoApply;
    }

    ///\ingroup PositionalTracking_group
    /// <summary>
    /// Lists the different states of region of interest auto detection.
    /// </summary>
    public enum REGION_OF_INTEREST_AUTO_DETECTION_STATE
    {
        /// <summary>
        /// The region of interest auto detection is initializing.
        /// </summary>
        RUNNING,
        /// <summary>
        /// The region of interest mask is ready, if auto_apply was enabled, the region of interest mask is being used.
        /// </summary>
        READY,
        /// <summary>
        ///  The region of interest auto detection is not enabled.
        /// </summary>
        NOT_ENABLED
    };

    ///\ingroup PositionalTracking_group
    /// <summary>
    /// Lists the different states of positional tracking.
    /// </summary>
    public enum POSITIONAL_TRACKING_STATE
    {
        /// <summary>
        /// The camera is searching for a previously known position to locate itself.
        /// </summary>
        SEARCHING,
        /// <summary>
        /// The positional tracking is working normally.
        /// </summary>
        OK,
        /// <summary>
        /// The positional tracking is not enabled.
        /// </summary>
        OFF,
        /// <summary>
        /// The effective FPS is too low to give proper results for motion tracking.
        /// \n Consider using performance parameters (sl.DEPTH_MODE.PERFORMANCE, low camera resolution (sl.RESOLUTION.VGA
        /// / sl.RESOLUTION.HDSVGA or sl.RESOLUTION.HD720).
        /// </summary>
        FPS_TOO_LOW,
        /// <summary>
        /// The camera is searching for the floor plane to locate itself with respect to it.\n The sl.REFERENCE_FRAME.WORLD will be set afterward.
        /// </summary>
        SEARCHING_FLOOR_PLANE
    }

    ///\ingroup PositionalTracking_group
    /// <summary>
    /// Lists the mode of positional tracking that can be used.
    /// </summary>
    public enum POSITIONAL_TRACKING_MODE
    {
        /// <summary>
        /// Default mode. Best compromise in performance and accuracy.
        /// </summary>
        STANDARD,
        /// <summary>
        /// Improve accuracy in more challenging scenes such as outdoor repetitive patterns like extensive fields.
        /// \n Currently works best with sl.DEPTH_MODE.ULTRA and requires more compute power.
        /// </summary>
        QUALITY
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists the different states of spatial memory area export.
    /// </summary>
    public enum AREA_EXPORTING_STATE
    {
        /// <summary>
        /// The spatial memory file has been successfully created.
        /// </summary>
        SUCCESS,
        /// <summary>
        /// The spatial memory is currently being written.
        /// </summary>
        RUNNING,
        /// <summary>
        /// The spatial memory file exportation has not been called.
        /// </summary>
        NOT_STARTED,
        /// <summary>
        /// The spatial memory contains no data, the file is empty.
        /// </summary>
        FILE_EMPTY,
        /// <summary>
        /// The spatial memory file has not been written because of a wrong file name.
        /// </summary>
        FILE_ERROR,
        /// <summary>
        /// The spatial memory learning is disabled. No file can be created.
        /// </summary>
        SPATIAL_MEMORY_DISABLED,
    }

    /// \ingroup PositionalTracking_group
    /// <summary>
    /// Lists possible types of position matrix used to store camera path and pose.
    /// </summary>
    public enum REFERENCE_FRAME
    {
        /// <summary>
        /// The transform of sl.Pose will contain the motion with reference to the world frame (previously called sl.PATH).
        /// </summary>
        WORLD,
        /// <summary>
        /// The transform of sl.Pose will contain the motion with reference to the previous camera frame (previously called sl.POSE).
        /// </summary>
        CAMERA
    };


    /// \ingroup PositionalTracking_group
    /// <summary>
    /// Part of the ZED (left/right sensor, center) that's considered its center for tracking purposes.
    /// </summary>
    public enum TRACKING_FRAME
    {
        /// <summary>
        /// Camera's center is at the left sensor.
        /// </summary>
		LEFT_EYE,
        /// <summary>
        /// Camera's center is in the camera's physical center, between the sensors.
        /// </summary>
		CENTER_EYE,
        /// <summary>
        /// Camera's center is at the right sensor.
        /// </summary>
		RIGHT_EYE
    };

    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Sensors  /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Sensors Module

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing data from the IMU sensor.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ImuData
    {
        /// <summary>
        /// Whether the IMU sensor is available in your camera.
        /// </summary>
        public bool available;
        /// <summary>
        /// Data acquisition timestamp in nanoseconds.
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// Angular velocity vector of the gyroscope in deg/s.
        /// </summary>
        /// The value is corrected from bias, scale and misalignment.
        /// \note The value can be directly ingested in an IMU fusion algorithm to extract a quaternion.
        /// \note Not available in SVO or STREAM mode.
		public Vector3 angularVelocity;
        /// <summary>
        /// Linear acceleration vector (3x1) of the gyroscope in m/s².
        /// </summary>
        /// The value is corrected from bias, scale and misalignment.
        /// \note The value can be directly ingested in an IMU fusion algorithm to extract a quaternion.
        /// \note Not available in SVO or STREAM mode.
		public Vector3 linearAcceleration;
        /// <summary>
        /// Angular velocity vector of the gyroscope in deg/s (uncorrected from the IMU calibration).
        /// </summary>
        /// \note The value is the exact raw values from the IMU.
        /// \note Not available in SVO or STREAM mode.
        public Vector3 angularVelocityUncalibrated;
        /// <summary>
        /// Linear acceleration vector of the gyroscope in m/s² (uncorrected from the IMU calibration).
        /// </summary>
        /// \note The value is the exact raw values from the IMU.
        /// \note Not available in SVO or STREAM mode.
		public Vector3 linearAccelerationUncalibrated;
        /// <summary>
        /// Orientation from the IMU sensor.
        /// </summary>
		public Quaternion fusedOrientation;
        /// <summary>
        /// Covariance matrix of the quaternion.
        /// </summary>
		public Matrix3x3 orientationCovariance;
        /// <summary>
        /// Covariance matrix of the angular velocity of the gyroscope in deg/s (\ref angularVelocity).
        /// </summary>
        /// \note Not available in SVO or STREAM mode.
		public Matrix3x3 angularVelocityCovariance;
        /// <summary>
        /// Accelerometer raw data covariance matrix.
        /// </summary>
        /// \note Not available in SVO or STREAM mode.
		public Matrix3x3 linearAccelerationCovariance;
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing data from the barometer sensor.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BarometerData
    {
        /// <summary>
        /// Whether the barometer sensor is available in your camera.
        /// </summary>
        public bool available;
        /// <summary>
        /// Data acquisition timestamp in nanoseconds.
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// Ambient air pressure in hectopascal (hPa).
        /// </summary>
        public float pressure;
        /// <summary>
        /// Relative altitude from first camera position (at sl.Camera.Open() time).
        /// </summary>
        public float relativeAltitude;
    };

    ///\ingroup  Sensors_group
    /// <summary>
    /// Lists the different states of the magnetic heading.
    /// </summary>
    public enum HEADING_STATE
    {
        /// <summary>
        /// The heading is reliable and not affected by iron interferences.
        /// </summary>
        GOOD,
        /// <summary>
        /// The heading is reliable, but affected by slight iron interferences.
        /// </summary>
        OK,
        /// <summary>
        /// The heading is not reliable because affected by strong iron interferences.
        /// </summary>
        NOT_GOOD,
        /// <summary>
        /// The magnetometer has not been calibrated.
        /// </summary>
        NOT_CALIBRATED,
        /// <summary>
        /// The magnetometer sensor is not available.
        /// </summary>
        MAG_NOT_AVAILABLE,
        ///@cond SHOWHIDDEN
        LAST
        ///@endcond
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing data from the magnetometer sensor.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct MagnetometerData
    {
        /// <summary>
        /// Whether the magnetometer sensor is available in your camera.
        /// </summary>
        public bool available;
        /// <summary>
        /// Data acquisition timestamp in nanoseconds.
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// Magnetic field local vector in microtesla (μT).
        /// </summary>
        /// \note To calibrate the magnetometer sensor, please use \b ZED \b Sensor \b Viewer tool after placing the camera in the final operating environment.
        public Vector3 magneticField;
        /// <summary>
        /// Uncalibrated magnetic field local vector in microtesla (μT).
        /// </summary>
        /// \note The magnetometer raw values are affected by soft and hard iron interferences.
        /// \note The sensor must be calibrated by placing the camera in the working environment and using \b ZED \b Sensor \b Viewer tool.
        /// \note Not available in SVO or STREAM mode.
        public Vector3 magneticFieldUncalibrated;
        /// <summary>
        /// Camera heading in degrees relative to the magnetic North Pole.
        /// </summary>
        /// \note The magnetic North Pole has an offset with respect to the geographic North Pole, depending on the geographic position of the camera.
        /// \note To get a correct magnetic heading, the magnetometer sensor must be calibrated using \b ZED \b Sensor \b Viewer tool.
        public float magneticHeading;
        /// <summary>
        /// State of \ref magneticHeading.
        /// </summary>
        public HEADING_STATE magnetic_heading_state;
        /// <summary>
        /// Accuracy of \ref magnetic_heading measure in the range [0.0, 1.0].
        /// </summary>
        /// \note A negative value means that the magnetometer must be calibrated using \b ZED \b Sensor \b Viewer tool.
        public float magnetic_heading_accuracy;
        /// <summary>
        /// Realtime data acquisition rate in hertz (Hz).
        /// </summary>
        public float effective_rate;
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing data from the temperature sensors.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct TemperatureSensorData
    {
        /// <summary>
        /// Temperature in °C at the IMU location (-100 if not available).
        /// </summary>
        public float imu_temp;
        /// <summary>
        /// Temperature in °C at the barometer location (-100 if not available).
        /// </summary>
        public float barometer_temp;
        /// <summary>
        /// Temperature in °C next to the left image sensor (-100 if not available).
        /// </summary>
        public float onboard_left_temp;
        /// <summary>
        /// Temperature in °C next to the right image sensor (-100 if not available).
        /// </summary>
        public float onboard_right_temp;
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing all sensors data (except image sensors) to be used for positional tracking or environment study.
    /// </summary>
    /// \note Some data are not available in SVO and streaming input mode.
    /// \note They are specified by a note "Not available in SVO or STREAM mode." in the documentation of a specific data.
    /// \note If nothing is mentioned in the documentation, they are available in all input modes.
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorsData
    {
        /// <summary>
        /// IMU data.
        /// </summary>
        public ImuData imu;
        /// <summary>
        /// Barometer data.
        /// </summary>
        public BarometerData barometer;
        /// <summary>
        /// Magnetometer data.
        /// </summary>
        public MagnetometerData magnetometer;
        /// <summary>
        /// Temperature data.
        /// </summary>
        public TemperatureSensorData temperatureSensor;
        /// <summary>
        /// Motion state of the camera.
        /// </summary>
        /// - static: 0
        /// - moving: 1
        /// - falling: 2
        public int camera_moving_state;
        /// <summary>
        /// Indicates if the sensors data has been taken during a frame capture on sensor.
        /// </summary>
        /// If the value is 1, the data has been taken during the same time than a frame has been acquired by the left sensor (the time precision is linked to the IMU rate, therefore 800Hz == 1.3ms).
        /// \n If the value is 0, the data has not been taken during a frame acquisition.
        public int image_sync_val;
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Lists available sensor types.
    /// \note Sensors are not available on sl.MODEL.ZED.
    /// </summary>
    public enum SENSOR_TYPE
    {
        /// <summary>
        /// Three-axis accelerometer sensor to measure the inertial accelerations.
        /// </summary>
        ACCELEROMETER,
        /// <summary>
        /// Three-axis gyroscope sensor to measure the angular velocities.
        /// </summary>
        GYROSCOPE,
        /// <summary>
        /// Three-axis magnetometer sensor to measure the orientation of the device with respect to the Earth's magnetic field.
        /// </summary>
        MAGNETOMETER,
        /// <summary>
        /// Barometer sensor to measure the atmospheric pressure.
        /// </summary>
        BAROMETER
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Lists available measurement units of onboard sensors.
    /// \note Sensors are not available on sl.MODEL.ZED.
    /// </summary>
    public enum SENSORS_UNIT
    {
        /// <summary>
        /// m/s² (acceleration)
        /// </summary>
        M_SEC_2,
        /// <summary>
        /// deg/s (angular velocity)
        /// </summary>
        DEG_SEC,
        /// <summary>
        /// μT (magnetic field)
        /// </summary>
        U_T,
        /// <summary>
        /// hPa (atmospheric pressure)
        /// </summary>
        HPA,
        /// <summary>
        /// °C (temperature)
        /// </summary>
        CELSIUS,
        /// <summary>
        /// Hz (frequency)
        /// </summary>
        HERTZ
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing information about a single sensor available in the current device.
    /// </summary>
    /// Information about the camera sensors is available in the sl.CameraInformation struct returned by sl.Camera.GetCameraInformation().
    /// \note This structure is meant to be used as a read-only container.
    /// \note Editing any of its fields will not impact the ZED SDK.
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorParameters
    {
        /// <summary>
        /// Type of the sensor.
        /// </summary>
        public SENSOR_TYPE type;
        /// <summary>
        /// Resolution of the sensor.
        /// </summary>
        public float resolution;
        /// <summary>
        /// Sampling rate (or ODR) of the sensor.
        /// </summary>
        public float sampling_rate;
        /// <summary>
        /// Range of the sensor (minimum: `range.x`, maximum: `range.y`).
        /// </summary>
        public float2 range;
        /// <summary>
        /// White noise density given as continuous (frequency-independent).
        /// </summary>
        /// \note The units will be expressed in ```sensor_unit / √(Hz)```.
        /// \note `NAN` if the information is not available.
        public float noise_density;
        /// <summary>
        /// Random walk derived from the Allan Variance given as continuous (frequency-independent).
        /// </summary>
        /// \note The units will be expressed in ```sensor_unit / √(Hz)```.
        /// \note `NAN` if the information is not available.
        public float random_walk;
        /// <summary>
        /// Unit of the sensor.
        /// </summary>
        public SENSORS_UNIT sensor_unit;
        /// <summary>
        /// Whether the sensor is available in your camera.
        /// </summary>
        public bool isAvailable;
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing information about all the sensors available in the current device.
    /// </summary>
    /// Information about the camera sensors is available in the sl.CameraInformation struct returned by sl.Camera.GetCameraInformation().
    /// \note This structure is meant to be used as a read-only container.
    /// \note Editing any of its fields will not impact the ZED SDK.
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorsConfiguration
    {
        /// <summary>
        /// Firmware version of the sensor module.
        /// </summary>
        /// \note 0 if no sensors are available (sl.MODEL.ZED).
        public uint firmware_version;
        /// <summary>
        /// IMU to left camera rotation (quaternion).
        /// </summary>
        /// \note It contains the rotation between the IMU frame and camera frame.
        public float4 camera_imu_rotation;
        /// <summary>
        /// IMU to left camera translation.
        /// </summary>
        /// \note It contains the rotation between the IMU frame and camera frame.
        public float3 camera_imu_translation;
        /// <summary>
        /// Magnetometer to IMU rotation (quaternion).
        /// </summary>
        /// \note It contains rotation between IMU frame and magnetometer frame.
        public float4 imu_magnometer_rotation;
        /// <summary>
        /// Magnetometer to IMU translation.
        /// </summary>
        /// \note It contains translation between IMU frame and magnetometer frame.
        public float3 imu_magnometer_translation;
        /// <summary>
        /// Configuration of the accelerometer.
        /// </summary>
        public SensorParameters accelerometer_parameters;
        /// <summary>
        /// Configuration of the gyroscope.
        /// </summary>
        public SensorParameters gyroscope_parameters;
        /// <summary>
        /// Configuration of the magnetometer.
        /// </summary>
        public SensorParameters magnetometer_parameters;
        /// <summary>
        /// Configuration of the barometer.
        /// </summary>
        public SensorParameters barometer_parameters;
        /// <summary>
        /// Checks if a sensor is available on the.
        /// </summary>
        /// <param name="sensor_type">Sensor type to check.</param>
        /// <returns>true if the sensor is available on the device, otherwise false.</returns>
        public bool isSensorAvailable(SENSOR_TYPE sensor_type) {
            switch (sensor_type) {
                case SENSOR_TYPE.ACCELEROMETER:
                    return accelerometer_parameters.isAvailable;
                case SENSOR_TYPE.GYROSCOPE:
                    return gyroscope_parameters.isAvailable;
                case SENSOR_TYPE.MAGNETOMETER:
                    return magnetometer_parameters.isAvailable;
                case SENSOR_TYPE.BAROMETER:
                    return barometer_parameters.isAvailable;
                default:
                    break;
            }
            return false;
        }
    };



    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Depth Sensing  ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Depth Sensing Module

    ///\ingroup Depth_group
    /// <summary>
    /// Class containing parameters that defines the behavior of sl.Camera.Grab().
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be user-adjusted at runtime.
    public class RuntimeParameters
    {
        /// <summary>
        /// Reference frame in which to provides the 3D measures (point cloud, normals, etc.).
        /// </summary>
        /// Default: sl.REFERENCE_FRAME.CAMERA
        public sl.REFERENCE_FRAME measure3DReferenceFrame;

        /// <summary>
        /// Defines if the depth map should be computed.
        /// </summary>
        /// Default: true
        /// \note If set to false, only the images are available.
        public bool enableDepth;

        /// <summary>
        /// Defines if the depth map should be completed or not.
        /// </summary>
        /// Default: false
        /// \note It is similar to the removed sl.SENSING_MODE.FILL.
        /// \warning Enabling this will override the confidence values \ref confidenceThreshold and
        /// \ref textureConfidenceThreshold as well as \ref removeSaturatedAreas.
        public bool enableFillMode;

        /// <summary>
        /// Threshold to reject depth values based on their confidence.
        /// </summary>
        /// Each depth pixel has a corresponding confidence sl.MEASURE.CONFIDENCE in the range [1, 100].
        /// \n Decreasing this value will remove depth data from both objects edges and low textured areas, to keep only confident depth estimation data.
        /// \n Default: 100 (no depth pixel will be rejected)
        /// \note Pixels with a value close to 100 are not to be trusted. Accurate depth pixels tends to be closer to lower values.
        /// \note It can be seen as a probability of error, scaled to 100.
        public int confidenceThreshold;

        /// <summary>
        /// Threshold to reject depth values based on their texture confidence.
        /// </summary>
        /// The texture confidence range is [1, 100].
        /// \n Decreasing this value will remove depth data from image areas which are uniform.
        /// \n Default: 100 (no depth pixel will be rejected)
        /// \note Pixels with a value close to 100 are not to be trusted. Accurate depth pixels tends to be closer to lower values.
        public int textureConfidenceThreshold;

        /// <summary>
        /// Defines if the saturated area (luminance>=255) must be removed from depth map estimation.
        /// </summary>
        /// Default: true
        /// \note It is recommended to keep this parameter at true because saturated area can create false detection.
        public bool removeSaturatedAreas;

        /// <summary>
        /// Default constructor.
        /// </summary>
        /// All the parameters are set to their default values.
        public RuntimeParameters(REFERENCE_FRAME reframe = REFERENCE_FRAME.CAMERA, bool depth = true, int cnf_threshold = 100, int txt_cnf_threshold = 100, bool removeSaturatedAreas_ = true, bool pEnableFillMode = false)
        {
            this.measure3DReferenceFrame = reframe;
            this.enableDepth = depth;
            this.confidenceThreshold = cnf_threshold;
            this.textureConfidenceThreshold = txt_cnf_threshold;
            this.removeSaturatedAreas = removeSaturatedAreas_;
            this.enableFillMode = pEnableFillMode;
        }
    }

    ///\ingroup Depth_group
    /// <summary>
    /// Structure containing the intrinsic parameters of a camera.
    /// </summary>
    /// \note \note For more info, read about the ZED SDK C++ struct it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/structsl_1_1CameraParameters.html">CameraParameters</a>
    [StructLayout(LayoutKind.Sequential)]
    public struct CameraParameters
    {
        /// <summary>
        /// Focal length in pixels along x axis.
        /// </summary>
        public float fx;
        /// <summary>
        /// Focal length in pixels along y axis.
        /// </summary>
        public float fy;
        /// <summary>
        /// Optical center along x axis, defined in pixels (usually close to width / 2).
        /// </summary>
        public float cx;
        /// <summary>
        /// Optical center along y axis, defined in pixels (usually close to height / 2).
        /// </summary>
        public float cy;

        /// <summary>
        /// Distortion factor : [k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4].
        /// </summary>
        /// Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3, s4) distortion.
        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.U8, SizeConst = 12)]
        public double[] disto;

        /// <summary>
        /// Vertical field of view, in degrees.
        /// </summary>
        public float vFOV;
        /// <summary>
        /// Horizontal field of view, in degrees.
        /// </summary>
        public float hFOV;
        /// <summary>
        /// Diagonal field of view, in degrees.
        /// </summary>
        public float dFOV;
        /// <summary>
        /// Size in pixels of the images given by the camera.
        /// </summary>
        public Resolution resolution;
        /// <summary>
        /// Real focal length in millimeters.
        /// </summary>
        public float focalLengthMetric;
    };

    ///\ingroup Depth_group
    /// <summary>
    /// Structure containing intrinsic and extrinsic parameters of the camera (translation and rotation).
    /// </summary>
    /// That information about the camera will be returned by sl.Camera.GetCameraInformation().
    /// \note The calibration/rectification process, called during sl.Camera.Open(), is using the raw parameters defined in the SNXXX.conf file, where XXX is the serial number of the camera.
    /// \note Those values may be adjusted or not by the self-calibration to get a proper image alignment.
    /// \note After sl.Camera.Open() is done (with or without self-calibration activated), most of the stereo parameters (except baseline of course) should be 0 or very close to 0.
    /// \note It means that images after rectification process (given by sl.Camera.RetrieveImage()) are aligned as if they were taken by a "perfect" stereo camera, defined by the new sl.CalibrationParameters.
    /// \warning CalibrationParameters are returned in sl.COORDINATE_SYSTEM.IMAGE, they are not impacted by the sl.InitParameters.coordinateSystem.
    /// \note For more info, read about the ZED SDK C++ struct it mirrors: 
    /// <a href="https://www.stereolabs.com/docs/api/structsl_1_1CalibrationParameters.html">CalibrationParameters</a>
    [StructLayout(LayoutKind.Sequential)]
    public struct CalibrationParameters
    {
        /// <summary>
        /// Intrinsic sl.CameraParameters of the left camera.
        /// </summary>
        public CameraParameters leftCam;
        /// <summary>
        /// Intrinsic sl.CameraParameters of the right camera.
        /// </summary>
        public CameraParameters rightCam;
        /// <summary>
        /// Left to right camera rotation, expressed in user coordinate system and unit (defined by sl.InitParameters.coordinateSystem).
        /// </summary>
        public Quaternion Rot;
        /// <summary>
        /// Left to right camera translation, expressed in user coordinate system and unit (defined by sl.InitParameters.coordinateSystem).
        /// </summary>
        public Vector3 Trans;
    };


    ///\ingroup Depth_group
    /// <summary>
    /// Lists available depth computation modes.
    /// </summary>
    /// \note For more info, read about the ZED SDK C++ enum it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/group__Depth__group.html#ga8d542017c9b012a19a15d46be9b7fa43">DEPTH_MODE</a>
    public enum DEPTH_MODE
    {
        /// <summary>
        /// No depth map computation.
        /// \n Only rectified stereo images will be available.
        /// </summary>
        NONE,
        /// <summary>
        /// Computation mode optimized for speed.
        /// </summary>
        PERFORMANCE,
        /// <summary>
        /// Computation mode designed for challenging areas with untextured surfaces.
        /// </summary>
        QUALITY,
        /// <summary>
        /// Computation mode that favors edges and sharpness.\n Requires more GPU memory and computation power.
        /// </summary>
		ULTRA,
        /// <summary>
        /// End to End Neural disparity estimation.\n Requires AI module.
        /// </summary>
        NEURAL
    };

    ///\ingroup Depth_group
    /// <summary>
    /// Lists retrievable measures.
    /// \note For more info, read about the ZED SDK C++ enum it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/group__Depth__group.html#ga798a8eed10c573d759ef7e5a5bcd545d">MEASURE</a>
    /// </summary>
    public enum MEASURE
    {
        /// <summary>
        /// Disparity map. Each pixel contains 1 float.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C1
        /// </summary>
        DISPARITY,
        /// <summary>
        /// Depth map in sl.UNIT defined in sl.InitParameters.coordinateUnits. Each pixel contains 1 float.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C1
        /// </summary>
        DEPTH,
        /// <summary>
        /// Certainty/confidence of the depth map. Each pixel contains 1 float.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C1
        /// </summary>
        CONFIDENCE,
        /// <summary>
        /// Point cloud. Each pixel contains 4 float (X, Y, Z, not used).
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZ,
        /// <summary>
        /// Colored point cloud. Each pixel contains 4 float (X, Y, Z, color).
        /// \n The color should to be read as an unsigned char[4] representing the RGBA color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZRGBA,
        /// <summary>
        /// Colored point cloud. Each pixel contains 4 float (X, Y, Z, color).
        /// \n The color should to be read as an unsigned char[4] representing the BGRA color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZBGRA,
        /// <summary>
        /// Colored point cloud. Each pixel contains 4 float (X, Y, Z, color).
        /// \n The color should to be read as an unsigned char[4] representing the ARGB color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZARGB,
        /// <summary>
        /// Colored point cloud. Each pixel contains 4 float (X, Y, Z, color).
        /// \n The color should to be read as an unsigned char[4] representing the ABGR color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZABGR,
        /// <summary>
        /// Normal vectors map. Each pixel contains 4 float (X, Y, Z, 0).
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        NORMALS,
        /// <summary>
        /// Disparity map for right sensor. Each pixel contains 1 float.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C1
        /// </summary>
        DISPARITY_RIGHT,
        /// <summary>
        /// Depth map for right sensor. Each pixel contains 1 float.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C1
        /// </summary>
        DEPTH_RIGHT,
        /// <summary>
        /// Point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, not used).
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZ_RIGHT,
        /// <summary>
        /// Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color).
        /// The color needs to be read as an unsigned char[4] representing the RGBA color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZRGBA_RIGHT,
        /// <summary>
        /// Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color).
        /// The color needs to be read as an unsigned char[4] representing the BGRA color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZBGRA_RIGHT,
        /// <summary>
        /// Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color).
        /// The color needs to be read as an unsigned char[4] representing the ARGB color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZARGB_RIGHT,
        /// <summary>
        /// Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color).
        /// The color needs to be read as an unsigned char[4] representing the ABGR color.
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        XYZABGR_RIGHT,
        /// <summary>
        ///  Normal vectors map for right view. Each pixel contains 4 float (X, Y, Z, 0).
        /// \n Type: sl.MAT_TYPE.MAT_32F_C4
        /// </summary>
        NORMALS_RIGHT,
        /// <summary>
        /// Depth map in millimeter whatever the sl.UNIT defined in sl.InitParameters.coordinateUnits.
        /// \n Invalid values are set to 0 and depth values are clamped at 65000.
        /// \n Each pixel contains 1 unsigned short.
        /// \n Type: sl.MAT_TYPE.MAT_16U_C1.
        /// </summary>
        DEPTH_U16_MM,
        /// <summary>
        /// Depth map in millimeter for right sensor. Each pixel contains 1 unsigned short.
        /// \n Type: sl.MAT_TYPE.MAT_16U_C1.
        /// </summary>
        DEPTH_U16_MM_RIGHT
    };

    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Video   //////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Video Module

    ///\ingroup Video_group
    /// <summary>
    /// Class containing the options used to initialize the sl.Camera object.
    ///
    /// This class allows you to select multiple parameters for the sl.Camera such as the selected camera, resolution, depth mode, coordinate system, and units of measurement.
    /// \n Once filled with the desired options, it should be passed to the sl.Camera.Open() method.
    /// \note For more info, read about the ZED SDK C++ class it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/structsl_1_1InitParameters.html">InitParameters</a>
    /// </summary>

    public class InitParameters
    {
        /// <summary>
        /// Defines the input source to initialize and open an sl.Camera object from.
        ///
        /// The SDK can handle different input types:
        /// - Select a camera by its ID (<i>/dev/videoX</i> on Linux, and 0 to N cameras connected on Windows)
        /// - Select a camera by its serial number
        /// - Open a recorded sequence in the SVO file format
        /// - Open a streaming camera from its IP address and port
        ///
        /// \note Available cameras and their id/serial number can be listed using sl.Camera.GetDeviceList() and sl.Camera.GetStreamingDeviceList().
        /// \note Each sl.Camera will create its own memory (CPU and GPU),
        ///  therefore the number of cameras used at the same time can be limited by the configuration of your computer (GPU/CPU memory and capabilities).
        ///
        /// Default : (empty)
        /// \note See sl.InputType for complementary information.
        /// </summary>
        public sl.INPUT_TYPE inputType;
        /// <summary>
        /// Desired camera resolution.
        /// \note Small resolutions offer higher framerate and lower computation time.
        /// \note In most situations, sl.RESOLUTION.HD720 at 60 FPS is the best balance between image quality and framerate.
        ///
        /// Default: <ul>
        /// <li>ZED X/X Mini: sl.RESOLUTION.HD1200</li>
        /// <li>other cameras: sl.RESOLUTION.HD720</li></ul>
        /// \note Available resolutions are listed here: sl.RESOLUTION.
        /// </summary>
        public sl.RESOLUTION resolution;
        /// <summary>
        /// Requested camera frame rate.
        ///
        /// If set to 0, the highest FPS of the specified \ref camera_resolution will be used.
        /// \n Default: 0
        /// \n\n See sl.RESOLUTION for a list of supported frame rates.
        /// \note If the requested \ref cameraFPS is unsupported, the closest available FPS will be used.
        /// </summary>
        public int cameraFPS;
        /// <summary>
        /// Id for identifying which camera to use from the connected cameras.
        /// </summary>
        public int cameraDeviceID;
        /// <summary>
        /// Path to a recorded SVO file to play, including filename.
        /// </summary>
        public string pathSVO = "";
        /// <summary>
        /// Defines if sl.Camera object return the frame in real time mode.
        ///
        /// When playing back an SVO file, each call to sl.Camera.Grab() will extract a new frame and use it.
        /// \n However, it ignores the real capture rate of the images saved in the SVO file.
        /// \n Enabling this parameter will bring the SDK closer to a real simulation when playing back a file by using the images' timestamps.
        /// \n Default: false
        /// \note sl.Camera.Grab() will return an error when trying to play too fast, and frames will be dropped when playing too slowly.
        /// </summary>
        public bool svoRealTimeMode;

        /// <summary>
        /// Unit of spatial data (depth, point cloud, tracking, mesh, etc.) for retrieval.
        ///
        /// Default: sl.UNIT.MILLIMETER
        /// </summary>
        public UNIT coordinateUnits;
        /// <summary>
        /// sl.COORDINATE_SYSTEM to be used as reference for positional tracking, mesh, point clouds, etc.
        ///
        /// This parameter allows you to select the sl.COORDINATE_SYSTEM used by the sl.Camera object to return its measures.
        /// \n This defines the order and the direction of the axis of the coordinate system.
        /// \n Default: sl.COORDINATE_SYSTEM.IMAGE
        /// </summary>
        public COORDINATE_SYSTEM coordinateSystem;
        /// <summary>
        /// sl.DEPTH_MODE to be used.
        ///
        /// The ZED SDK offers several sl.DEPTH_MODE, offering various levels of performance and accuracy.
        /// \n This parameter allows you to set the sl.DEPTH_MODE that best matches your needs.
        /// \n Default: \ref DEPTH_MODE "sl.DEPTH_MODE.PERFORMANCE"
        /// \note Available depth mode are listed here: sl.DEPTH_MODE.
        /// </summary>
        public sl.DEPTH_MODE depthMode;
        /// <summary>
        /// Minimum depth distance to be returned, measured in the sl.UNIT defined in \ref coordinateUnits.
        ///
        /// This parameter allows you to specify the minimum depth value (from the camera) that will be computed.
        ///
        /// \n In stereovision (the depth technology used by the camera), looking for closer depth values can have a slight impact on performance and memory consumption.
        /// \n On most of modern GPUs, performance impact will be low. However, the impact of memory footprint will be visible.
        /// \n In cases of limited computation power, increasing this value can provide better performance.
        /// \n Default: -1 (corresponding values are available <a href="https://www.stereolabs.com/docs/depth-sensing/depth-settings#depth-range">here</a>)
        /// \note \ref depthMinimumDistance value cannot be greater than 3 meters.
        /// \note 0 will imply that \ref depthMinimumDistance is set to the minimum depth possible for each camera
        /// (those values are available <a href="https://www.stereolabs.com/docs/depth-sensing/depth-settings#depth-range">here</a>).
        /// </summary>
        public float depthMinimumDistance;
        
        /// <summary>
        /// Maximum depth distance to be returned, measured in the sl.UNIT defined in \ref coordinateUnits.
        ///
        /// When estimating the depth, the ZED SDK uses this upper limit to turn higher values into <b>inf</b> ones.
        /// \note Changing this value has no impact on performance and doesn't affect the positional tracking nor the spatial mapping.
        /// \note It only change values the depth, point cloud and normals.
        /// </summary>
        public float depthMaximumDistance;
        /// <summary>
        /// Defines if a flip of the images is needed.
        ///
        /// If you are using the camera upside down, setting this parameter to sl.FLIP_MODE.ON will cancel its rotation.
        /// \n The images will be horizontally flipped.
        /// \n Default: sl.FLIP_MODE.AUTO
        /// \note From ZED SDK 3.2 a new sl.FLIP_MODE enum was introduced to add the automatic flip mode detection based on the IMU gravity detection.
        /// \note This does not work on sl.MODEL.ZED cameras since they do not have the necessary sensors.
        /// </summary>
        public FLIP_MODE cameraImageFlip;
        /// <summary>
        /// Enable the measurement computation on the right images.
        ///
        /// By default, the ZED SDK only computes a single depth map, aligned with the left camera image.
        /// \n This parameter allows you to enable sl.MEASURE.DEPTH_RIGHT and other \ref MEASURE "sl.MEASURE.XXX_RIGHT" at the cost of additional computation time.
        /// \n For example, mixed reality pass-through applications require one depth map per eye, so this parameter can be activated.
        /// \n Default: False
        /// </summary>
        public bool enableRightSideMeasure;
        /// <summary>
        /// Defines if a flip of the images is needed.
        ///
        /// At initialization, sl.Camera runs a self-calibration process that corrects small offsets from the device's factory calibration.
        /// \n A drawback is that calibration parameters will slightly change from one (live) run to another, which can be an issue for repeatability.
        /// \n If set to true, self-calibration will be disabled and calibration parameters won't be optimized, raw calibration parameters from the configuration file will be used.
        /// \n Default: false
        /// \note In most situations, self calibration should remain enabled.
        /// \note You can also trigger the self-calibration at anytime after sl.Camera.Open() by calling sl.Camera.UpdateSelfCalibration(), even if this parameter is set to true.
        /// </summary>
        public bool cameraDisableSelfCalib;

        /// <summary>
        /// Enable the ZED SDK verbose mode.
        ///
        /// This parameter allows you to enable the verbosity of the ZED SDK to get a variety of runtime information in the console.
        /// \n When developing an application, enabling verbose (<code>\ref sdkVerbose >= 1</code>) mode can help you understand the current ZED SDK behavior.
        /// \n However, this might not be desirable in a shipped version.
        /// \n Default: 0 (no verbose message)
        /// \note The verbose messages can also be exported into a log file.
        /// \note See \ref sdkVerboseLogFile for more.
        /// </summary>
        public int sdkVerbose;

        /// <summary>
        /// NVIDIA graphics card id to use.
        ///
        /// By default the SDK will use the most powerful NVIDIA graphics card found.
        /// \n However, when running several applications, or using several cameras at the same time, splitting the load over available GPUs can be useful.
        /// \n This parameter allows you to select the GPU used by the sl.Camera using an ID from 0 to n-1 GPUs in your PC.
        /// \n Default: -1
        /// \note A non-positive value will search for all CUDA capable devices and select the most powerful.
        /// </summary>
        public int sdkGPUId;
        /// <summary>
        /// File path to store the ZED SDK logs (if \ref sdkVerbose is enabled).
        ///
        /// The file will be created if it does not exist.
        /// \n Default: ""
        ///
        /// \note Setting this parameter to any value will redirect all standard output print calls of the entire program.
        /// \note This means that your own standard output print calls will be redirected to the log file.
        /// \note This parameter can be particularly useful for creating a log system, and with Unreal or Unity applications that don't provide a standard console output.
        /// \warning The log file won't be cleared after successive executions of the application.
        /// \warning This means that it can grow indefinitely if not cleared. 
        /// </summary>
        public string sdkVerboseLogFile = "";
        /// <summary>
        /// Defines whether the depth needs to be stabilized and to what extent.
        ///
        /// Regions of generated depth map can oscillate from one frame to another.
        /// \n These oscillations result from a lack of texture (too homogeneous) on an object and by image noise.
        /// \n This parameter controls a stabilization filter that reduces these oscillations.
        /// \n In the range [0-100]: <ul>
        /// <li>0 disable the depth stabilization (raw depth will be return)</li>
        /// <li>stabilization smoothness is linear from 1 to 100</li></ul>
        /// Default: 1
        /// \note The stabilization uses the positional tracking to increase its accuracy, 
        /// so the positional tracking module will be enabled automatically when set to a value different from 0.
        /// \note Note that calling sl.Camera.EnablePositionalTracking() with your own parameters afterwards is still possible.
        /// </summary>
        public int depthStabilization;
        /// <summary>
        /// Optional path where the ZED SDK has to search for the settings file (<i>SN<XXXX>.conf</i> file).
        ///
        /// This file contains the calibration information of the camera.
        /// \n Default: ""
        /// 
        /// \note The settings file will be searched in the default directory: <ul>
        /// <li><b>Linux</b>: <i>/usr/local/zed/settings/</i></li> 
        /// <li><b>Windows</b>: <i>C:/ProgramData/stereolabs/settings</i></li></ul>
        /// 
        /// \note If a path is specified and no file has been found, the ZED SDK will search the settings file in the default directory.
        /// \note An automatic download of the settings file (through <b>ZED Explorer</b> or the installer) will still download the files on the default path.
        /// </summary>
        public string optionalSettingsPath = "";
        /// <summary>
        /// Requires the successful opening of the motion sensors before opening the camera.
        ///
        /// Default: false.
        ///
        /// \note If set to false, the ZED SDK will try to <b>open and use</b> the IMU (second USB device on USB2.0) and will open the camera successfully even if the sensors failed to open.
        ///
        /// This can be used for example when using a USB3.0 only extension cable (some fiber extension for example).
        /// \note This parameter only impacts the LIVE mode.
        /// \note If set to true, sl.Camera.Open() will fail if the sensors cannot be opened.
        /// \note This parameter should be used when the IMU data must be available, such as object detection module or when the gravity is needed.
        /// 
        /// \n\note This setting is not taken into account for \ref sl.MODEL.ZED camera since it does not include sensors.
        /// </summary>
        public bool sensorsRequired;
        
        /// <summary>
        /// IP address of the streaming sender to connect to.
        /// </summary>
        
        public string ipStream = "";

        /// <summary>
        /// Port of the streaming sender to connect to.
        /// </summary>
        public ushort portStream = 30000;
        /// <summary>
        /// Enable the Enhanced Contrast Technology, to improve image quality.
        ///
        /// Default: True.
        /// 
        /// \n If set to true, image enhancement will be activated in camera ISP. Otherwise, the image will not be enhanced by the IPS.
        /// \note This only works for firmware version starting from 1523 and up.
        /// </summary>
        public bool enableImageEnhancement = true;
        /// <summary>
        /// Optional path where the ZED SDK can find a file containing the calibration information of the camera computed by OpenCV.
        ///
        /// \note Using this will disable the factory calibration of the camera.
        /// \note The file must be in a XML/YAML/JSON formatting provided by OpenCV.
        /// \note It also must contain the following keys: Size, K_LEFT (intrinsic left), K_RIGHT (intrinsic right),
        /// D_LEFT (distortion left), D_RIGHT (distortion right), R (extrinsic rotation), T (extrinsic translation).
        /// \warning Erroneous calibration values can lead to poor accuracy in all ZED SDK modules.
        /// </summary>
        public string optionalOpencvCalibrationFile;
        /// <summary>
        /// Define a timeout in seconds after which an error is reported if the sl.Camera.Open() method fails.
        ///
        /// Set to '-1' to try to open the camera endlessly without returning error in case of failure.
        /// \n Set to '0' to return error in case of failure at the first attempt.
        /// \n Default: 5.0
        /// \note This parameter only impacts the LIVE mode.
        /// </summary>
        public float openTimeoutSec;

        /// <summary>
        /// Define the behavior of the automatic camera recovery during sl.Camera.Grab() method call.
        ///
        /// When async is enabled and there's an issue with the communication with the sl.Camera object,
        /// sl.Camera.Grab() will exit after a short period and return the \ref ERROR_CODE "sl.ERROR_CODE.CAMERA_REBOOTING" warning.
        /// \n The recovery will run in the background until the correct communication is restored.
        /// \n When \ref asyncGrabCameraRecovery is false, the sl.Camera.Grab() method is blocking and will return
        /// only once the camera communication is restored or the timeout is reached. 
        /// \n Default: false
        /// </summary>

        public bool asyncGrabCameraRecovery;
        /// <summary>
        /// Define a computation upper limit to the grab frequency.
        ///
        /// This can be useful to get a known constant fixed rate or limit the computation load while keeping a short exposure time by setting a high camera capture framerate.
        /// \n The value should be inferior to the sl.InitParameters.camera_fps and strictly positive.
        /// \note  It has no effect when reading an SVO file.
        ///
        /// This is an upper limit and won't make a difference if the computation is slower than the desired compute capping FPS.
        /// \note Internally the sl.Camera.grab() method always tries to get the latest available image while respecting the desired FPS as much as possible.
        /// </summary>
        public float grabComputeCappingFPS = 0;
        /// <summary>
        /// Enable or disable the image validity verification.
        /// This will perform additional verification on the image to identify corrupted data.This verification is done in the grab function and requires some computations.
        /// If an issue is found, the grab function will output a warning as sl.ERROR_CODE.CORRUPTED_FRAME.
        /// This version doesn't detect frame tearing currently.
        ///  \n default: disabled
        /// </summary>
        public bool enableImageValidityCheck;


        /// <summary>
        /// Default constructor.
        ///
        /// All the parameters are set to their default and optimized values.
        /// </summary>
        public InitParameters()
        {
            this.inputType = sl.INPUT_TYPE.USB;
            this.resolution = RESOLUTION.HD720;
            this.cameraFPS = 60;
            this.cameraDeviceID = 0;
            this.pathSVO = "";
            this.svoRealTimeMode = false;
            this.coordinateUnits = UNIT.METER;
            this.coordinateSystem = COORDINATE_SYSTEM.IMAGE;
            this.depthMode = DEPTH_MODE.PERFORMANCE;
            this.depthMinimumDistance = -1;
            this.depthMaximumDistance = -1;
            this.cameraImageFlip = FLIP_MODE.AUTO;
            this.cameraDisableSelfCalib = false;
            this.sdkVerbose = 0;
            this.sdkGPUId = -1;
            this.sdkVerboseLogFile = "";
            this.enableRightSideMeasure = false;
            this.depthStabilization = 1;
            this.optionalSettingsPath = "";
            this.sensorsRequired = false;
            this.ipStream = "";
            this.portStream = 30000;
            this.enableImageEnhancement = true;
            this.optionalOpencvCalibrationFile = "";
            this.openTimeoutSec = 5.0f;
            this.asyncGrabCameraRecovery = false;
            this.grabComputeCappingFPS = 0;
            this.enableImageValidityCheck = false;
        }

    }

    ///\ingroup  Video_group
    /// <summary>
    /// Lists available input types in the ZED SDK.
    /// </summary>
    public enum INPUT_TYPE
    {
        /// <summary>
        /// USB input mode
        /// </summary>
        USB,
        /// <summary>
        /// SVO file input mode
        /// </summary>
        SVO,
        /// <summary>
        /// STREAM input mode (requires to use \ref Camera.EnableStreaming "EnableStreaming()" /
        /// \ref Camera.DisableStreaming "DisableStreaming()" on the "sender" side)
        /// </summary>
        STREAM,
        /// <summary>
        /// GMSL input mode
        /// </summary>
        GMSL
    };

    ///\ingroup Video_group
    /// <summary>
    /// Lists available LIVE input type in the ZED SDK.
    /// </summary>
    public enum BUS_TYPE
    {
        /// <summary>
        /// USB input mode
        /// </summary>
        USB,
        /// <summary>
        /// GMSL input mode
        /// \note Only on NVIDIA Jetson.
        /// </summary>
        GMSL,
        /// <summary>
        /// Automatically select the input type.\n Trying first for available USB cameras, then GMSL.
        /// </summary>
        AUTO,
        ///@cond SHOWHIDDEN 
        LAST
        ///@endcond 
    };



    ///\ingroup  Video_group
    /// <summary>
    /// Lists possible camera states.
    /// </summary>
    public enum CAMERA_STATE
    {
        /// <summary>
        /// The camera can be opened by the ZED SDK.
        /// </summary>
        AVAILABLE,
        /// <summary>
        /// The camera is already opened and unavailable.
        /// </summary>
        NOT_AVAILABLE
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Structure containing the options used to record.
    /// \note For more info, read about the ZED SDK C++ class it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/structsl_1_1RecordingParameters.html">RecordingParameters</a>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RecordingParameters
    {
        /// <summary>
        /// Filename of the file to save the recording into.
        /// </summary>
        public string videoFilename;
    
        /// <summary>
        /// Compression mode the recording.
        ///
        /// Default: sl.SVO_COMPRESSION_MODE.H264_BASED
        /// </summary>

        public SVO_COMPRESSION_MODE compressionMode;
        /// <summary>
        /// Framerate for the recording file.
        ///
        /// Default: 0 (camera framerate will be taken)
        /// \warning This framerate must be below or equal to the camera framerate and camera framerate must be a multiple of the target framerate.
        /// \warning It means that it must respect <code> cameraFPS%targetFPS == 0</code>.
        /// \warning Allowed framerates are 15,30, 60 or 100 if possible.
        /// \warning Any other values will be discarded and camera FPS will be taken.
        /// </summary>
        public int targetFPS;

        /// <summary>
        /// Overrides the default bitrate of the SVO file, in kbits/s.
        ///
        /// Default: 0 (the default values associated with the resolution)
        /// \note Only works if \ref compressionMode is sl.SVO_COMPRESSION_MODE.H264_BASED or sl.SVO_COMPRESSION_MODE.H265_BASED.
        /// \note Available range: 0 or [1000 - 60000]
        /// </summary>
        public uint bitrate;

        /// <summary>
        /// Defines whether to decode and re-encode a streaming source.
        ///
        /// Default: false
        /// \note If set to false, it will avoid decoding/re-encoding and convert directly streaming input into a SVO file.
        /// \note This saves a encoding session and can be especially useful on NVIDIA Geforce cards where the number of encoding session is limited.
        /// \note \ref compressionMode, \ref targetFPS and \ref bitrate will be ignored in this mode.
        /// </summary>
        public bool transcode;
        /// <summary>
        /// Default constructor.
        ///
        /// All the parameters are set to their default values.
        /// </summary>
        public RecordingParameters(string filename = "", SVO_COMPRESSION_MODE compression = SVO_COMPRESSION_MODE.H264_BASED, uint bitrate = 0, int fps = 0, bool transcode = false)
        {
            this.videoFilename = filename;
            this.compressionMode = compression;
            this.bitrate = bitrate;
            this.targetFPS = fps;
            this.transcode = transcode;
        }
    }

    ///\ingroup  Video_group
    /// <summary>
    /// Structure containing the options used to stream with the ZED SDK.
    /// \note For more info, read about the ZED SDK C++ class it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/structsl_1_1StreamingParameters.html">StreamingParameters</a>
    /// </summary>
    public struct StreamingParameters
    {
        /// <summary>
        /// Encoding used for streaming.
        /// </summary>
        public STREAMING_CODEC codec;

        /// <summary>
        /// Port used for streaming.
        /// \warning Port must be an even number. Any odd number will be rejected.
        /// \warning Port must be opened.
        /// </summary>
        public ushort port;
        /// <summary>
        /// Defines the streaming bitrate in Kbits/s.
        /// | sl.STREAMING_CODEC  | sl.RESOLUTION   | FPS   | Bitrate (kbps) |
        /// |------------------|--------------|-------|----------------|
        /// | H264             |  HD2K        |   15  |     8500       |
        /// | H264             |  HD1080      |   30  |    12500       |
        /// | H264             |  HD720       |   60  |     7000       |
        /// | H265             |  HD2K        |   15  |     7000       |
        /// | H265             |  HD1080      |   30  |    11000       |
        /// | H265             |  HD720       |   60  |     6000       |
        /// Default: 0 (it will be set to the best value depending on your resolution/FPS)
        /// \note Available range: [1000 - 60000]
        /// </summary>
        public uint bitrate;

        /// <summary>
        /// GOP size in number of frames.
        /// 
        /// Default: -1 (the GOP size will last at maximum 2 seconds, depending on camera FPS)
        /// \note The GOP size determines the maximum distance between IDR/I-frames. Very high GOP size will result in slightly more efficient compression, especially on static scenes. But latency will increase.
        /// \note Maximum value: 256
    
        /// </summary>
        public int gopSize;
        /// <summary>
        /// Defines whether the adaptive bitrate is enable.
        ///
        /// Default: false
        /// \note Bitrate will be adjusted depending the number of packet dropped during streaming.
        /// \note If activated, the bitrate can vary between [bitrate/4, bitrate].
        /// \warning Currently, the adaptive bitrate only works when "sending" device is a NVIDIA Jetson (X1, X2, Xavier, Nano).
        /// </summary>
    
        public bool adaptativeBitrate;
        /// <summary>
        /// Size of a single chunk.
        ///
        /// Default: 16084
        /// \note Stream buffers are divided into X number of chunks where each chunk is  \ref chunkSize bytes long.
        /// \note You can lower \ref chunkSize value if network generates a lot of packet lost: this will
        /// generates more chunk for a single image, but each chunk sent will be lighter to avoid inside-chunk corruption.
        /// \note Increasing this value can decrease latency.
        ///
        /// \n \note Available range: [1024 - 65000]
        /// </summary>
        public ushort chunkSize;

        /// <summary>
        /// Framerate for the streaming output.
        ///
        /// Default: 0 (camera framerate will be taken)
        /// \warning This framerate must be below or equal to the camera framerate.
        /// \warning Allowed framerates are 15, 30, 60 or 100 if possible.
        /// \warning Any other values will be discarded and camera FPS will be taken.
        /// </summary>
        public int targetFPS;

        /// <summary>
        /// Default constructor.
        ///
        /// All the parameters are set to their default values.
        /// </summary>
        public StreamingParameters(STREAMING_CODEC codec = STREAMING_CODEC.H264_BASED, ushort port = 3000, uint bitrate = 8000,
                                    int gopSize = -1, bool adaptativeBitrate = false, ushort chunkSize = 32789, int targetFPS = 0)
        {
            this.codec = codec;
            this.port = port;
            this.bitrate = bitrate;
            this.gopSize = gopSize;
            this.adaptativeBitrate = adaptativeBitrate;
            this.chunkSize = chunkSize;
            this.targetFPS = targetFPS;
        }
    }

    ///\ingroup  Video_group
    /// <summary>
    /// Structure containing information about the properties of a camera.
    /// </summary>
    /// \note A \ref cameraModel sl.MODEL.ZED_M with an id '-1' can be due to an inverted USB-C cable.
    [StructLayout(LayoutKind.Sequential)]
    public struct DeviceProperties
    {
        /// <summary>
        /// State of the camera.
        ///
        /// Default: Default: sl.CAMERA_STATE.NOT_AVAILABLE
        /// </summary>
        public sl.CAMERA_STATE cameraState;

        /// <summary>
        /// Id of the camera.
        /// 
        /// Default: -1
        /// </summary>
        public int id;

        /// <summary>
        /// System path of the camera.
        /// </summary>
        public string path;

        /// <summary>
        /// Model of the camera.
        /// </summary>
        public sl.MODEL cameraModel;

        /// <summary>
        /// Serial number of the camera.
        ///
        /// Default: 0
        /// \warning Not provided for Windows.
        /// </summary>
        public uint sn;

        /// <summary>
        /// Input type of the camera.
        /// </summary>
        public sl.INPUT_TYPE inputType;

    };

    ///\ingroup  Video_group
    /// <summary>
    /// Structure containing information about the properties of a streaming device. 
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct StreamingProperties
    {
        /// <summary>
        /// IP address of the streaming device.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 16)]
        public string ip;

        /// <summary>
        /// Streaming port of the streaming device.
        ///
        /// Default: 0
        /// </summary>
        public ushort port;

        /// <summary>
        /// Current bitrate of encoding of the streaming device.
        ///
        /// Default: 0
        /// </summary>
        public int currentBitrate;

        /// <summary>
        /// Current codec used for compression in streaming device.
        ///
        /// Default: sl.STREAMING_CODEC.H265_BASED
        /// </summary>
        public sl.STREAMING_CODEC codec;
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Structure containing information about the status of the recording.
    /// \note For more info, read about the ZED SDK C++ struct it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/structsl_1_1RecordingStatus.html">RecordingStatus</a>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RecordingStatus
    {
        /// <summary>
        /// Report if the recording has been enabled.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool is_recording;
        /// <summary>
        /// Report if the recording has been paused.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool is_paused;
        /// <summary>
        /// Status of current frame.
        ///
        /// True for success or false if the frame could not be written in the SVO file.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool status;
        /// <summary>
        /// Compression time for the current frame in milliseconds.
        /// </summary>
        public double current_compression_time;
        /// <summary>
        /// Compression ratio (% of raw size) for the current frame.
        /// </summary>
        public double current_compression_ratio;
        /// <summary>
        /// Average compression time in milliseconds since beginning of recording.
        /// </summary>
        public double average_compression_time;
        /// <summary>
        /// Average compression ratio (% of raw size) since beginning of recording.
        /// </summary>
        public double average_compression_ratio;
    }

    ///\ingroup  Video_group
    /// <summary>
    /// Lists available resolutions.
    /// \note The VGA resolution does not respect the 640*480 standard to better fit the camera sensor (672*376 is used).
    /// \warning All resolutions are not available for every camera.
    /// \warning You can find the available resolutions for each camera in <a href="https://www.stereolabs.com/docs/video/camera-controls#selecting-a-resolution">our documentation</a>.
    /// </summary>
    public enum RESOLUTION
    {
        /// <summary>
        /// 2208*1242 (x2)
        /// \n Available FPS: 15
        /// </summary>
        HD2K,
        /// <summary>
        /// 1920*1080 (x2)
        /// \n Available FPS: 15, 30
        /// </summary>
        HD1080,
        /// <summary>
        /// 1920*1200 (x2)
        /// \n Available FPS: 15, 30, 60
        /// </summary>
        HD1200,
        /// <summary>
        /// 1280*720 (x2)
        /// \n Available FPS: 15, 30, 60
        /// </summary>
        HD720,
        /// <summary>
        /// 960*600 (x2)
        /// \n Available FPS: 15, 30, 60, 120
        /// </summary>
        HDSVGA,
        /// <summary>
        /// 672*376 (x2)
        /// \n Available FPS: 15, 30, 60, 100
        /// </summary>
        VGA,
        /// <summary>
        /// Select the resolution compatible with the camera:
        /// - ZED X/X Mini: HD1200
        /// - other cameras: HD720
        /// </summary>
        AUTO
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Lists possible flip modes of the camera.
    /// </summary>
    public enum FLIP_MODE
    {
        /// <summary>
        /// No flip applied. Default behavior.
        /// </summary>
        OFF = 0,
        /// <summary>
        /// Images and camera sensors' data are flipped useful when your camera is mounted upside down.
        /// </summary>
        ON = 1,
        /// <summary>
        /// In LIVE mode, use the camera orientation (if an IMU is available) to set the flip mode.
        /// \n In SVO mode, read the state of this enum when recorded.
        /// </summary>
        AUTO = 2,
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Lists ZED camera model. <a href="https://www.stereolabs.com/docs/api/c/types__c_8h.html#aa71ca736d522c5f280cf90450665e749">SL_MODEL</a> in C wrapper.
    /// </summary>
    public enum MODEL
    {
        /// <summary>
        /// ZED camera model
        /// </summary>
	    ZED,
        /// <summary>
        /// ZED Mini (ZED M) camera model
        /// </summary>
	    ZED_M,
        /// <summary>
        /// ZED 2 camera model
        /// </summary>
        ZED2,
        /// <summary>
        /// ZED 2i camera model
        /// </summary>
        ZED2i,
        /// <summary>
        /// ZED X camera model
        /// </summary>
        ZED_X,
        /// <summary>
        /// ZED X Mini (ZED XM) camera model
        /// </summary>
        ZED_XM
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Lists available views.
    /// </summary>
    /// \note For more info, read about the ZED SDK C++ enum it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c">VIEW</a>
    /// </remarks>
    public enum VIEW
    {
        /// <summary>
        /// Left BGRA image. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// </summary>
        LEFT,
        /// <summary>
        ///  Right BGRA image. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// </summary>
        RIGHT,
        /// <summary>
        /// Left gray image. Each pixel contains 1 unsigned char.
        ///\n Type: sl.MAT_TYPE.MAT_8U_C1.
        /// </summary>
        LEFT_GREY,
        /// <summary>
        /// Right gray image. Each pixel contains 1 unsigned char.
        ///\n Type: sl.MAT_TYPE.MAT_8U_C1.
        /// </summary>
        RIGHT_GREY,
        /// <summary>
        /// Left BGRA unrectified image. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// </summary>
        LEFT_UNRECTIFIED,
        /// <summary>
        /// Right BGRA unrectified image. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// </summary>
        RIGHT_UNRECTIFIED,
        /// <summary>
        /// Left gray unrectified image. Each pixel contains 1 unsigned char.
        ///\n Type: sl.MAT_TYPE.MAT_8U_C1.
        /// </summary>
        LEFT_UNRECTIFIED_GREY,
        /// <summary>
        /// Right gray unrectified image. Each pixel contains 1 unsigned char.
        ///\n Type: sl.MAT_TYPE.MAT_8U_C1.
        /// </summary>
        RIGHT_UNRECTIFIED_GREY,
        /// <summary>
        /// Left and right image (the image width is therefore doubled). Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// </summary>
        SIDE_BY_SIDE,
        /// <summary>
        /// Color rendering of the depth. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// \note Use \ref MEASURE "sl.MEASURE.DEPTH" with sl.Camera.RetrieveMeasure() to get depth values.
        /// </summary>
        DEPTH,
        /// <summary>
        /// Color rendering of the depth confidence. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// \note Use \ref MEASURE "sl.MEASURE.CONFIDENCE" with sl.Camera.RetrieveMeasure() to get confidence values.
        /// </summary>
        CONFIDENCE,
        /// <summary>
        /// Color rendering of the normals. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// \note Use \ref MEASURE "sl.MEASURE.NORMALS" with sl.Camera.RetrieveMeasure() to get normal values.
        /// </summary>
        NORMALS,
        /// <summary>
        /// Color rendering of the right depth mapped on right sensor. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// \note Use \ref MEASURE "sl.MEASURE.DEPTH_RIGHT" with sl.Camera.RetrieveMeasure() to get depth right values.
        /// </summary>
        DEPTH_RIGHT,
        /// <summary>
        /// Color rendering of the normals mapped on right sensor. Each pixel contains 4 unsigned char (B, G, R, A).
        ///\n Type: sl.MAT_TYPE.MAT_8U_C4.
        /// \note Use \ref MEASURE "sl.MEASURE.NORMALS_RIGHT" with sl.Camera.RetrieveMeasure() to get normal right values.
        /// </summary>
        NORMALS_RIGHT,
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Lists available camera settings for the camera (contrast, hue, saturation, gain, ...).
    /// \warning GAIN and EXPOSURE are linked in auto/default mode (see \ref sl.Camera.SetCameraSettings()).
    /// </summary>
    public enum VIDEO_SETTINGS
    {
        /// <summary>
        /// Brightness control.
        /// \n Affected value should be between 0 and 8.
        /// \note Not available for ZED X/X Mini cameras.
        /// </summary>
        BRIGHTNESS,
        /// <summary>
        /// Contrast control
        /// \n Affected value should be between 0 and 8.
        /// \note Not available for ZED X/X Mini cameras.
        /// </summary>
        CONTRAST,
        /// <summary>
        /// Hue control
        /// \n Affected value should be between 0 and 11.
        /// \note Not available for ZED X/X Mini cameras.
        /// </summary>
        HUE,
        /// <summary>
        /// Saturation control
        /// \n Affected value should be between 0 and 8.
        /// </summary>
        SATURATION,
        /// <summary>
        /// Digital sharpening control
        /// \n Affected value should be between 0 and 8.
        /// </summary>
        SHARPNESS,
        /// <summary>
        /// ISP gamma control
        /// \n Affected value should be between 1 and 9.
        /// </summary>
        GAMMA,
        /// <summary>
        /// Gain control
        /// \n Affected value should be between 0 and 100 for manual control.
        /// \note If EXPOSURE is set to -1 (automatic mode), then \ref VIDEO_SETTINGS.GAIN "GAIN" will be automatic as well.
        /// </summary>
        GAIN,
        /// <summary>
        /// Exposure control
        /// \n Affected value should be between 0 and 100 for manual control.
        /// \n The exposition is mapped linearly in a percentage of the following max values.
        /// \n Special case for <code>EXPOSURE = 0</code> that corresponds to 0.17072ms.
        /// \n The conversion to milliseconds depends on the framerate: <ul>
        /// <li>15fps <code>EXPOSURE = 100</code> -> 19.97ms</li>
        /// <li>30fps <code>EXPOSURE = 100</code> -> 19.97ms</li>
        /// <li>60fps <code>EXPOSURE = 100</code> -> 10.84072ms</li>
        /// <li>100fps <code>EXPOSURE = 100</code> -> 10.106624ms</li></ul>
        /// </summary>
        EXPOSURE,
        /// <summary>
        /// Defines if the \ref VIDEO_SETTINGS.GAIN "GAIN" and \ref VIDEO_SETTINGS.EXPOSURE "EXPOSURE" are in automatic mode or not.
        /// \n Setting \ref VIDEO_SETTINGS.GAIN "GAIN" or \ref VIDEO_SETTINGS.EXPOSURE "EXPOSURE" values will automatically set this value to 0.
        /// </summary>
        AEC_AGC,
        /// <summary>
        /// Defines the region of interest for automatic exposure/gain computation.
        /// \n To be used with overloaded \ref Camera.SetCameraSettings(VIDEO_SETTINGS,SIDE,Rect,bool) "SetCameraSettings()"
        /// / \ref Camera.GetCameraSettings(VIDEO_SETTINGS,SIDE,ref Rect) "GetCameraSettings()" methods.
        /// </summary>
        AEC_AGC_ROI,
        /// <summary>
        /// Color temperature control
        /// \n Affected value should be between 2800 and 6500 with a step of 100.
        /// \note Setting a value will automatically set \ref sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO "WHITEBALANCE_AUTO" to 0.
        /// </summary>
        WHITEBALANCE_TEMPERATURE,
        /// <summary>
        /// Defines if the white balance is in automatic mode or not.
        /// </summary>
        WHITEBALANCE_AUTO,
        /// <summary>
        /// Status of the front LED of the camera.
        /// \n Set to 0 to disable the light, 1 to enable the light.
        /// \n Default value is on.
        /// \note Requires camera firmware 1523 at least.
        /// </summary>
        LED_STATUS,
        /// <summary>
        /// Real exposure time control in microseconds.
        /// \note Only available for ZED X/X Mini cameras.
        /// \note Replace \ref VIDEO_SETTINGS.EXPOSURE "EXPOSURE" setting.
        /// </summary>
        EXPOSURE_TIME,
        /// <summary>
        /// Real analog gain (sensor) control in mDB.
        /// \n The range is defined by Jetson DTS and by default [1000-16000].
        /// \note Only available for ZED X/X Mini cameras.
        /// \note Replace \ref VIDEO_SETTINGS.GAIN "GAIN" settings.
        /// </summary>
        ANALOG_GAIN,
        /// <summary>
        /// Real digital gain (ISP) as a factor.
        /// \n The range is defined by Jetson DTS and by default [1-256].
        /// \note Only available for ZED X/X Mini cameras.
        /// \note Replace \ref VIDEO_SETTINGS.GAIN "GAIN" settings.
        /// </summary>
        DIGITAL_GAIN,
        /// <summary>
        /// Range of exposure auto control in microseconds.
        /// \n Used with \ref Camera.SetCameraSettings(VIDEO_SETTINGS,int,int) "SetCameraSettings()".
        /// \n Min/max range between max range defined in DTS.
        /// \n By default : [28000 - <fps_time> or 19000] us.
        /// \note Only available for ZED X/X Mini cameras.
        /// </summary>
        AUTO_EXPOSURE_TIME_RANGE,
        /// <summary>
        /// Range of sensor gain in automatic control.
        /// \n Used with \ref Camera.SetCameraSettings(VIDEO_SETTINGS,int,int) "SetCameraSettings()".
        /// \n Min/max range between max range defined in DTS.
        /// \n By default: [1000 - 16000] mdB.
        /// \note Only available for ZED X/X Mini cameras.
        /// </summary>
        AUTO_ANALOG_GAIN_RANGE,
        /// <summary>
        /// Range of digital ISP gain in automatic control.
        /// \n Used with \ref Camera.SetCameraSettings(VIDEO_SETTINGS,int,int) "SetCameraSettings()".
        /// \n Min/max range between max range defined in DTS.
        /// \n By default: [1 - 256].
        /// \note Only available for ZED X/X Mini cameras.
        /// </summary>
        AUTO_DIGITAL_GAIN_RANGE,
        /// <summary>
        /// Exposure-target compensation made after auto exposure.
        /// \n Reduces the overall illumination target by factor of F-stops.
        /// \n Affected value should be between 0 and 100 (mapped between [-2.0,2.0]).
        /// \n Default value is 50, i.e. no compensation applied.
        /// \note Only available for ZED X/X Mini cameras.
        /// </summary>
        EXPOSURE_COMPENSATION,
        /// <summary>
        /// Level of denoising applied on both left and right images.
        /// \n Affected value should be between 0 and 100.
        /// \n Default value is 50.
        /// \note Only available for ZED X/X Mini cameras.
        /// </summary>
        DENOISING,
        ///@cond SHOWHIDDEN 
        LAST
        ///@endcond
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Lists possible time references for timestamps or data.
    /// </summary>
    public enum TIME_REFERENCE
    {
        /// <summary>
        /// The requested timestamp or data will be at the time of the frame extraction.
        /// </summary>
        IMAGE,
        /// <summary>
        /// The requested timestamp or data will be at the time of the function call.
        /// </summary>
        CURRENT
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Lists available compression modes for SVO recording.
    /// \note \ref sl.SVO_COMPRESSION_MODE.LOSSLESS_BASED "LOSSLESS_BASED" is an improvement of previous lossless compression (used in ZED Explorer),
    /// even if size may be bigger, compression time is much faster.
    /// </summary>
    public enum SVO_COMPRESSION_MODE
    {
        /// <summary>
        /// PNG/ZSTD (lossless) CPU based compression.
        /// \n Average size: 42% of RAW
        /// </summary>
        LOSSLESS_BASED,
        /// <summary>
        /// H264 (AVCHD) GPU based compression.
        /// \n Average size: 1% of RAW
        /// \note Requires a NVIDIA GPU.
        /// </summary>
        H264_BASED,
        /// <summary>
        /// H265 (HEVC) GPU based compression.
        /// \n Average size: 1% of RAW
        /// \note Requires a NVIDIA GPU.
        /// </summary>
        H265_BASED,
        /// <summary>
        /// H264 Lossless GPU/Hardware based compression.
        /// \n Average size: 25% of RAW
        /// \n Provides a SSIM/PSNR result (vs RAW) >= 99.9%.
        /// \note Requires a NVIDIA GPU.
        /// </summary>
        H264_LOSSLESS_BASED,
        /// <summary>
        /// H265 Lossless GPU/Hardware based compression.
        /// \n Average size: 25% of RAW
        /// \n Provides a SSIM/PSNR result (vs RAW) >= 99.9%.
        /// \note Requires a NVIDIA GPU.
        /// </summary>
        H265_LOSSLESS_BASED,
    }

    ///\ingroup  Video_group
    /// <summary>
    /// Lists the different encoding types for image streaming.
    /// </summary>
    public enum STREAMING_CODEC
    {
        /// <summary>
        /// AVCHD/H264 encoding
        /// </summary>
        H264_BASED,
        /// <summary>
        /// HEVC/H265 encoding
        /// </summary>
        H265_BASED
    }
    /// <summary>
    /// Lists possible sides on which to get data from.
    /// </summary>
    public enum SIDE
    {
        /// <summary>
        /// Left side only.
        /// </summary>
        LEFT = 0,
        /// <summary>
        /// Right side only.
        /// </summary>
        RIGHT = 1,
        /// <summary>
        /// Left and right side.
        /// </summary>
        BOTH = 2
    }
    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Spatial Mapping //////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Spatial Mapping Module

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Class containing a set of parameters for the plane detection functionality.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be adjusted by the user.
    public class PlaneDetectionParameters
    {
        /// <summary>
        /// Controls the spread of plane by checking the position difference.
        /// </summary>
        /// Default: 0.15 meters
        public float maxDistanceThreshold = 0.15f;

        /// <summary>
        /// Controls the spread of plane by checking the angle difference.
        /// </summary>
        /// Default: 15 degrees
        public float normalSimilarityThreshold = 15.0f;
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Class containing a set of parameters for the spatial mapping module.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be adjusted by the user.
    public class SpatialMappingParameters
    {
        /// <summary>
        /// Spatial mapping resolution in meters.
        /// </summary>
        public float resolutionMeter = 0.05f;
        /// <summary>
        ///  Depth range in meters.
        /// </summary>
        /// Can be different from the value set by sl.InitParameters.depthMaximumDistance.
        /// \note Set to 0 by default. In this case, the range is computed from \ref resolutionMeter
        /// and from the current internal parameters to fit your application.
        public float rangeMeter = 0.0f;
        /// <summary>
        /// Whether to save the texture.
        /// </summary>
        /// If set to true, you will be able to apply the texture to your mesh after it is created.
        /// \note This option will consume more memory.
        /// \note This option is only available for sl.SPATIAL_MAP_TYPE.MESH.
        public bool saveTexture = false;
        /// <summary>
        /// Whether to only use chunks.
        /// </summary>
        /// If set to false, you will ensure consistency between the mesh and its inner chunk data.
        /// \note Updating the mesh is time-consuming.
        /// \note Setting this to true results in better performance.
        public bool useChunkOnly = false;
        /// <summary>
        /// The maximum CPU memory (in MB) allocated for the meshing process.
        /// </summary>
        public int maxMemoryUsage = 2048;
        /// <summary>
        /// Whether to inverse the order of the vertices of the triangles.
        /// </summary>
        /// If your display process does not handle front and back face culling, you can use this to correct it.
        /// \note This option is only available for sl.SPATIAL_MAP_TYPE.MESH.
        public bool reverseVertexOrder = false;
        /// <summary>
        /// The type of spatial map to be created. This dictates the format that will be used for the mapping(e.g. mesh, point cloud). See \ref SPATIAL_MAP_TYPE
        /// </summary>
        public SPATIAL_MAP_TYPE map_type = SPATIAL_MAP_TYPE.MESH;
        /// <summary>
        /// Control the integration rate of the current depth into the mapping process.
        /// </summary>
        /// This parameter controls how many times a stable 3D points should be seen before it is integrated into the spatial mapping.
        /// \n Default: 0 (this will define the stability counter based on the mesh resolution, the higher the resolution, the higher the stability counter)
        public int stabilityCounter = 0;
        /// <summary>
        /// Default constructor.
        /// </summary>
        /// Sets all parameters to their default and optimized values.
        public SpatialMappingParameters(float resolutionMeter = 0.05f, float rangeMeter = 0.0f, bool saveTexture = false, SPATIAL_MAP_TYPE map_type = SPATIAL_MAP_TYPE.MESH,
                                        bool useChunkOnly = false, int maxMemoryUsage = 2048, bool reverseVertexOrder = false, int stabilityCounter = 0)
        {
            this.resolutionMeter = resolutionMeter;
            this.rangeMeter = rangeMeter;
            this.saveTexture = saveTexture;
            this.map_type = map_type;
            this.useChunkOnly = useChunkOnly;
            this.maxMemoryUsage = maxMemoryUsage;
            this.reverseVertexOrder = reverseVertexOrder;
            this.stabilityCounter = stabilityCounter;
        }
        /// <summary>
        /// Returns the value corresponding to a sl.MAPPING_RESOLUTION preset in meters.
        /// </summary>
        /// <param name="mappingResolution">The desired sl.MAPPING_RESOLUTION. Default: sl.MAPPING_RESOLUTION.MEDIUM</param>
        /// <returns>The value of \b mappingResolution in meters.</returns>
        public static float get(MAPPING_RESOLUTION mappingResolution = MAPPING_RESOLUTION.MEDIUM)
        {
            if (mappingResolution == MAPPING_RESOLUTION.HIGH)
            {
                return 0.05f;
            }
            else if (mappingResolution == MAPPING_RESOLUTION.MEDIUM)
            {
                return 0.10f;
            }
            if (mappingResolution == MAPPING_RESOLUTION.LOW)
            {
                return 0.15f;
            }
            return 0.10f;
        }
        /// <summary>
        /// Returns the value corresponding to a sl.MAPPING_RANGE preset in meters.
        /// </summary>
        /// <param name="mappingRange">The desired sl.MAPPING_RANGE. Default: sl.MAPPING_RANGE.MEDIUM</param>
        /// <returns>The value of \b mappingRange in meters.</returns>
        public static float get(MAPPING_RANGE mappingRange = MAPPING_RANGE.MEDIUM)
        {
            if (mappingRange == MAPPING_RANGE.NEAR)
            {
                return 3.5f;
            }
            else if (mappingRange == MAPPING_RANGE.MEDIUM)
            {
                return 5.0f;
            }
            if (mappingRange == MAPPING_RANGE.FAR)
            {
                return 10.0f;
            }
            return 5.0f;
        }
        /// <summary>
        /// Sets the resolution to a sl.MAPPING_RESOLUTION preset.
        /// </summary>
        /// <param name="mappingResolution">The desired sl.MAPPING_RESOLUTION. Default: sl.MAPPING_RESOLUTION.MEDIUM</param>
        public void set(MAPPING_RESOLUTION mappingResolution = MAPPING_RESOLUTION.MEDIUM)
        {
            if (mappingResolution == MAPPING_RESOLUTION.HIGH)
            {
                resolutionMeter = 0.05f;
            }
            else if (mappingResolution == MAPPING_RESOLUTION.MEDIUM)
            {
                resolutionMeter = 0.10f;
            }
            if (mappingResolution == MAPPING_RESOLUTION.LOW)
            {
                resolutionMeter = 0.15f;
            }
        }
        /// <summary>
        /// Sets the range to a sl.MAPPING_RANGE preset.
        /// </summary>
        /// <param name="mappingRange">The desired sl.MAPPING_RANGE. Default: sl.MAPPING_RANGE.MEDIUM</param>
        public void set(MAPPING_RANGE mappingRange = MAPPING_RANGE.MEDIUM)
        {
            if (mappingRange == MAPPING_RANGE.NEAR)
            {
                rangeMeter = 3.5f;
            }
            else if (mappingRange == MAPPING_RANGE.MEDIUM)
            {
                rangeMeter = 5.0f;
            }
            if (mappingRange == MAPPING_RANGE.FAR)
            {
                rangeMeter = 10.0f;
            }
        }
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists the spatial mapping resolution presets.
    /// </summary>
    public enum MAPPING_RESOLUTION
    {
        /// <summary>
        /// Creates a detailed geometry.
        /// \n Requires lots of memory.
        /// </summary>
        HIGH,
        /// <summary>
        /// Small variations in the geometry will disappear.
        /// \n Useful for big objects.
        /// </summary>
        MEDIUM,
        /// <summary>
        /// Keeps only huge variations of the geometry.
        /// \n Useful for outdoor purposes.
        /// </summary>
        LOW
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists the spatial mapping depth range presets.
    /// </summary>
    public enum MAPPING_RANGE
    {
        /// <summary>
        /// Geometry within 3.5 meters of the camera will be mapped.
        /// </summary>
        NEAR,
        /// <summary>
        /// Geometry within 5 meters of the camera will be mapped.
        /// </summary>
        MEDIUM,
        /// <summary>
        /// Objects as far as 10 meters away are mapped.
        /// \n Useful for outdoors.
        /// </summary>
        FAR
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists the types of spatial maps that can be created.
    /// </summary>
    public enum SPATIAL_MAP_TYPE
    {
        /// <summary>
        /// The geometry is represented by a set of vertices connected by edges and forming faces.
        /// \n No color information is available.
        /// </summary>
        MESH,
        /// <summary>
        /// The geometry is represented by a set of 3D colored points.
        /// </summary>
        FUSED_POINT_CLOUD
    };

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists available mesh file formats.
    /// </summary>
    public enum MESH_FILE_FORMAT
    {
        /// <summary>
        /// Contains only vertices and faces.
        /// </summary>
        PLY,
        /// <summary>
        /// Contains only vertices and faces encoded in binary.
        /// </summary>
        BIN,
        /// <summary>
        /// Contains vertices, normals, faces, and texture information (if possible).
        /// </summary>
        OBJ
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists available mesh filtering intensities.
    /// </summary>
    public enum MESH_FILTER
    {
        /// <summary>
        /// Clean the mesh by closing small holes and removing isolated faces.
        /// </summary>
        LOW,
        /// <summary>
        /// Soft faces decimation and smoothing.
        /// </summary>
        MEDIUM,
        /// <summary>
        /// Drastically reduce the number of faces and apply a soft smooth.
        /// </summary>
        HIGH,
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists the different states of spatial mapping.
    /// </summary>
    public enum SPATIAL_MAPPING_STATE
    {
        /// <summary>
        /// The spatial mapping is initializing.
        /// </summary>
        INITIALIZING,
        /// <summary>
        /// The depth and tracking data were correctly integrated in the mapping algorithm.
        /// </summary>
        OK,
        /// <summary>
        /// The maximum memory dedicated to the scanning has been reached.
        /// \n The mesh will no longer be updated.
        /// </summary>
        NOT_ENOUGH_MEMORY,
        /// <summary>
        /// sl.Camera.EnableSpatialMapping() wasn't called or the scanning was stopped and not relaunched.
        /// </summary>
        NOT_ENABLED,
        /// <summary>
        /// The effective FPS is too low to give proper results for spatial mapping.
        /// \n Consider using performance parameters (sl.DEPTH_MODE.PERFORMANCE, sl.MAPPING_RESOLUTION.LOW,
        /// low camera resolution (sl.RESOLUTION.VGA / sl.RESOLUTION.HDSVGA or sl.RESOLUTION.HD720).
        /// </summary>
        FPS_TOO_LOW
    }

    ///\ingroup Core_group
    /// <summary>
    /// Lists available units for measures.
    /// </summary>
    public enum UNIT
    {
        /// <summary>
        /// International System (1/1000 meters)
        /// </summary>
        MILLIMETER,
        /// <summary>
        /// International System (1/100 meters)
        /// </summary>
        CENTIMETER,
        /// <summary>
        /// International System (1 meter)
        /// </summary>
        METER,
        /// <summary>
        ///  Imperial Unit (1/12 foot)
        /// </summary>
        INCH,
        /// <summary>
        ///  Imperial Unit (1 feet)
        /// </summary>
        FOOT
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Lists the available plane types detected based on its orientation and whether detected by 
    /// sl.Camera.FindFloorPlane() or sl.Camera.FindPlaneAtHit().
    /// </summary>
    public enum PLANE_TYPE
    {
        /// <summary>
        /// Floor plane of a scene.
        /// \n Retrieved by sl.Camera.FindFloorPlane().
        /// </summary>
        FLOOR,
        /// <summary>
        /// Horizontal plane, such as a tabletop, floor, etc.
        /// \n Detected with sl.Camera.FindPlaneAtHit() using screen-space coordinates.
        /// </summary>
        HIT_HORIZONTAL,
        /// <summary>
        /// Vertical plane, such as a wall.
        /// \n Detected with sl.Camera.FindPlaneAtHit() using screen-space coordinates.
        /// </summary>
        HIT_VERTICAL,
        /// <summary>
        /// Plane at an angle neither parallel nor perpendicular to the floor.
        /// \n Detected with sl.Camera.FindPlaneAtHit() using screen-space coordinates.
        /// </summary>
        HIT_UNKNOWN
    };

    ///\ingroup SpatialMapping_group
    /// <summary>
    ///  Possible states of the ZED's spatial memory area export, for saving 3D features used
    ///  by the tracking system to relocalize the camera. This is used when saving a mesh generated
    ///  by spatial mapping when Save Mesh is enabled - a .area file is saved as well.
    /// </summary>
    public enum AREA_EXPORT_STATE
    {
        /// <summary>
        /// Spatial memory file has been successfully created.
        /// </summary>
        AREA_EXPORT_STATE_SUCCESS,
        /// <summary>
        /// Spatial memory file is currently being written to.
        /// </summary>
        AREA_EXPORT_STATE_RUNNING,
        /// <summary>
        /// Spatial memory file export has not been called.
        /// </summary>
        AREA_EXPORT_STATE_NOT_STARTED,
        /// <summary>
        /// Spatial memory contains no data; the file is empty.
        /// </summary>
        AREA_EXPORT_STATE_FILE_EMPTY,
        /// <summary>
        /// Spatial memory file has not been written to because of a bad file name.
        /// </summary>
        AREA_EXPORT_STATE_FILE_ERROR,
        /// <summary>
        /// Spatial memory has been disabled, so no file can be created.
        /// </summary>
        AREA_EXPORT_STATE_SPATIAL_MEMORY_DISABLED
    };

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Class representing a mesh and containing the geometric (and optionally texture) data of the scene captured by the spatial mapping module.
    /// </summary>
    /// By default the mesh is defined as a set of chunks.
    /// \n This way we update only the data that has to be updated avoiding a time consuming remapping process every time a small part of the sl.Mesh is updated.
    public class Mesh
    {
        /// <summary>
        /// Number of vertices per chunk/sub-mesh.
        /// </summary>
        public int[] nbVerticesInSubmesh = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Number of triangles per chunk/sub-mesh.
        /// </summary>
        public int[] nbTrianglesInSubmesh = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Number of indices per chunk/sub-mesh.
        /// </summary>
        public int[] updatedIndices = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Vertices count in current sub-mesh.
        /// </summary>
        public int nbVertices = 0;
        /// <summary>
        /// Triangle count in current sub-mesh.
        /// </summary>
        /// Every three values of \ref triangles are the indexes of the three vertices constituting a triangular face.
        public int nbTriangles = 0;
        /// <summary>
        /// Number of updated sub-meshes.
        /// </summary>
        public int nbUpdatedSubmesh = 0;
        /// <summary>
        /// Vector of vertices.
        /// </summary>
        public Vector3[] vertices = new Vector3[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Vector of of triangles/faces.
        /// </summary>
        /// Triangles are defined as a set of three vertices indexes ```{v1, v2, v3}```.
        public int[] triangles = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Vector of colors.
        /// </summary>
        public byte[] colors = new byte[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// UVs defines the 2D projection of each vertices onto the texture.
        /// </summary>
        /// Values are normalized [0, 1] and start from the bottom left corner of the texture (as requested by OpenGL).
        /// In order to display a textured mesh you need to bind the texture and then draw each triangle by picking its uv values.
        public Vector2[] uvs = null;
        /// <summary>
        /// Texture of the sl.Mesh.
        /// </summary>
        public IntPtr textures = IntPtr.Zero;
        /// <summary>
        /// Width and height of the sl.Mesh texture, if any.
        /// </summary>
        public int[] texturesSize = new int[2];
        /// <summary>
        /// Dictionary of all existing chunks.
        /// </summary>
        public Dictionary<int, Chunk> chunks = new Dictionary<int, Chunk>((int)Constant.MAX_SUBMESH);
    }

    /// <summary>
    /// A fused point cloud contains both geometric and color data of the scene captured by spatial mapping.
    /// </summary>
    public class FusedPointCloud
    {
        /// <summary>
        /// Array of vertices.
        /// </summary>
        /// Vertices are defined by colored 3D points ```{x, y, z, rgba}```.
        public Vector4[] vertices;
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Class representing a sub-mesh containing local vertices and triangles.
    /// </summary>
    public struct Chunk
    {
        /// <summary>
        /// Array of vertices.
        /// </summary>
        /// Vertices are defined by a 3D point.
        public Vector3[] vertices;
        /// <summary>
        /// Triangles (or faces) contains the index of its three vertices.
        /// </summary>
        /// It corresponds to the 3 vertices of the triangle ```{v1, v2, v3}```.
        public int[] triangles;
        /// <summary>
        /// Colors of the vertices.
        /// </summary>
        public byte[] colors;
    }

    /// <summary>
    /// Structure representing a plane defined by a point and a normal, or a plane equation.
    /// </summary>
    /// \note The plane measurements are expressed in reference defined by sl.RuntimeParameters.measure3DReferenceFrame.
    [StructLayout(LayoutKind.Sequential)]
    public struct PlaneData
    {
        /// <summary>
        /// sl.ERROR_CODE returned by the ZED SDK when the plane detection was attempted.
        /// </summary>
        public sl.ERROR_CODE ErrorCode;
        /// <summary>
        /// Type of the plane defined by its orientation.
        /// </summary>
        /// \note It is deduced from the gravity vector and is therefore not available with on sl.MODEL.ZED.
        public PLANE_TYPE Type;
        /// <summary>
        /// Plane normalized normal vector.
        /// </summary>
        public Vector3 PlaneNormal;
        /// <summary>
        /// Plane center point.
        /// </summary>
        public Vector3 PlaneCenter;
        /// <summary>
        /// Plane position relative to the global reference frame.
        /// </summary>
        public Vector3 PlaneTransformPosition;
        /// <summary>
        /// Plane orientation relative to the global reference frame.
        /// </summary>
        public Quaternion PlaneTransformOrientation;
        /// <summary>
        /// Plane equation coefficients ```{a, b, c, d}```.
        /// </summary>
        /// \note The plane equation has the following form: ```ax + by + cz = d```.
        public Vector4 PlaneEquation;
        /// <summary>
        /// Gets the width and height of the bounding rectangle around the plane contours.
        /// </summary>
        public Vector2 Extents;
        /// <summary>
        /// Size of \ref Bounds.
        /// </summary>
        public int BoundsSize;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
        /// <summary>
        /// Array of 3D points forming a polygon bounds corresponding to the current visible limits of the plane.
        /// </summary>
        public Vector3[] Bounds; //max 256 points
    }

    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Object Detection /////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Object Detection Module

    /// \ingroup Object_group
    /// <summary>
    /// Structure containing a set of parameters for batch object detection.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be user adjusted.
    [StructLayout(LayoutKind.Sequential)]
    public struct BatchParameters
    {
        /// <summary>
        /// Whether to enable the batch option in the object detection module.
        /// </summary>
        /// Batch queueing system provides:
        /// - deep-learning based re-identification
        /// - trajectory smoothing and filtering
        ///
        /// Default: false
        /// \note To activate this option, \ref enable must be set to true.
        [MarshalAs(UnmanagedType.U1)]
        public bool enable;
        /// <summary>
        /// Max retention time in seconds of a detected object.
        /// </summary>
        /// After this time, the same object will mostly have a different id.
        public float idRetentionTime;
        /// <summary>
        /// Trajectories will be output in batch with the desired latency in seconds.
        /// </summary>
        /// During this waiting time, re-identification of objects is done in the background.
        /// \note Specifying a short latency will limit the search (falling in timeout) for previously seen object ids but will be closer to real time output.
        /// \note Specifying a long latency will reduce the change of timeout in re-identification but increase difference with live output.
        public float latency;
    }


    /// <summary>
    /// Structure containing AI model status.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct AI_Model_status
    {
        /// <summary>
        /// The model file is currently present on the host.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool downloaded;
        /// <summary>
        /// An engine file with the expected architecture is found.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool optimized;
    };

    ///\ingroup Object_group
    /// <summary>
    /// Structure containing a set of parameters for the object detection module.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be user adjusted.
    [StructLayout(LayoutKind.Sequential)]
    public struct ObjectDetectionParameters
    {
        /// <summary>
        /// Id of the module instance.
        /// </summary>
        /// This is used to identify which object detection module instance is used.
        uint instanceModuleId;
        /// <summary>
        /// Whether the object detection is synchronized to the image or runs in a separate thread.
        /// </summary>
        /// If set to true, the detection is run on every sl.Camera.Grab().
        /// \n Otherwise, the thread runs at its own speed, which can lead to new detection once in a while.
        /// \n Default: true
        [MarshalAs(UnmanagedType.U1)]
        public bool imageSync;
        /// <summary>
        /// Whether the object detection system includes object tracking capabilities across a sequence of images.
        /// </summary>
        /// Default: true
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool enableObjectTracking;
        /// <summary>
        /// Whether the object masks will be computed.
        /// </summary>
        /// Default: false
        [MarshalAs(UnmanagedType.U1)]
        public bool enableSegmentation;
        /// <summary>
        /// sl.OBJECT_DETECTION_MODEL to use.
        /// </summary>
        /// Default: sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST
        public sl.OBJECT_DETECTION_MODEL detectionModel;

        /// <summary>
        /// Upper depth range for detections.
        /// </summary>
        /// Default: -1 (value set in sl.InitParameters.depthMaximumDistance)
        /// \note The value cannot be greater than sl.InitParameters.depthMaximumDistance and its unit is defined in sl.InitParameters.coordinateUnits.
        public float maxRange;
        /// <summary>
        /// Batching system parameters.
        /// </summary>
        /// Batching system (introduced in 3.5) performs short-term re-identification with deep-learning and trajectories filtering.
        /// \n sl.BatchParameters.enable must to be true to use this feature (by default disabled).
        public BatchParameters batchParameters;

        /// <summary>
        /// Filtering mode that should be applied to raw detections.
        /// </summary>
        /// Default: sl.OBJECT_FILTERING_MODE.NMS3D (same behavior as previous ZED SDK version)
        /// \note This parameter is only used in detection model [sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_XXX](\ref OBJECT_DETECTION_MODEL)
        /// and sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS.
        /// \note For custom object, it is recommended to use sl.OBJECT_FILTERING_MODE.NMS3D_PER_CLASS or sl.OBJECT_FILTERING_MODE.NONE.
        /// \note In this case, you might need to add your own NMS filter before ingesting the boxes into the object detection module.
        public OBJECT_FILTERING_MODE filteringMode;
        /// <summary>
        /// Prediction duration of the ZED SDK when an object is not detected anymore before switching its state to sl.OBJECT_TRACKING_STATE.SEARCHING.
        /// </summary>
        /// It prevents the jittering of the object state when there is a short misdetection.
        /// \n The user can define their own prediction time duration.
        /// \n Default: 0.2f
        /// \note During this time, the object will have sl.OBJECT_TRACKING_STATE.OK state even if it is not detected.
        /// \note The duration is expressed in seconds.
        /// \warning \ref predictionTimeout_s will be clamped to 1 second as the prediction is getting worse with time.
        /// \warning Setting this parameter to 0 disables the ZED SDK predictions.
        public float predictionTimeout_s;

        /// <summary>
        /// Whether to allow inference to run at a lower precision to improve runtime and memory usage.
        /// </summary>
        /// It might increase the initial optimization time and could include downloading calibration data or calibration cache and slightly reduce the accuracy.
        /// \note The fp16 is automatically enabled if the GPU is compatible and provides a speed up of almost x2 and reduce memory usage by almost half, no precision loss.
        /// \note This setting allow int8 precision which can speed up by another x2 factor (compared to fp16, or x4 compared to fp32) and half the fp16 memory usage, however some accuracy could be lost.
        /// \note The accuracy loss should not exceed 1-2% on the compatible models.
        /// \note The current compatible models are all [sl.AI_MODELS.HUMAN_BODY_XXXX](\ref AI_MODELS).
        public bool allowReducedPrecisionInference;
    };

    ///\ingroup Object_group
    /// <summary>
    /// Structure containing a set of runtime parameters for the object detection module.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be adjusted by the user.
    [StructLayout(LayoutKind.Sequential)]
    public struct ObjectDetectionRuntimeParameters
    {
        /// <summary>
        /// Confidence threshold.
        /// </summary>
        /// From 1 to 100, with 1 meaning a low threshold, more uncertain objects and 99 very few but very precise objects.
        /// \n Default: 20.f
        /// \note If the scene contains a lot of objects, increasing the confidence can slightly speed up the process, since every object instance is tracked.
        /// \note \ref detectionConfidenceThreshold is used as a fallback when sl::ObjectDetectionRuntimeParameters.objectConfidenceThreshold is partially set.
        public float detectionConfidenceThreshold;
        /// <summary>
        /// Defines which object types to detect and track.
        /// </summary>
        /// Default: ```new int[(int)sl.OBJECT_CLASS.LAST)]``` (all classes are tracked)
        /// \note Fewer object types can slightly speed up the process since every object is tracked.
        /// \note Will output only the selected classes.
        ///
        /// In order to get all the available classes, the filter list must be empty :
        /// \code
        /// objectClassFilter = new int[(int)sl.OBJECT_CLASS.LAST)];
        /// \endcode
        ///
        /// To select a set of specific object classes, like vehicles, persons and animals for instance:
        /// \code
        /// objectClassFilter[(int)sl.OBJECT_CLASS.PERSON] = Convert.ToInt32(true);
        /// objectClassFilter[(int)sl.OBJECT_CLASS.VEHICLE] = Convert.ToInt32(true);
        /// objectClassFilter[(int)sl.OBJECT_CLASS.ANIMAL] = Convert.ToInt32(true);
        /// \endcode
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)sl.OBJECT_CLASS.LAST)]
        public int[] objectClassFilter;

        /// <summary>
        /// Array of confidence thresholds for each class (can be empty for some classes).
        /// </summary>
        /// \note sl::ObjectDetectionRuntimeParameters.detectionConfidenceThreshold will be taken as fallback/default value.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)sl.OBJECT_CLASS.LAST)]
        public int[] objectConfidenceThreshold;
    };

    ///\ingroup Body_group
    /// <summary>
    /// Structure containing a set of parameters for the body tracking module.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be user adjusted.
    [StructLayout(LayoutKind.Sequential)]
    public struct BodyTrackingParameters
    {
        /// <summary>
        /// Id of the module instance.
        /// </summary>
        /// This is used to identify which body tracking module instance is used.
        uint instanceModuleId;
        /// <summary>
        /// Whether the body tracking is synchronized to the image or runs in a separate thread.
        /// </summary>
        /// If set to true, the detection is run on every sl.Camera.Grab().
        /// \n Otherwise, the thread runs at its own speed, which can lead to new detection once in a while.
        /// \n Default: true
        [MarshalAs(UnmanagedType.U1)]
        public bool imageSync;
        /// <summary>
        /// Whether the body tracking system includes body/person tracking capabilities across a sequence of images.
        /// </summary>
        /// Default: true
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool enableObjectTracking;
        /// <summary>
        /// Whether the body/person masks will be computed.
        /// </summary>
        /// Default: false
        [MarshalAs(UnmanagedType.U1)]
        public bool enableSegmentation;
        /// <summary>
        /// sl.BODY_TRACKING_MODEL to use.
        /// </summary>
        /// Default: sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
        public sl.BODY_TRACKING_MODEL detectionModel;
        /// <summary>
        /// Whether to apply the body fitting.
        /// </summary>
        /// Default: false
        [MarshalAs(UnmanagedType.U1)]
        public bool enableBodyFitting;
        /// <summary>
        /// Body format to be outputted by the ZED SDK with sl.Camera.RetrieveBodies().
        /// </summary>
        public sl.BODY_FORMAT bodyFormat;
        /// <summary>
        /// Upper depth range for detections.
        /// </summary>
        /// Default: -1 (value set in sl.InitParameters.depthMaximumDistance)
        /// \note The value cannot be greater than sl.InitParameters.depthMaximumDistance and its unit is defined in sl.InitParameters.coordinateUnits.
        public float maxRange;

#if false
        /// <summary>
        /// Batching system parameters.
        /// Batching system(introduced in 3.5) performs short-term re-identification with deep learning and trajectories filtering.
        /// BatchParameters.enable must to be true to use this feature (by default disabled)
        /// </summary>
        public BatchParameters batchParameters;
#endif
        /// <summary>
        /// Prediction duration of the ZED SDK when an object is not detected anymore before switching its state to sl.OBJECT_TRACKING_STATE.SEARCHING.
        /// </summary>
        /// It prevents the jittering of the object state when there is a short misdetection.
        /// \n The user can define their own prediction time duration.
        /// \n Default: 0.2f
        /// \note During this time, the object will have sl.OBJECT_TRACKING_STATE.OK state even if it is not detected.
        /// \note The duration is expressed in seconds.
        /// \warning \ref predictionTimeout_s will be clamped to 1 second as the prediction is getting worse with time.
        /// \warning Setting this parameter to 0 disables the ZED SDK predictions.
        public float predictionTimeout_s;

        /// <summary>
        /// Whether to allow inference to run at a lower precision to improve runtime and memory usage.
        /// </summary>
        /// It might increase the initial optimization time and could include downloading calibration data or calibration cache and slightly reduce the accuracy.
        /// \note The fp16 is automatically enabled if the GPU is compatible and provides a speed up of almost x2 and reduce memory usage by almost half, no precision loss.
        /// \note This setting allow int8 precision which can speed up by another x2 factor (compared to fp16, or x4 compared to fp32) and half the fp16 memory usage, however some accuracy could be lost.
        /// \note The accuracy loss should not exceed 1-2% on the compatible models.
        /// \note The current compatible models are all [sl.AI_MODELS.HUMAN_BODY_XXXX](\ref AI_MODELS).
        public bool allowReducedPrecisionInference;
    };

    /// \ingroup Body_group
    /// <summary>
    /// Structure containing a set of runtime parameters for the body tracking module.
    /// </summary>
    /// The default constructor sets all parameters to their default settings.
    /// \note Parameters can be adjusted by the user.
    [StructLayout(LayoutKind.Sequential)]
    public struct BodyTrackingRuntimeParameters
    {
        /// <summary>
        /// Confidence threshold.
        /// </summary>
        /// From 1 to 100, with 1 meaning a low threshold, more uncertain objects and 99 very few but very precise objects.
        /// \n Default: 20.f
        /// \note If the scene contains a lot of objects, increasing the confidence can slightly speed up the process, since every object instance is tracked.
        public float detectionConfidenceThreshold;

        /// <summary>
        /// Minimum threshold for the keypoints.
        /// </summary>
        /// The ZED SDK will only output the keypoints of the skeletons with threshold greater than this value.
        /// \n Default: 0
        /// \note It is useful, for example, to remove unstable fitting results when a skeleton is partially occluded.
        public int minimumKeypointsThreshold;

        /// <summary>
        /// Control of the smoothing of the fitted fused skeleton.
        /// </summary>
        /// It is ranged from 0 (low smoothing) and 1 (high smoothing).
        /// \n Default: 0
        public float skeletonSmoothing;
    };

    ///\ingroup Object_group
	/// <summary>
	/// Structure containing data of a detected object such as its \ref boundingBox, \ref label, \ref id and its 3D \ref position.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
    public struct ObjectData
    {
        /// <summary>
        /// Object identification number.
        /// </summary>
        /// It is used as a reference when tracking the object through the frames.
        /// \note Only available if sl.ObjectDetectionParameters.enableTracking is activated.
        /// \note Otherwise, it will be set to -1.
        public int id; 
        /// <summary>
        /// Unique id to help identify and track AI detections.
        /// </summary>
        /// It can be either generated externally, or by using sl.Camera.GenerateUniqueID() or left empty.
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 37)]
        public string uniqueObjectId;
        /// <summary>
        /// Object raw label.
        /// </summary>
        /// It is forwarded from sl.CustomBoxObjectData when using sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS.
        public int rawLabel;
        /// <summary>
        /// Object class/category to identify the object type.
        /// </summary>
		public sl.OBJECT_CLASS label;
        /// <summary>
        /// Object sub-class/sub-category to identify the object type.
        /// </summary>
        public sl.OBJECT_SUBCLASS sublabel;
        /// <summary>
        /// Object tracking state.
        /// </summary>
		public sl.OBJECT_TRACKING_STATE objectTrackingState;
        /// <summary>
        /// Object action state.
        /// </summary>
		public sl.OBJECT_ACTION_STATE actionState;
        /// <summary>
        /// Object 3D centroid.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        public Vector3 position;
        /// <summary>
        /// Detection confidence value of the object.
        /// </summary>
        /// From 0 to 100, a low value means the object might not be localized perfectly or the label (sl.OBJECT_CLASS) is uncertain.
		public float confidence;
        /// <summary>
        /// Mask defining which pixels which belong to the object (in \ref boundingBox and set to 255) and those of the background (set to 0).
        /// </summary>
        /// \warning The mask information is only available for tracked objects (sl.OBJECT_TRACKING_STATE.OK) that have a valid depth.
        /// \warning Otherwise, the mask will not be initialized.
		public System.IntPtr mask;
        /// <summary>
        /// 2D bounding box of the object represented as four 2D points starting at the top left corner and rotation clockwise.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \code
        /// A ------ B
        /// | Object |
        /// D ------ C
        /// \endcode
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public Vector2[] boundingBox2D;
        /// <summary>
        /// 3D centroid of the head of the object (a person).
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \warning Not available with [sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_XXX](\ref OBJECT_DETECTION_MODEL).
		public Vector3 headPosition; //object head position (only for HUMAN detectionModel)
        /// <summary>
        /// Object 3D velocity.
        /// </summary>
        /// \note It is defined in ```sl.InitParameters.coordinateUnits / s``` and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
		public Vector3 velocity; //object root velocity
        /// <summary>
        /// 3D object dimensions: width, height, length.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        public Vector3 dimensions;
        /// <summary>
        /// 3D bounding box of the object represented as eight 3D points.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \code
        ///    1 ------ 2
        ///   /        /|
        ///  0 ------ 3 |
        ///  | Object | 6
        ///  |        |/
        ///  4 ------ 7
        /// \endcode
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public Vector3[] boundingBox; // 3D Bounding Box of object
        /// <summary>
        /// 3D bounding box of the head of the object (a person) represented as eight 3D points.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \warning Not available with [sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_XXX](\ref OBJECT_DETECTION_MODEL).
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public Vector3[] headBoundingBox;// 3D Bounding Box of head (only for HUMAN detectionModel)
        /// <summary>
        /// 2D bounding box of the head of the object (a person) represented as four 2D points starting at the top left corner and rotation clockwise.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \warning Not available with [sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_XXX](\ref OBJECT_DETECTION_MODEL).
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public Vector2[] headBoundingBox2D;// 2D Bounding Box of head
        /// <summary>
        /// Covariance matrix of the 3D position.
        /// </summary>
        /// \note It is represented by its upper triangular matrix value
        /// \code
        ///      = [p0, p1, p2]
        ///        [p1, p3, p4]
        ///        [p2, p4, p5]
        /// \endcode
        /// where pi is ```positionCovariance[i]```
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] positionCovariance;// covariance matrix of the 3d position, represented by its upper triangular matrix value
    };

    ///\ingroup Object_group
    /// <summary>
    /// Structure that store externally detected objects.
    /// </summary>
    /// The objects can be ingested with sl.Camera.IngestCustomBoxObjects() to extract 3D and tracking information over time.
    [StructLayout(LayoutKind.Sequential)]
    public struct CustomBoxObjectData
    {
        /// <summary>
        /// Unique id to help identify and track AI detections.
        /// </summary>
        /// It can be either generated externally, or by using sl.Camera.GenerateUniqueID() or left empty.
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 37)]
        public string uniqueObjectID;
        /// <summary>
        /// 2D bounding box of the object represented as four 2D points starting at the top left corner and rotation clockwise.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \code
        /// A ------ B
        /// | Object |
        /// D ------ C
        /// \endcode
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public Vector2[] boundingBox2D;
        /// <summary>
        /// Object label.
        /// </summary>
        /// This information is passed-through and can be used to improve object tracking.
        /// \note It should define an object class. This means that any similar object (in classification) should share the same label number.
        public int label;
        /// <summary>
        /// Detection confidence value of the object.
        /// </summary>
        /// \note The value should be in ```[0-1]```.
        /// \note It can be used to improve the object tracking.
        public float probability;
        /// <summary>
        /// Provide hypothesis about the object movements (degrees of freedom or DoF) to improve the object tracking.
        /// </summary>
        /// - true: 2 DoF projected alongside the floor plane. Case for object standing on the ground such as person, vehicle, etc. 
        /// \n The projection implies that the objects cannot be superposed on multiple horizontal levels. 
        /// - false: 6 DoF (full 3D movements are allowed).
        ///
        /// \note This parameter cannot be changed for a given object tracking id.
        /// \note It is advised to set it by labels to avoid issues.
        [MarshalAs(UnmanagedType.U1)]
        public bool isGrounded;
    }

    ///\ingroup Object_group
    /// <summary>
    /// Structure containing the results of the object detection module.
    /// </summary>
    /// It contains the number of object in the scene (\ref numObject) and the \ref objectData structure for each object.
    /// \note Since the data is transmitted from C++ to C#, the size of the structure must be constant.
    /// \note Therefore, there is a limitation of 75 (sl.Constant.MAX_OBJECTS) objects in the image.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Objects
    {
        /// <summary>
        /// Number of detected objects.
        /// </summary>
        /// \note You can use it to iterate through the \ref objectData array.
        public int numObject;
        /// <summary>
        /// Timestamp corresponding to the frame acquisition.
        /// </summary>
        /// This value is especially useful for the async mode to synchronize the data.
        public ulong timestamp;
        /// <summary>
        /// Whether \ref objectData has already been retrieved or not.
        /// </summary>
        public int isNew;
        /// <summary>
        /// Whether both the object tracking and the world orientation has been setup.
        /// </summary>
        public int isTracked;
        /// <summary>
        /// Current sl.OBJECT_DETECTION_MODEL used.
        /// </summary>
        public sl.OBJECT_DETECTION_MODEL detectionModel;
        /// <summary>
        /// Array of detected objects.
        /// </summary>
        /// \note Since the data is transmitted from C++ to C#, the size of the structure must be constant.
        /// \note Therefore, there is a limitation of 75 (sl.Constant.MAX_OBJECTS) objects in the image.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)(Constant.MAX_OBJECTS))]
        public ObjectData[] objectData;

        /// <summary>
        /// Function that looks for a given object id in the current objects list.
        /// </summary>
        /// <param name="objectData">[Out] sl.ObjectData to fill if the search succeeded.</param>
        /// <param name="objectDataId">[In] Id of the sl.ObjectData to search.</param>
        /// <returns>True if found, otherwise False.</returns>
        public bool GetObjectDataFromId(ref sl.ObjectData objectData, int objectDataId)
        {
            bool output = false;
            objectData = new sl.ObjectData();
            for (int idx = 0; idx < this.numObject; idx++)
                if (this.objectData[idx].id == 0)
                {
                    objectData = this.objectData[idx];
                    output = true;
                }
            return output;
        }
    };

    /// <summary>
    /// Full covariance matrix for position (3x3). Only 6 values are necessary
    /// [p0, p1, p2]
    /// [p1, p3, p4]
    /// [p2, p4, p5]
    /// </summary>
    public struct CovarMatrix
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] values;// covariance matrix of the 3d position, represented by its upper triangular matrix value
    };

    /// \ingroup Body_group
	/// <summary>
	/// Structure containing data of a detected body/person such as its \ref headBoundingBox, \ref id and its 3D \ref position.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
    public struct BodyData
    {
        /// <summary>
        /// Body/person identification number.
        /// </summary>
        /// It is used as a reference when tracking the body through the frames.
        /// \note Only available if sl.BodyTrackingParameters.enableTracking is activated.
        /// \note Otherwise, it will be set to -1.
        public int id;
        /// <summary>
        /// Unique id to help identify and track AI detections.
        /// </summary>
        /// It can be either generated externally, or by using sl.Camera.GenerateUniqueID() or left empty.
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 37)]
        public string uniqueObjectId;
        /// <summary>
        /// Body/person tracking state.
        /// </summary>
		public sl.OBJECT_TRACKING_STATE trackingState;
        /// <summary>
        /// Body/person action state.
        /// </summary>
		public sl.OBJECT_ACTION_STATE actionState;
        /// <summary>
        /// Body/person 3D centroid.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        public Vector3 position;
        /// <summary>
        /// Body/person 3D velocity.
        /// </summary>
        /// \note It is defined in ```sl.InitParameters.coordinateUnits / s``` and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
		public Vector3 velocity; //object root velocity
        /// <summary>
        /// Covariance matrix of the 3D position.
        /// </summary>
        /// \note It is represented by its upper triangular matrix value
        /// \code
        ///      = [p0, p1, p2]
        ///        [p1, p3, p4]
        ///        [p2, p4, p5]
        /// \endcode
        /// where pi is ```positionCovariance[i]```
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public CovarMatrix positionCovariance;// covariance matrix of the 3d position, represented by its upper triangular matrix value
        /// <summary>
        /// Detection confidence value of the body/person.
        /// </summary>
        /// From 0 to 100, a low value means the body might not be localized perfectly.
		public float confidence;
        /// <summary>
        /// Mask defining which pixels which belong to the body/person (in \ref boundingBox and set to 255) and those of the background (set to 0).
        /// </summary>
        /// \warning The mask information is only available for tracked bodies (sl.OBJECT_TRACKING_STATE.OK) that have a valid depth.
        /// \warning Otherwise, the mask will not be initialized.
		public System.IntPtr mask;
        /// <summary>
        /// 2D bounding box of the body/person represented as four 2D points starting at the top left corner and rotation clockwise.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \code
        /// A ------ B
        /// | Object |
        /// D ------ C
        /// \endcode
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public Vector2[] boundingBox2D;
        /// <summary>
        /// 3D centroid of the head of the body/person.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
		public Vector3 headPosition; //object head position (only for HUMAN detectionModel)
        /// <summary>
        /// 3D body/person dimensions: width, height, length.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        public Vector3 dimensions;
        /// <summary>
        /// 3D bounding box of the body/person represented as eight 3D points.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \code
        ///    1 ------ 2
        ///   /        /|
        ///  0 ------ 3 |
        ///  | Object | 6
        ///  |        |/
        ///  4 ------ 7
        /// \endcode
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public Vector3[] boundingBox; // 3D Bounding Box of object
        /// <summary>
        /// 3D bounding box of the head of the body/person represented as eight 3D points.
        /// </summary>
        /// \note It is defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public Vector3[] headBoundingBox;// 3D Bounding Box of head (only for HUMAN detectionModel)
        /// <summary>
        /// 2D bounding box of the head of the body/person represented as four 2D points starting at the top left corner and rotation clockwise.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public Vector2[] headBoundingBox2D;// 2D Bounding Box of head
        /// <summary>
        /// Set of useful points representing the human body in 2D.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \warning In some cases, eg. body partially out of the image, some keypoints can not be detected. They will have negatives coordinates.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 38)]
        public Vector2[] keypoints2D;
        /// <summary>
        /// Set of useful points representing the human body in 3D.
        /// </summary>
        /// \note They are defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \warning In some cases, eg. body partially out of the image or missing depth data, some keypoints can not be detected. They will have non finite values.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 38)]
        public Vector3[] keypoints;// 3D position of the joints of the skeleton

        /// <summary>
        /// Array of detection confidences for each keypoint.
        /// </summary>
        /// \note They can not be lower than the sl.BodyTrackingRuntimeParameters.detectionConfidenceThreshold.
        /// \warning In some cases, eg. body partially out of the image or missing depth data, some keypoints can not be detected. They will have non finite values.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 38)]
        public float[] keypointConfidence;

        /// <summary>
        /// Array of detection covariance for each keypoint.
        /// </summary>
        /// \warning In some cases, eg. body partially out of the image or missing depth data, some keypoints can not be detected. Their covariances will be 0.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 38)]
        public CovarMatrix[] keypointCovariances;

        /// <summary>
        /// Array of local position (position of the child keypoint with respect to its parent expressed in its parent coordinate frame) for each keypoint.
        /// </summary>
        /// \note They are expressed in sl.REFERENCE_FRAME.CAMERA or sl.REFERENCE_FRAME.WORLD.
        /// \warning Not available with sl.BODY_FORMAT.BODY_18.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 38)]
        public Vector3[] localPositionPerJoint;
        /// <summary>
        /// Array of local orientation for each keypoint.
        /// </summary>
        /// \note The orientation is represented by a quaternion.
        /// \warning Not available with sl.BODY_FORMAT.BODY_18.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 38)]
        public Quaternion[] localOrientationPerJoint;
        /// <summary>
        /// Global root orientation of the skeleton.
        /// </summary>
        /// The orientation is also represented by a quaternion.
        /// \note The global root position is already accessible in \ref keypoint attribute by using the root index of a given sl.BODY_FORMAT.
        /// \warning Not available with sl.BODY_FORMAT.BODY_18.
        public Quaternion globalRootOrientation;
    };

    /// \ingroup Body_group
    /// <summary>
    /// Structure containing the results of the body tracking module.
    /// </summary>
    /// The detected bodies/persons are listed in \ref bodiesList.
    [StructLayout(LayoutKind.Sequential)]
    public struct Bodies
    {
        /// <summary>
        /// Number of detected bodies/persons.
        /// </summary>
        /// \note You can use it to iterate through the \ref bodiesList array.
        public int nbBodies;
        /// <summary>
        /// Timestamp corresponding to the frame acquisition.
        /// </summary>
        /// This value is especially useful for the async mode to synchronize the data.
        public ulong timestamp;
        /// <summary>
        /// Whether \ref bodiesList has already been retrieved or not.
        /// </summary>
        public int isNew;
        /// <summary>
        /// Whether both the body tracking and the world orientation has been setup.
        /// </summary>
        public int isTracked;
        /// <summary>
        /// Array of detected bodies/persons.
        /// </summary>
        /// \note Since the data is transmitted from C++ to C#, the size of the structure must be constant.
        /// \note Therefore, there is a limitation of 75 (sl.Constant.MAX_OBJECTS) objects in the image.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)(Constant.MAX_OBJECTS))]
        public BodyData[] bodiesList;

    }

    ///\ingroup Body_group
    /// <summary>
    /// Lists supported skeleton body models.
    /// </summary>
    public enum BODY_FORMAT
    {
        /// <summary>
        /// 18-keypoint model
        /// \n Basic body model
        /// </summary>
        BODY_18,
        /// <summary>
        /// 34-keypoint model
        /// \note Requires body fitting enabled.
        /// </summary>
        BODY_34,
        /// <summary>
        /// 38-keypoint model
        /// \n Including simplified face, hands and feet.
        /// \note Early Access
        /// </summary>
        BODY_38,
#if false
        /// <summary>
        /// Body model, including feet, simplified face and detailed hands
        /// </summary>
        BODY_70
#endif     
        };

    ///\ingroup Body_group
    /// <summary>
    /// Lists supported models for skeleton keypoints selection.
    /// </summary>
    public enum BODY_KEYPOINTS_SELECTION
    {
        /// <summary>
        /// Full keypoint model
        /// </summary>
        FULL,
        /// <summary>
        /// Upper body keypoint model
        /// \n Will output only upper body (from hip).
        /// </summary>
        UPPER_BODY
    };

    ///\ingroup Object_group
    /// <summary>
    /// Lists available object classes.
    /// </summary>
    public enum OBJECT_CLASS
    {
        /// <summary>
        /// For people detection
        /// </summary>
        PERSON = 0,
        /// <summary>
        /// For vehicle detection (cars, trucks, buses, motorcycles, etc.)
        /// </summary>
        VEHICLE = 1,
        /// <summary>
        /// For bag detection (backpack, handbag, suitcase, etc.)
        /// </summary>
        BAG = 2,
        /// <summary>
        /// For animal detection (cow, sheep, horse, dog, cat, bird, etc.)
        /// </summary>
        ANIMAL = 3,
        /// <summary>
        /// For electronic device detection (cellphone, laptop, etc.)
        /// </summary>
        ELECTRONICS = 4,
        /// <summary>
        /// For fruit and vegetable detection (banana, apple, orange, carrot, etc.)
        /// </summary>
        FRUIT_VEGETABLE = 5,
        /// <summary>
        /// For sport-related object detection (sport ball, etc.)
        /// </summary>
        SPORT = 6,
        ///@cond SHOWHIDDEN
        LAST = 7
        ///@endcond
    };

    ///\ingroup Object_group
    /// <summary>
    /// List available object subclasses.
    ///
    /// Given as hint, when using object tracking an object can change of sl.OBJECT_SUBCLASS while keeping the same sl.OBJECT_CLASS
    /// (i.e.: frame n: sl.OBJECT_SUBCLASS.MOTORBIKE, frame n+1: sl.OBJECT_SUBCLASS.BICYCLE).
    /// </summary>
    public enum OBJECT_SUBCLASS
    {
        /// <summary>
        /// sl.OBJECT_CLASS.PERSON
        /// </summary>
        PERSON = 0,
        /// <summary>
        /// sl.OBJECT_CLASS.VEHICLE
        /// </summary>
        BICYCLE = 1,
        /// <summary>
        /// sl.OBJECT_CLASS.VEHICLE
        /// </summary>
        CAR = 2,
        /// <summary>
        /// sl.OBJECT_CLASS.VEHICLE
        /// </summary>
        MOTORBIKE = 3,
        /// <summary>
        /// sl.OBJECT_CLASS.VEHICLE
        /// </summary>
        BUS = 4,
        /// <summary>
        /// sl.OBJECT_CLASS.VEHICLE
        /// </summary>
        TRUCK = 5,
        /// <summary>
        /// sl.OBJECT_CLASS.VEHICLE
        /// </summary>
        BOAT = 6,
        /// <summary>
        /// sl.OBJECT_CLASS.BAG
        /// </summary>
        BACKPACK = 7,
        /// <summary>
        /// sl.OBJECT_CLASS.BAG
        /// </summary>
        HANDBAG = 8,
        /// <summary>
        /// sl.OBJECT_CLASS.BAG
        /// </summary>
        SUITCASE = 9,
        /// <summary>
        /// sl.OBJECT_CLASS.ANIMAL
        /// </summary>
        BIRD = 10,
        /// <summary>
        /// sl.OBJECT_CLASS.ANIMAL
        /// </summary>
        CAT = 11,
        /// <summary>
        /// sl.OBJECT_CLASS.ANIMAL
        /// </summary>
        DOG = 12,
        /// <summary>
        /// sl.OBJECT_CLASS.ANIMAL
        /// </summary>
        HORSE = 13,
        /// <summary>
        /// sl.OBJECT_CLASS.ANIMAL
        /// </summary>
        SHEEP = 14,
        /// <summary>
        /// sl.OBJECT_CLASS.ANIMAL
        /// </summary>
        COW = 15,
        /// <summary>
        /// sl.OBJECT_CLASS.ELECTRONICS
        /// </summary>
        CELLPHONE = 16,
        /// <summary>
        /// sl.OBJECT_CLASS.ELECTRONICS
        /// </summary>
        LAPTOP = 17,
        /// <summary>
        /// sl.OBJECT_CLASS.FRUIT_VEGETABLE
        /// </summary>
        BANANA = 18,
        /// <summary>
        /// sl.OBJECT_CLASS.FRUIT_VEGETABLE
        /// </summary>
        APPLE = 19,
        /// <summary>
        /// sl.OBJECT_CLASS.FRUIT_VEGETABLE
        /// </summary>
        ORANGE = 20,
        /// <summary>
        /// sl.OBJECT_CLASS.FRUIT_VEGETABLE
        /// </summary>
        CARROT = 21,
        /// <summary>
        /// sl.OBJECT_CLASS.PERSON
        /// </summary>
        PERSON_HEAD = 22,
        /// <summary>
        /// sl.OBJECT_CLASS.SPORT
        /// </summary>
        SPORTSBALL = 23,
        ///@cond SHOWHIDDEN
        LAST = 24
        ///@endcond
    };

    ///\ingroup Object_group
    /// <summary>
    /// Lists the different states of object tracking.
    /// </summary>
    public enum OBJECT_TRACKING_STATE
    {
        /// <summary>
        /// The tracking is not yet initialized.
        /// \n The object id is not usable.
        /// </summary>
		OFF,
        /// <summary>
        /// The object is tracked.
        /// </summary>
		OK,
        /// <summary>
        /// The object could not be detected in the image and is potentially occluded.
        /// \n The trajectory is estimated.
        /// </summary>
		SEARCHING,
        /// <summary>
        /// This is the last searching state of the track.
        /// \n The track will be deleted in the next sl.Camera.RetrieveObjects().
        /// </summary>
        TERMINATE
    };

    ///\ingroup Object_group
    /// <summary>
    /// Lists the different states of an object's actions.
    /// </summary>
	public enum OBJECT_ACTION_STATE
    {
        /// <summary>
        /// The object is staying static.
        /// </summary>
		IDLE = 0,
        /// <summary>
        /// The object is moving.
        /// </summary>
		MOVING = 1
    };

    ///\ingroup Object_group
	/// <summary>
	/// Lists available models for the object detection module.
	/// </summary>
	public enum OBJECT_DETECTION_MODEL {
        /// <summary>
        /// Any objects, bounding box based.
        /// </summary>
		MULTI_CLASS_BOX_FAST,
        /// <summary>
        /// Any objects, bounding box based, compromise between accuracy and speed.
        /// </summary>
        MULTI_CLASS_BOX_MEDIUM,
        /// <summary>
        /// Any objects, bounding box based, more accurate but slower than the base model.
        /// </summary>
        MULTI_CLASS_BOX_ACCURATE,
        /// <summary>
        /// Bounding box detector specialized in person heads particularly well suited for crowded environments.
        /// The person localization is also improved.
        /// </summary>
        PERSON_HEAD_BOX_FAST,
        /// <summary>
        /// Bounding box detector specialized in person heads, particularly well suited for crowded environments.
        /// The person localization is also improved, more accurate but slower than the base model.
        /// </summary>
        PERSON_HEAD_BOX_ACCURATE,
        /// <summary>
        /// For external inference, using your own custom model and/or frameworks.
        /// This mode disables the internal inference engine, the 2D bounding box detection must be provided.
        /// </summary>
        CUSTOM_BOX_OBJECTS
    };

    ///\ingroup Body_group
	/// <summary>
	/// Lists available models for the body tracking module.
	/// </summary>
	public enum BODY_TRACKING_MODEL
    {
        /// <summary>
        /// Keypoints based, specific to human skeleton, real time performance even on Jetson or low end GPU cards.
        /// </summary>
        HUMAN_BODY_FAST,
        /// <summary>
        /// Keypoints based, specific to human skeleton, compromise between accuracy and speed.
        /// </summary>
		HUMAN_BODY_MEDIUM,
        /// <summary>
        /// Keypoints based, specific to human skeleton, state of the art accuracy, requires powerful GPU.
        /// </summary>
        HUMAN_BODY_ACCURATE
    };

    /// <summary>
    /// Lists available  AI models.
    /// </summary>
    public enum AI_MODELS
    {
        /// <summary>
        /// Related to sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST
        /// </summary>
        MULTI_CLASS_DETECTION,
        /// <summary>
        /// Related to sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM
        /// </summary>
        MULTI_CLASS_MEDIUM_DETECTION,
        /// <summary>
        /// Related to sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_ACCURATE
        /// </summary>
        MULTI_CLASS_ACCURATE_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
        /// </summary>
        HUMAN_BODY_FAST_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM
        /// </summary>
        HUMAN_BODY_MEDIUM_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
        /// </summary>
        HUMAN_BODY_ACCURATE_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
        /// </summary>
        HUMAN_BODY_38_FAST_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM
        /// </summary>
        HUMAN_BODY_38_MEDIUM_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
        /// </summary>
        HUMAN_BODY_38_ACCURATE_DETECTION,
#if false
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
        /// </summary>
        HUMAN_BODY_70_FAST_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM
        /// </summary>
        HUMAN_BODY_70_MEDIUM_DETECTION,
        /// <summary>
        /// Related to sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
        /// </summary>
        HUMAN_BODY_70_ACCURATE_DETECTION,
#endif
        /// <summary>
        /// Related to sl.OBJECT_DETECTION_MODEL.PERSON_HEAD_BOX_FAST
        /// </summary>
        PERSON_HEAD_DETECTION,
        /// <summary>
        /// Related to sl.OBJECT_DETECTION_MODEL.PERSON_HEAD_BOX_ACCURATE
        /// </summary>
        PERSON_HEAD_ACCURATE_DETECTION,
        /// <summary>
        /// Related to sl.BatchParameters.enable
        /// </summary>
        REID_ASSOCIATION,
        /// <summary>
        /// Related to sl.DEPTH_MODE.NEURAL
        /// </summary>
        NEURAL_DEPTH,
        ///@cond SHOWHIDDEN
        LAST
        ///@endcond
    };

    /// <summary>
    /// Lists supported bounding box preprocessing.
    /// </summary>
    public enum OBJECT_FILTERING_MODE
    {
        /// <summary>
        /// The ZED SDK will not apply any preprocessing to the detected objects.
        /// </summary>
        NONE,
        /// <summary>
        /// The ZED SDK will remove objects that are in the same 3D position as an already tracked object (independent of class id).
        /// </summary>
        NMS3D,
        /// <summary>
        /// The ZED SDK will remove objects that are in the same 3D position as an already tracked object of the same class id.
        /// </summary>
        NMS3D_PER_CLASS
    };

    ///\ingroup Body_group
    /// <summary>
    /// Semantic of human body parts and order of \ref sl.BodyData.keypoints for \ref sl.BODY_FORMAT.BODY_18.
    /// </summary>
    public enum BODY_18_PARTS
    {
        NOSE = 0, /**< 0*/
        NECK = 1, /**< 1*/
        RIGHT_SHOULDER = 2, /**< 2*/
        RIGHT_ELBOW = 3, /**< 3*/
        RIGHT_WRIST = 4, /**< 4*/
        LEFT_SHOULDER = 5, /**< 5*/
        LEFT_ELBOW = 6, /**< 6*/
        LEFT_WRIST = 7, /**< 7*/
        RIGHT_HIP = 8, /**< 8*/
        RIGHT_KNEE = 9, /**< 9*/
        RIGHT_ANKLE = 10, /**< 10*/
        LEFT_HIP = 11, /**< 11*/
        LEFT_KNEE = 12, /**< 12*/
        LEFT_ANKLE = 13, /**< 13*/
        RIGHT_EYE = 14, /**< 14*/
        LEFT_EYE = 15, /**< 15*/
        RIGHT_EAR = 16, /**< 16*/
        LEFT_EAR = 17, /**< 17*/
        ///@cond SHOWHIDDEN
        LAST = 18
        ///@endcond
    };

    ///\ingroup Body_group
    /// <summary>
    /// Semantic of human body parts and order of \ref sl.BodyData.keypoints for \ref sl.BODY_FORMAT.BODY_34.
    /// </summary>
    public enum BODY_34_PARTS
    {
        PELVIS = 0, /**< 0*/
        NAVAL_SPINE = 1, /**< 1*/
        CHEST_SPINE = 2, /**< 2*/
        NECK = 3, /**< 3*/
        LEFT_CLAVICLE = 4, /**< 4*/
        LEFT_SHOULDER = 5, /**< 5*/
        LEFT_ELBOW = 6, /**< 6*/
        LEFT_WRIST = 7, /**< 7*/
        LEFT_HAND = 8, /**< 8*/
        LEFT_HANDTIP = 9, /**< 9*/
        LEFT_THUMB = 10, /**< 10*/
        RIGHT_CLAVICLE = 11, /**< 11*/
        RIGHT_SHOULDER = 12, /**< 12*/
        RIGHT_ELBOW = 13, /**< 13*/
        RIGHT_WRIST = 14, /**< 14*/
        RIGHT_HAND = 15, /**< 15*/
        RIGHT_HANDTIP = 16, /**< 16*/
        RIGHT_THUMB = 17, /**< 17*/
        LEFT_HIP = 18, /**< 18*/
        LEFT_KNEE = 19, /**< 19*/
        LEFT_ANKLE = 20, /**< 20*/
        LEFT_FOOT = 21, /**< 21*/
        RIGHT_HIP = 22, /**< 22*/
        RIGHT_KNEE = 23, /**< 23*/
        RIGHT_ANKLE = 24, /**< 24*/
        RIGHT_FOOT = 25, /**< 25*/
        HEAD = 26, /**< 26*/
        NOSE = 27, /**< 27*/
        LEFT_EYE = 28, /**< 28*/
        LEFT_EAR = 29, /**< 29*/
        RIGHT_EYE = 30, /**< 30*/
        RIGHT_EAR = 31, /**< 31*/
        LEFT_HEEL = 32, /**< 32*/
        RIGHT_HEEL = 33, /**< 33*/
        ///@cond SHOWHIDDEN
        LAST = 34
        ///@endcond
    };

    ///\ingroup Body_group
    /// <summary>
    /// Semantic of human body parts and order of \ref sl.BodyData.keypoints for \ref sl.BODY_FORMAT.BODY_38.
	/// </summary>
	public enum BODY_38_PARTS 
    {
        PELVIS, /**< 0*/
        SPINE_1, /**< 1*/
        SPINE_2, /**< 2*/
        SPINE_3, /**< 3*/
        NECK, /**< 4*/
        NOSE, /**< 5*/
        LEFT_EYE, /**< 6*/
        RIGHT_EYE, /**< 7*/
        LEFT_EAR, /**< 8*/
        RIGHT_EAR, /**< 9*/
        LEFT_CLAVICLE, /**< 10*/
        RIGHT_CLAVICLE, /**< 11*/
        LEFT_SHOULDER, /**< 12*/
        RIGHT_SHOULDER, /**< 13*/
        LEFT_ELBOW, /**< 14*/
        RIGHT_ELBOW, /**< 15*/
        LEFT_WRIST, /**< 16*/
        RIGHT_WRIST, /**< 17*/
        LEFT_HIP, /**< 18*/
        RIGHT_HIP, /**< 19*/
        LEFT_KNEE, /**< 20*/
        RIGHT_KNEE, /**< 21*/
        LEFT_ANKLE, /**< 22*/
        RIGHT_ANKLE, /**< 23*/
        LEFT_BIG_TOE, /**< 24*/
        RIGHT_BIG_TOE, /**< 25*/
        LEFT_SMALL_TOE, /**< 26*/
        RIGHT_SMALL_TOE, /**< 27*/
        LEFT_HEEL, /**< 28*/
        RIGHT_HEEL, /**< 29*/
        // Hands
        LEFT_HAND_THUMB_4, /**< 30*/
        RIGHT_HAND_THUMB_4, /**< 31*/
        LEFT_HAND_INDEX_1, /**< 32*/
        RIGHT_HAND_INDEX_1, /**< 33*/
        LEFT_HAND_MIDDLE_4, /**< 34*/
        RIGHT_HAND_MIDDLE_4, /**< 35*/
        LEFT_HAND_PINKY_1, /**< 36*/
        RIGHT_HAND_PINKY_1, /**< 37*/
        ///@cond SHOWHIDDEN
        LAST = 38
        ///@endcond
    };

#if false
    ///\ingroup Object_group
    /// <summary>
    /// ssemantic of human body parts and order keypoints for BODY_FORMAT.POSE_70.
    /// </summary>
    public enum BODY_70_PARTS
    {
        PELVIS,
        SPINE_1,
        SPINE_2,
        SPINE_3,
        NECK,
        NOSE,
        LEFT_EYE,
        RIGHT_EYE,
        LEFT_EAR,
        RIGHT_EAR,
        LEFT_CLAVICLE,
        RIGHT_CLAVICLE,
        LEFT_SHOULDER,
        RIGHT_SHOULDER,
        LEFT_ELBOW,
        RIGHT_ELBOW,
        LEFT_WRIST,
        RIGHT_WRIST,
        LEFT_HIP,
        RIGHT_HIP,
        LEFT_KNEE,
        RIGHT_KNEE,
        LEFT_ANKLE,
        RIGHT_ANKLE,
        LEFT_BIG_TOE,
        RIGHT_BIG_TOE,
        LEFT_SMALL_TOE,
        RIGHT_SMALL_TOE,
        LEFT_HEEL,
        RIGHT_HEEL,
        // Hands
        // Left
        LEFT_HAND_THUMB_1,
        LEFT_HAND_THUMB_2,
        LEFT_HAND_THUMB_3,
        LEFT_HAND_THUMB_4,
        LEFT_HAND_INDEX_1,
        LEFT_HAND_INDEX_2,
        LEFT_HAND_INDEX_3,
        LEFT_HAND_INDEX_4,
        LEFT_HAND_MIDDLE_1,
        LEFT_HAND_MIDDLE_2,
        LEFT_HAND_MIDDLE_3,
        LEFT_HAND_MIDDLE_4,
        LEFT_HAND_RING_1,
        LEFT_HAND_RING_2,
        LEFT_HAND_RING_3,
        LEFT_HAND_RING_4,
        LEFT_HAND_PINKY_1,
        LEFT_HAND_PINKY_2,
        LEFT_HAND_PINKY_3,
        LEFT_HAND_PINKY_4,
        //Right
        RIGHT_HAND_THUMB_1,
        RIGHT_HAND_THUMB_2,
        RIGHT_HAND_THUMB_3,
        RIGHT_HAND_THUMB_4,
        RIGHT_HAND_INDEX_1,
        RIGHT_HAND_INDEX_2,
        RIGHT_HAND_INDEX_3,
        RIGHT_HAND_INDEX_4,
        RIGHT_HAND_MIDDLE_1,
        RIGHT_HAND_MIDDLE_2,
        RIGHT_HAND_MIDDLE_3,
        RIGHT_HAND_MIDDLE_4,
        RIGHT_HAND_RING_1,
        RIGHT_HAND_RING_2,
        RIGHT_HAND_RING_3,
        RIGHT_HAND_RING_4,
        RIGHT_HAND_PINKY_1,
        RIGHT_HAND_PINKY_2,
        RIGHT_HAND_PINKY_3,
        RIGHT_HAND_PINKY_4,
        LAST

    };

#endif
    ///\ingroup Object_group
    /// <summary>
    /// Class containing batched data of a detected objects from the object detection module.
    /// </summary>
    /// This class can be used to store trajectories.
    [StructLayout(LayoutKind.Sequential)]
    public class ObjectsBatch
    {
        /// <summary>
        /// Number of objects in the sl.ObjectsBatch.
        /// </summary>
        /// Use this to iterate through the top of \ref positions / \ref velocities / \ref boundingBoxes / etc.
        /// \note Objects with greater indexes are empty.
        public int numData = 0;
        /// <summary>
        /// Id of the batch.
        /// </summary>
        public int id = 0;
        /// <summary>
        /// Objects class/category to identify the object type.
        /// </summary>
        public OBJECT_CLASS label = OBJECT_CLASS.LAST;
        /// <summary>
        /// Objects sub-class/sub-category to identify the object type.
        /// </summary>
        public OBJECT_SUBCLASS sublabel = OBJECT_SUBCLASS.LAST;
        /// <summary>
        /// Objects tracking state.
        /// </summary>
        public OBJECT_TRACKING_STATE trackingState = OBJECT_TRACKING_STATE.TERMINATE;
        /// <summary>
        /// Array of positions for each object.
        /// </summary>
        public Vector3[] positions = new Vector3[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of positions' covariances for each object.
        /// </summary>
        public float[,] positionCovariances = new float[(int)Constant.MAX_BATCH_SIZE, 6];
        /// <summary>
        /// Array of 3D velocities for each object.
        /// </summary>
        public Vector3[] velocities = new Vector3[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of timestamps for each object.
        /// </summary>
        public ulong[] timestamps = new ulong[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of 3D bounding boxes for each object.
        /// \note They are defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \code
        ///    1 ------ 2
        ///   /        /|
        ///  0 ------ 3 |
        ///  | Object | 6
        ///  |        |/
        ///  4 ------ 7
        /// \endcode
        /// </summary>
        public Vector3[,] boundingBoxes = new Vector3[(int)Constant.MAX_BATCH_SIZE, 8];
        /// <summary>
        /// Array of 2D bounding boxes for each object.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \code
        /// A ------ B
        /// | Object |
        /// D ------ C
        /// \endcode
        public Vector2[,] boundingBoxes2D = new Vector2[(int)Constant.MAX_BATCH_SIZE, 4];
        /// <summary>
        /// Array of confidences for each object.
        /// </summary>
        public float[] confidences = new float[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of action states for each object.
        /// </summary>
        public OBJECT_ACTION_STATE[] actionStates = new OBJECT_ACTION_STATE[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of 2D bounding box of the head for each object (person).
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \warning Not available with [sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_XXX](\ref OBJECT_DETECTION_MODEL).
        public Vector2[,] headBoundingBoxes2D = new Vector2[(int)Constant.MAX_BATCH_SIZE, 8];
        /// <summary>
        /// Array of 3D bounding box of the head for each object (person).
        /// </summary>
        /// \note They are defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \warning Not available with [sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_XXX](\ref OBJECT_DETECTION_MODEL).
        public Vector3[,] headBoundingBoxes = new Vector3[(int)Constant.MAX_BATCH_SIZE, 8];
        /// <summary>
        /// Array of 3D centroid of the head for each object (person).
        /// </summary>
        /// \note They are defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \warning Not available with [sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_XXX](\ref OBJECT_DETECTION_MODEL).
        public Vector3[] headPositions = new Vector3[(int)Constant.MAX_BATCH_SIZE];
    }

#if false

    /// \ingroup Body_group
    /// <summary>
    /// Class containing batched data of a detected bodies/persons from the body tracking module.
    /// </summary>
    public class BodiesBatch
    {
        /// <summary>
        /// Number of detected bodies/persons.
        /// </summary>
        /// \note You can use it to iterate through the \ref positions / \ref velocities / \ref boundingBoxes / etc. array.
        /// </summary>
        public int numData = 0;
        /// <summary>
        /// Id of the batch.
        /// </summary>
        public int id = 0;
        /// <summary>
        /// Bodies/persons tracking state.
        /// </summary>
        public OBJECT_TRACKING_STATE trackingState = OBJECT_TRACKING_STATE.TERMINATE;
        /// <summary>
        /// Array of positions for each body/person.
        /// </summary>
        public Vector3[] positions = new Vector3[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of positions' covariances for each body/person.
        /// </summary>
        public float[,] positionCovariances = new float[(int)Constant.MAX_BATCH_SIZE, 6];
        /// <summary>
        /// Array of 3D velocities for each body/person.
        /// </summary>
        public Vector3[] velocities = new Vector3[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of timestamps for each body/person.
        /// </summary>
        public ulong[] timestamps = new ulong[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of 3D bounding boxes for each body/person.
        /// \note They are defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        /// \code
        ///    1 ------ 2
        ///   /        /|
        ///  0 ------ 3 |
        ///  | Object | 6
        ///  |        |/
        ///  4 ------ 7
        /// \endcode
        /// </summary>
        public Vector3[,] boundingBoxes = new Vector3[(int)Constant.MAX_BATCH_SIZE, 8];
        /// <summary>
        /// Array of 2D bounding boxes for each body/person.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        /// \code
        /// A ------ B
        /// | Object |
        /// D ------ C
        /// \endcode
        public Vector2[,] boundingBoxes2D = new Vector2[(int)Constant.MAX_BATCH_SIZE, 4];
        /// <summary>
        /// Array of confidences for each body/person.
        /// </summary>
        public float[] confidences = new float[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of action states for each body/person.
        /// </summary>
        public OBJECT_ACTION_STATE[] actionStates = new OBJECT_ACTION_STATE[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of 2D keypoints for each body/person.
        /// </summary>
        /// \warning In some cases, eg. body partially out of the image or missing depth data, some keypoints can not be detected. They will have non finite values.
        public Vector2[,] keypoints2D = new Vector2[(int)Constant.MAX_BATCH_SIZE, 70];
        /// <summary>
        /// Array of 3D keypoints for each body/person.
        /// </summary>
        /// \warning In some cases, eg. body partially out of the image or missing depth data, some keypoints can not be detected. They will have non finite values.
        public Vector3[,] keypoints = new Vector3[(int)Constant.MAX_BATCH_SIZE, 70];
        /// <summary>
        /// Array of 2D bounding box of the head for each body/person.
        /// </summary>
        /// \note Expressed in pixels on the original image resolution, ```[0, 0]``` is the top left corner.
        public Vector2[,] headBoundingBoxes2D = new Vector2[(int)Constant.MAX_BATCH_SIZE, 8];
        /// <summary>
        /// Array of 3D bounding box of the head for each body/person.
        /// </summary>
        /// \note They are defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        public Vector3[,] headBoundingBoxes = new Vector3[(int)Constant.MAX_BATCH_SIZE, 8];
        /// <summary>
        /// Array of 3D centroid of the head for each body/person.
        /// </summary>
        /// \note They are defined in sl.InitParameters.coordinateUnits and expressed in sl.RuntimeParameters.measure3DReferenceFrame.
        public Vector3[] headPositions = new Vector3[(int)Constant.MAX_BATCH_SIZE];
        /// <summary>
        /// Array of detection confidences array for each keypoint for each body/person.
        /// </summary>
        /// \note They can not be lower than the sl.BodyTrackingRuntimeParameters.detectionConfidenceThreshold.
        /// \warning In some cases, eg. body partially out of the image or missing depth data, some keypoints can not be detected. They will have non finite values.
        public float[,] keypointsConfidences = new float[(int)Constant.MAX_BATCH_SIZE, 70];
    }

#endif



    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Fusion API ///////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Fusion API Module

    /// \ingroup Fusion_group
    /// <summary>
    /// Lists the types of error that can be raised by the Fusion.
    /// </summary>
    public enum FUSION_ERROR_CODE
    {
        /// <summary>
        /// The senders are using different body formats.
        /// \n Consider changing them.
        /// </summary>
        WRONG_BODY_FORMAT = -7,
        /// <summary>
        /// The following module was not enabled.
        /// </summary>
        NOT_ENABLE = -6,
        /// <summary>
        /// Some sources are provided by SVO and others by LIVE stream.
        /// </summary>
        INPUT_FEED_MISMATCH = -5,
        /// <summary>
        /// Connection timed out. Unable to reach the sender.
        /// \n Verify the sender's IP/port.
        /// </summary>
        CONNECTION_TIMED_OUT = -4,
        /// <summary>
        /// Intra-process shared memory allocation issue.
        /// \n Multiple connections to the same data.
        /// </summary>
        MEMORY_ALREADY_USED = -3,
        /// <summary>
        /// The provided IP address format is incorrect.
        /// \n Please provide the IP in the format 'a.b.c.d', where (a, b, c, d) are numbers between 0 and 255.
        /// </summary>
        BAD_IP_ADDRESS = -2,
        /// <summary>
        /// Standard code for unsuccessful behavior.
        /// </summary>
        FAILURE = -1,
        /// <summary>
        /// Standard code for successful behavior.
        /// </summary>
        SUCCESS = 0,
        /// <summary>
        /// Significant differences observed between sender's FPS.
        /// </summary>
        ERRATIC_FPS = 1,
        /// <summary>
        /// At least one sender has an FPS lower than 10 FPS.
        /// </summary>
        FPS_TOO_LOW = 2,
        /// <summary>
        /// Problem detected with the ingested timestamp.
        /// \n Sample data will be ignored.
        /// </summary>
        INVALID_TIMESTAMP = 3,
        /// <summary>
        /// Problem detected with the ingested covariance.
        /// \n Sample data will be ignored.
        /// </summary>
        INVALID_COVARIANCE = 4,
        /// <summary>
        /// All data from all sources has been consumed.
        /// \n No new data is available for processing.
        /// </summary>
        NO_NEW_DATA_AVAILABLE = 5
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Lists the types of error that can be raised during the Fusion by senders.
    /// </summary>
    public enum SENDER_ERROR_CODE
    {
        /// <summary>
        /// The sender has been disconnected.
        /// </summary>
        DISCONNECTED = -1,
        /// <summary>
        ///  Standard code for successful behavior.
        /// </summary>
        SUCCESS = 0,
        /// <summary>
        /// The sender encountered a grab error.
        /// </summary>
        GRAB_ERROR = 1,
        /// <summary>
        /// The sender does not run with a constant frame rate.
        /// </summary>
        ERRATIC_FPS = 2,
        /// <summary>
        /// The frame rate of the sender is lower than 10 FPS.
        /// </summary>
        FPS_TOO_LOW = 3
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the options used to initialize the \ref Fusion object.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct InitFusionParameters
    {
        /// <summary>
        /// This parameter allows you to select the unit to be used for all metric values of the SDK (depth, point cloud, tracking, mesh, and others).
	    /// Default : \ref UNIT "UNIT::MILLIMETER"
        /// </summary>
        public UNIT coordinateUnits;

        /// <summary>
        /// Positional tracking, point clouds and many other features require a given \ref COORDINATE_SYSTEM to be used as reference.
        /// This parameter allows you to select the \ref COORDINATE_SYSTEM used by the \ref Camera to return its measures.
    	/// \n This defines the order and the direction of the axis of the coordinate system.
    	/// \n Default : \ref COORDINATE_SYSTEM "COORDINATE_SYSTEM::IMAGE"
        /// </summary>
        public COORDINATE_SYSTEM coordinateSystem;

        /// <summary>
        /// It allows users to extract some stats of the Fusion API like drop frame of each camera, latency, etc...
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool outputPerformanceMetrics;

        /// <summary>
        /// Enable the verbosity mode of the SDK.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool verbose;

        /// <summary>
        /// If specified change the number of period necessary for a source to go in timeout without data. For example, if you set this to 5
        /// then, if any source do not receive data during 5 period, these sources will go to timeout and will be ignored.	    /// if any source does not receive data during 5 periods, these sources will go into timeout and will be ignored.
        /// </summary>
        public uint timeoutPeriodsNumber;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Lists the different types of communications available for Fusion module.
    /// </summary>
    public enum COMM_TYPE
    {
        /// <summary>
        /// The sender and receiver are on the same local network and communicate by RTP.
        /// \n The communication can be affected by the local network load.
        /// </summary>
        LOCAL_NETWORK, 
        /// <summary>
        /// Both sender and receiver are declared by the same process and can be in different threads.
        /// \n This type of communication is optimized.
        /// </summary>
        INTRA_PROCESS 
    };

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the communication parameter to configure the connection between senders and receiver
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CommunicationParameters
    {
        /// <summary>
        /// Type of communication
        /// </summary>
        public COMM_TYPE communicationType;
        /// <summary>
        /// The comm port used for streaming the data
        /// </summary>
	    uint ipPort;
        /// <summary>
        /// The IP address of the sender
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
        char[] ipAdd;
    };

    /// \ingroup Fusion_group
    /// <summary>
    /// Stores the Fusion configuration, can be read from /write to a Json file.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct FusionConfiguration
    {
        /// <summary>
        /// The serial number of the used ZED camera.
        /// </summary>
        public int serialnumber;
        /// <summary>
        /// The communication parameters to connect this camera to the Fusion.
        /// </summary>
        public CommunicationParameters commParam;
        /// <summary>
        /// The WORLD position of the camera for Fusion.
        /// </summary>
        public Vector3 position;
        /// <summary>
        /// The WORLD rotation of the camera for Fusion.
        /// </summary>
        public Quaternion rotation;
        /// <summary>
        /// The input type for the current camera.
        /// </summary>
        public INPUT_TYPE inputType;
    };


    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the options used to initialize the body tracking module of the \ref Fusion.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BodyTrackingFusionParameters
    {
        /// <summary>
        /// Defines if the object detection will track objects across images flow.
        ///
        /// Default: true
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool enableTracking;

        /// <summary>
        /// Defines if the body fitting will be applied.
        ///
        /// Default: false
        /// \note If you enable it and the camera provides data as BODY_18 the fused body format will be BODY_34.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool enableBodyFitting;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the options used to change the behavior of the body tracking module at runtime.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BodyTrackingFusionRuntimeParameters
    {
        /// <summary>
        /// If the fused skeleton has less than skeleton_minimum_allowed_keypoints keypoints, it will be discarded.
        ///
        /// Default: -1.
        /// </summary>
        public int skeletonMinimumAllowedKeypoints;

        /// <summary>
        /// If a skeleton was detected in less than skeleton_minimum_allowed_camera cameras, it will be discarded.
        ///
        /// Default: -1.
        /// </summary>
        public int skeletonMinimumAllowedCameras;

        /// <summary>
	    /// This value controls the smoothing of the tracked or fitted fused skeleton.
        ///
        /// It is ranged from 0 (low smoothing) and 1 (high smoothing).
        /// \n Default: 0.
        /// </summary>
        public float skeletonSmoothing;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Used to identify a specific camera in the Fusion API
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CameraIdentifier
    {
        /// <summary>
        /// Serial Number of the camera.
        /// </summary>
        public ulong sn;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the metrics of a sender in the fusion process.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CameraMetrics
    {
        public CameraIdentifier uuid;

        /// <summary>
        /// FPS of the received data.
        /// </summary>
        public float receivedFps;

        /// <summary>
        /// Latency (in seconds) of the received data.
        /// </summary>
        public float receivedLatency;

        /// <summary>
        /// Latency (in seconds) after Fusion synchronization.
        /// </summary>
        public float syncedLatency;

        /// <summary>
        /// If no data present is set to false.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool isPresent;

        /// <summary>
        /// Percent of detection par image during the last second in %, a low value means few detections occurs lately.
        /// </summary>
        public float ratioDetection;

        /// <summary>
        /// Average time difference for the current fused data.
        /// </summary>
        public float deltaTs;

    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the metrics of the fusion process.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct FusionMetrics
    {
        /// <summary>
        /// Mean number of camera that provides data during the past second.
        /// </summary>
        public float meanCameraFused;

        /// <summary>
        /// Standard deviation of the data timestamp fused, the lower the better.
        /// </summary>
        public float meanStdevBetweenCamera;

        /// <summary>
        /// Sender metrics.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)(Constant.MAX_FUSED_CAMERAS))]
        public CameraMetrics[] cameraIndividualStats;
    };

    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////  GNSS API ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region GNSS API

    /// \ingroup Fusion_group
    /// <summary>
    /// Lists the different states of the GNSS calibration.
    /// </summary>
    public enum GNSS_CALIBRATION_STATE
    {
        /// <summary>
        /// The GNSS/VIO calibration has not been completed yet.
        /// \n Please continue moving the robot while ingesting GNSS data to perform the calibration.
        /// </summary>
        GNSS_CALIBRATION_STATE_NOT_CALIBRATED = 0,
        /// <summary>
        /// The GNSS/VIO calibration is completed.
        /// </summary>
        GNSS_CALIBRATION_STATE_CALIBRATED = 1,
        /// <summary>
        /// A GNSS/VIO re-calibration is in progress in the background.
        /// \n Current geo-tracking services may not be accurate.
        /// </summary>
        GNSS_CALIBRATION_STATE_RE_CALIBRATION_IN_PROGRESS = 2
    };

    /// \ingroup Sensor_group
    /// <summary>
    /// Structure containing GNSS data to be used for positional tracking as prior.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GNSSData
    {
        /// <summary>
        /// Longitude in radian.
        /// </summary>
        public double longitude;
        /// <summary>
        /// Latitude in radian.
        /// </summary>
        public double latitude;
        /// <summary>
        /// Altitude in meter.
        /// </summary>
        public double altitude;
        /// <summary>
        /// Timestamp of the GNSS position in nanoseconds (must be aligned with the camera time reference).
        /// </summary>
        public ulong ts;
        /// <summary>
        /// Covariance of the position in meter (must be expressed in the ENU coordinate system).
        /// </summary>
        /// For eph, epv GNSS sensors, set it as follow: ```{eph*eph, 0, 0, 0, eph*eph, 0, 0, 0, epv*epv}```.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
        public double[] positionCovariance;
        /// <summary>
        /// Longitude standard deviation.
        /// </summary>
        public double longitudeStd;
        /// <summary>
        /// Latitude standard deviation.
        /// </summary>
        public double latitudeStd;
        /// <summary>
        /// Altitude standard deviation.
        /// </summary>
        public double altitudeStd;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds Geo reference position.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GeoPose
    {
        /// <summary>
        /// The translation defining the pose in ENU.
        /// </summary>
        public Vector3 translation;
        /// <summary>
        /// The rotation defining the pose in ENU.
        /// </summary>
        public Quaternion rotation;
        /// <summary>
        /// The pose covariance in ENU.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
        public float[] poseCovariance;
        /// <summary>
        /// The horizontal accuracy.
        /// </summary>
        public double horizontalAccuracy;
        /// <summary>
        /// The vertical accuracy.
        /// </summary>
        public double verticalAccuracy;
        /// <summary>
        /// The latitude, longitude, altitude.
        /// </summary>
        public LatLng latCoordinate;
        /// <summary>
        /// The heading.
        /// </summary>
        public double heading;
        /// <summary>
        /// The timestamp of GeoPose.
        /// </summary>
        public ulong timestamp;
    };

    /// \ingroup Fusion_group
    /// <summary>
    /// Represents a world position in ECEF format.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ECEF
    {
        /// <summary>
        /// x coordinate of ECEF.
        /// </summary>
        public double x;
        /// <summary>
        /// y coordinate of ECEF.
        /// </summary>
        public double y;
        /// <summary>
        /// z coordinate of ECEF.
        /// </summary>
        public double z;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Represents a world position in LatLng format.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct LatLng
    {
        /// <summary>
        /// Latitude in radian.
        /// </summary>
        public double latitude;
        /// <summary>
        /// Longitude in radian.
        /// </summary>
        public double longitude;
        /// <summary>
        /// Altitude in meter.
        /// </summary>
        public double altitude;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Represents a world position in UTM format.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct UTM
    {
        /// <summary>
        /// Northing coordinate.
        /// </summary>
        public double northing;
        /// <summary>
        /// Easting coordinate.
        /// </summary>
        public double easting;
        /// <summary>
        /// Gamma coordinate.
        /// </summary>
        public double gamma;
        /// <summary>
        /// UTMZone of the coordinate.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
        public string UTMZone;
    }

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the options used for calibrating GNSS / VIO.
    /// </summary>
    public class GNSSCalibrationParameters
    {
        /// <summary>
        /// This parameter defines the target yaw uncertainty at which the calibration process between GNSS and VIO concludes.
        /// The unit of this parameter is in radian.
        /// 
        /// Default: 0.1 radians
        /// </summary>
        public float targetYawUncertainty = 0.1f;
        /// <summary>
        /// When this parameter is enabled (set to true), the calibration process between GNSS and VIO accounts for the uncertainty in the determined translation, thereby facilitating the calibration termination. 
        /// The maximum allowable uncertainty is controlled by the 'target_translation_uncertainty' parameter.
	    ///
        /// Default: false
        /// </summary>
        public bool enableTranslationUncertaintyTarget = false;
        /// <summary>
        /// This parameter defines the target translation uncertainty at which the calibration process between GNSS and VIO concludes.
        ///
        /// Default: 10e-2 (10 centimeters)
        /// </summary>
        public float targetTranslationUncertainty = 10e-2f;
        /// <summary>
        /// This parameter determines whether reinitialization should be performed between GNSS and VIO fusion when a significant disparity is detected between GNSS data and the current fusion data.
        /// It becomes particularly crucial during prolonged GNSS signal loss scenarios.
        /// 
        /// Default: true
        /// </summary>
        public bool enableReinitialization = true;
        /// <summary>
        /// This parameter determines the threshold for GNSS/VIO reinitialization.
        /// If the fused position deviates beyond out of the region defined by the product of the GNSS covariance and the gnss_vio_reinit_threshold, a reinitialization will be triggered.
        /// 
        /// Default: 5
        /// </summary>
        public float gnssVioReinitThreshold = 5;
        /// <summary>
        /// If this parameter is set to true, the fusion algorithm will used a rough VIO / GNSS calibration at first and then refine it.
        /// This allow you to quickly get a fused position.
        ///
        ///  Default: true
        /// </summary>     
        public bool enableRollingCalibration = true;

        public GNSSCalibrationParameters(float targetYawUncertainty_ = 0.1f, bool enableTranslationUncertaintyTarget_ = false, float targetTranslationUncertainty_ = 0.01f,
            bool enableReinitialization_ = true, float gnssVioReinitThreshold_ = 5, bool enableRollingCalibration_ = true)
        {
            targetYawUncertainty = targetYawUncertainty_;
            enableTranslationUncertaintyTarget = enableTranslationUncertaintyTarget_;
            targetTranslationUncertainty = targetTranslationUncertainty_;
            enableReinitialization = enableReinitialization_;
            gnssVioReinitThreshold = gnssVioReinitThreshold_;
            enableRollingCalibration = enableRollingCalibration_;
        }
    };

    /// \ingroup Fusion_group
    /// <summary>
    /// Holds the options used for initializing the positional tracking fusion module.
    /// </summary>
    public class PositionalTrackingFusionParameters
    {
        /// <summary>
        /// This attribute is responsible for enabling or not GNSS positional tracking fusion.
        /// </summary>
        public bool enableGNSSFusion = false;
        /// <summary>
        /// Control the VIO / GNSS calibration process.
        /// </summary>
        public GNSSCalibrationParameters gnssCalibrationParameters;
        /// <summary>
        /// Constructor
        /// </summary>
        public PositionalTrackingFusionParameters()
        {
            enableGNSSFusion = false;
            gnssCalibrationParameters = new GNSSCalibrationParameters();
        }
    }

    #endregion
}// end namespace sl
