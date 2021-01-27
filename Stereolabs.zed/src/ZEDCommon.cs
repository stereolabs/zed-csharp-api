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
/// \defgroup Sensors_group Sensors Module

namespace sl
{

    public class ZEDCommon
    {
        public const string NameDLL = "sl_zed_interface.dll";

    }

    /// <summary>
    /// Constant for plugin. Should not be changed
    /// </summary>
    public enum Constant
    {
        MAX_OBJECTS = 100,
        /// <summary>
        /// Maximum number of chunks. It's best to get relatively few chunks and to update them quickly.
        /// </summary>
        MAX_SUBMESH = 1000
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Core  ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Core Module

    /// \ingroup Core_group
    /// <summary>
    /// Holds a 3x3 matrix that can be marshaled between the 
    /// wrapper and C# scripts.
    /// </summary>
    public struct Matrix3x3
    {
        /// <summary>
        /// 3x3 matrix as the float array.
        /// </summary>
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
        public float[] m; //3x3 matrix.

        /// <summary>
        /// Gives the result of the addition between a Matrix3x3 and a specified scalar value.
        /// </summary>
        /// <param name="a">The scalar value</param>
        /// <returns></returns>
        public float3 multiply(float3 a)
        {
            float3 result = new float3();

            result.x = m[0] * a.x + m[1] * a.y + m[2] * a.z;
            result.y = m[3] * a.x + m[4] * a.y + m[5] * a.z;
            result.z = m[6] * a.x + m[7] * a.y + m[8] * a.z;

            return result;
        }
    };

    /// <summary>
    /// \ingroup Core_group
    /// Holds a camera resolution as two pointers (for height and width) for easy
    /// passing back and forth to the ZED wrapper.
    /// </summary>
    public struct Resolution
    {
        /// <summary>
        /// Resolution of the image.
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        public Resolution(uint width, uint height)
        {
            this.width = (System.UIntPtr)width;
            this.height = (System.UIntPtr)height;
        }
        /// <summary>
        /// width in pixels.
        /// </summary>
        public System.UIntPtr width;
        /// <summary>
        /// height in pixels.
        /// </summary>
        public System.UIntPtr height;
    };

    /// <summary>
    /// \ingroup Core_group
    /// Rect structure to define a rectangle or a ROI in pixels
    /// Use to set ROI target for AEC/AGC
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
        /// width of the rectangle in pixels.
        /// </summary>
        public int width;
        /// <summary>
        /// height of the rectangle in pixels.
        /// </summary>
        public int height;
    };

    ///\ingroup  Core_group
    /// <summary>
    /// List of error codes in the ZED SDK.
    /// </summary><remarks>
    /// Mirrors ERROR_CODE in the ZED C++ SDK. For more info, read:
    /// https://www.stereolabs.com/docs/api/group__Core__group.html#ga4db9ee29f2ff83c71567c12f6bfbf28c
    /// </remarks>
    public enum ERROR_CODE
    {
        /// <summary>
        /// Operation was successful.
        /// </summary>
        SUCCESS,
        /// <summary>
        /// Standard, generic code for unsuccessful behavior when no other code is more appropriate.
        /// </summary>
        FAILURE,
        /// <summary>
        /// No GPU found, or CUDA capability of the device is not supported.
        /// </summary>
        NO_GPU_COMPATIBLE,
        /// <summary>
        /// Not enough GPU memory for this depth mode. Try a different mode (such as PERFORMANCE).
        /// </summary>
        NOT_ENOUGH_GPUMEM,
        /// <summary>
        /// The ZED camera is not plugged in or detected.
        /// </summary>
        CAMERA_NOT_DETECTED,
        /// <summary>
        /// a ZED Mini is detected but the inertial sensor cannot be opened. (Never called for original ZED)
        /// </summary>
        SENSOR_NOT_DETECTED,
        /// <summary>
        /// For Nvidia Jetson X1 only - resolution not yet supported (USB3.0 bandwidth).
        /// </summary>
        INVALID_RESOLUTION,
        /// <summary>
        /// USB communication issues. Occurs when the camera FPS cannot be reached, due to a lot of corrupted frames.
        /// Try changing the USB port.
        /// </summary>
        LOW_USB_BANDWIDTH,
        /// <summary>
        /// ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.
        /// </summary>
        CALIBRATION_FILE_NOT_AVAILABLE,
        /// <summary>
        /// ZED calibration file is not valid. Try downloading the factory one or recalibrating using the ZED Calibration tool.
        /// </summary>
        INVALID_CALIBRATION_FILE,
        /// <summary>
        /// The provided SVO file is not valid.
        /// </summary>
        INVALID_SVO_FILE,
        /// <summary>
        /// An SVO recorder-related error occurred (such as not enough free storage or an invalid file path).
        /// </summary>
        SVO_RECORDING_ERROR,
        /// <summary>
        /// An SVO related error when NVIDIA based compression cannot be loaded
        /// </summary>
        SVO_UNSUPPORTED_COMPRESSION,
        /// <summary>
        /// The requested coordinate system is not available.
        /// </summary>
        INVALID_COORDINATE_SYSTEM,
        /// <summary>
        /// The firmware of the ZED is out of date. Update to the latest version.
        /// </summary>
        INVALID_FIRMWARE,
        /// <summary>
        ///  An invalid parameter has been set for the function.
        /// </summary>
        INVALID_FUNCTION_PARAMETERS,
        /// <summary>
        /// In grab() only, the current call return the same frame as last call. Not a new frame.
        /// </summary>
        NOT_A_NEW_FRAME,
        /// <summary>
        /// In grab() only, a CUDA error has been detected in the process. Activate wrapperVerbose in ZEDManager.cs for more info.
        /// </summary>
        CUDA_ERROR,
        /// <summary>
        /// In grab() only, ZED SDK is not initialized. Probably a missing call to sl::Camera::open.
        /// </summary>
        CAMERA_NOT_INITIALIZED,
        /// <summary>
        /// Your NVIDIA driver is too old and not compatible with your current CUDA version.
        /// </summary>
        NVIDIA_DRIVER_OUT_OF_DATE,
        /// <summary>
        /// The function call is not valid in the current context. Could be a missing a call to sl::Camera::open.
        /// </summary>
        INVALID_FUNCTION_CALL,
        /// <summary>
        ///  The SDK wasn't able to load its dependencies, the installer should be launched.
        /// </summary>
        CORRUPTED_SDK_INSTALLATION,
        /// <summary>
        /// The installed SDK is not the SDK used to compile the program.
        /// </summary>
        INCOMPATIBLE_SDK_VERSION,
        /// <summary>
        /// The given area file does not exist. Check the file path.
        /// </summary>
        INVALID_AREA_FILE,
        /// <summary>
        /// The area file does not contain enough data to be used ,or the sl::DEPTH_MODE used during the creation of the
        /// area file is different from the one currently set.
        /// </summary>
        INCOMPATIBLE_AREA_FILE,
        /// <summary>
        /// Camera failed to set up.
        /// </summary>
        CAMERA_FAILED_TO_SETUP,
        /// <summary>
        /// Your ZED cannot be opened. Try replugging it to another USB port or flipping the USB-C connector (if using ZED Mini).
        /// </summary>
        CAMERA_DETECTION_ISSUE,
        /// <summary>
        /// The Camera is already in use by another process.
        /// </summary>
        CAMERA_ALREADY_IN_USE,
        /// <summary>
        /// No GPU found or CUDA is unable to list it. Can be a driver/reboot issue.
        /// </summary>
        NO_GPU_DETECTED,
        /// <summary>
        /// Plane not found. Either no plane is detected in the scene, at the location or corresponding to the floor,
        /// or the floor plane doesn't match the prior given.
        /// </summary>
        PLANE_NOT_FOUND,
        /// <summary>
        /// Missing or corrupted AI module ressources.
        /// Please reinstall the ZED SDK with the AI (object detection) module to fix this issue
        /// </summary>
        AI_MODULE_NOT_AVAILABLE,
        /// <summary>
        /// The cuDNN library cannot be loaded, or is not compatible with this version of the ZED SDK
        /// </summary>
        INCOMPATIBLE_CUDNN_VERSION,
        /// <summary>
        /// internal sdk timestamp is not valid
        /// </summary>
        AI_INVALID_TIMESTAMP,
        /// <summary>
        /// an error occur while tracking objects
        /// </summary>
        AI_UNKNOWN_ERROR,
        /// <summary>
        /// End of ERROR_CODE
        /// </summary>
        ERROR_CODE_LAST
    };

    ///\ingroup  Core_group
    /// <summary>
    /// List of available coordinate systems.
    /// </summary>
    public enum COORDINATE_SYSTEM
    {
        /// <summary>
        /// Standard coordinates system used in computer vision.
        /// Used in OpenCV. See: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        /// </summary>
        IMAGE,
        /// <summary>
        /// Left-Handed with Y up and Z forward. Used in Unity3D with DirectX
        /// </summary>
        LEFT_HANDED_Y_UP,
        /// <summary>
        ///  Right-Handed with Y pointing up and Z backward. Used in OpenGL.
        /// </summary>
        RIGHT_HANDED_Y_UP,
        /// <summary>
        /// Right-Handed with Z pointing up and Y forward. Used in 3DSMax.
        /// </summary>
        RIGHT_HANDED_Z_UP,
        /// <summary>
        /// Left-Handed with Z axis pointing up and X forward. Used in Unreal Engine.
        /// </summary>
        LEFT_HANDED_Z_UP,
        /// <summary>
        /// Right-Handed with Z pointing up and X forward. Used in ROS (REP 103)
        /// </summary>
        RIGHT_HANDED_Z_UP_X_FWD
    }

    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Tracking  ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    #region Tracking Module

    /// \ingroup PositionalTracking_group
    /// <summary>
    /// Parameters for positional tracking initialization.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public class PositionalTrackingParameters
    {
        /// <summary>
        /// Rotation of the camera in the world frame when the camera is started.By default, it should be identity.
        /// </summary>
        public Quaternion initialWorldRotation = Quaternion.Identity;
        /// <summary>
        /// Position of the camera in the world frame when the camera is started. By default, it should be zero.
        /// </summary>
        public Vector3 initialWorldPosition = Vector3.Zero;
        /// <summary>
        /// This mode enables the camera to remember its surroundings. This helps correct positional tracking drift, and can be helpful for positioning different cameras relative to one other in space.
        /// </summary>
        public bool enableAreaMemory = true;
        /// <summary>
        /// This mode enables smooth pose correction for small drift correction.
        /// </summary>
        public bool enablePoseSmothing = false;
        /// <summary>
        /// This mode initializes the tracking to be aligned with the floor plane to better position the camera in space.
        /// </summary>
        public bool setFloorAsOrigin = false;
        /// <summary>
        /// This setting allows you define the camera as static. If true, it will not move in the environment. This allows you to set its position using initial_world_transform.
        /// </summary>
        public bool setAsStatic = false;
        /// <summary>
        /// This setting allows you to enable or disable IMU fusion. When set to false, only the optical odometry will be used.
        /// </summary>
        public bool enableIMUFusion = true;
        /// <summary>
        /// Area localization file that describes the surroundings, saved from a previous tracking session.
        /// </summary>
        public string areaFilePath = "";


    }
    /// \ingroup PositionalTracking_group 
    /// <summary>
    /// Pose structure with data on timing and validity in addition to
    /// position and rotation.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Pose
    {
        /// <summary>
        /// boolean that indicates if tracking is activated or not. You should check that first if something wrong.
        /// </summary>
        public bool valid;
        /// <summary>
        /// Timestamp of the pose. This timestamp should be compared with the camera timestamp for synchronization.
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// orientation from the pose.
        /// </summary>
        public Quaternion rotation;
        /// <summary>
        /// translation from the pose.
        /// </summary>
        public Vector3 translation;
        /// <summary>
        /// Confidence/Quality of the pose estimation for the target frame.
        /// A confidence metric of the tracking[0 - 100], 0 means that the tracking is lost, 100 means that the tracking can be fully trusted.
        /// </summary>
        public int pose_confidence;
    };

    ///\ingroup PositionalTracking_group
    /// <summary>
    /// Possible states of the ZED's Tracking system.
    /// </summary>
    public enum POSITIONAL_TRACKING_STATE
    {
        /// <summary>
        /// Tracking is searching for a match from the database to relocate to a previously known position.
        /// </summary>
        SEARCHING,
        /// <summary>
        /// Tracking is operating normally; tracking data should be correct.
        /// </summary>
        OK,
        /// <summary>
        /// Tracking is not enabled.
        /// </summary>
        OFF,
        /// <summary>
        ///Effective FPS is too low to give proper results for motion tracking. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA, HD720))
        /// </summary>
        FPS_TOO_LOW
    }

    /// \ingroup PositionalTracking_group
    /// <summary>
    /// Reference frame (world or camera) for tracking and depth sensing.
    /// </summary>
    public enum REFERENCE_FRAME
    {
        /// <summary>
        /// Matrix contains the total displacement from the world origin/the first tracked point.
        /// </summary>
        WORLD,
        /// <summary>
        /// Matrix contains the displacement from the previous camera position to the current one.
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
    /// Full IMU data structure.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ImuData
    {
        /// <summary>
        /// Indicates if imu data is available
        /// </summary>
        public bool available;
        /// <summary>
        /// IMU Data timestamp in ns
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// Gyroscope calibrated data in degrees/second.
        /// </summary>
		public Vector3 angularVelocity;
        /// <summary>
        /// Accelerometer calibrated data in m/s².
        /// </summary>
		public Vector3 linearAcceleration;
        /// <summary>
        /// Gyroscope raw/uncalibrated data in degrees/second.
        /// </summary>
        public Vector3 angularVelocityUncalibrated;
        /// <summary>
        /// Accelerometer raw/uncalibrated data in m/s².
        /// </summary>
		public Vector3 linearAccelerationUncalibrated;
        /// <summary>
        /// Orientation from gyro/accelerator fusion.
        /// </summary>
		public Quaternion fusedOrientation;
        /// <summary>
        /// Covariance matrix of the quaternion.
        /// </summary>
		public Matrix3x3 orientationCovariance;
        /// <summary>
        /// Gyroscope raw data covariance matrix.
        /// </summary>
		public Matrix3x3 angularVelocityCovariance;
        /// <summary>
        /// Accelerometer raw data covariance matrix.
        /// </summary>
		public Matrix3x3 linearAccelerationCovariance;
    };

    /// \ingroup Sensors_group
    [StructLayout(LayoutKind.Sequential)]
    public struct BarometerData
    {
        /// <summary>
        /// Indicates if mag data is available
        /// </summary>
        public bool available;
        /// <summary>
        /// mag Data timestamp in ns
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// Barometer ambient air pressure in hPa
        /// </summary>
        public float pressure;
        /// <summary>
        /// Relative altitude from first camera position
        /// </summary>
        public float relativeAltitude;
    };

    /// \ingroup Sensors_group
    [StructLayout(LayoutKind.Sequential)]
    public struct MagnetometerData
    {
        /// <summary>
        /// Indicates if mag data is available
        /// </summary>
        public bool available;
        /// <summary>
        /// mag Data timestamp in ns
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// Magnetic field calibrated values in uT
        /// </summary>
        public Vector3 magneticField;
        /// <summary>
        /// (3x1) Vector for magnetometer raw values (uncalibrated) In other words, the current magnetic field (uT), along with the x, y, and z axes.
        /// </summary>
        public Vector3 magneticFieldUncalibrated;
        /// <summary>
        /// (3x1) Vector for magnetometer values (after user calibration) In other words, the current magnetic field (uT), along with the x, y, and z axes.
        /// <remarks>To calibrate the magnetometer sensor please use the ZED Sensor Viewer tool</remarks>
        /// </summary>
        public Vector3 magneticFieldCalibrated;
    };

    /// \ingroup Sensors_group
    [StructLayout(LayoutKind.Sequential)]
    public struct TemperatureSensorData
    {
        /// <summary>
        /// Temperature from IMU device ( -100 if not available)
        /// </summary>
        public float imu_temp;
        /// <summary>
        /// Temperature from Barometer device ( -100 if not available)
        /// </summary>
        public float barometer_temp;
        /// <summary>
        /// Temperature from Onboard left analog temperature sensor ( -100 if not available)
        /// </summary>
        public float onboard_left_temp;
        /// <summary>
        /// Temperature from Onboard right analog temperature sensor ( -100 if not available)
        /// </summary>
        public float onboard_right_temp;
    };

    /// \ingroup Sensors_group
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorsData
    {
        /// <summary>
        /// Contains Imu Data
        /// </summary>
        public ImuData imu;
        /// <summary>
        /// Contains Barometer Data
        /// </summary>
        public BarometerData barometer;
        /// <summary>
        /// Contains Mag Data
        /// </summary>
        public MagnetometerData magnetometer;
        /// <summary>
        /// Contains Temperature Data
        /// </summary>
        public TemperatureSensorData temperatureSensor;
        /// <summary>
        /// Indicated if camera is :
        /// -> Static : 0
        /// -> Moving : 1
        /// -> Falling : 2
        /// </summary>
        public int camera_moving_state;
        /// <summary>
        /// Indicates if the current sensors data is sync to the current image (>=1). Otherwise, value will be 0.
        /// </summary>
        public int image_sync_val;
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// List of the available onboard sensors
    /// </summary>
    public enum SENSOR_TYPE
    {
        /// <summary>
        /// Three axis Accelerometer sensor to measure the inertial accelerations.
        /// </summary>
        ACCELEROMETER,
        /// <summary>
        /// Three axis Gyroscope sensor to measure the angular velocitiers.
        /// </summary>
        GYROSCOPE,
        /// <summary>
        /// Three axis Magnetometer sensor to measure the orientation of the device respect to the earth magnetic field.
        /// </summary>
        MAGNETOMETER,
        /// <summary>
        /// Barometer sensor to measure the atmospheric pressure.
        /// </summary>
        BAROMETER,

        LAST
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// List of the available onboard sensors measurement units
    /// </summary>
    public enum SENSORS_UNIT
    {
        /// <summary>
        /// Acceleration [m/s²].
        /// </summary>
        M_SEC_2,
        /// <summary>
        /// Angular velocity [deg/s].
        /// </summary>
        DEG_SEC,
        /// <summary>
        /// Magnetic Fiels [uT].
        /// </summary>
        U_T,
        /// <summary>
        /// Atmospheric pressure [hPa].
        /// </summary>
        HPA,
        /// <summary>
        /// Temperature [°C].
        /// </summary>
        CELSIUS,
        /// <summary>
        /// Frequency [Hz].
        /// </summary>
        HERTZ,
        /// <summary>
        /// 
        /// </summary>
        LAST
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing information about a single sensor available in the current device
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorParameters
    {
        /// <summary>
        /// The type of the sensor as \ref DEVICE_SENSORS.
        /// </summary>
        public SENSOR_TYPE type;
        /// <summary>
        /// The resolution of the sensor.
        /// </summary>
        public float resolution;
        /// <summary>
        /// The sampling rate (or ODR) of the sensor.
        /// </summary>
        public float sampling_rate;
        /// <summary>
        /// The range values of the sensor. MIN: `range.x`, MAX: `range.y`
        /// </summary>
        public float2 range;
        /// <summary>
        /// also known as white noise, given as continous (frequency independant). Units will be expressed in sensor_unit/√(Hz). `NAN` if the information is not available.
        /// </summary>
        public float noise_density;
        /// <summary>
        /// derived from the Allan Variance, given as continous (frequency independant). Units will be expressed in sensor_unit/s/√(Hz).`NAN` if the information is not available.
        /// </summary>
        public float random_walk;
        /// <summary>
        /// The string relative to the measurement unit of the sensor.
        /// </summary>
        public SENSORS_UNIT sensor_unit;
        /// <summary>
        /// 
        /// </summary>
        public bool isAvailable;
    };

    /// \ingroup Sensors_group
    /// <summary>
    /// Structure containing information about all the sensors available in the current device
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorsConfiguration
    {
        /// <summary>
        /// The firmware version of the sensor module, 0 if no sensors are available (ZED camera model).
        /// </summary>
        public uint firmware_version;
        /// <summary>
        /// contains rotation between IMU frame and camera frame.
        /// </summary>
        public float4 camera_imu_rotation;
        /// <summary>
        /// contains translation between IMU frame and camera frame.
        /// </summary>
        public float3 camera_imu_translation;
        /// <summary>
        /// Configuration of the accelerometer device.
        /// </summary>
        public SensorParameters accelerometer_parameters;
        /// <summary>
        /// Configuration of the gyroscope device.
        /// </summary>
        public SensorParameters gyroscope_parameters;
        /// <summary>
        /// Configuration of the magnetometer device.
        /// </summary>
        public SensorParameters magnetometer_parameters;
        /// <summary>
        /// Configuration of the barometer device
        /// </summary>
        public SensorParameters barometer_parameters;
        /// <summary>
        /// if a sensor type is available on the device 
        /// </summary>
        /// <param name="sensor_type"></param>
        /// <returns></returns>
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

    ///\ingroup Depth_group
    /// <summary>
    /// Runtime parameters used by the ZEDCamera.Grab() function, and its Camera::grab() counterpart in the SDK.
    /// </summary>
    public class RuntimeParameters
    {
        /// <summary>
        /// Defines the algorithm used for depth map computation, more info : \ref SENSING_MODE definition.
        /// </summary>
        public sl.SENSING_MODE sensingMode;
        /// <summary>
        /// Provides 3D measures (point cloud and normals) in the desired reference frame (default is REFERENCE_FRAME_CAMERA).
        /// </summary>
        public sl.REFERENCE_FRAME measure3DReferenceFrame;
        /// <summary>
        /// Defines whether the depth map should be computed.
        /// </summary>
        public bool enableDepth;
        /// <summary>
        ///  Defines the confidence threshold for the depth. Based on stereo matching score.
        /// </summary>
        public int confidenceThreshold;
        /// <summary>
        /// Defines texture confidence threshold for the depth. Based on textureness confidence.
        /// </summary>
        public int textureConfidenceThreshold;

        /// <summary>
        /// Constructor
        /// </summary>
        public RuntimeParameters(sl.SENSING_MODE smode = SENSING_MODE.STANDARD, REFERENCE_FRAME reframe = REFERENCE_FRAME.CAMERA, bool depth = true, int cnf_threshold = 100, int txt_cnf_threshold = 100)
        {
            this.sensingMode = smode;
            this.measure3DReferenceFrame = reframe;
            this.enableDepth = depth;
            this.confidenceThreshold = cnf_threshold;
            this.textureConfidenceThreshold = txt_cnf_threshold;
        }
    }

    #region Depth Sensing Module

    ///\ingroup Depth_group
    /// <summary>
    /// Calibration information for an individual sensor on the ZED (left or right). </summary>
    /// <remarks>For more information, see:
    /// https://www.stereolabs.com/docs/api/structsl_1_1CameraParameters.html </remarks>
    [StructLayout(LayoutKind.Sequential)]
    public struct CameraParameters
    {
        /// <summary>
        /// Focal X.
        /// </summary>
        public float fx;
        /// <summary>
        /// Focal Y.
        /// </summary>
        public float fy;
        /// <summary>
        /// Optical center X.
        /// </summary>
        public float cx;
        /// <summary>
        /// Optical center Y.
        /// </summary>
        public float cy;

        /// <summary>
        /// Distortion coefficients.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.U8, SizeConst = 5)]
        public double[] disto;

        /// <summary>
        /// Vertical field of view after stereo rectification.
        /// </summary>
        public float vFOV;
        /// <summary>
        /// Horizontal field of view after stereo rectification.
        /// </summary>
        public float hFOV;
        /// <summary>
        /// Diagonal field of view after stereo rectification.
        /// </summary>
        public float dFOV;
        /// <summary>
        /// Camera's current resolution.
        /// </summary>
        public Resolution resolution;
    };

    ///\ingroup Depth_group
    /// <summary>
    /// Holds calibration information about the current ZED's hardware, including per-sensor
    /// calibration and offsets between the two sensors.
    /// </summary> <remarks>For more info, see:
    /// https://www.stereolabs.com/docs/api/structsl_1_1CalibrationParameters.html </remarks>
    [StructLayout(LayoutKind.Sequential)]
    public struct CalibrationParameters
    {
        /// <summary>
        /// Parameters of the left sensor.
        /// </summary>
        public CameraParameters leftCam;
        /// <summary>
        /// Parameters of the right sensor.
        /// </summary>
        public CameraParameters rightCam;
        /// <summary>
        /// Rotation (using Rodrigues' transformation) between the two sensors. Defined as 'tilt', 'convergence' and 'roll'.
        /// </summary>
        public Quaternion Rot;
        /// <summary>
        /// Translation between the two sensors. T[0] is the distance between the two cameras in meters.
        /// </summary>
        public Vector3 Trans;
    };

    ///\ingroup Depth_group
    /// <summary>
    /// Lists available depth computation modes. Each mode offers better accuracy than the
    /// mode before it, but at a performance cost.
    /// </summary><remarks>
    /// Mirrors DEPTH_MODE in the ZED C++ SDK. For more info, see:
    /// https://www.stereolabs.com/docs/api/group__Depth__group.html#ga8d542017c9b012a19a15d46be9b7fa43
    /// </remarks>
    public enum DEPTH_MODE
    {
        /// <summary>
        /// Does not compute any depth map. Only rectified stereo images will be available.
        /// </summary>
        NONE,
        /// <summary>
        /// Fastest mode for depth computation.
        /// </summary>
        PERFORMANCE,
        /// <summary>
        /// Balanced quality mode. Depth map is robust in most environment and requires medium compute power.
        /// </summary>
        QUALITY,
        /// <summary>
        /// Native depth. Very accurate, but at a large performance cost.
        /// </summary>
        ULTRA
    };

    ///\ingroup  Depth_group
    /// <summary>
    /// Lists available sensing modes - whether to produce the original depth map (STANDARD) or one with
    /// smoothing and other effects added to fill gaps and roughness (FILL).
    /// </summary>
    public enum SENSING_MODE
    {
        /// <summary>
        /// This mode outputs the standard ZED depth map that preserves edges and depth accuracy.
        /// However, there will be missing data where a depth measurement couldn't be taken, such as from
        /// a surface being occluded from one sensor but not the other.
        /// Better for: Obstacle detection, autonomous navigation, people detection, 3D reconstruction.
        /// </summary>
        STANDARD,
        /// <summary>
        /// This mode outputs a smooth and fully dense depth map. It doesn't have gaps in the data
        /// like STANDARD where depth can't be calculated directly, but the values it fills them with
        /// is less accurate than a real measurement.
        /// Better for: AR/VR, mixed-reality capture, image post-processing.
        /// </summary>
        FILL
    };

    ///\ingroup Depth_group
    /// <summary>
    /// Lists available measure types retrieved from the camera, used for creating precise measurement maps
    /// (Measure-type textures).
    /// Based on the MEASURE enum in the ZED C++ SDK. For more info, see:
    /// https://www.stereolabs.com/docs/api/group__Depth__group.html#ga798a8eed10c573d759ef7e5a5bcd545d
    /// </summary>
    public enum MEASURE
    {
        /// <summary>
        /// Disparity map. As a ZEDMat, MAT_TYPE is set to MAT_32F_C1.
        /// </summary>
        DISPARITY,
        /// <summary>
        /// Depth map. As a ZEDMat, MAT_TYPE is set to MAT_32F_C1.
        /// </summary>
        DEPTH,
        /// <summary>
        /// Certainty/confidence of the disparity map. As a ZEDMat, MAT_TYPE is set to MAT_32F_C1.
        /// </summary>
        CONFIDENCE,
        /// <summary>
        /// 3D coordinates of the image points. Used for point clouds in ZEDPointCloudManager.
        /// As a ZEDMat, MAT_TYPE is set to MAT_32F_C4. The 4th channel may contain the colors.
        /// </summary>
        XYZ,
        /// <summary>
        /// 3D coordinates and color of the image. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// The 4th channel encodes 4 UCHARs for colors in R-G-B-A order.
        /// </summary>
        XYZRGBA,
        /// <summary>
        /// 3D coordinates and color of the image. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// The 4th channel encode 4 UCHARs for colors in B-G-R-A order.
        /// </summary>
        XYZBGRA,
        /// <summary>
        /// 3D coordinates and color of the image. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// The 4th channel encodes 4 UCHARs for color in A-R-G-B order.
        /// </summary>
        XYZARGB,
        /// <summary>
        /// 3D coordinates and color of the image. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// Channel 4 contains color in A-B-G-R order.
        /// </summary>
        XYZABGR,
        /// <summary>
        /// 3D coordinates and color of the image. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// The 4th channel encode 4 UCHARs for color in A-B-G-R order.
        /// </summary>
        NORMALS,
        /// <summary>
        /// Disparity map for the right sensor. As a ZEDMat, MAT_TYPE is set to  MAT_32F_C1.
        /// </summary>
        DISPARITY_RIGHT,
        /// <summary>
        /// Depth map for right sensor. As a ZEDMat, MAT_TYPE is set to MAT_32F_C1.
        /// </summary>
        DEPTH_RIGHT,
        /// <summary>
        /// Point cloud for right sensor. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4. Channel 4 is empty.
        /// </summary>
        XYZ_RIGHT,
        /// <summary>
        /// Colored point cloud for right sensor. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// Channel 4 contains colors in R-G-B-A order.
        /// </summary>
        XYZRGBA_RIGHT,
        /// <summary>
        /// Colored point cloud for right sensor. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// Channel 4 contains colors in B-G-R-A order.
        /// </summary>
        XYZBGRA_RIGHT,
        /// <summary>
        ///  Colored point cloud for right sensor. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        ///  Channel 4 contains colors in A-R-G-B order.
        /// </summary>
        XYZARGB_RIGHT,
        /// <summary>
        /// Colored point cloud for right sensor. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        /// Channel 4 contains colors in A-B-G-R order.
        /// </summary>
        XYZABGR_RIGHT,
        /// <summary>
        ///  Normals vector for right view. As a ZEDMat, MAT_TYPE is set to MAT_32F_C4.
        ///  Channel 4 is empty (set to 0).
        /// </summary>
        NORMALS_RIGHT

    };

    #endregion

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Video   //////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///\ingroup Video_group
    /// <summary>
    /// Struct containing all parameters passed to the SDK when initializing the ZED.
    /// These parameters will be fixed for the whole execution life time of the camera.
    /// <remarks>For more details, see the InitParameters class in the SDK API documentation:
    /// https://www.stereolabs.com/docs/api/structsl_1_1InitParameters.html
    /// </remarks>
    /// </summary>

    public class InitParameters
    {
        public sl.INPUT_TYPE inputType;
        /// <summary>
        /// Resolution the ZED will be set to.
        /// </summary>
        public sl.RESOLUTION resolution;
        /// <summary>
        /// Requested FPS for this resolution. Setting it to 0 will choose the default FPS for this resolution.
        /// </summary>
        public int cameraFPS;
        /// <summary>
        /// ID for identifying which of multiple connected ZEDs to use.
        /// </summary>
        public int cameraDeviceID;
        /// <summary>
        /// Path to a recorded SVO file to play, including filename.
        /// </summary>
        public string pathSVO = "";
        /// <summary>
        /// In SVO playback, this mode simulates a live camera and consequently skipped frames if the computation framerate is too slow.
        /// </summary>
        public bool svoRealTimeMode;
        /// <summary>
        ///  Define a unit for all metric values (depth, point clouds, tracking, meshes, etc.).
        /// </summary>
        public UNIT coordinateUnits;
        /// <summary>
        /// This defines the order and the direction of the axis of the coordinate system.
        /// </summary>
        public COORDINATE_SYSTEM coordinateSystem;
        /// <summary>
        /// Quality level of depth calculations. Higher settings improve accuracy but cost performance.
        /// </summary>
        public sl.DEPTH_MODE depthMode;
        /// <summary>
        /// Minimum distance from the camera from which depth will be computed, in the defined coordinateUnit.
        /// </summary>
        public float depthMinimumDistance;
        /// <summary>
        ///   When estimating the depth, the SDK uses this upper limit to turn higher values into \ref TOO_FAR ones.
        ///   The current maximum distance that can be computed in the defined \ref UNIT.
        ///   Changing this value has no impact on performance and doesn't affect the positional tracking nor the spatial mapping. (Only the depth, point cloud, normals)
        /// </summary>
        public float depthMaximumDistance;
        /// <summary>
        ///  Defines if images are horizontally flipped.
        /// </summary>
        public FLIP_MODE cameraImageFlip;
        /// <summary>
        /// Defines if measures relative to the right sensor should be computed (needed for MEASURE_<XXX>_RIGHT).
        /// </summary>
        public bool enableRightSideMeasure;
        /// <summary>
        /// True to disable self-calibration and use the optional calibration parameters without optimizing them.
        /// False is recommended, so that calibration parameters can be optimized.
        /// </summary>
        public bool cameraDisableSelfCalib;
        /// <summary>
        /// True for the SDK to provide text feedback.
        /// </summary>
        public bool sdkVerbose;
        /// <summary>
        /// ID of the graphics card on which the ZED's computations will be performed.
        /// </summary>
        public int sdkGPUId;
        /// <summary>
        /// If set to verbose, the filename of the log file into which the SDK will store its text output.
        /// </summary>
        public string sdkVerboseLogFile = "";
        /// <summary>
        /// True to stabilize the depth map. Recommended.
        /// </summary>
        public bool depthStabilization;
        /// <summary>
        /// Optional path for searching configuration (calibration) file SNxxxx.conf. (introduced in ZED SDK 2.6)
        /// </summary>
        public string optionalSettingsPath = "";
        /// <summary>
        /// True to stabilize the depth map. Recommended.
        /// </summary>
        public bool sensorsRequired;
        /// <summary>
        /// Path to a recorded SVO file to play, including filename.
        /// </summary>
        public string ipStream = "";
        /// <summary>
        /// Path to a recorded SVO file to play, including filename.
        /// </summary>
        public ushort portStream = 30000;
        /// <summary>
        /// Whether to enable improved color/gamma curves added in ZED SDK 3.0.
        /// </summary>
        public bool enableImageEnhancement = true;
        /// <summary>
        /// Set an optional file path where the SDK can find a file containing the calibration information of the camera computed by OpenCV.
        /// <remarks> Using this will disable the factory calibration of the camera. </remarks>
        /// <warning> Erroneous calibration values can lead to poor SDK modules accuracy. </warning>
        /// </summary>
        public string optionalOpencvCalibrationFile;

        /// <summary>
        /// Constructor. Sets default initialization parameters.
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
            this.sdkVerbose = false;
            this.sdkGPUId = -1;
            this.sdkVerboseLogFile = "";
            this.enableRightSideMeasure = false;
            this.depthStabilization = true;
            this.optionalSettingsPath = "";
            this.sensorsRequired = false;
            this.ipStream = "";
            this.portStream = 30000;
            this.enableImageEnhancement = true;
            this.optionalOpencvCalibrationFile = "";
        }

    }

    #region Video Module

    ///\ingroup  Video_group
    /// <summary>
    /// Lists available input type in SDK.
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
        /// STREAM input mode (requires to use enableStreaming()/disableStreaming() on the "sender" side)
        /// </summary>
        STREAM
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Sets the recording parameters.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RecordingParameters
    {
        /// <summary>
        /// filename of the SVO file.
        /// </summary>
        public string videoFilename;
        /// <summary>
        /// compression_mode : can be one of the SVO_COMPRESSION_MODE enum
        /// </summary>
        public SVO_COMPRESSION_MODE compressionMode;
        /// <summary>
        /// defines the target framerate for the recording module.
        /// </summary>
        public int targetFPS;
        /// <summary>
        /// bitrate : override default bitrate of the SVO file, in KBits/s. Only works if SVO_COMPRESSION_MODE is H264 or H265. : 0 means default values (depends on the resolution)
        /// </summary>
        public uint bitrate;
        /// <summary>
        /// In case of streaming input, if set to false, it will avoid decoding/re-encoding and convert directly streaming input into a SVO file. 
        /// This saves a encoding session and can be especially useful on NVIDIA Geforce cards where the number of encoding session is limited.
        /// </summary>
        public bool transcode;
        /// <summary>
        /// Constructor
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
    /// Sets the streaming parameters.
    /// </summary>
    public struct StreamingParameters
    {
        /// <summary>
        /// Defines the codec used for streaming.
        /// </summary>
        STREAMING_CODEC codec;
        /// <summary>
        /// Defines the port used for streaming.
        /// </summary>
        public ushort port;
        /// <summary>
        /// Defines the streaming bitrate in Kbits/s.
        /// </summary>
        public uint bitrate;
        /// <summary>
        /// Defines the gop size in number of frames.
        /// </summary>
        public int gopSize;
        /// <summary>
        /// Enable/Disable adaptive bitrate.
        /// </summary>
        public bool adaptativeBitrate;
        /// <summary>
        /// Defines a single chunk size.
        /// </summary>
        public ushort chunkSize;
        /// <summary>
        /// defines the target framerate for the streaming output.
        /// </summary>
        public int targetFPS;
        /// <summary>
        /// Constructor
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
    /// Container for information about the current SVO recording process.
    /// </summary><remarks>
    /// Mirrors RecordingStatus in the ZED C++ SDK. For more info, visit:
    /// https://www.stereolabs.com/docs/api/structsl_1_1RecordingStatus.html
    /// </remarks>
    [StructLayout(LayoutKind.Sequential)]
    public struct RecordingStatus
    {
        /// <summary>
        /// Recorder status, true if enabled.
        /// </summary>
        public bool is_recording;
        /// <summary>
        /// Recorder status, true if the pause is enabled.
        /// </summary>
        public bool is_paused;
        /// <summary>
        /// Status of the current frame. True if recording was successful, false if frame could not be written.
        /// </summary>
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
        /// Average compression time in millisecond since beginning of recording.
        /// </summary>
        public double average_compression_time;
        /// <summary>
        /// Compression ratio (% of raw size) since recording was started.
        /// </summary>
        public double average_compression_ratio;
    }

    ///\ingroup  Video_group
    /// <summary>
    /// Represents the available resolution options.
    /// </summary>
    public enum RESOLUTION
    {
        /// <summary>
        /// 2208*1242. Supported frame rate: 15 FPS.
        /// </summary>
        HD2K,
        /// <summary>
        /// 1920*1080. Supported frame rates: 15, 30 FPS.
        /// </summary>
        HD1080,
        /// <summary>
        /// 1280*720. Supported frame rates: 15, 30, 60 FPS.
        /// </summary>
        HD720,
        /// <summary>
        /// 672*376. Supported frame rates: 15, 30, 60, 100 FPS.
        /// </summary>
        VGA
    };

    ///\ingroup  Video_group
    /// <summary>
    ///brief Lists available compression modes for SVO recording.
    /// </summary>
    public enum FLIP_MODE
    {
        /// <summary>
        /// default behavior.
        /// </summary>
        OFF = 0,
        /// <summary>
        /// Images and camera sensors data are flipped, useful when your camera is mounted upside down.
        /// </summary>
        ON = 1,
        /// <summary>
        /// in live mode: use the camera orientation (if an IMU is available) to set the flip mode, in SVO mode, read the state of this enum when recorded
        /// </summary>
        AUTO = 2,
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Types of compatible ZED cameras.
    /// </summary>
    public enum MODEL
    {
        /// <summary>
        /// ZED(1)
        /// </summary>
	    ZED,
        /// <summary>
        /// ZED Mini.
        /// </summary>
	    ZED_M,
        /// <summary>
        /// ZED2.
        /// </summary>
        ZED2
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Lists available view types retrieved from the camera, used for creating human-viewable (Image-type) textures.
    /// </summary><remarks>
    /// Based on the VIEW enum in the ZED C++ SDK. For more info, see:
    /// https://www.stereolabs.com/docs/api/group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c
    /// </remarks>
    public enum VIEW
    {
        /// <summary>
        /// Left RGBA image. As a Mat, MAT_TYPE is set to MAT_TYPE_8U_C4.
        /// </summary>
        LEFT,
        /// <summary>
        /// Right RGBA image. As a Mat, MAT_TYPE is set to sl::MAT_TYPE_8U_C4.
        /// </summary>
        RIGHT,
        /// <summary>
        /// Left GRAY image. As a Mat, MAT_TYPE is set to sl::MAT_TYPE_8U_C1.
        /// </summary>
        LEFT_GREY,
        /// <summary>
        /// Right GRAY image. As a Mat, MAT_TYPE is set to sl::MAT_TYPE_8U_C1.
        /// </summary>
        RIGHT_GREY,
        /// <summary>
        /// Left RGBA unrectified image. As a Mat, MAT_TYPE is set to sl::MAT_TYPE_8U_C4.
        /// </summary>
        LEFT_UNRECTIFIED,
        /// <summary>
        /// Right RGBA unrectified image. As a Mat, MAT_TYPE is set to sl::MAT_TYPE_8U_C4.
        /// </summary>
        RIGHT_UNRECTIFIED,
        /// <summary>
        /// Left GRAY unrectified image. As a ZEDMat, MAT_TYPE is set to sl::MAT_TYPE_8U_C1.
        /// </summary>
        LEFT_UNRECTIFIED_GREY,
        /// <summary>
        /// Right GRAY unrectified image. As a Mat, MAT_TYPE is set to sl::MAT_TYPE_8U_C1.
        /// </summary>
        RIGHT_UNRECTIFIED_GREY,
        /// <summary>
        ///  Left and right image. Will be double the width to hold both. As a Mat, MAT_TYPE is set to MAT_8U_C4.
        /// </summary>
        SIDE_BY_SIDE,
        /// <summary>
        /// Normalized depth image. As a Mat, MAT_TYPE is set to sl::MAT_TYPE_8U_C4.
        /// <para>Use an Image texture for viewing only. For measurements, use a Measure type instead
        /// (ZEDCamera.RetrieveMeasure()) to preserve accuracy. </para>
        /// </summary>
        DEPTH,
        /// <summary>
        /// Normalized confidence image. As a Mat, MAT_TYPE is set to MAT_8U_C4.
        /// <para>Use an Image texture for viewing only. For measurements, use a Measure type instead
        /// (ZEDCamera.RetrieveMeasure()) to preserve accuracy. </para>
        /// </summary>
        CONFIDENCE,
        /// <summary>
        /// Color rendering of the normals. As a Mat, MAT_TYPE is set to MAT_8U_C4.
        /// <para>Use an Image texture for viewing only. For measurements, use a Measure type instead
        /// (ZEDCamera.RetrieveMeasure()) to preserve accuracy. </para>
        /// </summary>
        NORMALS,
        /// <summary>
        /// Color rendering of the right depth mapped on right sensor. As a Mat, MAT_TYPE is set to MAT_8U_C4.
        /// <para>Use an Image texture for viewing only. For measurements, use a Measure type instead
        /// (ZEDCamera.RetrieveMeasure()) to preserve accuracy. </para>
        /// </summary>
        DEPTH_RIGHT,
        /// <summary>
        /// Color rendering of the normals mapped on right sensor. As a Mat, MAT_TYPE is set to MAT_8U_C4.
        /// <para>Use an Image texture for viewing only. For measurements, use a Measure type instead
        /// (ZEDCamera.RetrieveMeasure()) to preserve accuracy. </para>
        /// </summary>
        NORMALS_RIGHT,
        /// <summary>
        /// Depth map in millimeter. Each pixel  contains 1 unsigned short. As a Mat, MAT_TYPE is set to MAT_U16_C1.
        /// </summary>
        DEPTH_U16_MM,
        /// <summary>
        /// Depth map in millimeter for right sensor. Each pixel  contains 1 unsigned short. As a Mat, MAT_TYPE is set to MAT_U16_C1.
        /// </summary>
        DEPTH_U16_MM_RIGHT
    };

    ///\ingroup  Video_group
    /// <summary>
    ///  Lists available camera settings for the ZED camera (contrast, hue, saturation, gain, etc.)
    /// </summary>
    public enum VIDEO_SETTINGS
    {
        /// <summary>
        /// Brightness control. Value should be between 0 and 8.
        /// </summary>
        BRIGHTNESS,
        /// <summary>
        /// Contrast control. Value should be between 0 and 8.
        /// </summary>
        CONTRAST,
        /// <summary>
        /// Hue control. Value should be between 0 and 11.
        /// </summary>
        HUE,
        /// <summary>
        /// Saturation control. Value should be between 0 and 8.
        /// </summary>
        SATURATION,
        /// <summary>
        /// Sharpness control. Value should be between 0 and 8.
        /// </summary>
        SHARPNESS,
        /// <summary>
        /// Gamma control. Value should be between 1 and 9
        /// </summary>
        GAMMA,
        /// <summary>
        /// Gain control. Value should be between 0 and 100 for manual control.
        /// If ZED_EXPOSURE is set to -1 (automatic mode), then gain will be automatic as well.
        /// </summary>
        GAIN,
        /// <summary>
        /// Exposure control. Value can be between 0 and 100.
        /// Setting to -1 enables auto exposure and auto gain.
        /// Setting to 0 disables auto exposure but doesn't change the last applied automatic values.
        /// Setting to 1-100 disables auto mode and sets exposure to the chosen value.
        /// </summary>
        EXPOSURE,
        /// <summary>
        /// Auto-exposure and auto gain. Setting this to true switches on both. Assigning a specifc value to GAIN or EXPOSURE will set this to 0.
        /// </summary>
        AEC_AGC,
        /// <summary>
        /// ROI for auto exposure/gain. ROI defines the target where the AEC/AGC will be calculated
        /// Use overloaded function for this enum
        /// </summary>
        AEC_AGC_ROI,
        /// <summary>
        /// Color temperature control. Value should be between 2800 and 6500 with a step of 100.
        /// </summary>
        WHITEBALANCE,
        /// <summary>
        /// Defines if the white balance is in automatic mode or not.
        /// </summary>
        AUTO_WHITEBALANCE,
        /// <summary>
        /// front LED status (1==enable, 0 == disable)
        /// </summary>
        LED_STATUS
    };

    ///\ingroup  Video_group
    /// <summary>
    /// Categories indicating when a timestamp is captured.
    /// </summary>
    public enum TIME_REFERENCE
    {
        /// <summary>
        /// Timestamp from when the image was received over USB from the camera, defined
        /// by when the entire image was available in memory.
        /// </summary>
        IMAGE,
        /// <summary>
        /// Timestamp from when the relevant function was called.
        /// </summary>
        CURRENT
    };

    ///\ingroup  Video_group
    /// <summary>
    /// SVO compression modes.
    /// </summary>
    public enum SVO_COMPRESSION_MODE
    {
        /// <summary>
        /// Lossless compression based on png/zstd. Average size = 42% of RAW.
        /// </summary>
        LOSSLESS_BASED,
        /// <summary>
        /// AVCHD Based compression (H264). Available since ZED SDK 2.7
        /// </summary>
        H264_BASED,
        /// <summary>
        /// HEVC Based compression (H265). Available since ZED SDK 2.7
        /// </summary>
        H265_BASED,
    }

    ///\ingroup  Video_group
    /// <summary>
    /// Streaming codecs
    /// </summary>
    public enum STREAMING_CODEC
    {
        /// <summary>
        /// AVCHD/H264 Based compression
        /// </summary>
        H264_BASED,
        /// <summary>
        /// HEVC/H265 Based compression
        /// </summary>
        H265_BASED
    }
    /// <summary>
    /// defines left,right,both to distinguish between left and right or both sides
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
        /// Left and Right side.
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
    /// Sets the spatial mapping parameters.
    /// </summary>
    public class SpatialMappingParameters
    {
        /// <summary>
        /// Spatial mapping resolution in meters.
        /// </summary>
        public float resolutionMeter;
        /// <summary>
        /// Depth range in meters. 
        /// </summary>
        public float rangeMeter;
        /// <summary>
        /// Set to true if you want to be able to apply the texture to your mesh after its creation.
        /// </summary>
        public bool saveTexture = false;
        /// <summary>
        /// 
        /// </summary>
        public SPATIAL_MAP_TYPE map_type;
        /// <summary>
        /// Constructor
        /// </summary>
        public SpatialMappingParameters(float resolutionMeter = 0.05f, float rangeMeter = 0.0f, bool saveTexture = false, SPATIAL_MAP_TYPE map_type = SPATIAL_MAP_TYPE.MESH)
        {
            this.resolutionMeter = resolutionMeter;
            this.rangeMeter = rangeMeter;
            this.saveTexture = saveTexture;
            this.map_type = map_type;
        }
        /// <summary>
        /// Returns the resolution corresponding to the given MAPPING_RESOLUTION preset.
        /// </summary>
        /// <param name="mappingResolution">The desired MAPPING_RESOLUTION.</param>
        /// <returns>The resolution in meters.</returns>
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
        /// Returns the resolution corresponding to the given MAPPING_RANGE preset.
        /// </summary>
        /// <param name="mappingRange">The desired MAPPING_RANGE. Default: MAPPING_RANGE::MEDIUM.</param>
        /// <returns>The range in meters.</returns>
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
        /// Sets the resolution corresponding to the given MAPPING_RESOLUTION preset.
        /// </summary>
        /// <param name="mappingResolution">The desired MAPPING_RESOLUTION.</param>
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
        /// Sets the maximum value of the depth corresponding to the given MAPPING_RANGE preset.
        /// </summary>
        /// <param name="mappingRange">The desired MAPPING_RANGE. Default: MAPPING_RANGE::MEDIUM.</param>
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
    /// Spatial mapping depth resolution presets.
    /// </summary>
    public enum MAPPING_RESOLUTION
    {
        /// <summary>
        /// Create detailed geometry. Requires lots of memory.
        /// </summary>
        HIGH,
        /// <summary>
        /// Small variations in the geometry will disappear. Useful for large objects.
        /// </summary>
        ///
        MEDIUM,
        /// <summary>
        /// Keeps only large variations of the geometry. Useful for outdoors.
        /// </summary>
        LOW
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    ///  Spatial mapping depth range presets.
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
        /// Objects as far as 10 meters away are mapped. Useful for outdoors.
        /// </summary>
        FAR
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Spatial Mapping type (default is mesh)
    /// </summary>m
    public enum SPATIAL_MAP_TYPE
    {
        /// <summary>
        /// Represent a surface with faces, 3D points are linked by edges, no color information
        /// </summary>
        MESH,
        /// <summary>
        ///  Geometry is represented by a set of 3D colored points.
        /// </summary>
        FUSED_POINT_CLOUD
    };

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Mesh formats that can be saved/loaded with spatial mapping.
    /// </summary>
    public enum MESH_FILE_FORMAT
    {
        /// <summary>
        /// Contains only vertices and faces.
        /// </summary>
        PLY,
        /// <summary>
        /// Contains only vertices and faces, encoded in binary.
        /// </summary>
        BIN,
        /// <summary>
        /// Contains vertices, normals, faces, and texture information (if possible).
        /// </summary>
        OBJ
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Presets for filtering meshes scannedw ith spatial mapping. Higher values reduce total face count by more.
    /// </summary>
    public enum MESH_FILTER
    {
        /// <summary>
        /// Soft decimation and smoothing.
        /// </summary>
        LOW,
        /// <summary>
        /// Decimate the number of faces and apply a soft smooth.
        /// </summary>
        MEDIUM,
        /// <summary>
        /// Drastically reduce the number of faces.
        /// </summary>
        HIGH,
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Possible states of the ZED's Spatial Mapping system.
    /// </summary>
    public enum SPATIAL_MAPPING_STATE
    {
        /// <summary>
        /// Spatial mapping is initializing.
        /// </summary>
        INITIALIZING,
        /// <summary>
        /// Depth and tracking data were correctly integrated into the fusion algorithm.
        /// </summary>
        OK,
        /// <summary>
        /// Maximum memory dedicated to scanning has been reached; the mesh will no longer be updated.
        /// </summary>
        NOT_ENOUGH_MEMORY,
        /// <summary>
        /// EnableSpatialMapping() wasn't called (or the scanning was stopped and not relaunched).
        /// </summary>
        NOT_ENABLED,
        /// <summary>
        /// Effective FPS is too low to give proper results for spatial mapping.
        /// Consider using performance-friendly parameters (DEPTH_MODE_PERFORMANCE, VGA or HD720 camera resolution,
        /// and LOW spatial mapping resolution).
        /// </summary>
        FPS_TOO_LOW
    }

    ///\ingroup Core_group
    /// <summary>
    /// Units used by the SDK for measurements and tracking.
    /// </summary>
    public enum UNIT
    {
        /// <summary>
        /// International System, 1/1000 meters.
        /// </summary>
        MILLIMETER,
        /// <summary>
        /// International System, 1/100 meters.
        /// </summary>
        CENTIMETER,
        /// <summary>
        /// International System, 1/1 meters.
        /// </summary>
        METER,
        /// <summary>
        ///  Imperial Unit, 1/12 feet.
        /// </summary>
        INCH,
        /// <summary>
        ///  Imperial Unit, 1/1 feet.
        /// </summary>
        FOOT
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Type of the plane, determined by its orientation and whether detected by ZEDPlaneDetectionManager's
    /// DetectFloorPlane() or DetectPlaneAtHit().
    /// </summary>
    public enum PLANE_TYPE
    {
        /// <summary>
        /// Floor plane of a scene. Retrieved by ZEDPlaneDetectionManager.DetectFloorPlane().
        /// </summary>
        FLOOR,
        /// <summary>
        /// Horizontal plane, such as a tabletop, floor, etc. Detected with DetectPlaneAtHit() using screen-space coordinates. 
        /// </summary>
        HIT_HORIZONTAL,
        /// <summary>
        /// Vertical plane, such as a wall. Detected with DetectPlaneAtHit() using screen-space coordinates. 
        /// </summary>
        HIT_VERTICAL,
        /// <summary>
        /// Plane at an angle neither parallel nor perpendicular to the floor. Detected with DetectPlaneAtHit() using screen-space coordinates. 
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
    /// A mesh contains the geometric (and optionally texture) data of the scene captured by spatial mapping.
    /// </summary>
    public class Mesh
    {
        /// <summary>
        /// Total vertices in each chunk/submesh. 
        /// </summary>
        public int[] nbVerticesInSubmesh = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Total triangles in each chunk/submesh. 
        /// </summary>
        public int[] nbTrianglesInSubmesh = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Total indices per chunk/submesh. 
        /// </summary>
        public int[] updatedIndices = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Vertex count in current submesh. 
        /// </summary>
        public int nbVertices = 0;
        /// <summary>
        /// Triangle point counds in current submesh. (Every three values are the indexes of the three vertexes that make up one triangle)
        /// </summary>
        public int nbTriangles = 0;
        /// <summary>
        /// How many submeshes were updated. 
        /// </summary>
        public int nbUpdatedSubmesh = 0;
        /// <summary>
        /// Vertices of the mesh.
        /// </summary>
        public Vector3[] vertices = new Vector3[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// Contains the index of the vertices.
        /// </summary>
        public int[] triangles = new int[(int)Constant.MAX_SUBMESH];
        /// <summary>
        /// UVs defines the 2D projection of each vertices onto the Texture.
        /// Values are normalized [0;1], starting from the bottom left corner of the texture (as requested by opengl).
        /// In order to display a textured mesh you need to bind the Texture and then draw each triangles by picking its uv values.
        /// </summary>
        public Vector2[] uvs = null;
        /// <summary>
        /// Texture of the mesh
        /// </summary>
        public IntPtr textures = IntPtr.Zero;
        /// <summary>
        /// Width and height of the mesh texture, if any. 
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
        /// Vertices are defined by colored 3D points {x, y, z, rgba}.
        /// </summary>
        public Vector4[] vertices;
    }

    ///\ingroup SpatialMapping_group
    /// <summary>
    /// Represents a sub-mesh, it contains local vertices and triangles. 
    /// </summary>
    public struct Chunk
    {
        /// <summary>
        /// Vertices are defined by a 3D point {x,y,z}.
        /// </summary>
        public Vector3[] vertices;
        /// <summary>
        /// Triangles (or faces) contains the index of its three vertices. It corresponds to the 3 vertices of the triangle {v1, v2, v3}.
        /// </summary>
        public int[] triangles;
    }

    #endregion

    /// <summary>
    /// Structure that defines a new plane, holding information directly from the ZED SDK.
    /// Data within is relative to the camera;
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct PlaneData
    {
        /// <summary>
        /// Error code returned by the ZED SDK when the plane detection was attempted. 
        /// </summary>
        public sl.ERROR_CODE ErrorCode;
        /// <summary>
        /// Type of the plane (floor, hit_vertical, etc.) 
        /// </summary>
        public PLANE_TYPE Type;
        /// <summary>
        /// Normalized vector of the direction the plane is facing. 
        /// </summary>
        public Vector3 PlaneNormal;
        /// <summary>
        /// Camera-space position of the center of the plane. 
        /// </summary>
        public Vector3 PlaneCenter;
        /// <summary>
        /// Camera-space position of the center of the plane. 
        /// </summary>
        public Vector3 PlaneTransformPosition;
        /// <summary>
        /// Camera-space rotation/orientation of the plane. 
        /// </summary>
        public Quaternion PlaneTransformOrientation;
        /// <summary>
        /// The mathematical Vector4 equation of the plane. 
        /// </summary>
        public Vector4 PlaneEquation;
        /// <summary>
        /// How wide and long/tall the plane is in meters. 
        /// </summary>
        public Vector2 Extents;
        /// <summary>
        /// How many points make up the plane's bounds, eg. the array length of Bounds. 
        /// </summary>
        public int BoundsSize;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
        ///Positions of the points that make up the edges of the plane's mesh. 
        public Vector3[] Bounds; //max 256 points
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////  Object Detection /////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

#region Object Detection Module


    ///\ingroup Object_group
    /// <summary>
    /// Sets the object detection parameters.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObjectDetectionParameters
    {
        /// <summary>
        /// Defines if the object detection is synchronized to the image or runs in a separate thread.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool imageSync;
        /// <summary>
        /// Defines if the object detection will track objects across multiple images, instead of an image-by-image basis.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool enableObjectTracking;
        /// <summary>
        /// Defines if the SDK will calculate 2D masks for each object. Requires more performance, so don't enable if you don't need these masks.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool enable2DMask;
        /// <summary>
        /// Defines the AI model used for detection
        /// </summary>
        public sl.DETECTION_MODEL detectionModel;
        /// <summary>
        /// Defines if the body fitting will be applied.
        /// </summary>
        public bool enableBodyFitting;
        /// <summary>
        /// Defines a upper depth range for detections.
        /// Defined in  UNIT set at  sl.Camera.Open.
        /// Default value is set to sl.Initparameters.depthMaximumDistance (can not be higher).
        /// </summary>
        public float maxRange;
    };

    ///\ingroup Object_group
    /// <summary>
    /// Sets the object detection runtime parameters.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObjectDetectionRuntimeParameters
    {
        /// <summary>
        /// The detection confidence threshold between 1 and 99. 
        /// A confidence of 1 means a low threshold, more uncertain objects and 99 very few but very precise objects.
        /// Ex: If set to 80, then the SDK must be at least 80% sure that a given object exists before reporting it in the list of detected objects. 
        /// If the scene contains a lot of objects, increasing the confidence can slightly speed up the process, since every object instance is tracked.
        /// Default confidence threshold value, used as a fallback when ObjectDetectionRuntimeParameters.object_confidence_threshold is partially set
        /// </summary>
        public float detectionConfidenceThreshold;
        /// <summary>
        /// Select which object type to detect and track. Fewer objects type can slightly speed up the process, since every objects are tracked. Only the selected classes in the vector will be outputted.
        /// In order to get all the available classes, the filter vector must be empty :
        /// <c> object_class_filter = new int[(int)sl.OBJECT_CLASS.LAST)]; </c>
        /// To select a set of specific object classes, like person and vehicle for instance:
        /// objectClassFilter[(int)sl.OBJECT_CLASS.PERSON] = Convert.ToInt32(true);
        /// objectClassFilter[(int)sl.OBJECT_CLASS.VEHICLE] = Convert.ToInt32(true);
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)sl.OBJECT_CLASS.LAST)]
        public int[] objectClassFilter;

        /// <summary>
        /// Defines a detection threshold for each classes, can be empty for some classes, ObjectDetectionRuntimeParameters.detectionConfidenceThreshold will be taken as fallback/default value.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)sl.OBJECT_CLASS.LAST)]
        public int[] objectConfidenceThreshold;
    };

    ///\ingroup Object_group
	/// <summary>
	/// Object data structure directly from the SDK. Represents a single object detection.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct ObjectData
	{
        /// <summary>
        /// Object identification number, used as a reference when tracking the object through the frames.
        /// </summary>
        public int id; //person ID
        /// <summary>
        /// Object category. Identify the object type.
        /// </summary>
		public sl.OBJECT_CLASS label;
        /// <summary>
        /// Object subclass.
        /// </summary>
        public sl.OBJECT_SUBCLASS sublabel;
        /// <summary>
        /// Defines the object tracking state.
        /// </summary>
		public sl.OBJECT_TRACKING_STATE objectTrackingState;
        /// <summary>
        /// Defines the object action state.
        /// </summary>
		public sl.OBJECT_ACTION_STATE actionState;
        /// <summary>
        /// Defines the detection confidence value of the object. A lower confidence value means the object might not be localized perfectly or the label (OBJECT_CLASS) is uncertain.
        /// </summary>
		public float confidence;
        /// <summary>
        /// Defines for the bounding_box_2d the pixels which really belong to the object (set to 255) and those of the background (set to 0).
        /// </summary>
		public System.IntPtr mask;

		/// <summary>
		/// Image data.
		/// Note that Y in these values is relative from the top of the image.
        /// If using this raw value, subtract Y from the
		/// image height to get the height relative to the bottom.
		/// </summary>
		///  0 ------- 1
		///  |   obj   |
		///  3-------- 2
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public Vector2[] boundingBox2D;


        /// <summary>
        /// Defines the object 3D centroid.
        /// </summary>
        public Vector3 position; //object root position
        /// <summary>
        /// 3D head centroid.
        /// </summary>
		public Vector3 headPosition; //object head position (only for HUMAN detectionModel)
        /// <summary>
        /// Defines the object 3D velocity.
        /// </summary>
		public Vector3 velocity; //object root velocity

		/// <summary>
		/// The 3D space bounding box. given as array of vertices
		/// </summary>
		///   1 ---------2
		///  /|         /|
		/// 0 |--------3 |
		/// | |        | |
		/// | 5--------|-6
		/// |/         |/
		/// 4 ---------7
		///
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
		public Vector3[] boundingBox; // 3D Bounding Box of object
        /// <summary>
        /// bounds the head with eight 3D points.
        /// </summary>
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
		public Vector3[] headBoundingBox;// 3D Bounding Box of head (only for HUMAN detectionModel)

        /// <summary>
        /// A set of useful points representing the human body, expressed in 3D. We use a classic 18 points representation, the points semantic and order is given by BODY_PARTS.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 18)]
		public Vector3[] keypoints;// 3D position of the joints of the skeleton

        /// <summary>
        /// Full covariance matrix for position (3x3). Only 6 values are necessary
        /// [p0, p1, p2]
        /// [p1, p3, p4]
        /// [p2, p4, p5]
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
		public float[] positionCovariance;// covariance matrix of the 3d position, represented by its upper triangular matrix value

        /// <summary>
        ///  Per keypoint detection confidence, can not be lower than the \ref ObjectDetectionRuntimeParameters.detectionConfidenceThreshold.
        ///  Not available with DETECTION_MODEL.MULTI_CLASS_BOX.
        ///  in some cases, eg. body partially out of the image or missing depth data, some keypoint can not be detected, they will have non finite values.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 18)]
        public float[] keypointConfidence;
    };

    ///\ingroup Object_group
	/// <summary>
	/// Object Scene data directly from the ZED SDK. Represents all detections given during a single image frame.
	/// Contains the number of object in the scene and the objectData structure for each object.
	/// Since the data is transmitted from C++ to C#, the size of the structure must be constant. Therefore, there is a limitation of 200 (MAX_OBJECT constant) objects in the image.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct Objects
	{
		/// <summary>
		/// How many objects were detected this frame. Use this to iterate through the top of objectData; objects with indexes greater than numObject are empty.
		/// </summary>
		public int numObject;
		/// <summary>
		/// Timestamp of the image where these objects were detected.
		/// </summary>
		public ulong timestamp;
		/// <summary>
		/// Defines if the object frame is new (new timestamp)
		/// </summary>
		public int isNew;
		/// <summary>
		/// Defines if the object is tracked
		/// </summary>
		public int isTracked;
		/// <summary>
		/// Current detection model used.
		/// </summary>
		public sl.DETECTION_MODEL detectionModel;
		/// <summary>
		/// Array of objects
		/// </summary>
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)(Constant.MAX_OBJECTS))]
		public ObjectData[] objectData;
    };

    ///\ingroup Object_group
    /// <summary>
    /// Lists available object class
    /// </summary>
    public enum OBJECT_CLASS
    {
        PERSON = 0,
        VEHICLE = 1,
        BAG = 2,
        ANIMAL = 3,
        ELECTRONICS = 4,
        FRUIT_VEGETABLE = 5,
        LAST = 6
    };

    ///\ingroup Object_group
    /// <summary>
    /// Lists available object subclass.
    /// </summary>
    public enum OBJECT_SUBCLASS
    {
        PERSON = 0,
        // VEHICLES
        BICYCLE = 1,
        CAR = 2,
        MOTORBIKE = 3,
        BUS = 4,
        TRUCK = 5,
        BOAT = 6,
        // BAGS
        BACKPACK = 7,
        HANDBAG = 8,
        SUITCASE = 9,
        // ANIMALS
        BIRD = 10,
        CAT = 11,
        DOG = 12,
        HORSE = 13,
        SHEEP = 14,
        COW = 15,
        // ELECTRONICS
        CELLPHONE = 16,
        LAPTOP = 17,
        // FRUITS/VEGETABLES
        BANANA = 18,
        APPLE = 19,
        ORANGE = 20,
        CARROT = 21,
        LAST = 22
    };

    ///\ingroup Object_group
    /// <summary>
    /// Tracking state of an individual object.
    /// </summary>
    public enum OBJECT_TRACKING_STATE
	{
        /// <summary>
        /// The tracking is not yet initialized, the object ID is not usable.
        /// </summary>
		OFF,
        /// <summary>
        /// The object is tracked.
        /// </summary>
		OK,
        /// <summary>
        /// The object couldn't be detected in the image and is potentially occluded, the trajectory is estimated.
        /// </summary>
		SEARCHING,
        /// <summary>
        /// This is the last searching state of the track, the track will be deleted in the next retreiveObject.
        /// </summary>
        TERMINATE
    };

    ///\ingroup Object_group
    /// <summary>
    /// Lists available object action state.
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
		MOVING = 1,
		LAST = 2
	};

    ///\ingroup Object_group
	/// <summary>
	/// List available models for detection
	/// </summary>
	public enum DETECTION_MODEL {
        /// <summary>
        /// Any objects, bounding box based.
        /// </summary>
		MULTI_CLASS_BOX,
        /// <summary>
        /// Any objects, bounding box based.
        /// </summary>
        MULTI_CLASS_BOX_ACCURATE,
        /// <summary>
        /// Keypoints based, specific to human skeleton, real time performance even on Jetson or low end GPU cards.
        /// </summary>
        HUMAN_BODY_FAST,
        /// <summary>
        ///  Keypoints based, specific to human skeleton, state of the art accuracy, requires powerful GPU.
        /// </summary>
		HUMAN_BODY_ACCURATE
    };

    ///\ingroup Object_group
	/// <summary>
	/// semantic and order of human body keypoints.
	/// </summary>
	public enum BODY_PARTS {
		NOSE = 0,
		NECK = 1,
		RIGHT_SHOULDER = 2,
		RIGHT_ELBOW= 3,
		RIGHT_WRIST = 4,
		LEFT_SHOULDER = 5,
		LEFT_ELBOW = 6,
		LEFT_WRIST = 7,
		RIGHT_HIP = 8,
		RIGHT_KNEE = 9,
		RIGHT_ANKLE = 10,
		LEFT_HIP = 11,
		LEFT_KNEE = 12,
		LEFT_ANKLE = 13,
		RIGHT_EYE = 14,
		LEFT_EYE = 15,
		RIGHT_EAR = 16,
		LEFT_EAR = 17,
		LAST = 18
	};

#endregion

}// end namespace sl

