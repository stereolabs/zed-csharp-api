//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Reflection;
using System.Numerics;

namespace sl
{
    public class ZEDCamera
    {
        /// <summary>
        /// Type of textures requested.
        /// </summary>
        public enum TYPE_VIEW
        {
            /// <summary>
            /// Image-type texture. Human-viewable but loses measurement accuracy.
            /// </summary>
            RETRIEVE_IMAGE,
            /// <summary>
            /// Measure-type texture. Preserves measurement accuracy but isn't human-viewable.
            /// </summary>
            RETRIEVE_MEASURE
        }

        /********* Camera members ********/

        /// <summary>
        /// DLL name, used for extern calls to the wrapper.
        /// </summary>
        const string nameDll = sl.ZEDCommon.NameDLL;

        /// <summary>
        /// Width of the textures in pixels. Corresponds to the ZED's current resolution setting.
        /// </summary>
        private int imageWidth;
        /// <summary>
        /// Width of the images returned by the ZED in pixels. Corresponds to the ZED's current resolution setting.
        /// </summary>
        public int ImageWidth
        {
            get
            {
                return imageWidth;
            }
        }

        public const float Deg2Rad = 0.0174532924F;
        public const float Rad2Deg = 57.29578F;

        /// <summary>
        /// Height of the textures in pixels. Corresponds to the ZED's current resolution setting.
        /// </summary>
        private int imageHeight;
        /// <summary>
        /// Height of the images returned by the ZED in pixels. Corresponds to the ZED's current resolution setting.
        /// </summary>
        public int ImageHeight
        {
            get
            {
                return imageHeight;
            }
        }

        /// <summary>
        /// True if the ZED SDK is installed.
        /// </summary>
        private static bool pluginIsReady = true;

        /// <summary>
        /// Mutex for the image acquisition thread.
        /// </summary>
        public object grabLock = new object();

        /// <summary>
        /// Current ZED resolution setting. Set at initialization.
        /// </summary>
        private RESOLUTION currentResolution;
        
        /// <summary>
        /// Callback for c++ debugging. Should not be used in C#.
        /// </summary>
        private delegate void DebugCallback(string message);

        /// <summary>
        /// Desired FPS from the ZED camera. This is the maximum FPS for the ZED's current
        /// resolution unless a lower setting was specified in Init().
        /// Maximum values are bound by the ZED's output, not system performance.
        /// </summary>
        private uint fpsMax = 60; //Defaults to HD720 resolution's output.
        /// <summary>
        /// Desired FPS from the ZED camera. This is the maximum FPS for the ZED's current
        /// resolution unless a lower setting was specified in Init().
        /// Maximum values are bound by the ZED's output, not system performance.
        /// </summary>
        public float GetRequestedCameraFPS()
        {
            return fpsMax;
        }
        /// <summary>
        /// Holds camera settings like brightness/contrast, gain/exposure, etc.
        /// </summary>
        private ZEDCameraSettings cameraSettingsManager = new ZEDCameraSettings();

        /// <summary>
        /// Camera's stereo baseline (distance between the cameras). Extracted from calibration files.
        /// </summary>
        private float baseline = 0.0f;
        /// <summary>
        /// Camera's stereo baseline (distance between the cameras). Extracted from calibration files.
        /// </summary>
        public float Baseline
        {
            get { return baseline; }
        }
        /// <summary>
        /// ZED's current horizontal field of view in degrees.
        /// </summary>
        private float fov_H = 0.0f;
        /// <summary>
        /// ZED's current vertical field of view in degrees.
        /// </summary>
        private float fov_V = 0.0f;
        /// <summary>
        /// ZED's current horizontal field of view in degrees.
        /// </summary>
        public float HorizontalFieldOfView
        {
            get { return fov_H; }
        }
        /// <summary>
        /// ZED's current vertical field of view in degrees.
        /// </summary>
        public float VerticalFieldOfView
        {
            get { return fov_V; }
        }
        /// <summary>
        /// Structure containing information about all the sensors available in the current device
        /// </summary>
        private SensorsConfiguration sensorsConfiguration;
        /// <summary>
        /// Stereo parameters for current ZED camera prior to rectification (distorted).
        /// </summary>
        private CalibrationParameters calibrationParametersRaw;
        /// <summary>
        /// Stereo parameters for current ZED camera after rectification (undistorted).
        /// </summary>
        private CalibrationParameters calibrationParametersRectified;
        /// <summary>
        /// Camera model - ZED or ZED Mini.
        /// </summary>
        private sl.MODEL cameraModel;

        /// <summary>
        /// Whether the camera has been successfully initialized.
        /// </summary>
        private bool cameraReady = false;

        public SensorsConfiguration SensorsConfiguration
        {
            get { return sensorsConfiguration; }
        }
        /// <summary>
        /// Stereo parameters for current ZED camera prior to rectification (distorted).
        /// </summary>
        public CalibrationParameters CalibrationParametersRaw
        {
            get { return calibrationParametersRaw; }
        }
        /// <summary>
        /// Stereo parameters for current ZED camera after rectification (undistorted).
        /// </summary>
        public CalibrationParameters CalibrationParametersRectified
        {
            get { return calibrationParametersRectified; }
        }
        /// <summary>
        /// Camera model - ZED or ZED Mini.
        /// </summary>
        public sl.MODEL CameraModel
        {
            get { return cameraModel; }
        }
        /// <summary>
        /// Whether the camera has been successfully initialized.
        /// </summary>
        public bool IsCameraReady
        {
            get { return cameraReady; }
        }

        /// <summary>
        /// Camera ID (for multiple cameras)
        /// </summary>
        public int CameraID = 0;

        public const int brightnessDefault = 4;
        public const int contrastDefault = 4;
        public const int hueDefault = 0;
        public const int saturationDefault = 4;
        public const int sharpnessDefault = 3;
        public const int gammaDefault = 5;
        public const int whitebalanceDefault = 2600;


        #region DLL Calls

        /*
          * Utils function.
          */

        [DllImport(nameDll, EntryPoint = "dllz_unload_all_instances")]
        private static extern void dllz_unload_all_instances();

        [DllImport(nameDll, EntryPoint = "dllz_unload_instance")]
        private static extern void dllz_unload_instance(int id);

        [DllImport(nameDll, EntryPoint = "dllz_find_usb_device")]
        private static extern bool dllz_find_usb_device(USB_DEVICE dev);

        /*
          * Create functions
          */
        [DllImport(nameDll, EntryPoint = "dllz_create_camera")]
        private static extern bool dllz_create_camera(int cameraID, bool verbose);


        /*
        * Opening function (Opens camera and creates textures).
        */
        [DllImport(nameDll, EntryPoint = "dllz_open")]
        private static extern int dllz_open(int cameraID, ref dll_initParameters parameters, System.Text.StringBuilder svoPath, System.Text.StringBuilder ipStream, int portStream, System.Text.StringBuilder output, System.Text.StringBuilder opt_settings_path);

        /*
         * Close function.
         */
        [DllImport(nameDll, EntryPoint = "dllz_close")]
        private static extern void dllz_close(int cameraID);


        /*
         * Grab function.
         */
        [DllImport(nameDll, EntryPoint = "dllz_grab")]
        private static extern int dllz_grab(int cameraID, ref dll_RuntimeParameters runtimeParameters);

        /*
        * Recording functions.
        */
        [DllImport(nameDll, EntryPoint = "dllz_enable_recording")]
        private static extern int dllz_enable_recording(int cameraID, byte[] video_filename, int compresssionMode, int bitrate, int target_fps, bool transcode);

        [DllImport(nameDll, EntryPoint = "dllz_disable_recording")]
        private static extern bool dllz_disable_recording(int cameraID);


        /*
        * Texturing functions.
        */
       /* [DllImport(nameDll, EntryPoint = "dllz_retrieve_textures")]
        private static extern void dllz_retrieve_textures(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_updated_textures_timestamp")]
        private static extern ulong dllz_get_updated_textures_timestamp(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_swap_textures")]
        private static extern void dllz_swap_textures(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_register_texture_image_type")]
        private static extern int dllz_register_texture_image_type(int cameraID, int option, IntPtr id, Resolution resolution);

        [DllImport(nameDll, EntryPoint = "dllz_register_texture_measure_type")]
        private static extern int dllz_register_texture_measure_type(int cameraID, int option, IntPtr id, Resolution resolution);

        [DllImport(nameDll, EntryPoint = "dllz_unregister_texture_measure_type")]
        private static extern int dllz_unregister_texture_measure_type(int cameraID, int option);

        [DllImport(nameDll, EntryPoint = "dllz_unregister_texture_image_type")]
        private static extern int dllz_unregister_texture_image_type(int cameraID, int option);

        [DllImport(nameDll, EntryPoint = "dllz_get_copy_mat_texture_image_type")]
        private static extern IntPtr dllz_get_copy_mat_texture_image_type(int cameraID, int option);

        [DllImport(nameDll, EntryPoint = "dllz_get_copy_mat_texture_measure_type")]
        private static extern IntPtr dllz_get_copy_mat_texture_measure_type(int cameraID, int option);
        */

        /*
         * Camera control functions.
         */

        [DllImport(nameDll, EntryPoint = "dllz_set_video_settings")]
        private static extern void dllz_set_video_settings(int id, int mode, int value);

        [DllImport(nameDll, EntryPoint = "dllz_get_video_settings")]
        private static extern int dllz_get_video_settings(int id, int mode);

        [DllImport(nameDll, EntryPoint = "dllz_set_roi_for_aec_agc")]
        private static extern int dllz_set_roi_for_aec_agc(int id, int side, iRect roi,bool reset);

        [DllImport(nameDll, EntryPoint = "dllz_get_roi_for_aec_agc")]
        private static extern int dllz_get_roi_for_aec_agc(int id, int side, ref iRect roi);


        [DllImport(nameDll, EntryPoint = "dllz_get_input_type")]
        private static extern int dllz_get_input_type(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_set_camera_fps")]
        private static extern void dllz_set_camera_fps(int cameraID, int fps);

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_fps")]
        private static extern float dllz_get_camera_fps(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_width")]
        private static extern int dllz_get_width(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_height")]
        private static extern int dllz_get_height(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_calibration_parameters")]
        private static extern IntPtr dllz_get_calibration_parameters(int cameraID, bool raw);

        [DllImport(nameDll, EntryPoint = "dllz_get_sensors_configuration")]
        private static extern IntPtr dllz_get_sensors_configuration(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_model")]
        private static extern int dllz_get_camera_model(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_firmware")]
        private static extern int dllz_get_camera_firmware(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_sensors_firmware")]
        private static extern int dllz_get_sensors_firmware(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_zed_serial")]
        private static extern int dllz_get_zed_serial(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_imu_transform")]
        private static extern void dllz_get_camera_imu_transform(int cameraID, ulong timeStamp, bool useLatency, out Vector3 translation, out Quaternion rotation);

        [DllImport(nameDll, EntryPoint = "dllz_is_zed_connected")]
        private static extern int dllz_is_zed_connected();

        [DllImport(nameDll, EntryPoint = "dllz_get_camera_Timestamp")]
        private static extern ulong dllz_get_camera_timestamp(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_current_Timestamp")]
        private static extern ulong dllz_get_current_timestamp(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_image_updater_time_stamp")]
        private static extern ulong dllz_get_image_updater_time_stamp(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_frame_dropped_count")]
        private static extern uint dllz_get_frame_dropped_count(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_frame_dropped_percent")]
        private static extern float dllz_get_frame_dropped_percent(int cameraID);

        /*
         * SVO control functions.
         */

        [DllImport(nameDll, EntryPoint = "dllz_set_svo_position")]
        private static extern void dllz_set_svo_position(int cameraID, int frame);

        [DllImport(nameDll, EntryPoint = "dllz_get_svo_number_of_frames")]
        private static extern int dllz_get_svo_number_of_frames(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_svo_position")]
        private static extern int dllz_get_svo_position(int cameraID);


        /*
         * Depth Sensing utils functions.
         */
         /* Removed as of ZED SDK v3.0.
        [DllImport(nameDll, EntryPoint = "dllz_set_confidence_threshold")]
        private static extern void dllz_set_confidence_threshold(int cameraID, int threshold);
        [DllImport(nameDll, EntryPoint = "dllz_set_depth_max_range_value")]
        private static extern void dllz_set_depth_max_range_value(int cameraID, float distanceMax);
        */

        [DllImport(nameDll, EntryPoint = "dllz_get_confidence_threshold")]
        private static extern int dllz_get_confidence_threshold(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_depth_max_range_value")]
        private static extern float dllz_get_depth_max_range_value(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_depth_value")]
        private static extern float dllz_get_depth_value(int cameraID, uint x, uint y);

        [DllImport(nameDll, EntryPoint = "dllz_get_distance_value")]
        private static extern float dllz_get_distance_value(int cameraID, uint x, uint y);

        [DllImport(nameDll, EntryPoint = "dllz_get_normal_value")]
        private static extern int dllz_get_normal_value(int cameraID, uint x, uint y, out Vector4 value);

        [DllImport(nameDll, EntryPoint = "dllz_get_xyz_value")]
        private static extern int dllz_get_xyz_value(int cameraID, uint x, uint y, out Vector4 value);

        [DllImport(nameDll, EntryPoint = "dllz_get_depth_min_range_value")]
        private static extern float dllz_get_depth_min_range_value(int cameraID);


        /*
         * Motion Tracking functions.
         */
        [DllImport(nameDll, EntryPoint = "dllz_enable_tracking")]
        private static extern int dllz_enable_tracking(int cameraID, ref Quaternion quat, ref Vector3 vec, bool enableSpatialMemory = false, bool enablePoseSmoothing = false, bool enableFloorAlignment = false, 
            bool trackingIsStatic = false, bool enableIMUFusion = true, System.Text.StringBuilder aeraFilePath = null);

        [DllImport(nameDll, EntryPoint = "dllz_disable_tracking")]
        private static extern void dllz_disable_tracking(int cameraID, System.Text.StringBuilder path);

        [DllImport(nameDll, EntryPoint = "dllz_save_current_area")]
        private static extern int dllz_save_current_area(int cameraID, System.Text.StringBuilder path);

        [DllImport(nameDll, EntryPoint = "dllz_get_position_data")]
        private static extern int dllz_get_position_data(int cameraID, ref Pose pose, int reference_frame);

        [DllImport(nameDll, EntryPoint = "dllz_get_position")]
        private static extern int dllz_get_position(int cameraID, ref Quaternion quat, ref Vector3 vec, int reference_frame);

        [DllImport(nameDll, EntryPoint = "dllz_get_position_at_target_frame")]
        private static extern int dllz_get_position_at_target_frame(int cameraID, ref Quaternion quaternion, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation, int reference_frame);

        [DllImport(nameDll, EntryPoint = "dllz_transform_pose")]
        private static extern void dllz_transform_pose(ref Quaternion quaternion, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation);

        [DllImport(nameDll, EntryPoint = "dllz_reset_tracking")]
        private static extern int dllz_reset_tracking(int cameraID, Quaternion rotation, Vector3 translation);

        [DllImport(nameDll, EntryPoint = "dllz_reset_tracking_with_offset")]
        private static extern int dllz_reset_tracking_with_offset(int cameraID, Quaternion rotation, Vector3 translation, Quaternion offsetQuaternion, Vector3 offsetTranslation);

        [DllImport(nameDll, EntryPoint = "dllz_estimate_initial_position")]
        private static extern int dllz_estimate_initial_position(int cameraID, ref Quaternion quaternion, ref Vector3 translation, int countSuccess, int countTimeout);

        [DllImport(nameDll, EntryPoint = "dllz_set_imu_prior_orientation")]
        private static extern int dllz_set_imu_prior_orientation(int cameraID, Quaternion rotation);

        [DllImport(nameDll, EntryPoint = "dllz_get_internal_imu_orientation")]
        private static extern int dllz_get_internal_imu_orientation(int cameraID, ref Quaternion rotation, int reference_time);

        [DllImport(nameDll, EntryPoint = "dllz_get_internal_sensors_data")]
        private static extern int dllz_get_internal_sensors_data(int cameraID, ref SensorsData imuData, int reference_time);

        [DllImport(nameDll, EntryPoint = "dllz_get_area_export_state")]
        private static extern int dllz_get_area_export_state(int cameraID);

        /*
        * Spatial Mapping functions.
        */
        [DllImport(nameDll, EntryPoint = "dllz_enable_spatial_mapping")]
        private static extern int dllz_enable_spatial_mapping(int cameraID, int type, float resolution_meter, float max_range_meter, int saveTexture, int max_memory_usage);

        [DllImport(nameDll, EntryPoint = "dllz_disable_spatial_mapping")]
        private static extern void dllz_disable_spatial_mapping(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_pause_spatial_mapping")]
        private static extern void dllz_pause_spatial_mapping(int cameraID, bool status);

        [DllImport(nameDll, EntryPoint = "dllz_request_mesh_async")]
        private static extern void dllz_request_mesh_async(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_mesh_request_status_async")]
        private static extern int dllz_get_mesh_request_status_async(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_update_mesh")]
        private static extern int dllz_update_mesh(int cameraID, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "dllz_retrieve_mesh")]
        private static extern int dllz_retrieve_mesh(int cameraID, Vector3[] vertices, int[] triangles, int nbSubmesh, Vector2[] uvs, IntPtr textures);
        [DllImport(nameDll, EntryPoint = "dllz_extract_whole_spatial_map")]
        private static extern int dllz_extract_whole_spatial_map(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_save_mesh")]
        private static extern bool dllz_save_mesh(int cameraID, string filename, MESH_FILE_FORMAT format);

        [DllImport(nameDll, EntryPoint = "dllz_load_mesh")]
        private static extern bool dllz_load_mesh(int cameraID, string filename, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbMaxSubmesh, int[] textureSize = null);

        [DllImport(nameDll, EntryPoint = "dllz_apply_texture")]
        private static extern bool dllz_apply_texture(int cameraID, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int[] textureSize, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "dllz_filter_mesh")]
        private static extern bool dllz_filter_mesh(int cameraID, FILTER meshFilter, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "dllz_get_spatial_mapping_state")]
        private static extern int dllz_get_spatial_mapping_state(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_spatial_mapping_merge_chunks")]
        private static extern void dllz_spatial_mapping_merge_chunks(int cameraID, int numberFaces, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "dllz_spatial_mapping_get_gravity_estimation")]
        private static extern void dllz_spatial_mapping_get_gravity_estimation(int cameraID, ref Vector3 v);

        /*
         * Plane Detection functions (starting v2.4)
         */
        [DllImport(nameDll, EntryPoint = "dllz_find_floor_plane")]
        private static extern IntPtr dllz_find_floor_plane(int cameraID, out Quaternion rotation, out Vector3 translation, Quaternion priorQuaternion, Vector3 priorTranslation);

        [DllImport(nameDll, EntryPoint = "dllz_find_plane_at_hit")]
        private static extern IntPtr dllz_find_plane_at_hit(int cameraID, Vector2 HitPixel, bool refine);

        [DllImport(nameDll, EntryPoint = "dllz_convert_floorplane_to_mesh")]
        private static extern int dllz_convert_floorplane_to_mesh(int cameraID, Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles);

        [DllImport(nameDll, EntryPoint = "dllz_convert_hitplane_to_mesh")]
        private static extern int dllz_convert_hitplane_to_mesh(int cameraID, Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles);


        /*
         * Streaming Module functions (starting v2.8)
         */
        [DllImport(nameDll, EntryPoint = "dllz_enable_streaming")]
        private static extern int dllz_enable_streaming(int cameraID, sl.STREAMING_CODEC codec, uint bitrate, ushort port, int gopSize, int adaptativeBitrate, int chunk_size, int target_fps);

        [DllImport(nameDll, EntryPoint = "dllz_is_streaming_enabled")]
        private static extern int dllz_is_streaming_enabled(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_disable_streaming")]
        private static extern void dllz_disable_streaming(int cameraID);


        /*
        * Objects Detection functions (starting v3.0)
        */
        [DllImport(nameDll, EntryPoint = "dllz_enable_objects_detection")]
        private static extern int dllz_enable_objects_detection(int cameraID, ref ObjectDetectionParameters od_params);

        [DllImport(nameDll, EntryPoint = "dllz_disable_objects_detection")]
        private static extern void dllz_disable_objects_detection(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_pause_objects_detection")]
        private static extern void dllz_pause_objects_detection(int cameraID, bool status);

        [DllImport(nameDll, EntryPoint = "dllz_retrieve_objects_data")]
        private static extern int dllz_retrieve_objects_data(int cameraID, ref ObjectDetectionRuntimeParameters od_params, ref ObjectsFrameSDK objFrame);


        /*
        * Save utils function
        */
        [DllImport(nameDll, EntryPoint = "dllz_save_current_image")]
        private static extern int dllz_save_current_image(int cameraID, VIEW view,string filename);

        [DllImport(nameDll, EntryPoint = "dllz_save_current_depth")]
        private static extern int dllz_save_current_depth(int cameraID, int side,string filename);

        [DllImport(nameDll, EntryPoint = "dllz_save_current_point_cloud")]
        private static extern int dllz_save_current_point_cloud(int cameraID, int side,  string filename);

        /*
         * Specific plugin functions
         */
        [DllImport(nameDll, EntryPoint = "dllz_check_plugin")]
        private static extern int dllz_check_plugin(int major, int minor);

        [DllImport(nameDll, EntryPoint = "dllz_get_sdk_version")]
        private static extern IntPtr dllz_get_sdk_version();

        [DllImport(nameDll, EntryPoint = "dllz_compute_offset")]
        private static extern void dllz_compute_offset(float[] A, float[] B, int nbVectors, float[] C);

        [DllImport(nameDll, EntryPoint = "dllz_compute_optical_center_offsets")]
        private static extern System.IntPtr dllz_compute_optical_center_offsets(ref Vector4 calibLeft, ref Vector4 calibRight, sl.Resolution imageResolution, float planeDistance);


        /*
         * Retreieves used by mat
         */
        [DllImport(nameDll, EntryPoint = "dllz_retrieve_measure")]
        private static extern int dllz_retrieve_measure(int cameraID, System.IntPtr ptr, int type, int mem, sl.Resolution resolution);

        [DllImport(nameDll, EntryPoint = "dllz_retrieve_image")]
        private static extern int dllz_retrieve_image(int cameraID, System.IntPtr ptr, int type, int mem, sl.Resolution resolution);

        #endregion

        public static void UnloadPlugin()
        {
            dllz_unload_all_instances();
        }

        public static void UnloadInstance(int id)
        {
            dllz_unload_instance(id);
        }


        /// <summary>
        /// Return a string from a pointer to a char. Used in GetSDKVersion().
        /// </summary>
        /// <param name="ptr">Pointer to a char.</param>
        /// <returns>The char as a string.</returns>
        private static string PtrToStringUtf8(IntPtr ptr)
        {
            if (ptr == IntPtr.Zero)
            {
                return "";
            }
            int len = 0;
            while (Marshal.ReadByte(ptr, len) != 0)
                len++;
            if (len == 0)
            {
                return "";
            }
            byte[] array = new byte[len];
            Marshal.Copy(ptr, array, 0, len);
            return System.Text.Encoding.ASCII.GetString(array);
        }

        /// <summary>
        /// Convert a pointer to a char into an array of bytes. Used to send file names to SDK for SVO recording.
        /// </summary>
        /// <param name="ptr">Pointer to a char.</param>
        /// <returns>The array.</returns>
        private static byte[] StringUtf8ToByte(string str)
        {
            byte[] array = System.Text.Encoding.ASCII.GetBytes(str);
            return array;
        }

        /// <summary>
        /// Gets the max FPS for each resolution setting. Higher FPS will cause lower GPU performance.
        /// </summary>
        /// <param name="reso"></param>
        /// <returns>The resolution</returns>
        static private uint GetFpsForResolution(RESOLUTION reso)
        {
            if (reso == RESOLUTION.HD1080) return 30;
            else if (reso == RESOLUTION.HD2K) return 15;
            else if (reso == RESOLUTION.HD720) return 60;
            else if (reso == RESOLUTION.VGA) return 100;
            return 30;
        }

        /// <summary>
        /// Performs a rigid transform.
        /// </summary>
        /// <param name="quaternion"></param>
        /// <param name="translation"></param>
        /// <param name="targetQuaternion"></param>
        /// <param name="targetTranslation"></param>
        public static void TransformPose(ref Quaternion quaternion, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation)
        {
            dllz_transform_pose(ref quaternion, ref translation, ref targetQuaternion, ref targetTranslation);
        }

        /// <summary>
        /// Closes the camera.
        /// Once destroyed, you need to recreate a camera instance to restart again.
        /// </summary>
        public void Close()
        {
            cameraReady = false;
            dllz_close(CameraID);
        }

        /// <summary>
        /// DLL-friendly version of InitParameters (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct dll_initParameters
        {
            public sl.INPUT_TYPE inputType;
            /// <summary>
            /// Resolution the ZED will be set to.
            /// </summary>
            public sl.RESOLUTION resolution;
            /// <summary>
            /// Desired camera FPS. Max is set by resolution.
            /// </summary>
            public int cameraFps;
            /// <summary>
            /// ID for identifying which of multiple connected ZEDs to use.
            /// </summary>
            public int cameraDeviceID;
            /// <summary>
            /// True to flip images horizontally.
            /// </summary>
            public int cameraImageFlip;
            /// <summary>
            /// True to disable self-calibration, using unoptimized optional calibration parameters.
            /// False is recommended for optimized calibration.
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool cameraDisableSelfCalib;
            /// <summary>
            /// True if depth relative to the right sensor should be computed.
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool enableRightSideMeasure;
            /// <summary>
            /// True to skip dropped frames during SVO playback.
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool svoRealTimeMode;
            /// <summary>
            /// Quality level of depth calculations. Higher settings improve accuracy but cost performance.
            /// </summary>
            public sl.DEPTH_MODE depthMode;
            /// <summary>
            /// True to stabilize the depth map. Recommended.
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool depthStabilization;
            /// <summary>
            /// Minimum distance from the camera from which depth will be computed, in the defined coordinateUnit.
            /// </summary>
            public float depthMinimumDistance;
            /// <summary>
            /// Maximum distance that can be computed.
            /// </summary>
            public float depthMaximumDistance;
            /// <summary>
            /// Coordinate unit for all measurements (depth, tracking, etc.).
            /// </summary>
            public UNIT coordinateUnit;
            /// <summary>
            /// Defines order and direction of coordinate system axes.
            /// </summary>
            public COORDINATE_SYSTEM coordinateSystem;
            /// <summary>
            /// ID of the graphics card on which the ZED's computations will be performed.
            /// </summary>
            public int sdkGPUId;
            /// <summary>
            /// True for the SDK to provide text feedback.
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool sdkVerbose;
            /// <summary>
            /// True if sensors are required, false will not trigger an error if sensors are missing.
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool sensorsRequired;
            /// <summary>
            /// Whether to enable improved color/gamma curves added in ZED SDK 3.0. 
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool enableImageEnhancement;

            /// <summary>
            /// Copy constructor.
            /// </summary>
            /// <param name="init"></param>
            public dll_initParameters(InitParameters init)
            {
                inputType = init.inputType;
                resolution = init.resolution;
                cameraFps = init.cameraFPS;
                svoRealTimeMode = init.svoRealTimeMode;
                coordinateUnit = init.coordinateUnit;
                depthMode = init.depthMode;
                depthMinimumDistance = init.depthMinimumDistance;
                depthMaximumDistance = init.depthMaximumDistance;
                cameraImageFlip = (int)init.cameraImageFlip;
                enableRightSideMeasure = init.enableRightSideMeasure;
                cameraDisableSelfCalib = init.cameraDisableSelfCalib;
                sdkVerbose = init.sdkVerbose;
                sdkGPUId = init.sdkGPUId;
                cameraDeviceID = init.cameraDeviceID;
                coordinateSystem = init.coordinateSystem;
                depthStabilization = init.depthStabilization;
                sensorsRequired = init.sensorsRequired;
                enableImageEnhancement = init.enableImageEnhancement;
            }
        }

        /// <summary>
        /// DLL-friendly version of RuntimeParameters (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct dll_RuntimeParameters
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
            [MarshalAs(UnmanagedType.U1)]
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
            /// Copy constructor.
            /// </summary>
            public dll_RuntimeParameters(RuntimeParameters rt)
            {
                sensingMode = rt.sensingMode;
                measure3DReferenceFrame = rt.measure3DReferenceFrame;
                enableDepth = rt.enableDepth;
                confidenceThreshold = rt.confidenceThreshold;
                textureConfidenceThreshold = rt.textureConfidenceThreshold;
            }
        }


        public ZEDCamera(int id)
        {
            CameraID = id;
        }
        /// <summary>
        /// Checks if the ZED camera is plugged in and  opens it.
        /// </summary>
        /// <param name="initParameters">Class with all initialization settings.
        /// A newly-instantiated InitParameters will have recommended default values.</param>
        /// <returns>ERROR_CODE: The error code gives information about the internal connection process.
        /// If SUCCESS is returned, the camera is ready to use. Every other code indicates an error.</returns>
        public ERROR_CODE Init(ref InitParameters initParameters)
        {
            //Update values with what we're about to pass to the camera.
            currentResolution = initParameters.resolution;
            fpsMax = GetFpsForResolution(currentResolution);
            if (initParameters.cameraFPS == 0)
            {
                initParameters.cameraFPS = (int)fpsMax;
            }

            dll_initParameters initP = new dll_initParameters(initParameters); //DLL-friendly version of InitParameters.
            initP.coordinateSystem = initParameters.coordinateSystem; //Left-hand
            int v = dllz_open(CameraID, ref initP,
                new System.Text.StringBuilder(initParameters.pathSVO, initParameters.pathSVO.Length),
                new System.Text.StringBuilder(initParameters.ipStream, initParameters.ipStream.Length),
                initParameters.portStream,
                new System.Text.StringBuilder(initParameters.sdkVerboseLogFile, initParameters.sdkVerboseLogFile.Length),
                new System.Text.StringBuilder(initParameters.optionalSettingsPath, initParameters.optionalSettingsPath.Length));

            if ((ERROR_CODE)v != ERROR_CODE.SUCCESS)
            {
                cameraReady = false;
                return (ERROR_CODE)v;
            }

            //Set more values if the initialization was successful.
            imageWidth = dllz_get_width(CameraID);
            imageHeight = dllz_get_height(CameraID);

            if (imageWidth > 0 && imageHeight > 0)
            {
                GetCalibrationParameters(false);
                cameraModel = GetCameraModel();
                cameraReady = true;
                return (ERROR_CODE)v;
            }
            else
                return sl.ERROR_CODE.CAMERA_NOT_INITIALIZED;
        }

        /// <summary>
        /// Grabs a new image, rectifies it, and computes the disparity map and (optionally) the depth map.
        /// The grabbing function is typically called in the main loop in a separate thread.
        /// </summary><remarks>For more info, read about the SDK function it calls:
        /// https://www.stereolabs.com/developers/documentation/API/v2.5.1/classsl_1_1Camera.html#afa3678a18dd574e162977e97d7cbf67b </remarks>
        /// <param name="runtimeParameters">Struct holding all grab parameters. </param>
        /// <returns>the function returns false if no problem was encountered,
        /// true otherwise.</returns>
        public sl.ERROR_CODE Grab(ref sl.RuntimeParameters runtimeParameters)
        {
            dll_RuntimeParameters rt_params = new dll_RuntimeParameters(runtimeParameters);
            return (sl.ERROR_CODE)dllz_grab(CameraID, ref rt_params);
        }

        /// <summary>
        /// Return the INPUT_TYPE currently used
        /// </summary>
        /// <returns></returns>
        public sl.INPUT_TYPE GetInputType()
        {
            return (sl.INPUT_TYPE)dllz_get_input_type(CameraID);
        }

        /// <summary>
        /// Creates a file for recording the ZED's output into a .SVO or .AVI video.
        /// </summary><remarks>An SVO is Stereolabs' own format designed for the ZED. It holds the video feed with timestamps
        /// as well as info about the camera used to record it.</remarks>
        /// <param name="videoFileName">Filename. Whether it ends with .svo or .avi defines its file type.</param>
        /// <param name="compressionMode">How much compression to use</param>
        /// <param name="bitrate">Override default bitrate with a custom bitrate (Kbits/s)</param>
        /// <param name="target_fps">Use another fps than camera fps. Must respect camera_fps%target_fps == 0</param>
        /// <param name="transcode">If input is in streaming mode, dump directly into SVO file (transcode=false) or decode/encode (transcode=true)</param>
        /// <returns>An ERROR_CODE that defines if the file was successfully created and can be filled with images.</returns>
        public ERROR_CODE EnableRecording(string videoFileName, SVO_COMPRESSION_MODE compressionMode = SVO_COMPRESSION_MODE.H264_BASED, int bitrate = 0, int target_fps = 0, bool transcode = false)
        {
            return (ERROR_CODE)dllz_enable_recording(CameraID, StringUtf8ToByte(videoFileName), (int)compressionMode, bitrate, target_fps, transcode);
        }

        /// <summary>
        /// Stops recording to an SVO/AVI, if applicable, and closes the file.
        /// </summary>
        public bool DisableRecording()
        {
            return dllz_disable_recording(CameraID);
        }

        /// <summary>
        /// Sets a new target frame rate for the camera. If it's not possible with the current resolution,
        /// the SDK will target the closest possible frame rate instead.
        /// </summary>
        /// <param name="fps">New target FPS.</param>
        public void SetCameraFPS(int fps)
        {
            if (GetFpsForResolution(currentResolution) >= fps)
            {
                fpsMax = (uint)fps;
            }

            dllz_set_camera_fps(CameraID, fps);
        }

        /// <summary>
        /// Sets the position of the SVO file currently being read to a desired frame.
        /// </summary>
        /// <param name="frame">Index of the desired frame to be decoded.</param>
        public void SetSVOPosition(int frame)
        {
            dllz_set_svo_position(CameraID, frame);
        }

        /// <summary>
        /// Gets the current confidence threshold value for the disparity map (and by extension the depth map).
        /// Values below the given threshold are removed from the depth map.
        /// </summary>
        /// <returns>Filtering value between 0 and 100.</returns>
        public int GetConfidenceThreshold()
        {
            return dllz_get_confidence_threshold(CameraID);
        }

        /// <summary>
        /// Gets the timestamp at the time the latest grabbed frame was extracted from the USB stream.
        /// This is the closest timestamp you can get from when the image was taken. Must be called after calling grab().
        /// </summary>
        /// <returns>Current timestamp in nanoseconds. -1 means it's is not available, such as with an .SVO file without compression.</returns>
        public ulong GetCameraTimeStamp()
        {
            return dllz_get_camera_timestamp(CameraID);
        }

        /// <summary>
        /// Gets the current timestamp at the time the function is called. Can be compared to the camera timestamp
        /// for synchronization, since they have the same reference (the computer's start time).
        /// </summary>
        /// <returns>The timestamp in nanoseconds.</returns>
        public ulong GetCurrentTimeStamp()
        {
            return dllz_get_current_timestamp(CameraID);
        }

        /// <summary>
        /// Timestamp from the most recent image update. Based on the computer's start time.
        /// </summary>
        /// <returns>The timestamp in nanoseconds.</returns>
        public ulong GetImageUpdaterTimeStamp()
        {
            return dllz_get_image_updater_time_stamp(CameraID);
        }

        /// <summary>
        /// Get the current position of the SVO being recorded to.
        /// </summary>
        /// <returns>Index of the frame being recorded to.</returns>
        public int GetSVOPosition()
        {
            return dllz_get_svo_position(CameraID);
        }

        /// <summary>
        /// Gets the total number of frames in the loaded SVO file.
        /// </summary>
        /// <returns>Total frames in the SVO file. Returns -1 if the SDK is not reading an SVO.</returns>
        public int GetSVONumberOfFrames()
        {
            return dllz_get_svo_number_of_frames(CameraID);
        }

        /// <summary>
        /// Gets the closest measurable distance by the camera, according to the camera type and depth map parameters.
        /// </summary>
        /// <returns>The nearest possible depth value.</returns>
        public float GetDepthMinRangeValue()
        {
            return dllz_get_depth_min_range_value(CameraID);
        }

        /// <summary>
        /// Returns the current maximum distance of depth/disparity estimation.
        /// </summary>
        /// <returns>The closest depth</returns>
        public float GetDepthMaxRangeValue()
        {
            return dllz_get_depth_max_range_value(CameraID);
        }

        /// <summary>
        /// Initialize and Start the tracking functions
        /// </summary>
        /// <param name="quat"> rotation used as initial world transform. By default it should be identity.</param>
        /// <param name="vec"> translation used as initial world transform. By default it should be identity.</param>
        /// <param name="enableSpatialMemory">  (optional) define if spatial memory is enable or not.</param>
        /// <param name="areaFilePath"> (optional) file of spatial memory file that has to be loaded to relocate in the scene.</param>
        /// <returns></returns>
        public sl.ERROR_CODE EnableTracking(ref Quaternion quat, ref Vector3 vec, bool enableSpatialMemory = true, bool enablePoseSmoothing = false, bool enableFloorAlignment = false, bool trackingIsStatic = false,
            bool enableIMUFusion = true, string areaFilePath = "")
        {
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            trackingStatus = (sl.ERROR_CODE)dllz_enable_tracking(CameraID, ref quat, ref vec, enableSpatialMemory, enablePoseSmoothing, enableFloorAlignment,
                trackingIsStatic, enableIMUFusion, new System.Text.StringBuilder(areaFilePath, areaFilePath.Length));
            return trackingStatus;
        }

        /// <summary>
        ///  Stop the motion tracking, if you want to restart, call enableTracking().
        /// </summary>
        /// <param name="path">The path to save the area file</param>
        public void DisableTracking(string path = "")
        {
            dllz_disable_tracking(CameraID, new System.Text.StringBuilder(path, path.Length));
        }

        /// <summary>
        /// Reset tracking
        /// </summary>
        /// <param name="rotation"></param>
        /// <param name="translation"></param>
        /// <returns></returns>
        public sl.ERROR_CODE ResetTracking(Quaternion rotation, Vector3 translation)
        {
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            trackingStatus = (sl.ERROR_CODE)dllz_reset_tracking(CameraID, rotation, translation);
            return trackingStatus;
        }


        /// <summary>
        /// Returns the current camera FPS. This is limited primarily by resolution but can also be lower due to
        /// setting a lower desired resolution in Init() or from USB connection/bandwidth issues.
        /// </summary>
        /// <returns>The current fps</returns>
        public float GetCameraFPS()
        {
            return dllz_get_camera_fps(CameraID);
        }

        public CalibrationParameters GetCalibrationParameters(bool raw = false)
        {

            IntPtr p = dllz_get_calibration_parameters(CameraID, raw);

            if (p == IntPtr.Zero)
            {
                return new CalibrationParameters();
            }
            CalibrationParameters parameters = (CalibrationParameters)Marshal.PtrToStructure(p, typeof(CalibrationParameters));

            if (raw)
                calibrationParametersRaw = parameters;
            else
                calibrationParametersRectified = parameters;


            return parameters;

        }

        public SensorsConfiguration GetInternalSensorsConfiguration()
        {
            IntPtr p = dllz_get_sensors_configuration(CameraID);

            if (p == IntPtr.Zero)
            {
                return new SensorsConfiguration();
            }
            SensorsConfiguration configuration = (SensorsConfiguration)Marshal.PtrToStructure(p, typeof(SensorsConfiguration));

            return configuration;
        }

        /// <summary>
        /// Gets the ZED camera model (ZED or ZED Mini).
        /// </summary>
        /// <returns>Model of the ZED as sl.MODEL.</returns>
        public sl.MODEL GetCameraModel()
        {
            return (sl.MODEL)dllz_get_camera_model(CameraID);
        }

        /// <summary>
        /// Gets the ZED's camera firmware version.
        /// </summary>
        /// <returns>Firmware version.</returns>
        public int GetCameraFirmwareVersion()
        {
            return dllz_get_camera_firmware(CameraID);
        }

        /// <summary>
        /// Gets the ZED's sensors firmware version.
        /// </summary>
        /// <returns>Firmware version.</returns>
        public int GetSensorsFirmwareVersion()
        {
            return dllz_get_sensors_firmware(CameraID);
        }

        /// <summary>
        /// Gets the ZED's serial number.
        /// </summary>
        /// <returns>Serial number</returns>
        public int GetZEDSerialNumber()
        {
            return dllz_get_zed_serial(CameraID);
        }

        /// <summary>
        /// Returns the ZED's vertical field of view in radians.
        /// </summary>
        /// <returns>Vertical field of view.</returns>
        public float GetFOV()
        {
            return GetCalibrationParameters(false).leftCam.vFOV * Deg2Rad;
        }

        /*
        /// <summary>
        /// Computes textures from the ZED. The new textures will not be displayed until an event is sent to the render thread.
        /// This event is called from UpdateTextures().
        /// </summary>
        public void RetrieveTextures()
        {
            dllz_retrieve_textures(CameraID);
        }

        /// <summary>
        /// Swaps textures safely between the acquisition and rendering threads.
        /// </summary>
        public void SwapTextures()
        {
            dllz_swap_textures(CameraID);
        }

        /// <summary>
        /// Timestamp of the images used the last time the ZED wrapper updated textures.
        /// </summary>
        /// <returns></returns>
        public ulong GetImagesTimeStamp()
        {
            return dllz_get_updated_textures_timestamp(CameraID);
        }
        */
        /// <summary>
        /// Gets the number of frames dropped since Grab() was called for the first time.
        /// Based on camera timestamps and an FPS comparison.
        /// </summary><remarks>Similar to the Frame Drop display in the ZED Explorer app.</remarks>
        /// <returns>Frames dropped since first Grab() call.</returns>
        public uint GetFrameDroppedCount()
        {
            return dllz_get_frame_dropped_count(CameraID);
        }

        /// <summary>
        /// Gets the percentage of frames dropped since Grab() was called for the first time.
        /// </summary>
        /// <returns>Percentage of frames dropped.</returns>
        public float GetFrameDroppedPercent()
        {
            return dllz_get_frame_dropped_percent(CameraID);
        }

        /// <summary>
        /// Gets the position of the camera and the current state of the ZED Tracking.
        /// </summary>
        /// <param name="rotation">Quaternion filled with the current rotation of the camera depending on its reference frame.</param>
        /// <param name="position">Vector filled with the current position of the camera depending on its reference frame.</param>
        /// <param name="referenceType">Reference frame for setting the rotation/position. CAMERA gives movement relative to the last pose.
        /// WORLD gives cumulative movements since tracking started.</param>
        /// <returns>State of ZED's Tracking system (off, searching, ok).</returns>
        public TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 position, REFERENCE_FRAME referenceType = REFERENCE_FRAME.WORLD)
        {
            return (TRACKING_STATE)dllz_get_position(CameraID, ref rotation, ref position, (int)referenceType);
        }

        /// <summary>
        /// Gets the current position of the camera and state of the tracking, with an optional offset to the tracking frame.
        /// </summary>
        /// <param name="rotation">Quaternion filled with the current rotation of the camera depending on its reference frame.</param>
        /// <param name="position">Vector filled with the current position of the camera depending on its reference frame.</param>
        /// <param name="targetQuaternion">Rotational offset applied to the tracking frame.</param>
        /// <param name="targetTranslation">Positional offset applied to the tracking frame.</param>
        /// <param name="referenceFrame">Reference frame for setting the rotation/position. CAMERA gives movement relative to the last pose.
        /// WORLD gives cumulative movements since tracking started.</param>
        /// <returns>State of ZED's Tracking system (off, searching, ok).</returns>
        public TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation, REFERENCE_FRAME referenceFrame = REFERENCE_FRAME.WORLD)
        {
            return (TRACKING_STATE)dllz_get_position_at_target_frame(CameraID, ref rotation, ref translation, ref targetQuaternion, ref targetTranslation, (int)referenceFrame);
        }


        /// <summary>
        /// Gets the current position of the camera and state of the tracking, with a defined tracking frame.
        /// A tracking frame defines what part of the ZED is its center for tracking purposes. See ZEDCommon.TRACKING_FRAME.
        /// </summary>
        /// <param name="rotation">Quaternion filled with the current rotation of the camera depending on its reference frame.</param>
        /// <param name="position">Vector filled with the current position of the camera depending on its reference frame.</param>
        /// <param name="trackingFrame">Center of the ZED for tracking purposes (left eye, center, right eye).</param>
        /// <param name="referenceFrame">Reference frame for setting the rotation/position. CAMERA gives movement relative to the last pose.
        /// WORLD gives cumulative movements since tracking started.</param>
        /// <returns>State of ZED's Tracking system (off, searching, ok).</returns>
        public TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 translation, TRACKING_FRAME trackingFrame, REFERENCE_FRAME referenceFrame = REFERENCE_FRAME.WORLD)
        {
            Quaternion rotationOffset = Quaternion.Identity;
            Vector3 positionOffset = Vector3.Zero;
            switch (trackingFrame) //Add offsets to account for different tracking frames.
            {
                case sl.TRACKING_FRAME.LEFT_EYE:
                    positionOffset = new Vector3(0, 0, 0);
                    break;
                case sl.TRACKING_FRAME.RIGHT_EYE:
                    positionOffset = new Vector3(Baseline, 0, 0);
                    break;
                case sl.TRACKING_FRAME.CENTER_EYE:
                    positionOffset = new Vector3(Baseline / 2.0f, 0, 0);
                    break;
            }

            return (TRACKING_STATE)dllz_get_position_at_target_frame(CameraID, ref rotation, ref translation, ref rotationOffset, ref positionOffset, (int)referenceFrame);
        }

        /// <summary>
        /// Gets the current position of the camera and state of the tracking, filling a Pose struct useful for AR pass-through.
        /// </summary>
        /// <param name="pose">Current pose.</param>
        /// <param name="referenceType">Reference frame for setting the rotation/position. CAMERA gives movement relative to the last pose.
        /// WORLD gives cumulative movements since tracking started.</param>
        /// <returns>State of ZED's Tracking system (off, searching, ok).</returns>
        public TRACKING_STATE GetPosition(ref Pose pose, REFERENCE_FRAME referenceType = REFERENCE_FRAME.WORLD)
        {
            return (TRACKING_STATE)dllz_get_position_data(CameraID, ref pose, (int)referenceType);
        }


        /// <summary>
        /// Sets a prior to the IMU orientation (only for ZED-M).
        /// Prior must come from a external IMU, such as the HMD orientation and should be in a time frame
        /// that's as close as possible to the camera.
        /// </summary>
        /// <returns>Error code status.</returns>
        /// <param name="rotation">Prior rotation.</param>
        public ERROR_CODE SetIMUOrientationPrior(ref Quaternion rotation)
        {
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            trackingStatus = (sl.ERROR_CODE)dllz_set_imu_prior_orientation(CameraID, rotation);
            return trackingStatus;
        }

        /// <summary>
        /// Gets the rotation given by the ZED-M/ZED2 IMU. Return an error if using ZED (v1) which does not contains internal sensors
        /// </summary>
        /// <returns>Error code status.</returns>
        /// <param name="rotation">Rotation from the IMU.</param>
        public ERROR_CODE GetInternalIMUOrientation(ref Quaternion rotation, TIME_REFERENCE referenceTime = TIME_REFERENCE.IMAGE)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            err = (sl.ERROR_CODE)dllz_get_internal_imu_orientation(CameraID, ref rotation, (int)referenceTime);
            return err;
        }

        /// <summary>
        /// Gets the full Sensor data from the ZED-M or ZED2 . Return an error if using ZED (v1) which does not contains internal sensors
        /// </summary>
        /// <returns>Error code status.</returns>
        /// <param name="rotation">Rotation from the IMU.</param>
        public ERROR_CODE GetInternalSensorsData(ref SensorsData data, TIME_REFERENCE referenceTime = TIME_REFERENCE.IMAGE)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            err = (sl.ERROR_CODE)dllz_get_internal_sensors_data(CameraID, ref data, (int)referenceTime);
            return err;
        }
        
        /// <summary>
        /// Sets a value in the ZED's camera settings.
        /// </summary>
        /// <param name="settings">Setting to be changed (brightness, contrast, gain, exposure, etc.)</param>
        /// <param name="value">New value.</param>
        /// <param name="usedefault">True to set the settings to their default values.</param>
        public void SetCameraSettings(CAMERA_SETTINGS settings, int value)
        {
            AssertCameraIsReady();
            //cameraSettingsManager.SetCameraSettings(CameraID, settings, value);
            dllz_set_video_settings(CameraID, (int)settings, value);
        }

        /// <summary>
        /// Gets the value of a given setting from the ZED camera.
        /// </summary>
        /// <param name="settings">Setting to be retrieved (brightness, contrast, gain, exposure, etc.)</param>
        public int GetCameraSettings(CAMERA_SETTINGS settings)
        {
            AssertCameraIsReady();
            return dllz_get_video_settings(CameraID, (int)settings);
            //return cameraSettingsManager.GetCameraSettings(CameraID, settings);
        }

        /// <summary>
        /// Overloaded function for CAMERA_SETTINGS.AEC_AGC_ROI (requires iRect as input)
        /// </summary>
        /// <param name="settings"> Must be set to CAMERA_SETTINGS.AEC_AGC_ROI. Otherwise will return -1.</param>
        /// <param name="side"> defines left=0 or right=1 or both=2 sensor target</param>
        /// <param name="roi">the roi defined as a sl.Rect</param>
        /// <param name="reset">Defines if the target must be reset to full sensor</param>
        /// <returns></returns>
        public int SetCameraSettings(CAMERA_SETTINGS settings, int side, iRect roi,bool reset)
        {
            AssertCameraIsReady();
            if (settings == CAMERA_SETTINGS.AEC_AGC_ROI)
                return dllz_set_roi_for_aec_agc(CameraID, side, roi, reset);
            else
                return -1;
        }

        /// <summary>
        /// Overloaded function for CAMERA_SETTINGS.AEC_AGC_ROI (requires iRect as input)
        /// </summary>
        /// <param name="settings"> Must be set to CAMERA_SETTINGS.AEC_AGC_ROI. Otherwise will return -1.</param>
        /// <param name="side"> defines left=0 or right=1 or both=2 sensor target.</param>
        /// <param name="roi"> Roi that will be filled.</param>
        /// <returns></returns>
        public int GetCameraSettings(CAMERA_SETTINGS settings, int side,ref iRect roi)
        {
            AssertCameraIsReady();
            if (settings == CAMERA_SETTINGS.AEC_AGC_ROI)
                return dllz_get_roi_for_aec_agc(CameraID, side, ref roi);
            else
                return -1;
        }

        /// <summary>
        /// Reset camera settings to default
        /// </summary>
        public void ResetCameraSettings()
        {
            AssertCameraIsReady();
            //cameraSettingsManager.ResetCameraSettings(this);

            SetCameraSettings(sl.CAMERA_SETTINGS.BRIGHTNESS, sl.ZEDCamera.brightnessDefault);
            SetCameraSettings(sl.CAMERA_SETTINGS.CONTRAST, sl.ZEDCamera.contrastDefault);
            SetCameraSettings(sl.CAMERA_SETTINGS.HUE, sl.ZEDCamera.hueDefault);
            SetCameraSettings(sl.CAMERA_SETTINGS.SATURATION, sl.ZEDCamera.saturationDefault);
            SetCameraSettings(sl.CAMERA_SETTINGS.SHARPNESS, sl.ZEDCamera.sharpnessDefault);
            SetCameraSettings(sl.CAMERA_SETTINGS.GAMMA, sl.ZEDCamera.gammaDefault);
            SetCameraSettings(sl.CAMERA_SETTINGS.AUTO_WHITEBALANCE, 1);
            SetCameraSettings(sl.CAMERA_SETTINGS.AEC_AGC, 1);
            SetCameraSettings(sl.CAMERA_SETTINGS.LED_STATUS, 1);

            SetCameraSettings(sl.CAMERA_SETTINGS.AEC_AGC_ROI,2, new sl.iRect(), true);
        }

        /// <summary>
        /// Loads camera settings (brightness, contrast, hue, saturation, gain, exposure) from a file in the
        /// project's root directory.
        /// </summary>
        /// <param name="path">Filename.</param>
        public void LoadCameraSettings(string path)
        {
            cameraSettingsManager.LoadCameraSettings(this, path);
        }

        /// <summary>
        /// Save the camera settings (brightness, contrast, hue, saturation, gain, exposure) to a file
        /// relative to the project's root directory.
        /// </summary>
        /// <param name="path">Filename.</param>
        public void SaveCameraSettings(string path)
        {
            cameraSettingsManager.SaveCameraSettings(path);
        }

        /// <summary>
        /// Retrieves camera settings from the ZED camera and loads them into a CameraSettings instance
        /// handled by ZEDCameraSettingsManager.
        /// </summary>
        public void RetrieveCameraSettings()
        {
            cameraSettingsManager.RetrieveSettingsCamera(this);
        }

        /// <summary>
        /// Returns if the camera's exposure mode is set to automatic.
        /// </summary>
        /// <returns><c>True</c> if automatic, <c>false</c> if manual.</returns>
        public bool GetExposureUpdateType()
        {
            return cameraSettingsManager.auto;
        }

        /// <summary>
        /// Returns if the camera's white balance  is set to automatic.
        /// </summary>
        /// <returns><c>True</c> if automatic, <c>false</c> if manual.</returns>
        public bool GetWhiteBalanceUpdateType()
        {
            return cameraSettingsManager.whiteBalanceAuto;
        }

        /// <summary>
        /// Applies all the settings registered in the ZEDCameraSettingsManager instance to the actual ZED camera.
        /// </summary>
        public void SetCameraSettings()
        {
            cameraSettingsManager.SetSettings(this);
        }

        /// <summary>
        /// Checks if the ZED camera is connected.
        /// </summary>
        /// <remarks>The C++ SDK version of this call returns the number of connected ZEDs.</remarks>
        /// <returns>True if ZED is connected.</returns>
        public static bool IsZedConnected()
        {
            return Convert.ToBoolean(dllz_is_zed_connected());
        }

        /// <summary>
        /// Gets the version of the currently installed ZED SDK.
        /// </summary>
        /// <returns>ZED SDK version as a string in the format MAJOR.MINOR.PATCH.</returns>
        public static string GetSDKVersion()
        {
            return PtrToStringUtf8(dllz_get_sdk_version());
        }

        /// <summary>
        /// Checks if the camera has been initialized and the plugin has been loaded. Throws exceptions otherwise.
        /// </summary>
        private void AssertCameraIsReady()
        {
            if (!cameraReady)
                throw new Exception("ZED camera is not connected or Init() was not called.");

            if (!pluginIsReady)
                throw new Exception("Could not resolve ZED plugin dependencies.");

        }

        /// <summary>
        /// Deploys an event that causes the textures to be updated with images received from the ZED.
        /// Should be called after RetrieveTextures() so there are new images available.
        /// </summary>
        /*public void UpdateTextures()
        {
            GL.IssuePluginEvent(GetRenderEventFunc(), 1);
        }*/


        ///////////////////////////// SINGLE PIXEL UTILITY FUNCTIONS ////////////////////////////////

        public float GetDepthValue(int posX, int posY)
        {
            if (!cameraReady)
            {
                return -1;
            }

            float d = dllz_get_depth_value(CameraID, (uint)posX, (uint)posY);
            return d;
        }

        public float GetDistanceValue(int posX, int posY)
        {
            if (!cameraReady) //Do nothing if the ZED isn't initialized.
            {
                return -1;
            }

            return dllz_get_distance_value(CameraID, (uint)posX, (uint)posY);
        }

        public bool GetXYZValue(int posX, int posY, out Vector4 xyz)
        {
            if (!cameraReady) //Do nothing if the ZED isn't initialized.
            {
                xyz = Vector4.Zero;
                return false;
            }

            bool r = dllz_get_xyz_value(CameraID, (uint)posX, (uint)posY, out xyz) != 0;
            return r;
        }

        public bool GetNormalValue(int posX, int posY, out Vector4 normal)
        {
            if (!cameraReady) //Do nothing if the ZED isn't initialized.
            {
                normal = Vector4.Zero;
                return false;
            }

            bool r = dllz_get_normal_value(CameraID, (uint)posX, (uint)posY, out normal) != 0;
            return r;
        }

        ///////////////////////////// SPATIAL MAPPING ////////////////////////////////

        /// <summary>
        /// Updates the range to match the specified preset.
        /// </summary>
        static public float ConvertRangePreset(MAPPING_RANGE rangePreset)
        {

            if (rangePreset == MAPPING_RANGE.NEAR)
            {
                return 3.5f;
            }
            else if (rangePreset == MAPPING_RANGE.MEDIUM)
            {
                return 5.0f;
            }
            if (rangePreset == MAPPING_RANGE.FAR)
            {
                return 10.0f;
            }
            return 5.0f;
        }

        /// <summary>
        /// Updates the resolution to match the specified preset.
        /// </summary>
        static public float ConvertResolutionPreset(MAPPING_RESOLUTION resolutionPreset)
        {
            if (resolutionPreset == MAPPING_RESOLUTION.HIGH)
            {
                return 0.05f;
            }
            else if (resolutionPreset == MAPPING_RESOLUTION.MEDIUM)
            {
                return 0.10f;
            }
            if (resolutionPreset == MAPPING_RESOLUTION.LOW)
            {
                return 0.15f;
            }
            return 0.10f;
        }

        /// <summary>
        /// Initializes and begins the spatial mapping processes.
        /// </summary>
        /// <param name="resolution_meter">Spatial mapping resolution in meters.</param>
        /// <param name="max_range_meter">Maximum scanning range in meters.</param>
        /// <param name="saveTexture">True to scan surface textures in addition to geometry.</param>
        /// <returns></returns>
        public sl.ERROR_CODE EnableSpatialMapping(SPATIAL_MAP_TYPE type, float resolution_meter, float max_range_meter, bool saveTexture = false)
        {
            sl.ERROR_CODE spatialMappingStatus = ERROR_CODE.FAILURE;
            lock (grabLock)
            {
                spatialMappingStatus = (sl.ERROR_CODE)dllz_enable_spatial_mapping(CameraID, (int)type, resolution_meter, max_range_meter, System.Convert.ToInt32(saveTexture), 2048);
            }
            return spatialMappingStatus;
        }

        /// <summary>
        /// Initializes and begins the spatial mapping processes.
        /// </summary>
        /// <param name="resolution_meter">Spatial mapping resolution in meters.</param>
        /// <param name="max_range_meter">Maximum scanning range in meters.</param>
        /// <param name="saveTexture">True to scan surface textures in addition to geometry.</param>
        /// <returns></returns>
        public sl.ERROR_CODE EnableSpatialMapping(SPATIAL_MAP_TYPE type, MAPPING_RESOLUTION mapping_resolution, MAPPING_RANGE mapping_range, bool saveTexture = false)
        {
            return EnableSpatialMapping(type, ConvertResolutionPreset(mapping_resolution), ConvertRangePreset(mapping_range), saveTexture);
        }

        /// <summary>
        /// Disables the Spatial Mapping process.
        /// </summary>
        public void DisableSpatialMapping()
        {
            lock (grabLock)
            {
                dllz_disable_spatial_mapping(CameraID);
            }
        }

        /// <summary>
        /// Updates the internal version of the mesh and returns the sizes of the meshes.
        /// </summary>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each submesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each submesh.</param>
        /// <param name="nbSubmeshes">Number of submeshes.</param>
        /// <param name="updatedIndices">List of all submeshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all submeshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all submeshes.</param>
        /// <param name="nbSubmeshMax">Maximum number of submeshes that can be handled.</param>
        /// <returns>Error code indicating if the update was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE UpdateMesh(int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmeshMax)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;
            err = (sl.ERROR_CODE)dllz_update_mesh(CameraID, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, nbSubmeshMax);
            return err;
        }

        /// <summary>
        /// Retrieves all chunks of the generated mesh. Call UpdateMesh() before calling this.
        /// Vertex and triangle arrays must be at least of the sizes returned by UpdateMesh (nbVertices and nbTriangles).
        /// </summary>
        /// <param name="vertices">Vertices of the mesh.</param>
        /// <param name="triangles">Triangles, formatted as the index of each triangle's three vertices in the vertices array.</param>
        /// <param name="nbSubmeshMax">Maximum number of submeshes that can be handled.</param>
        /// <returns>Error code indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveMesh(Vector3[] vertices, int[] triangles, int nbSubmeshMax, Vector2[] uvs, IntPtr textures)
        {
            return (sl.ERROR_CODE)dllz_retrieve_mesh(CameraID, vertices, triangles, nbSubmeshMax, uvs, textures);
        }

        ///Extracts the current spatial map from the spatial mapping process.
        ///If the object to be filled already contains a previous version of the mesh, only changes will be updated, optimizing performance.
        ///return \ref SUCCESS if the mesh is filled and available, otherwise \ref ERROR_CODE::FAILURE.
        ///warning This is a blocking function.You should either call it in a thread or at the end of the mapping process.
        public ERROR_CODE extractWholeSpatialMap()
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;
            err = (sl.ERROR_CODE)dllz_extract_whole_spatial_map(CameraID);
            return err;
        }
        /// <summary>
        /// Starts the mesh generation process in a thread that doesn't block the spatial mapping process.
        /// ZEDSpatialMappingHelper calls this each time it has finished applying the last mesh update.
        /// </summary>
        public void RequestMesh()
        {
            dllz_request_mesh_async(CameraID);
        }

        /// <summary>
        /// Sets the pause state of the data integration mechanism for the ZED's spatial mapping.
        /// </summary>
        /// <param name="status">If true, the integration is paused. If false, the spatial mapping is resumed.</param>
        public void PauseSpatialMapping(bool status)
        {
            dllz_pause_spatial_mapping(CameraID, status);
        }

        /// <summary>
        /// Returns the mesh generation status. Useful for knowing when to update and retrieve the mesh.
        /// </summary>
        public sl.ERROR_CODE GetMeshRequestStatus()
        {
            return (sl.ERROR_CODE)dllz_get_mesh_request_status_async(CameraID);
        }

        /// <summary>
        /// Saves the scanned mesh in a specific file format.
        /// </summary>
        /// <param name="filename">Path and filename of the mesh.</param>
        /// <param name="format">File format (extension). Can be .obj, .ply or .bin.</param>
        public bool SaveMesh(string filename, MESH_FILE_FORMAT format)
        {
            return dllz_save_mesh(CameraID, filename, format);
        }

        /// <summary>
        /// Loads a saved mesh file. ZEDSpatialMapping then configures itself as if the loaded mesh was just scanned.
        /// </summary>
        /// <param name="filename">Path and filename of the mesh. Should include the extension (.obj, .ply or .bin).</param>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each submesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each submesh.</param>
        /// <param name="nbSubmeshes">Number of submeshes.</param>
        /// <param name="updatedIndices">List of all submeshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all submeshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all submeshes.</param>
        /// <param name="nbSubmeshMax">Maximum number of submeshes that can be handled.</param>
        /// <param name="textureSize">Array containing the sizes of all the textures (width, height) if applicable.</param>
        public bool LoadMesh(string filename, int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices,
            ref int nbVertices, ref int nbTriangles, int nbSubmeshMax, int[] textureSize = null)
        {
            return dllz_load_mesh(CameraID, filename, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices,
                ref nbTriangles, nbSubmeshMax, textureSize);
        }

        /// <summary>
        /// Filters a mesh to remove triangles while still preserving its overall shape (though less accurate).
        /// </summary>
        /// <param name="filterParameters">Filter level. Higher settings remove more triangles.</param>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each submesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each submesh.</param>
        /// <param name="nbSubmeshes">Number of submeshes.</param>
        /// <param name="updatedIndices">List of all submeshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all submeshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all submeshes.</param>
        /// <param name="nbSubmeshMax">Maximum number of submeshes that can be handled.</param>
        public bool FilterMesh(FILTER filterParameters, int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmeshMax)
        {
            return dllz_filter_mesh(CameraID, filterParameters, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, nbSubmeshMax);
        }

        /// <summary>
        /// Applies the scanned texture onto the internal scanned mesh.
        /// </summary>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each submesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each submesh.</param>
        /// <param name="nbSubmeshes">Number of submeshes.</param>
        /// <param name="updatedIndices">List of all submeshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all submeshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all submeshes.</param>
        /// <param name="textureSize"> Vector containing the size of all the texture (width, height). </param>
        /// <param name="nbSubmeshMax">Maximum number of submeshes that can be handled.</param>
        /// <returns></returns>
        public bool ApplyTexture(int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int[] textureSize, int nbSubmeshMax)
        {
            return dllz_apply_texture(CameraID, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, textureSize, nbSubmeshMax);
        }

        /// <summary>
        /// Gets the current state of spatial mapping.
        /// </summary>
        /// <returns></returns>
        public SPATIAL_MAPPING_STATE GetSpatialMappingState()
        {
            return (sl.SPATIAL_MAPPING_STATE)dllz_get_spatial_mapping_state(CameraID);
        }

        /// <summary>
        /// Gets a vector pointing toward the direction of gravity. This is estimated from a 3D scan of the environment,
        /// and as such, a scan must be started/finished for this value to be calculated.
        /// If using the ZED Mini, this isn't required thanks to its IMU.
        /// </summary>
        /// <returns>Vector3 pointing downward.</returns>
        public Vector3 GetGravityEstimate()
        {
            Vector3 v = Vector3.Zero;
            dllz_spatial_mapping_get_gravity_estimation(CameraID, ref v);
            return v;
        }

        /// <summary>
        /// Consolidates the chunks from a scan. This is used to turn lots of small meshes (which are efficient for
        /// the scanning process) into several large meshes (which are more convenient to work with).
        /// </summary>
        /// <param name="numberFaces"></param>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each submesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each submesh.</param>
        /// <param name="nbSubmeshes">Number of submeshes.</param>
        /// <param name="updatedIndices">List of all submeshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all submeshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all submeshes.</param>
        public void MergeChunks(int numberFaces, int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh)
        {
            dllz_spatial_mapping_merge_chunks(CameraID, numberFaces, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, nbSubmesh);
        }

        /// <summary>
        /// Retrieves a measure texture from the ZED SDK and loads it into a ZEDMat. Use this to get an individual
        /// texture from the last grabbed frame with measurements in every pixel - such as a depth map, confidence map, etc.
        /// Measure textures are not human-viewable but don't lose accuracy, unlike image textures.
        /// </summary><remarks>
        /// If you want to access the texture via script, you'll usually want to specify CPU memory. Then you can use
        /// Marshal.Copy to move them into a new byte array, which you can load into a Texture2D.
        /// RetrieveMeasure() calls Camera::retrieveMeasure() in the C++ SDK. For more info, read:
        /// https://www.stereolabs.com/developers/documentation/API/v2.5.1/classsl_1_1Camera.html#af799d12342a7b884242fffdef5588a7f
        /// </remarks>
        /// <param name="mat">ZEDMat to fill with the new texture.</param>
        /// <param name="measure">Measure type (depth, confidence, xyz, etc.)</param>
        /// <param name="mem">Whether the image should be on CPU or GPU memory.</param>
        /// <param name="resolution">Resolution of the texture.</param>
        /// <returns>Error code indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveMeasure(sl.ZEDMat mat, sl.MEASURE measure, sl.MEM mem = sl.MEM.MEM_CPU, sl.Resolution resolution = new sl.Resolution())
        {
            return (sl.ERROR_CODE)(dllz_retrieve_measure(CameraID, mat.MatPtr, (int)measure, (int)mem, resolution));
        }

        /// <summary>
        /// Retrieves an image texture from the ZED SDK and loads it into a ZEDMat. Use this to get an individual
        /// texture from the last grabbed frame in a human-viewable format. Image textures work for when you want the result to be visible,
        /// such as the direct RGB image from the camera, or a greyscale image of the depth. However it will lose accuracy if used
        /// to show measurements like depth or confidence, unlike measure textures.
        /// </summary><remarks>
        /// If you want to access the texture via script, you'll usually want to specify CPU memory. Then you can use
        /// Marshal.Copy to move them into a new byte array, which you can load into a Texture2D. Note that you may need to
        /// change the color space and/or flip the image.
        /// RetrieveMeasure() calls Camera::retrieveMeasure() in the C++ SDK. For more info, read:
        /// https://www.stereolabs.com/developers/documentation/API/v2.5.1/classsl_1_1Camera.html#ac40f337ccc76cacd3412b93f7f4638e2
        /// </remarks>
        /// <param name="mat">ZEDMat to fill with the new texture.</param>
        /// <param name="view">Image type (left RGB, right depth map, etc.)</param>
        /// <param name="mem">Whether the image should be on CPU or GPU memory.</param>
        /// <param name="resolution">Resolution of the texture.</param>
        /// <returns>Error code indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveImage(sl.ZEDMat mat, sl.VIEW view, sl.MEM mem = sl.MEM.MEM_CPU, sl.Resolution resolution = new sl.Resolution())
        {
            return (sl.ERROR_CODE)(dllz_retrieve_image(CameraID, mat.MatPtr, (int)view, (int)mem, resolution));
        }

        ////////////////////////
        /// Plane Detection  ///
        ////////////////////////


        /// <summary>
        /// Looks for a plane in the visible area that is likely to represent the floor.
        /// Use ZEDPlaneDetectionManager.DetectFloorPlane for a higher-level version that turns planes into GameObjects.
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="playerHeight">Height of the camera from the newly-detected floor.</param>
        /// <param name="priorQuat">Prior rotation.</param>
        /// <param name="priorTrans">Prior position.</param>
        /// <returns></returns>
        public sl.ERROR_CODE findFloorPlane(ref PlaneData plane, out float playerHeight, Quaternion priorQuat, Vector3 priorTrans)
        {
            IntPtr p = IntPtr.Zero;
            Quaternion out_quat = Quaternion.Identity;
            Vector3 out_trans = Vector3.Zero;
            p = dllz_find_floor_plane(CameraID, out out_quat, out out_trans, priorQuat, priorTrans);
            plane.Bounds = new Vector3[256];
            playerHeight = 0;

            if (p != IntPtr.Zero)
            {
                plane = (PlaneData)Marshal.PtrToStructure(p, typeof(PlaneData));
                playerHeight = out_trans.Y;
                return (sl.ERROR_CODE)plane.ErrorCode;
            }
            else
                return sl.ERROR_CODE.FAILURE;
        }

        /// <summary>
        /// Using data from a detected floor plane, updates supplied vertex and triangle arrays with
        /// data needed to make a mesh that represents it. These arrays are updated directly from the wrapper.
        /// </summary>
        /// <param name="vertices">Array to be filled with mesh vertices.</param>
        /// <param name="triangles">Array to be filled with mesh triangles, stored as indexes of each triangle's points.</param>
        /// <param name="numVertices">Total vertices in the mesh.</param>
        /// <param name="numTriangles">Total triangle indexes (3x number of triangles).</param>
        /// <returns></returns>
        public int convertFloorPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_floorplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
        }

        /// <summary>
        /// Checks for a plane in the real world at given screen-space coordinates.
        /// Use ZEDPlaneDetectionManager.DetectPlaneAtHit() for a higher-level version that turns planes into GameObjects.
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="screenPos">Point on the ZED image to check for a plane.</param>
        /// <returns></returns>
        public sl.ERROR_CODE findPlaneAtHit(ref PlaneData plane, Vector2 coord)
        {
            IntPtr p = IntPtr.Zero;
            Quaternion out_quat = Quaternion.Identity;
            Vector3 out_trans = Vector3.Zero;

            p = dllz_find_plane_at_hit(CameraID, coord, true);
            plane.Bounds = new Vector3[256];

            if (p != IntPtr.Zero)
            {
                plane = (PlaneData)Marshal.PtrToStructure(p, typeof(PlaneData));
                return (sl.ERROR_CODE)plane.ErrorCode;
            }
            else
                return sl.ERROR_CODE.FAILURE;
        }

        /// <summary>
        /// Using data from a detected hit plane, updates supplied vertex and triangle arrays with
        /// data needed to make a mesh that represents it. These arrays are updated directly from the wrapper.
        /// </summary>
        /// <param name="vertices">Array to be filled with mesh vertices.</param>
        /// <param name="triangles">Array to be filled with mesh triangles, stored as indexes of each triangle's points.</param>
        /// <param name="numVertices">Total vertices in the mesh.</param>
        /// <param name="numTriangles">Total triangle indexes (3x number of triangles).</param>
        /// <returns></returns>
        public int convertHitPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_hitplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
        }

        ////////////////////////
        /// Streaming Module ///
        ////////////////////////

        /// <summary>
        /// Creates an streaming pipeline.
        /// </summary>
        /// <params>
        /// Streaming parameters: See sl::StreamingParameters of ZED SDK. See ZED SDK API doc for more informations
        /// </params>
        /// <returns>An ERROR_CODE that defines if the streaming pipe was successfully created</returns>
        public ERROR_CODE EnableStreaming(STREAMING_CODEC codec = STREAMING_CODEC.AVCHD_BASED, uint bitrate = 8000, ushort port = 30000, int gopSize = -1, bool adaptativeBitrate = false, int chunk_size = 8096, int target_fps = 0)
        {
            int doAdaptBitrate = adaptativeBitrate ? 1 : 0;
            return (ERROR_CODE)dllz_enable_streaming(CameraID, codec, bitrate, port, gopSize, doAdaptBitrate, chunk_size, target_fps);
        }

        /// <summary>
        /// Tells if streaming is running or not.
        /// </summary>
        /// <returns> false if streaming is not enabled, true if streaming is on</returns>
        public bool IsStreamingEnabled()
        {
            int res = dllz_is_streaming_enabled(CameraID);
            if (res == 1)
                return true;
            else
                return false;
        }

        /// <summary>
        /// Stops the streaming pipeline.
        /// </summary>
        public void DisableStreaming()
        {
            dllz_disable_streaming(CameraID);
        }


        ////////////////////////
        /// Save utils fct   ///
        ////////////////////////

        /// <summary>
        /// Save current image (specified by view) in a file defined by filename
        /// Supported formats are jpeg and png. Filename must end with either .jpg or .png
        /// </summary>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE SaveCurrentImageInFile(sl.VIEW view, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_image(CameraID, view, filename);
            return err;
        }

        /// <summary>
        /// Save the current depth in a file defined by filename.
        /// Supported formats are PNG,PFM and PGM
        /// </summary>
        /// <param name="side"> defines left (0) or right (1) depth</param>
        /// <param name="filename"> filename must end with .png, .pfm or .pgm</param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE SaveCurrentDepthInFile(int side, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_depth(CameraID, side, filename);
            return err;
        }

        /// <summary>
        /// Save the current point cloud in a file defined by filename.
        /// Supported formats are PLY,VTK,XYZ and PCD 
        /// </summary>
        /// <param name="side">defines left (0) or right (1) point cloud</param>
        /// <param name="filename"> filename must end with .ply, .xyz , .vtk or .pcd </param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE SaveCurrentPointCloudInFile(int side, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_point_cloud(CameraID, side, filename);
            return err;
        }

        ////////////////////////
        /// Object detection ///
        ////////////////////////

        /// <summary>
        /// Enable object detection module
        /// </summary>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE EnableObjectsDetection(ref ObjectDetectionParameters od_params)
        {
            sl.ERROR_CODE objDetectStatus = ERROR_CODE.FAILURE;
            lock (grabLock)
            {
                objDetectStatus = (sl.ERROR_CODE)dllz_enable_objects_detection(CameraID, ref od_params);
            }

            return objDetectStatus;
        }

        /// <summary>
        /// Disable object detection module and release the resources.
        /// </summary>
        public void DisableObjectsDetection()
        {
            lock (grabLock)
            {
                dllz_disable_objects_detection(CameraID);
            }
        }

        /// <summary>
        /// Pause or Unpause the object detection
        /// </summary>
        /// <param name="status"></param>
        public void PauseObjectsDetection(bool status)
        {
            lock (grabLock)
            {
                dllz_pause_objects_detection(CameraID, status);
            }
        }

        /// <summary>
        /// Retrieve object detection data 
        /// </summary>
        /// <param name="od_params"> Object detection runtime parameters </param>
        /// <param name="objFrame"> ObjectsFrameSDK that contains all the detection data </param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE RetrieveObjectsDetectionData(ref ObjectDetectionRuntimeParameters od_params, ref ObjectsFrameSDK objFrame)
        {
            return (sl.ERROR_CODE)dllz_retrieve_objects_data(CameraID, ref od_params, ref objFrame);
        }



    }
}
