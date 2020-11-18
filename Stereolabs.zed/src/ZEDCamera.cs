//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Reflection;
using System.Numerics;

namespace sl
{
    ///\ingroup  Video_group
    /// <summary>
    /// This class is the main interface with the camera and the SDK features, such as: video, depth, tracking, mapping, and more.
    /// </summary>
    public class Camera
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
        /// resolution unless a lower setting was specified in Open().
        /// Maximum values are bound by the ZED's output, not system performance.
        /// </summary>
        private uint fpsMax = 60; //Defaults to HD720 resolution's output.
        /// <summary>
        /// Desired FPS from the ZED camera. This is the maximum FPS for the ZED's current
        /// resolution unless a lower setting was specified in Open().
        /// Maximum values are bound by the ZED's output, not system performance.
        /// </summary>
        public float GetRequestedCameraFPS()
        {
            return fpsMax;
        }

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
        private static extern int dllz_enable_recording(int cameraID, byte[] video_filename, int compresssionMode, uint bitrate, int target_fps, bool transcode);

        [DllImport(nameDll, EntryPoint = "dllz_disable_recording")]
        private static extern bool dllz_disable_recording(int cameraID);

        /*
         * Camera control functions.
         */

        [DllImport(nameDll, EntryPoint = "dllz_set_video_settings")]
        private static extern void dllz_set_video_settings(int id, int mode, int value);

        [DllImport(nameDll, EntryPoint = "dllz_get_video_settings")]
        private static extern int dllz_get_video_settings(int id, int mode);

        [DllImport(nameDll, EntryPoint = "dllz_set_roi_for_aec_agc")]
        private static extern int dllz_set_roi_for_aec_agc(int id, int side, Rect roi,bool reset);

        [DllImport(nameDll, EntryPoint = "dllz_get_roi_for_aec_agc")]
        private static extern int dllz_get_roi_for_aec_agc(int id, int side, ref Rect roi);


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

        [DllImport(nameDll, EntryPoint = "dllz_get_init_parameters")]
        private static extern IntPtr dllz_get_init_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_runtime_parameters")]
        private static extern IntPtr dllz_get_runtime_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_get_positional_tracking_parameters")]
        private static extern IntPtr dllz_get_positional_tracking_parameters(int cameraID);
        

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

        [DllImport(nameDll, EntryPoint = "dllz_update_chunks")]
        private static extern int dllz_update_chunks(int cameraID, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "dllz_retrieve_mesh")]
        private static extern int dllz_retrieve_mesh(int cameraID, [In, Out] Vector3[] vertices, int[] triangles, int nbSubmesh, [In, Out] Vector2[] uvs, IntPtr textures);

        [DllImport(nameDll, EntryPoint = "dllz_retrieve_chunks")]
        private static extern int dllz_retrieve_chunks(int cameraID, int maxSubmesh, [In, Out] Vector3[] vertices, int[] triangles);

        [DllImport(nameDll, EntryPoint = "dllz_extract_whole_spatial_map")]
        private static extern int dllz_extract_whole_spatial_map(int cameraID);

        [DllImport(nameDll, EntryPoint = "dllz_update_fused_point_cloud")]
        private static extern int dllz_update_fused_point_cloud(int cameraID, ref int pbPoints);

        [DllImport(nameDll, EntryPoint = "dllz_retrieve_fused_point_cloud")]
        private static extern int dllz_retrieve_fused_point_cloud(int cameraID, [In, Out] Vector4[] points);

        [DllImport(nameDll, EntryPoint = "dllz_save_mesh")]
        private static extern bool dllz_save_mesh(int cameraID, string filename, MESH_FILE_FORMAT format);

        [DllImport(nameDll, EntryPoint = "dllz_save_point_cloud")]
        private static extern bool dllz_save_point_cloud(int cameraID, string filename, MESH_FILE_FORMAT format);
        
        [DllImport(nameDll, EntryPoint = "dllz_load_mesh")]
        private static extern bool dllz_load_mesh(int cameraID, string filename, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbMaxSubmesh, int[] textureSize = null);

        [DllImport(nameDll, EntryPoint = "dllz_apply_texture")]
        private static extern bool dllz_apply_texture(int cameraID, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int[] textureSize, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "dllz_filter_mesh")]
        private static extern bool dllz_filter_mesh(int cameraID, MESH_FILTER meshFilter, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

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
        private static extern int dllz_convert_floorplane_to_mesh(int cameraID, [In, Out] Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles);

        [DllImport(nameDll, EntryPoint = "dllz_convert_hitplane_to_mesh")]
        private static extern int dllz_convert_hitplane_to_mesh(int cameraID, [In, Out] Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles);


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
        private static extern int dllz_retrieve_objects_data(int cameraID, ref ObjectDetectionRuntimeParameters od_params, ref Objects objs);


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
         * Retrieves used by mat
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
        /// Constructor that creates an empty Camera object;
        /// </summary>
        /// <param name="id"></param>
        public Camera(int id)
        {
            CameraID = id;
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
        struct dll_initParameters
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
            public UNIT coordinateUnits;
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
                coordinateUnits = init.coordinateUnits;
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
        struct dll_RuntimeParameters
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

        /// <summary>
        /// DLL-friendly version of PositionalTrackingParameters (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        struct dll_PositionalTrackingParameters
        {
            public Quaternion initialWorldRotation;
            public Vector3 initialWorldPosition;
            public bool enableAreaMemory;
            public bool enablePoseSmothing;
            public bool setFloorAsOrigin;
            public bool setAsStatic;
            public bool enableIMUFusion;
            public string areaFilePath;
        };

        /// <summary>
        /// Checks if the ZED camera is plugged in and  opens it.
        /// </summary>
        /// <param name="initParameters">Class with all initialization settings.
        /// A newly-instantiated InitParameters will have recommended default values.</param>
        /// <returns>ERROR_CODE: The error code gives information about the internal connection process.
        /// If SUCCESS is returned, the camera is ready to use. Every other code indicates an error.</returns>
        public ERROR_CODE Open(ref InitParameters initParameters)
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
        /// https://www.stereolabs.com/docs/api/classsl_1_1Camera.html#afa3678a18dd574e162977e97d7cbf67b </remarks>
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

        ///@name Video

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
        /// https://www.stereolabs.com/docs/api/classsl_1_1Camera.html#a01dce4f0af6f8959a9c974ffaca656b5
        /// </remarks>
        /// <param name="mat">ZEDMat to fill with the new texture.</param>
        /// <param name="view">Image type (left RGB, right depth map, etc.)</param>
        /// <param name="mem">Whether the image should be on CPU or GPU memory.</param>
        /// <param name="resolution">Resolution of the texture.</param>
        /// <returns>Error code indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveImage(sl.Mat mat, sl.VIEW view, sl.MEM mem = sl.MEM.CPU, sl.Resolution resolution = new sl.Resolution())
        {
            return (sl.ERROR_CODE)(dllz_retrieve_image(CameraID, mat.MatPtr, (int)view, (int)mem, resolution));
        }

        /// <summary>
        /// Returns the init parameters used. Correspond to the structure send when the open() function was called.
        /// </summary>
        /// <returns>InitParameters containing the parameters used to initialize the Camera object.</returns>
        public InitParameters GetInitParameters()
        {
            IntPtr p = dllz_get_init_parameters(CameraID);
            if (p == IntPtr.Zero)
            {
                return new InitParameters();
            }
            dll_initParameters dll_parameters = (dll_initParameters)Marshal.PtrToStructure(p, typeof(dll_initParameters));
            InitParameters parameters = new InitParameters()
            {
                cameraDeviceID = dll_parameters.cameraDeviceID,
                inputType = dll_parameters.inputType,
                resolution = dll_parameters.resolution,
                cameraFPS = dll_parameters.cameraFps,
                svoRealTimeMode = dll_parameters.svoRealTimeMode,
                coordinateUnits = dll_parameters.coordinateUnits,
                coordinateSystem = dll_parameters.coordinateSystem,
                depthMaximumDistance = dll_parameters.depthMaximumDistance,
                depthMinimumDistance = dll_parameters.depthMinimumDistance,
                depthMode = dll_parameters.depthMode,
                cameraImageFlip = (FLIP_MODE)dll_parameters.cameraImageFlip,
                enableImageEnhancement = dll_parameters.enableImageEnhancement,
                enableRightSideMeasure = dll_parameters.enableRightSideMeasure,
                cameraDisableSelfCalib = dll_parameters.cameraDisableSelfCalib,
                sdkGPUId = dll_parameters.sdkGPUId,
                sdkVerbose = dll_parameters.sdkVerbose,
                depthStabilization = dll_parameters.depthStabilization,
                sensorsRequired = dll_parameters.sensorsRequired

            };
            return parameters;
        }
        /// <summary>
        /// Returns the runtime parameters used. Correspond to the structure send when the grab() function was called.
        /// </summary>
        /// <returns>RuntimeParameters containing the parameters that defines the behavior of the grab.</returns>
        public RuntimeParameters GetRuntimeParameters()
        {
            IntPtr p = dllz_get_runtime_parameters(CameraID);
            if (p == IntPtr.Zero)
            {
                return new RuntimeParameters();
            }

            dll_RuntimeParameters dll_parameters = (dll_RuntimeParameters)Marshal.PtrToStructure(p, typeof(dll_RuntimeParameters));
            RuntimeParameters parameters = new RuntimeParameters()
            {
                sensingMode = dll_parameters.sensingMode,
                textureConfidenceThreshold = dll_parameters.textureConfidenceThreshold,
                measure3DReferenceFrame = dll_parameters.measure3DReferenceFrame,
                enableDepth = dll_parameters.enableDepth,
                confidenceThreshold = dll_parameters.confidenceThreshold,
            };
            return parameters;
        }

        public PositionalTrackingParameters GetPositionalTrackingParameters()
        {
            IntPtr p = dllz_get_positional_tracking_parameters(CameraID);
            if (p == IntPtr.Zero)
            {
                return new PositionalTrackingParameters();
            }
            dll_PositionalTrackingParameters dll_positionalTracking = (dll_PositionalTrackingParameters)Marshal.PtrToStructure(p, typeof(dll_PositionalTrackingParameters));
            PositionalTrackingParameters trackingParams = new PositionalTrackingParameters()
            {
                initialWorldPosition = dll_positionalTracking.initialWorldPosition,
                initialWorldRotation = dll_positionalTracking.initialWorldRotation,
                enableIMUFusion = dll_positionalTracking.enableIMUFusion,
                enableAreaMemory = dll_positionalTracking.enableAreaMemory,
                enablePoseSmothing = dll_positionalTracking.enablePoseSmothing,
                areaFilePath = dll_positionalTracking.areaFilePath,
                setAsStatic = dll_positionalTracking.setAsStatic,
                setFloorAsOrigin = dll_positionalTracking.setFloorAsOrigin
            };

            return trackingParams;
        }


        /// <summary>
        /// Sets a value in the ZED's camera settings.
        /// </summary>
        /// <param name="settings">Setting to be changed (brightness, contrast, gain, exposure, etc.)</param>
        /// <param name="value">New value.</param>
        /// <param name="usedefault">True to set the settings to their default values.</param>
        public void SetCameraSettings(VIDEO_SETTINGS settings, int value)
        {
            AssertCameraIsReady();
            //cameraSettingsManager.SetCameraSettings(CameraID, settings, value);
            dllz_set_video_settings(CameraID, (int)settings, value);
        }

        /// <summary>
        /// Gets the value of a given setting from the ZED camera.
        /// </summary>
        /// <param name="settings">Setting to be retrieved (brightness, contrast, gain, exposure, etc.)</param>
        /// <returns>The current value for the corresponding setting. Returns -1 if encounters an error.</returns>
        public int GetCameraSettings(VIDEO_SETTINGS settings)
        {
            AssertCameraIsReady();
            return dllz_get_video_settings(CameraID, (int)settings);
            //return cameraSettingsManager.GetCameraSettings(CameraID, settings);
        }

        /// <summary>
        /// Overloaded function for CAMERA_SETTINGS.AEC_AGC_ROI (requires Rect as input)
        /// </summary>
        /// <param name="settings"> Must be set to CAMERA_SETTINGS.AEC_AGC_ROI. Otherwise will return -1.</param>
        /// <param name="side"> defines left=0 or right=1 or both=2 sensor target</param>
        /// <param name="roi">the roi defined as a sl.Rect</param>
        /// <param name="reset">Defines if the target must be reset to full sensor</param>
        /// <returns>ERROR_CODE.SUCCESS if ROI has been applied. Other ERROR_CODE otherwise.</returns>
        public int SetCameraSettings(VIDEO_SETTINGS settings, SIDE side, Rect roi, bool reset)
        {
            AssertCameraIsReady();
            if (settings == VIDEO_SETTINGS.AEC_AGC_ROI)
                return dllz_set_roi_for_aec_agc(CameraID, (int)side, roi, reset);
            else
                return -1;
        }

        /// <summary>
        /// Overloaded function for CAMERA_SETTINGS.AEC_AGC_ROI (requires Rect as input)
        /// </summary>
        /// <param name="settings"> Must be set to CAMERA_SETTINGS.AEC_AGC_ROI. Otherwise will return -1.</param>
        /// <param name="side"> defines left=0 or right=1 or both=2 sensor target.</param>
        /// <param name="roi"> Roi that will be filled.</param>
        /// <returns><see>ERROR_CODE.SUCCESS if ROI has been applied. Other ERROR_CODE otherwise.</returns>
        public int GetCameraSettings(VIDEO_SETTINGS settings, SIDE side, ref Rect roi)
        {
            AssertCameraIsReady();
            if (settings == VIDEO_SETTINGS.AEC_AGC_ROI)
                return dllz_get_roi_for_aec_agc(CameraID, (int)side, ref roi);
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

            SetCameraSettings(sl.VIDEO_SETTINGS.BRIGHTNESS, sl.Camera.brightnessDefault);
            SetCameraSettings(sl.VIDEO_SETTINGS.CONTRAST, sl.Camera.contrastDefault);
            SetCameraSettings(sl.VIDEO_SETTINGS.HUE, sl.Camera.hueDefault);
            SetCameraSettings(sl.VIDEO_SETTINGS.SATURATION, sl.Camera.saturationDefault);
            SetCameraSettings(sl.VIDEO_SETTINGS.SHARPNESS, sl.Camera.sharpnessDefault);
            SetCameraSettings(sl.VIDEO_SETTINGS.GAMMA, sl.Camera.gammaDefault);
            SetCameraSettings(sl.VIDEO_SETTINGS.AUTO_WHITEBALANCE, 1);
            SetCameraSettings(sl.VIDEO_SETTINGS.AEC_AGC, 1);
            SetCameraSettings(sl.VIDEO_SETTINGS.LED_STATUS, 1);

            SetCameraSettings(sl.VIDEO_SETTINGS.AEC_AGC_ROI, SIDE.BOTH, new sl.Rect(), true);
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
        /// Sets the position of the SVO file currently being read to a desired frame.
        /// </summary>
        /// <param name="frame">Index of the desired frame to be decoded.</param>
        public void SetSVOPosition(int frame)
        {
            dllz_set_svo_position(CameraID, frame);
        }

        /// <summary>
        /// Returns the current camera FPS. This is limited primarily by resolution but can also be lower due to
        /// setting a lower desired resolution in Open() or from USB connection/bandwidth issues.
        /// </summary>
        /// <returns>The current fps</returns>
        public float GetCameraFPS()
        {
            return dllz_get_camera_fps(CameraID);
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

        ///@}

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
                throw new Exception("ZED camera is not connected or Open() was not called.");

            if (!pluginIsReady)
                throw new Exception("Could not resolve ZED plugin dependencies.");

        }

        ///////////////////////////// SINGLE PIXEL UTILITY FUNCTIONS ////////////////////////////////

        /// <summary>
        /// Gets the current depth value of a pixel in the UNITS specified when the camera was started with Open().
        /// May result in errors if the ZED image does not fill the whole screen.
        /// </summary>
        /// <param name="posX">x component of the pixel position</param>
        /// <param name="posY">y component of the pixel position</param>
        /// <returns>Depth value as a float.</returns>
        public float GetDepthValue(int posX, int posY)
        {
            if (!cameraReady)
            {
                return -1;
            }

            float d = dllz_get_depth_value(CameraID, (uint)posX, (uint)posY);
            return d;
        }
        /// <summary>
        ///  Gets the current Euclidean distance (sqrt(x²+y²+z²)) of the targeted pixel of the screen to the camera.
        /// May result in errors if the ZED image does not fill the whole screen.
        /// </summary>
        /// <param name="posX">x component of the pixel position</param>
        /// <param name="posY">y component of the pixel position</param>
        /// <returns>Distance as a float.</returns>
        public float GetDistanceValue(int posX, int posY)
        {
            if (!cameraReady) //Do nothing if the ZED isn't initialized.
            {
                return -1;
            }

            return dllz_get_distance_value(CameraID, (uint)posX, (uint)posY);
        }
        /// <summary>
        /// Gets the position of a camera-space pixel relative to the camera frame.
        /// </summary>
        /// <param name="posX">x component of the pixel position</param>
        /// <param name="posY">y component of the pixel position</param>
        /// <param name="xyz">Position relative to the camera.</param>
        /// <returns>True if successful.</returns>
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
        /// <summary>
        /// Gets the normal of a camera-space pixel. The normal is relative to the camera.
        /// </summary>
        /// <param name="posX">x component of the pixel position</param>
        /// <param name="posY">y component of the pixel position</param>
        /// <param name="normal">Normal value of the pixel as a Vector4.</param>
        /// <returns>True if successful.</returns>
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

        ///@{
        /// @name Depth Sensing

        /// <summary>
        /// Retrieves a measure texture from the ZED SDK and loads it into a ZEDMat. Use this to get an individual
        /// texture from the last grabbed frame with measurements in every pixel - such as a depth map, confidence map, etc.
        /// Measure textures are not human-viewable but don't lose accuracy, unlike image textures.
        /// </summary><remarks>
        /// If you want to access the texture via script, you'll usually want to specify CPU memory. Then you can use
        /// Marshal.Copy to move them into a new byte array, which you can load into a Texture2D.
        /// RetrieveMeasure() calls Camera::retrieveMeasure() in the C++ SDK. For more info, read:
        /// https://www.stereolabs.com/docs/api/classsl_1_1Camera.html#a9e0773c0c14ce5156c1fa2fde501c13e
        /// </remarks>
        /// <param name="mat">ZEDMat to fill with the new texture.</param>
        /// <param name="measure">Measure type (depth, confidence, xyz, etc.)</param>
        /// <param name="mem">Whether the image should be on CPU or GPU memory.</param>
        /// <param name="resolution">Resolution of the texture.</param>
        /// <returns>Error code indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveMeasure(sl.Mat mat, sl.MEASURE measure, sl.MEM mem = sl.MEM.CPU, sl.Resolution resolution = new sl.Resolution())
        {
            return (sl.ERROR_CODE)(dllz_retrieve_measure(CameraID, mat.MatPtr, (int)measure, (int)mem, resolution));
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

        ///@}

        ///@{
        /// @name Positional Tracking

        /// <summary>
        /// Initialize and Start the tracking functions
        /// </summary>
        /// <param name="quat"> rotation used as initial world transform. By default it should be identity.</param>
        /// <param name="vec"> translation used as initial world transform. By default it should be identity.</param>
        /// <param name="enableSpatialMemory">  (optional) define if spatial memory is enable or not.</param>
        /// <param name="areaFilePath"> (optional) file of spatial memory file that has to be loaded to relocate in the scene.</param>
        /// <returns> ERROR_CODE.FAILURE if the area_file_path file wasn't found, SUCCESS otherwise.</returns>
        public sl.ERROR_CODE EnablePositionalTracking(ref Quaternion quat, ref Vector3 vec, bool enableAreaMemory = true, bool enablePoseSmoothing = false, bool setFloorAsOrigin = false, bool setAsStatic = false,
            bool enableIMUFusion = true, string areaFilePath = "")
        {
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            trackingStatus = (sl.ERROR_CODE)dllz_enable_tracking(CameraID, ref quat, ref vec, enableAreaMemory, enablePoseSmoothing, setFloorAsOrigin,
                setAsStatic, enableIMUFusion, new System.Text.StringBuilder(areaFilePath, areaFilePath.Length));
            return trackingStatus;
        }
        /// <summary>
        /// Initialize and Start the tracking functions
        /// </summary>
        /// <param name="positionalTrackingParameters"> struct that contains all positional tracking parameters</param>
        /// <returns>ERROR_CODE.FAILURE if the area_file_path file wasn't found, SUCCESS otherwise.</returns>
        public sl.ERROR_CODE EnablePositionalTracking(ref PositionalTrackingParameters positionalTrackingParameters)
        {
            return EnablePositionalTracking(ref positionalTrackingParameters.initialWorldRotation, ref positionalTrackingParameters.initialWorldPosition, positionalTrackingParameters.enableAreaMemory,
                                            positionalTrackingParameters.enablePoseSmothing, positionalTrackingParameters.setFloorAsOrigin, positionalTrackingParameters.setAsStatic, positionalTrackingParameters.enableIMUFusion,
                                            positionalTrackingParameters.areaFilePath);
        }

        /// <summary>
        ///  Stop the motion tracking, if you want to restart, call enableTracking().
        /// </summary>
        /// <param name="path">The path to save the area file</param>
        public void DisablePositionalTracking(string path = "")
        {
            dllz_disable_tracking(CameraID, new System.Text.StringBuilder(path, path.Length));
        }

        /// <summary>
        /// Reset tracking
        /// </summary>
        /// <param name="rotation"></param>
        /// <param name="translation"></param>
        /// <returns></returns>
        public sl.ERROR_CODE ResetPositionalTracking(Quaternion rotation, Vector3 translation)
        {
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            trackingStatus = (sl.ERROR_CODE)dllz_reset_tracking(CameraID, rotation, translation);
            return trackingStatus;
        }

        public SensorsConfiguration GetSensorsConfiguration()
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
        /// Gets the position of the camera and the current state of the ZED Tracking.
        /// </summary>
        /// <param name="rotation">Quaternion filled with the current rotation of the camera depending on its reference frame.</param>
        /// <param name="position">Vector filled with the current position of the camera depending on its reference frame.</param>
        /// <param name="referenceType">Reference frame for setting the rotation/position. CAMERA gives movement relative to the last pose.
        /// WORLD gives cumulative movements since tracking started.</param>
        /// <returns>State of ZED's Tracking system (off, searching, ok).</returns>
        public POSITIONAL_TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 position, REFERENCE_FRAME referenceType = REFERENCE_FRAME.WORLD)
        {
            return (POSITIONAL_TRACKING_STATE)dllz_get_position(CameraID, ref rotation, ref position, (int)referenceType);
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
        public POSITIONAL_TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation, REFERENCE_FRAME referenceFrame = REFERENCE_FRAME.WORLD)
        {
            return (POSITIONAL_TRACKING_STATE)dllz_get_position_at_target_frame(CameraID, ref rotation, ref translation, ref targetQuaternion, ref targetTranslation, (int)referenceFrame);
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
        public POSITIONAL_TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 translation, TRACKING_FRAME trackingFrame, REFERENCE_FRAME referenceFrame = REFERENCE_FRAME.WORLD)
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

            return (POSITIONAL_TRACKING_STATE)dllz_get_position_at_target_frame(CameraID, ref rotation, ref translation, ref rotationOffset, ref positionOffset, (int)referenceFrame);
        }

        /// <summary>
        /// Gets the current position of the camera and state of the tracking, filling a Pose struct useful for AR pass-through.
        /// </summary>
        /// <param name="pose">Current pose.</param>
        /// <param name="referenceType">Reference frame for setting the rotation/position. CAMERA gives movement relative to the last pose.
        /// WORLD gives cumulative movements since tracking started.</param>
        /// <returns>State of ZED's Tracking system (off, searching, ok).</returns>
        public POSITIONAL_TRACKING_STATE GetPosition(ref Pose pose, REFERENCE_FRAME referenceType = REFERENCE_FRAME.WORLD)
        {
            return (POSITIONAL_TRACKING_STATE)dllz_get_position_data(CameraID, ref pose, (int)referenceType);
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
        public ERROR_CODE GetIMUOrientation(ref Quaternion rotation, TIME_REFERENCE referenceTime = TIME_REFERENCE.IMAGE)
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
        public ERROR_CODE GetSensorsData(ref SensorsData data, TIME_REFERENCE referenceTime = TIME_REFERENCE.IMAGE)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            err = (sl.ERROR_CODE)dllz_get_internal_sensors_data(CameraID, ref data, (int)referenceTime);
            return err;
        }

        ///@}

        ///@{
        /// @name Spatial Mapping


        ///////////////////////////// SPATIAL MAPPING ////////////////////////////////

        /// <summary>
        /// Initializes and begins the spatial mapping processes.
        /// </summary>
        /// <param name="spatialMappingParameters">Spatial mapping parameters.</param>
        /// <returns>SUCCES if everything went fine, FAILURE otherwise</returns>
        public sl.ERROR_CODE EnableSpatialMapping(ref SpatialMappingParameters spatialMappingParameters)
        {
            return EnableSpatialMapping(spatialMappingParameters.map_type, spatialMappingParameters.resolutionMeter, spatialMappingParameters.rangeMeter,
                    spatialMappingParameters.saveTexture);
        }

        /// <summary>
        /// Initializes and begins the spatial mapping processes.
        /// </summary>
        /// <param name="resolutionMeter">Spatial mapping resolution in meters.</param>
        /// <param name="maxRangeMeter">Maximum scanning range in meters.</param>
        /// <param name="saveTexture">True to scan surface textures in addition to geometry.</param>
        /// <returns>SUCCES if everything went fine, FAILURE otherwise</returns>
        public sl.ERROR_CODE EnableSpatialMapping(SPATIAL_MAP_TYPE type, float resolutionMeter, float maxRangeMeter, bool saveTexture = false)
        {
            sl.ERROR_CODE spatialMappingStatus = ERROR_CODE.FAILURE;
            //lock (grabLock)
            {
                spatialMappingStatus = (sl.ERROR_CODE)dllz_enable_spatial_mapping(CameraID, (int)type, resolutionMeter, maxRangeMeter, System.Convert.ToInt32(saveTexture), 4096);
            }
            return spatialMappingStatus;
        }

        /// <summary>
        /// Initializes and begins the spatial mapping processes.
        /// </summary>
        /// <param name="resolutionMeter">Spatial mapping resolution in meters.</param>
        /// <param name="maxRangeMeter">Maximum scanning range in meters.</param>
        /// <param name="saveTexture">True to scan surface textures in addition to geometry.</param>
        /// <returns>SUCCES if everything went fine, FAILURE otherwise</returns>
        public sl.ERROR_CODE EnableSpatialMapping(SPATIAL_MAP_TYPE type = SPATIAL_MAP_TYPE.MESH, MAPPING_RESOLUTION mappingResolution = MAPPING_RESOLUTION.MEDIUM, MAPPING_RANGE mappingRange = MAPPING_RANGE.MEDIUM, bool saveTexture = false)
        {
            return EnableSpatialMapping(type, ConvertResolutionPreset(mappingResolution), ConvertRangePreset(mappingRange), saveTexture);
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
        /// <param name="nbUpdatedSubmesh">Number of updated submeshes.</param>
        /// <param name="updatedIndices">List of all submeshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all submeshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all submeshes.</param>
        /// <param name="nbSubmeshMax">Maximum number of submeshes that can be handled.</param>
        /// <returns>Error code indicating if the update was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE UpdateMesh(int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbUpdatedSubmesh, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmeshMax)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;
            err = (sl.ERROR_CODE)dllz_update_mesh(CameraID, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbUpdatedSubmesh, updatedIndices, ref nbVertices, ref nbTriangles, nbSubmeshMax);
            return err;
        }

        /// <summary>
        /// Updates the internal version of the mesh and returns the sizes of the meshes.
        /// </summary>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns>Error code indicating if the update was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE UpdateMesh(ref Mesh mesh)
        {
            ERROR_CODE err = UpdateMesh(mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles, (int)Constant.MAX_SUBMESH);

            return err;
        }

        /// <summary>
        /// Retrieves all chunks of the current generated mesh. Call UpdateMesh() before calling this.
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
             
        /// <summary>
        /// Retrieves all chunks of the current generated mesh. Call UpdateMesh() before calling this.
        /// Vertex and triangle arrays must be at least of the sizes returned by UpdateMesh (nbVertices and nbTriangles). 
        /// </summary>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns>Error code indicating if the update was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveMesh(ref Mesh mesh)
        {
            ERROR_CODE err = RetrieveMesh(mesh.vertices, mesh.triangles, (int)Constant.MAX_SUBMESH, mesh.uvs, mesh.textures);
            int verticesOffset = 0;
            int trianglesOffset = 0;

            for (int i = 0; i < mesh.nbUpdatedSubmesh; i++)
            {
                SetMesh(ref mesh, i, ref verticesOffset, ref trianglesOffset);
            }
            return err;
        }
        /// <summary>
        /// Retrieve all chunks of the generated mesh.Call UpdateMesh() before calling this. Used for mesh vizualisation.
        /// </summary>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns></returns>
        public sl.ERROR_CODE RetrieveChunks(ref Mesh mesh)
        {
            dllz_update_chunks(CameraID, mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles, (int)Constant.MAX_SUBMESH);

            mesh.vertices = new Vector3[mesh.nbVertices];
            mesh.triangles = new int[mesh.nbTriangles * 3];

            ERROR_CODE err = (sl.ERROR_CODE)dllz_retrieve_chunks(CameraID, (int)Constant.MAX_SUBMESH, mesh.vertices, mesh.triangles);
            int verticesOffset = 0;
            int trianglesOffset = 0;

            for (int i = 0; i < mesh.nbUpdatedSubmesh; i++)
            {
                SetMesh(ref mesh, i, ref verticesOffset, ref trianglesOffset);
            }
            return err;

        }
        /// <summary>
        /// Process data from a submesh retrieved from the ZED SDK into a chunk
        /// </summary>
        /// <param name="mesh"> Mesh data retrieved from the zed sdk</param>
        /// <param name="indexUpdate">Index of the submesh/chunk to be updated.</param>
        /// <param name="verticesOffset">Starting index in the vertices stack.</param>
        /// <param name="trianglesOffset">Starting index in the triangles stack.</param>
        /// <param name="uvsOffset">Starting index in the UVs stack.</param>
        private void SetMesh(ref Mesh mesh, int indexUpdate, ref int verticesOffset, ref int trianglesOffset)
        {
            if (!mesh.chunks.TryGetValue(indexUpdate, out Chunk subMesh)) //Use the existing chunk/submesh if already in the dictionary. Otherwise, make a new one. 
            {
                subMesh = new Chunk();
                mesh.chunks.Add(indexUpdate, subMesh);
            }
            //If the dynamicMesh's triangle and vertex arrays are unassigned or are the wrong size, redo the array. 
            if (subMesh.triangles == null || subMesh.triangles.Length != 3 * mesh.nbTrianglesInSubmesh[indexUpdate])
            {
                subMesh.triangles = new int[3 * mesh.nbTrianglesInSubmesh[indexUpdate]];
            }
            if (subMesh.vertices == null || subMesh.vertices.Length != mesh.nbVerticesInSubmesh[indexUpdate])
            {
                subMesh.vertices = new Vector3[mesh.nbVerticesInSubmesh[indexUpdate]];
            }

            //Clear the old mesh data. 
            Array.Clear(subMesh.vertices, 0, subMesh.vertices.Length);
            Array.Clear(subMesh.triangles, 0, subMesh.triangles.Length);

            //Copy data retrieved from the ZED SDK into the current chunk. 
            System.Array.Copy(mesh.vertices, verticesOffset, subMesh.vertices, 0, mesh.nbVerticesInSubmesh[indexUpdate]);
            verticesOffset += mesh.nbVerticesInSubmesh[indexUpdate];
            System.Buffer.BlockCopy(mesh.triangles, trianglesOffset * sizeof(int), subMesh.triangles, 0, 3 * mesh.nbTrianglesInSubmesh[indexUpdate] * sizeof(int)); //Block copy has better performance than Array.
            trianglesOffset += 3 * mesh.nbTrianglesInSubmesh[indexUpdate];

            mesh.chunks[indexUpdate] = subMesh;
        }

        /// <summary>
        /// Retrieves the current generated mesh.
        /// </summary>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns></returns>
        public sl.ERROR_CODE RetrieveSpatialMap(ref Mesh mesh)
        {
            UpdateMesh(ref mesh);
            //Resize the mesh buffer according to how many vertices are needed.
            mesh.vertices = new Vector3[mesh.nbVertices]; //Allocation is faster than resizing.
            mesh.triangles = new int[mesh.nbTriangles * 3];
            return RetrieveMesh(ref mesh);
        }
        /// <summary>
        /// Retrieves the current fused point cloud.
        /// </summary>
        /// <param name="fusedPointCloud">The Fused Point Cloud to be filled with the generated spatial map.</param>
        /// <returns></returns>
        public sl.ERROR_CODE RetrieveSpatialMap(ref FusedPointCloud fusedPointCloud)
        {
            int nbVertices = 0;
            UpdateFusedPointCloud(ref nbVertices);
            // Resize the vertice array according to how many vertices are needed.
            fusedPointCloud.vertices = new Vector4[nbVertices];
            if (nbVertices > 0)
                return RetrieveFusedPointCloud(fusedPointCloud.vertices);
            else
                return ERROR_CODE.FAILURE;
        }

        /// <summary>
        /// Updates the fused point cloud (if spatial map type was FUSED_POINT_CLOUD)
        /// </summary>
        /// <returns>Error code indicating if the update was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE UpdateFusedPointCloud(ref int nbVertices)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;
            err = (sl.ERROR_CODE)dllz_update_fused_point_cloud(CameraID, ref nbVertices);
            return err;
        }

        /// <summary>
        /// Retrieves all points of the fused point cloud. Call UpdateFusedPointCloud() before calling this.
        /// Vertex arrays must be at least of the sizes returned by UpdateFusedPointCloud
        /// </summary>
        /// <param name="vertices">Points of the fused point cloud.</param>
        /// <returns>Error code indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveFusedPointCloud(Vector4[] vertices)
        {
            return (sl.ERROR_CODE)dllz_retrieve_fused_point_cloud(CameraID, vertices);
        }

        ///Extracts the current spatial map from the spatial mapping process.
        ///If the object to be filled already contains a previous version of the mesh, only changes will be updated, optimizing performance.
        ///return \ref SUCCESS if the mesh is filled and available, otherwise \ref ERROR_CODE::FAILURE.
        ///warning This is a blocking function.You should either call it in a thread or at the end of the mapping process.
        public ERROR_CODE ExtractWholeSpatialMap()
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;
            err = (sl.ERROR_CODE)dllz_extract_whole_spatial_map(CameraID);
            return err;
        }
        /// <summary>
        /// Starts the mesh generation process in a thread that doesn't block the spatial mapping process.
        /// ZEDSpatialMappingHelper calls this each time it has finished applying the last mesh update.
        /// </summary>
        public void RequestSpatialMap()
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
        /// Saves the scanned point cloud in a specific file format.
        /// </summary>
        /// <param name="filename">Path and filename of the point cloud.</param>
        /// <param name="format">File format (extension). Can be .obj, .ply or .bin.</param>
        public bool SavePointCloud(string filename, MESH_FILE_FORMAT format)
        {
            return dllz_save_point_cloud(CameraID, filename, format);
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
        /// <returns>True if the filtering was successful, false otherwise.</returns>
        public bool FilterMesh(MESH_FILTER filterParameters, int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmeshMax)
        {
            return dllz_filter_mesh(CameraID, filterParameters, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, nbSubmeshMax);
        }
        /// <summary>
        /// Filters a mesh to remove triangles while still preserving its overall shape (though less accurate).
        /// </summary>
        /// <param name="filterParameters">Filter level. Higher settings remove more triangles.</param>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns>True if the filtering was successful, false otherwise.</returns>
        public bool FilterMesh(MESH_FILTER filterParameters, ref Mesh mesh)
        {
            return dllz_filter_mesh(CameraID, filterParameters, mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles,
                                    (int)Constant.MAX_SUBMESH);
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

        public bool ApplyTexture(ref Mesh mesh)
        {
            return dllz_apply_texture(CameraID, mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles, mesh.texturesSize, (int)Constant.MAX_SUBMESH);
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

        ///@}

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
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="screenPos">Point on the ZED image to check for a plane.</param>
        /// <returns></returns>
        public sl.ERROR_CODE findPlaneAtHit(ref PlaneData plane, Vector2 coord)
        {
            IntPtr p = IntPtr.Zero;
            Quaternion out_quat = Quaternion.Identity;
            Vector3 out_trans = Vector3.Zero;

            p = dllz_find_plane_at_hit(CameraID, coord, false);
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

        ///@{
        /// @name Recording

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
        public ERROR_CODE EnableRecording(string videoFileName, SVO_COMPRESSION_MODE compressionMode = SVO_COMPRESSION_MODE.H264_BASED, uint bitrate = 0, int targetFPS = 0, bool transcode = false)
        {
            return (ERROR_CODE)dllz_enable_recording(CameraID, StringUtf8ToByte(videoFileName), (int)compressionMode, bitrate, targetFPS, transcode);
        }

        public ERROR_CODE EnableRecording(RecordingParameters recordingParameters)
        {
            return (ERROR_CODE)dllz_enable_recording(CameraID, StringUtf8ToByte(recordingParameters.videoFilename), (int)recordingParameters.compressionMode, recordingParameters.bitrate,
                    recordingParameters.targetFPS, recordingParameters.transcode);
        }

        /// <summary>
        /// Stops recording to an SVO/AVI, if applicable, and closes the file.
        /// </summary>
        public bool DisableRecording()
        {
            return dllz_disable_recording(CameraID);
        }

        ///@}

        ///@{
        /// @name Streaming

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
        public ERROR_CODE EnableStreaming(STREAMING_CODEC codec = STREAMING_CODEC.H264_BASED, uint bitrate = 8000, ushort port = 30000, int gopSize = -1, bool adaptativeBitrate = false, int chunkSize = 32768, int targetFPS = 0)
        {
            int doAdaptBitrate = adaptativeBitrate ? 1 : 0;
            return (ERROR_CODE)dllz_enable_streaming(CameraID, codec, bitrate, port, gopSize, doAdaptBitrate, chunkSize, targetFPS);
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

        ///@}

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
        public sl.ERROR_CODE SaveCurrentDepthInFile(SIDE side, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_depth(CameraID, (int)side, filename);
            return err;
        }

        /// <summary>
        /// Save the current point cloud in a file defined by filename.
        /// Supported formats are PLY,VTK,XYZ and PCD 
        /// </summary>
        /// <param name="side">defines left (0) or right (1) point cloud</param>
        /// <param name="filename"> filename must end with .ply, .xyz , .vtk or .pcd </param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE SaveCurrentPointCloudInFile(SIDE side, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_point_cloud(CameraID, (int)side, filename);
            return err;
        }

        ///@{
        /// @name Object Detection


        ////////////////////////
        /// Object detection ///
        ////////////////////////

        /// <summary>
        /// Enable object detection module
        /// </summary>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE EnableObjectDetection(ref ObjectDetectionParameters od_params)
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
        public void DisableObjectDetection()
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
        public void PauseObjectDetection(bool status)
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
        public sl.ERROR_CODE RetrieveObjects(ref Objects objs, ref ObjectDetectionRuntimeParameters od_params)
        {
            return (sl.ERROR_CODE)dllz_retrieve_objects_data(CameraID, ref od_params, ref objs);
        }

        ///@}

    }

}
