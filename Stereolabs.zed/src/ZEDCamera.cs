//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Numerics;

namespace sl
{
    ///\ingroup  Video_group
    /// <summary>
    /// This class serves as the primary interface between the camera and the various features provided by the SDK.
    ///
    /// It enables seamless integration and access to a wide array of capabilities, including video streaming, depth sensing, object tracking, mapping, and much more.
    /// </summary>
    public class Camera
    {
        /********* Camera members ********/

        /// <summary>
        /// DLL name, used for extern calls to the wrapper.
        /// </summary>
        const string nameDll = sl.ZEDCommon.NameDLL;

        /// <summary>
        /// Width of the textures in pixels.
        ///
        /// It corresponds to the camera's current resolution setting.
        /// </summary>
        private int imageWidth;
        /// <summary>
        /// Width of the images returned by the camera in pixels.
        ///
        /// It corresponds to the camera's current resolution setting.
        /// </summary>
        public int ImageWidth
        {
            get
            {
                return imageWidth;
            }
        }

        private const float Deg2Rad = 0.0174532924F;
        private const float Rad2Deg = 57.29578F;

        /// <summary>
        /// Height of the textures in pixels.
        ///
        /// It corresponds to the camera's current resolution setting.
        /// </summary>
        private int imageHeight;
        /// <summary>
        /// Height of the images returned by the camera in pixels.
        ///
        /// It corresponds to the camera's current resolution setting.
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
        /// Baseline of the camera (distance between the cameras).
        ///
        /// Extracted from calibration files.
        /// </summary>
        private float baseline = 0.0f;
        /// <summary>
        /// Baseline of the camera (distance between the cameras).
        ///
        /// Extracted from calibration files.
        /// </summary>
        public float Baseline
        {
            get { return baseline; }
        }
        /// <summary>
        /// Current horizontal field of view in degrees of the camera.
        /// </summary>
        private float fov_H = 0.0f;
        /// <summary>
        /// Current vertical field of view in degrees of the camera.
        /// </summary>
        private float fov_V = 0.0f;
        /// <summary>
        /// Current horizontal field of view in degrees of the camera.
        /// </summary>
        public float HorizontalFieldOfView
        {
            get { return fov_H; }
        }
        /// <summary>
        /// Current vertical field of view in degrees of the camera.
        /// </summary>
        public float VerticalFieldOfView
        {
            get { return fov_V; }
        }
        /// <summary>
        /// Structure containing information about all the sensors available in the current device.
        /// </summary>
        private SensorsConfiguration sensorsConfiguration;
        /// <summary>
        /// Stereo parameters for the current camera prior to rectification (distorted).
        /// </summary>
        private CalibrationParameters calibrationParametersRaw;
        /// <summary>
        /// Stereo parameters for the current camera after rectification (undistorted).
        /// </summary>
        private CalibrationParameters calibrationParametersRectified;
        /// <summary>
        /// Model of the camera.
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
        /// Model of the camera.
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

        private const int brightnessDefault = 4;
        private const int contrastDefault = 4;
        private const int hueDefault = 0;
        private const int saturationDefault = 4;
        private const int sharpnessDefault = 3;
        private const int gammaDefault = 5;
        private const int whitebalanceDefault = 2600;

        #region DLL Calls

        /*
          * Utils function.
        */
        [DllImport(nameDll, EntryPoint = "sl_free")]
        public static extern void dllz_free(IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_unload_all_instances")]
        private static extern void dllz_unload_all_instances();

        [DllImport(nameDll, EntryPoint = "sl_unload_instance")]
        private static extern void dllz_unload_instance(int id);

        [DllImport(nameDll, EntryPoint = "sl_generate_unique_id")]
        private static extern int dllz_generate_unique_id([In, Out] byte[] id);

        /*
          * Create functions
          */
        [DllImport(nameDll, EntryPoint = "sl_create_camera")]
        private static extern bool dllz_create_camera(int cameraID);


        /*
        * Opening function (Opens camera and creates textures).
        */
        [DllImport(nameDll, EntryPoint = "sl_open_camera")]
        private static extern int dllz_open(int cameraID, ref sl_initParameters parameters, uint serialNumber, System.Text.StringBuilder svoPath, System.Text.StringBuilder ipStream, int portStream, int gmslPort, System.Text.StringBuilder output, System.Text.StringBuilder opt_settings_path, System.Text.StringBuilder opencv_calib_path);

        [DllImport(nameDll, EntryPoint = "sl_start_publishing")]
        private static extern ERROR_CODE dllz_start_publishing(int cameraID, ref CommunicationParameters commParams);

        [DllImport(nameDll, EntryPoint = "sl_stop_publishing")]
        private static extern ERROR_CODE dllz_stop_publishing(int cameraID);

        /*
         * Close function.
         */
        [DllImport(nameDll, EntryPoint = "sl_close_camera")]
        private static extern void dllz_close(int cameraID);


        /*
         * Grab function.
         */
        [DllImport(nameDll, EntryPoint = "sl_grab")]
        private static extern int dllz_grab(int cameraID, ref sl_RuntimeParameters runtimeParameters);

        /*
         * GetDeviceList function
         */
        [DllImport(nameDll, EntryPoint = "sl_get_device_list")]
        private static extern void dllz_get_device_list([Out] sl.DeviceProperties[] deviceList, out int nbDevices);

        /*
         * GetStreamingDeviceList function
         */
        [DllImport(nameDll, EntryPoint = "sl_get_streaming_device_list")]
        private static extern void dllz_get_streaming_device_list([Out] sl.StreamingProperties[] streamingDeviceList, out int nbDevices);

        /*
         * Reboot function.
         */
        [DllImport(nameDll, EntryPoint = "sl_reboot")]
        private static extern int dllz_reboot(int serialNumber, bool fullReboot);

        /*
        * Recording functions.
        */
        [DllImport(nameDll, EntryPoint = "sl_enable_recording")]
        private static extern int dllz_enable_recording(int cameraID, byte[] video_filename, int compresssionMode, uint bitrate, int target_fps, bool transcode);

        [DllImport(nameDll, EntryPoint = "sl_get_recording_status")]
        private static extern IntPtr dllz_get_recording_status(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_recording_parameters")]
        private static extern IntPtr dllz_get_recording_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_disable_recording")]
        private static extern void dllz_disable_recording(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_pause_recording")]
        private static extern void dllz_pause_recording(int cameraID, bool status);

        // Recording Gen 2 functions

        [DllImport(nameDll, EntryPoint = "sl_ingest_data_into_svo")]
        private static extern ERROR_CODE dllz_ingest_data_into_svo(int cameraID, ref SVOData data);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_data_size")]
        private static extern int dllz_get_svo_data_size(int cameraID, string key, ulong ts_begin, ulong ts_end);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_svo_data")]
        private static extern ERROR_CODE dllz_retrieve_svo_data(int cameraID, string key, int nb_data, out IntPtr data, ulong ts_begin, ulong ts_end);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_data_keys_size")]
        private static extern int dllz_get_svo_data_keys_size(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_data_keys")]
        private static extern void dllz_get_svo_data_keys(int cameraID, int nb_keys, out IntPtr keys);

        /*
         * Camera control functions.
         */

        [DllImport(nameDll, EntryPoint = "sl_is_camera_setting_supported")]
        private static extern bool dllz_is_camera_setting_supported(int id, VIDEO_SETTINGS setting);

        [DllImport(nameDll, EntryPoint = "sl_set_camera_settings")]
        private static extern ERROR_CODE dllz_set_camera_settings(int id, int mode, int value);

        [DllImport(nameDll, EntryPoint = "sl_set_camera_settings_min_max")]
        private static extern ERROR_CODE dllz_set_camera_settings_min_max(int id, int mode, int minvalue, int maxvalue);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_settings")]
        private static extern ERROR_CODE dllz_get_camera_settings(int id, VIDEO_SETTINGS settingToRetrieve, ref int value);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_settings_min_max")]
        private static extern ERROR_CODE dllz_get_camera_settings_min_max(int id, VIDEO_SETTINGS settingToRetrieve, ref int minvalue, ref int maxvalue);

        [DllImport(nameDll, EntryPoint = "sl_set_roi_for_aec_agc")]
        private static extern ERROR_CODE dllz_set_roi_for_aec_agc(int id, int side, Rect roi,bool reset);

        [DllImport(nameDll, EntryPoint = "sl_get_roi_for_aec_agc")]
        private static extern ERROR_CODE dllz_get_roi_for_aec_agc(int id, int side, ref Rect roi);

        [DllImport(nameDll, EntryPoint = "sl_get_input_type")]
        private static extern int dllz_get_input_type(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_fps")]
        private static extern float dllz_get_camera_fps(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_is_opened")]
        private static extern bool dllz_is_opened(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_width")]
        private static extern int dllz_get_width(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_height")]
        private static extern int dllz_get_height(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_update_self_calibration")]
        private static extern IntPtr dllz_update_self_calibration(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_calibration_parameters")]
        private static extern IntPtr dllz_get_calibration_parameters(int cameraID, bool raw);

        [DllImport(nameDll, EntryPoint = "sl_get_sensors_configuration")]
        private static extern IntPtr dllz_get_sensors_configuration(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_information")]
        private static extern IntPtr dllz_get_camera_information(int cameraID, int width, int height);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_model")]
        private static extern int dllz_get_camera_model(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_firmware")]
        private static extern int dllz_get_camera_firmware(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_sensors_firmware")]
        private static extern int dllz_get_sensors_firmware(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_zed_serial")]
        private static extern int dllz_get_zed_serial(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_imu_transform")]
        private static extern void dllz_get_camera_imu_transform(int cameraID, out Vector3 translation, out Quaternion rotation);

        [DllImport(nameDll, EntryPoint = "sl_get_image_timestamp")]
        private static extern ulong dllz_get_image_timestamp(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_current_timestamp")]
        private static extern ulong dllz_get_current_timestamp(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_frame_dropped_count")]
        private static extern uint dllz_get_frame_dropped_count(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_init_parameters")]
        private static extern IntPtr dllz_get_init_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_runtime_parameters")]
        private static extern IntPtr dllz_get_runtime_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_positional_tracking_parameters")]
        private static extern IntPtr dllz_get_positional_tracking_parameters(int cameraID);


        /*
         * SVO control functions.
         */

        [DllImport(nameDll, EntryPoint = "sl_set_svo_position")]
        private static extern ERROR_CODE dllz_set_svo_position(int cameraID, int position);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_number_of_frames")]
        private static extern int dllz_get_svo_number_of_frames(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_position")]
        private static extern int dllz_get_svo_position(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_position_at_timestamp")]
        private static extern int dllz_get_svo_position_at_timestamp(int cameraID, ulong timestamp);

        [DllImport(nameDll, EntryPoint = "sl_get_confidence_threshold")]
        private static extern int dllz_get_confidence_threshold(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_depth_max_range_value")]
        private static extern float dllz_get_depth_max_range_value(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_depth_min_range_value")]
        private static extern float dllz_get_depth_min_range_value(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_current_min_max_depth")]
        private static extern float dllz_get_current_min_max_depth(int cameraID, ref float min, ref float max);

        /*
         * Motion Tracking functions.
         */
        [DllImport(nameDll, EntryPoint = "sl_enable_positional_tracking")]
        private static extern int dllz_enable_tracking(int cameraID, ref sl_PositionalTrackingParameters tracking_params, System.Text.StringBuilder aeraFilePath = null);

        [DllImport(nameDll, EntryPoint = "sl_disable_positional_tracking")]
        private static extern void dllz_disable_tracking(int cameraID, System.Text.StringBuilder path);

        [DllImport(nameDll, EntryPoint = "sl_is_positional_tracking_enabled")]
        private static extern bool dllz_is_positional_tracking_enabled(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_save_area_map")]
        private static extern int dllz_save_area_map(int cameraID, System.Text.StringBuilder path);

        [DllImport(nameDll, EntryPoint = "sl_get_position_data")]
        private static extern int dllz_get_position_data(int cameraID, ref Pose pose, int reference_frame);

        [DllImport(nameDll, EntryPoint = "sl_get_position")]
        private static extern int dllz_get_position(int cameraID, ref Quaternion quat, ref Vector3 vec, int reference_frame);

        [DllImport(nameDll, EntryPoint = "sl_get_positional_tracking_status")]
        private static extern IntPtr dllz_get_positional_tracking_status(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_positional_tracking_landmarks")]
        private static extern int dllz_get_positional_tracking_landmarks(int cameraID, ref IntPtr landmarks, ref int count);

        [DllImport(nameDll, EntryPoint = "sl_get_positional_tracking_landmarks_2d")]
        private static extern int dllz_get_positional_tracking_landmarks_2d(int cameraID, ref IntPtr landmarks, ref int count);

        [DllImport(nameDll, EntryPoint = "sl_get_position_at_target_frame")]
        private static extern int dllz_get_position_at_target_frame(int cameraID, ref Quaternion quaternion, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation, int reference_frame);

        [DllImport(nameDll, EntryPoint = "sl_reset_positional_tracking")]
        private static extern int dllz_reset_tracking(int cameraID, Quaternion rotation, Vector3 translation);

        [DllImport(nameDll, EntryPoint = "sl_reset_positional_tracking_with_offset")]
        private static extern int dllz_reset_positional_tracking_with_offset(int cameraID, Quaternion rotation, Vector3 translation, Quaternion offsetQuaternion, Vector3 offsetTranslation);

        [DllImport(nameDll, EntryPoint = "sl_set_imu_prior_orientation")]
        private static extern int dllz_set_imu_prior_orientation(int cameraID, Quaternion rotation);

        [DllImport(nameDll, EntryPoint = "sl_get_imu_orientation")]
        private static extern int dllz_get_internal_imu_orientation(int cameraID, ref Quaternion rotation, int reference_time);

        [DllImport(nameDll, EntryPoint = "sl_get_sensors_data")]
        private static extern int dllz_get_internal_sensors_data(int cameraID, ref SensorsData imuData, int reference_time);

        [DllImport(nameDll, EntryPoint = "sl_get_sensors_data_batch_count")]
        private static extern int dllz_get_sensors_data_batch_count(int cameraID, out int count);

        [DllImport(nameDll, EntryPoint = "sl_get_sensors_data_batch")]
        private static extern int dllz_get_sensors_data_batch(int cameraID, ref SensorsData[] imuData);

        [DllImport(nameDll, EntryPoint = "sl_get_area_export_state")]
        private static extern int dllz_get_area_export_state(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_set_region_of_interest")]
        private static extern int dllz_set_region_of_interest(int cameraID, IntPtr roiMask, bool[] module);

        [DllImport(nameDll, EntryPoint = "sl_get_region_of_interest")]
        private static extern int dllz_get_region_of_interest(int cameraID, IntPtr roiMask, int width, int height, MODULE module);

        [DllImport(nameDll, EntryPoint = "sl_start_region_of_interest_auto_detection")]
        private static extern int dllz_start_region_of_interest_auto_detection(int cameraID, ref RegionOfInterestParameters roiParams);

        [DllImport(nameDll, EntryPoint = "sl_get_region_of_interest_auto_detection_status")]
        private static extern int dllz_get_region_of_interest_auto_detection_status(int cameraID);

        /*
        * Spatial Mapping functions.
        */
        [DllImport(nameDll, EntryPoint = "sl_enable_spatial_mapping")]
        private static extern int dllz_enable_spatial_mapping(int cameraID, ref sl_SpatialMappingParameters map_params);

        [DllImport(nameDll, EntryPoint = "sl_disable_spatial_mapping")]
        private static extern void dllz_disable_spatial_mapping(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_spatial_mapping_parameters")]
        private static extern IntPtr dllz_get_spatial_mapping_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_pause_spatial_mapping")]
        private static extern void dllz_pause_spatial_mapping(int cameraID, bool status);

        [DllImport(nameDll, EntryPoint = "sl_request_mesh_async")]
        private static extern void dllz_request_mesh_async(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_mesh_request_status_async")]
        private static extern int dllz_get_mesh_request_status_async(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_update_mesh")]
        private static extern int dllz_update_mesh(int cameraID, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_update_chunks")]
        private static extern int dllz_update_chunks(int cameraID, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_mesh")]
        private static extern int dllz_retrieve_mesh(int cameraID, [In, Out] Vector3[] vertices, [In, Out] int[] triangles, [In, Out] byte[] colors, [In, Out] Vector2[] uvs, IntPtr texture, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_chunks")]
        private static extern int dllz_retrieve_chunks(int cameraID, [In, Out] Vector3[] vertices, [In, Out] int[] triangles, [In, Out] byte[] colors, [In, Out] Vector2[] uvs, IntPtr texture, int maxSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_extract_whole_spatial_map")]
        private static extern int dllz_extract_whole_spatial_map(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_update_fused_point_cloud")]
        private static extern int dllz_update_fused_point_cloud(int cameraID, ref int nbPoints);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_fused_point_cloud")]
        private static extern int dllz_retrieve_fused_point_cloud(int cameraID, [In, Out] Vector4[] points);

        [DllImport(nameDll, EntryPoint = "sl_save_mesh")]
        private static extern bool dllz_save_mesh(int cameraID, string filename, MESH_FILE_FORMAT format);

        [DllImport(nameDll, EntryPoint = "sl_save_point_cloud")]
        private static extern bool dllz_save_point_cloud(int cameraID, string filename, MESH_FILE_FORMAT format);

        [DllImport(nameDll, EntryPoint = "sl_load_mesh")]
        private static extern bool dllz_load_mesh(int cameraID, string filename, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int[] textureSize = null, int nbMaxSubmesh = 1000);

        [DllImport(nameDll, EntryPoint = "sl_apply_texture")]
        private static extern bool dllz_apply_texture(int cameraID, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int[] textureSize, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_filter_mesh")]
        private static extern bool dllz_filter_mesh(int cameraID, MESH_FILTER meshFilter, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_get_spatial_mapping_state")]
        private static extern int dllz_get_spatial_mapping_state(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_spatial_mapping_merge_chunks")]
        private static extern void dllz_spatial_mapping_merge_chunks(int cameraID, int numberFaces, int[] nbVerticesInSubemeshes, int[] nbTrianglesInSubemeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_spatial_mapping_get_gravity_estimation")]
        private static extern void dllz_spatial_mapping_get_gravity_estimation(int cameraID, ref Vector3 v);

        /*
         * Plane Detection functions (starting v2.4)
         */
        [DllImport(nameDll, EntryPoint = "sl_find_floor_plane")]
        private static extern IntPtr dllz_find_floor_plane(int cameraID, out Quaternion rotation, out Vector3 translation, Quaternion priorQuaternion, Vector3 priorTranslation);

        [DllImport(nameDll, EntryPoint = "sl_find_plane_at_hit")]
        private static extern IntPtr dllz_find_plane_at_hit(int cameraID, Vector2 HitPixel, ref sl_PlaneDetectionParameters plane_params, bool refine);

        [DllImport(nameDll, EntryPoint = "sl_convert_floorplane_to_mesh")]
        private static extern int dllz_convert_floorplane_to_mesh(int cameraID, [In, Out] Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles);

        [DllImport(nameDll, EntryPoint = "sl_convert_hitplane_to_mesh")]
        private static extern int dllz_convert_hitplane_to_mesh(int cameraID, [In, Out] Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles);


        /*
         * Streaming Module functions (starting v2.8)
         */
        [DllImport(nameDll, EntryPoint = "sl_enable_streaming")]
        private static extern int dllz_enable_streaming(int cameraID, sl.STREAMING_CODEC codec, uint bitrate, ushort port, int gopSize, int adaptativeBitrate, int chunk_size, int target_fps);

        [DllImport(nameDll, EntryPoint = "sl_is_streaming_enabled")]
        private static extern int dllz_is_streaming_enabled(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_disable_streaming")]
        private static extern void dllz_disable_streaming(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_streaming_parameters")]
        private static extern IntPtr dllz_get_streaming_parameters(int cameraID);

        /*
        * Objects Detection functions (starting v3.0)
        */

        [DllImport(nameDll, EntryPoint = "sl_enable_body_tracking")]
        private static extern int dllz_enable_body_tracking(int cameraID, ref BodyTrackingParameters bodyTrackingParameters);

        [DllImport(nameDll, EntryPoint = "sl_get_body_tracking_parameters")]
        private static extern IntPtr dllz_get_body_tracking_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_disable_body_tracking")]
        private static extern IntPtr dllz_disable_body_tracking(int cameraID, uint instanceID, bool forceDisableAllInstances);

        [DllImport(nameDll, EntryPoint = "sl_check_AI_model_status")]
        private static extern IntPtr dllz_check_AI_model_status(AI_MODELS model, int gpu_id);

        [DllImport(nameDll, EntryPoint = "sl_optimize_AI_model")]
        private static extern int dllz_optimize_AI_model(AI_MODELS model, int gpu_id);

        [DllImport(nameDll, EntryPoint = "sl_enable_object_detection")]
        private static extern int dllz_enable_object_detection(int cameraID, ref ObjectDetectionParameters od_params); 

        [DllImport(nameDll, EntryPoint = "sl_get_object_detection_parameters")]
        private static extern IntPtr dllz_get_object_detection_parameters(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_set_object_detection_runtime_parameters")]
        private static extern int dllz_set_object_detection_runtime_parameters(int cameraID, ObjectDetectionRuntimeParameters od_params, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_disable_object_detection")]
        private static extern void dllz_disable_object_detection(int cameraID, uint instanceID, bool forceDisableAllInstances);

        [DllImport(nameDll, EntryPoint = "sl_ingest_custom_mask_objects")]
        private static extern int dllz_ingest_custom_mask_objects(int cameraID, int nb_objects, CustomMaskObjectData[] objects_in);

        [DllImport(nameDll, EntryPoint = "sl_ingest_custom_box_objects")]
        private static extern int dllz_ingest_custom_box_objects(int cameraID, int nb_objects, CustomBoxObjectData[] objects_in);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_custom_objects")]
        private static extern int dllz_retrieve_custom_objects(int cameraID, ref CustomObjectDetectionRuntimeParameters od_params, IntPtr objs, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_set_custom_object_detection_runtime_parameters")]
        private static extern int dllz_set_custom_object_detection_runtime_parameters(int cameraID, CustomObjectDetectionRuntimeParameters custom_od_params, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_objects")]
        private static extern int dllz_retrieve_objects_data(int cameraID, ref ObjectDetectionRuntimeParameters od_params, IntPtr objs, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_bodies")]
        private static extern int dllz_retrieve_bodies_data(int cameraID, ref BodyTrackingRuntimeParameters bt_params, IntPtr objs, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_set_body_tracking_runtime_parameters")]
        private static extern int dllz_set_body_tracking_runtime_parameters(int cameraID, BodyTrackingRuntimeParameters bt_params, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_update_objects_batch")]
        private static extern int dllz_update_objects_batch(int cameraID, out int nbBatches);

        [DllImport(nameDll, EntryPoint = "sl_get_objects_batch_csharp")]
        private static extern int dllz_get_objects_batch_data(int cameraID, int batch_index, ref int numData, ref int id, ref OBJECT_CLASS label, ref OBJECT_SUBCLASS sublabel, ref OBJECT_TRACKING_STATE trackingState,
            [In, Out] Vector3[] position, [In, Out] float[,] positionCovariances, [In, Out] Vector3[] velocities, [In, Out] ulong[] timestamps, [In, Out] Vector2[,] boundingBoxes2D, [In, Out] Vector3[,] boundingBoxes,
            [In, Out] float[] confidences, [In, Out] OBJECT_ACTION_STATE[] actionStates, [In, Out] Vector2[,] headBoundingBoxes2D, [In, Out] Vector3[,] headBoundingBoxes, [In, Out] Vector3[] headPositions);
        /*
        * Save utils function
        */
        [DllImport(nameDll, EntryPoint = "sl_save_current_image")]
        private static extern int dllz_save_current_image(int cameraID, VIEW view,string filename);

        [DllImport(nameDll, EntryPoint = "sl_save_current_depth")]
        private static extern int dllz_save_current_depth(int cameraID, int side,string filename);

        [DllImport(nameDll, EntryPoint = "sl_save_current_point_cloud")]
        private static extern int dllz_save_current_point_cloud(int cameraID, int side,  string filename);

        /*
         * Specific plugin functions
         */

        [DllImport(nameDll, EntryPoint = "sl_get_sdk_version")]
        private static extern IntPtr dllz_get_sdk_version();

        /*
         * Change the coordinate system of a transform matrix.
        */
        [DllImport(nameDll, EntryPoint = "sl_convert_coordinate_system")]
        private static extern int dllz_convert_coordinate_system(ref Quaternion rotation, ref Vector3 translation, sl.COORDINATE_SYSTEM coordSystemSrc, sl.COORDINATE_SYSTEM coordSystemDest);

        /*
         * Retrieves used by mat
         */
        [DllImport(nameDll, EntryPoint = "sl_retrieve_measure")]
        private static extern int dllz_retrieve_measure(int cameraID, System.IntPtr ptr, int type, int mem, int width, int height, IntPtr cudaStream);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_image")]
        private static extern int dllz_retrieve_image(int cameraID, System.IntPtr ptr, int type, int mem, int width, int height, IntPtr cudaStream);

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
        /// Gets the max FPS for each resolution setting.
        /// Higher FPS will cause lower GPU performance.
        /// </summary>
        /// <param name="reso"></param>
        /// <returns>The resolution</returns>
        private static uint GetFpsForResolution(RESOLUTION reso)
        {
            if (reso == RESOLUTION.HD1080) return 30;
            else if (reso == RESOLUTION.HD2K) return 15;
            else if (reso == RESOLUTION.HD720) return 60;
            else if (reso == RESOLUTION.VGA) return 100;
            return 30;
        }

        /// <summary>
        /// Generate a UUID like unique id to help identify and track AI detections.
        /// </summary>
        /// <returns>A UUID like unique id.</returns>
        public static string GenerateUniqueID()
        {
            byte[] array = new byte[37];
            int size = dllz_generate_unique_id(array);

            return new string(System.Text.Encoding.ASCII.GetChars(array));
        }

        /// <summary>
        /// Default constructor.
        ///
        /// Creates an empty Camera object.
        /// </summary>
        /// <param name="id"></param>
        public Camera(int id)
        {
            CameraID = id;
        }

        /// <summary>
        /// DLL-friendly version of InitParameters (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        struct sl_initParameters
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
            /// This sets the depth stabilizer temporal smoothing strength.
	        /// the depth stabilize smooth range is [0, 100]
            /// 0 means a low temporal smmoothing behavior(for highly dynamic scene),
            /// 100 means a high temporal smoothing behavior(for static scene)
            /// </summary>
            public int depthStabilization;
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
            [MarshalAs(UnmanagedType.I4)]
            public int sdkVerbose;
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
            /// Define a timeout in seconds after which an error is reported if the \ref open() command fails.
            /// Set to '-1' to try to open the camera endlessly without returning error in case of failure.
            /// Set to '0' to return error in case of failure at the first attempt.
            /// This parameter only impacts the LIVE mode.
            /// </summary>
            public float openTimeoutSec;
            /// <summary>
            /// Define the behavior of the automatic camera recovery during grab() function call. When async is enabled and there's an issue with the communication with the camera
            /// the grab() will exit after a short period and return the ERROR_CODE::CAMERA_REBOOTING warning.The recovery will run in the background until the correct communication is restored.
            /// When async_grab_camera_recovery is false, the grab() function is blocking and will return only once the camera communication is restored or the timeout is reached.
            /// The default behavior is synchronous (false), like previous ZED SDK versions
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool asyncGrabCameraRecovery;
            /// <summary>
            /// Define a computation upper limit to the grab frequency.
            /// This can be useful to get a known constant fixed rate or limit the computation load while keeping a short exposure time by setting a high camera capture framerate.
            /// The value should be inferior to the InitParameters::camera_fps and strictly positive.It has no effect when reading an SVO file.
            /// This is an upper limit and won't make a difference if the computation is slower than the desired compute capping fps.
            /// \note Internally the grab function always tries to get the latest available image while respecting the desired fps as much as possible.
            /// </summary>
            public float grabComputeCappingFPS;
            /// <summary>
            /// Enable or disable the image validity verification.
            /// This will perform additional verification on the image to identify corrupted data.This verification is done in the grab function and requires some computations.
            /// If an issue is found, the grab function will output a warning as sl.ERROR_CODE.CORRUPTED_FRAME.
            /// This version doesn't detect frame tearing currently.
            ///  \n default: disabled
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool enableImageValidityCheck;
            /// <summary>
            ///  Set a maximum size for all SDK output, like retrieveImage and retrieveMeasure functions.
            ///  This will override the default (0,0) and instead of outputting native image size sl::Mat, the ZED SDK will take this size as default.
	        ///A custom lower size can also be used at runtime, but not bigger.This is used for internal optimization of compute and memory allocations
	        /// The default is similar to previous version with(0,0), meaning native image size
	        /// 
	        /// \note: if maximum_working_resolution field are lower than 64, it will be interpreted as dividing scale factor;
	        /// - maximum_working_resolution = sl::Resolution(1280, 2) -> 1280 x(image_height/2) = 1280 x(half height)
	        /// - maximum_working_resolution = sl::Resolution(4, 4) -> (image_width/4) x(image_height/4) = quarter size
            /// </summary>
            public Resolution maximumWorkingResolution;

            /// <summary>
            /// Copy constructor.
            /// </summary>
            /// <param name="init"></param>
            public sl_initParameters(InitParameters init)
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
                openTimeoutSec = init.openTimeoutSec;
                asyncGrabCameraRecovery = init.asyncGrabCameraRecovery;
                grabComputeCappingFPS = init.grabComputeCappingFPS;
                enableImageValidityCheck = init.enableImageValidityCheck;
                maximumWorkingResolution = init.maximumWorkingResolution;
            }
        }

        /// <summary>
        /// DLL-friendly version of RuntimeParameters (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        struct sl_RuntimeParameters
        {
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
            /// Defines if the depth map should be completed or not, similar to the removed SENSING_MODE::FILL.
            /// Warning: Enabling this will override the confidence values confidenceThreshold and textureConfidenceThreshold as well as removeSaturatedAreas
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool enableFillMode;
            /// <summary>
            ///  Defines the confidence threshold for the depth. Based on stereo matching score.
            /// </summary>
            public int confidenceThreshold;
            /// <summary>
            /// Defines texture confidence threshold for the depth. Based on textureness confidence.
            /// </summary>
            public int textureConfidenceThreshold;
            /// <summary>
            /// Defines if the saturated area (Luminance>=255) must be removed from depth map estimation
            /// </summary>
            [MarshalAs(UnmanagedType.U1)]
            public bool removeSaturatedAreas;

            /// <summary>
            /// Copy constructor.
            /// </summary>
            public sl_RuntimeParameters(RuntimeParameters rt)
            {
                measure3DReferenceFrame = rt.measure3DReferenceFrame;
                enableDepth = rt.enableDepth;
                enableFillMode = rt.enableFillMode;
                confidenceThreshold = rt.confidenceThreshold;
                textureConfidenceThreshold = rt.textureConfidenceThreshold;
                removeSaturatedAreas = rt.removeSaturatedAreas;
            }
        }

        /// <summary>
        /// DLL-friendly version of PositionalTrackingParameters (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        struct sl_PositionalTrackingParameters
        {
            public Quaternion initialWorldRotation;
            public Vector3 initialWorldPosition;
            [MarshalAs(UnmanagedType.U1)]
            public bool enableAreaMemory;
            [MarshalAs(UnmanagedType.U1)]
            public bool enablePoseSmothing;
            [MarshalAs(UnmanagedType.U1)]
            public bool setFloorAsOrigin;
            [MarshalAs(UnmanagedType.U1)]
            public bool setAsStatic;
            [MarshalAs(UnmanagedType.U1)]
            public bool enableIMUFusion;
            public float depthMinRange;
            public bool setGravityAsOrigin;
            public sl.POSITIONAL_TRACKING_MODE mode;
        };

        /// <summary>
        /// DLL-friendly version of SpatialMappingPara (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        struct sl_SpatialMappingParameters
        {
            public float resolutionMeter;
            public float rangeMeter;
            [MarshalAs(UnmanagedType.U1)]
            public bool saveTexture;
            [MarshalAs(UnmanagedType.U1)]
            public bool useChunkOnly;
            public int maxMemoryUsage;
            [MarshalAs(UnmanagedType.U1)]
            public bool reverseVertexOrder;
            public SPATIAL_MAP_TYPE mapType;
            public int stabilityCounter;
            public float disparity_std;
            public float decay;
            [MarshalAs(UnmanagedType.U1)]
            public bool enable_forget_past;
        };

        /// <summary>
        /// DLL-friendly version of PlaneDetectionParameters (found in ZEDCommon.cs).
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        struct sl_PlaneDetectionParameters
        {
            public float maxDistanceThreshold;
            public float normalSimilarityThreshold;
        };

        /// <summary>
        /// Opens the ZED camera from the provided InitParameters.
        ///
        /// The method will also check the hardware requirements and run a self-calibration.
        /// </summary>
        /// <param name="initParameters">A structure containing all the initial parameters. Default: a preset of InitParameters.
        /// </param>
        /// <returns>
        /// An error code giving information about the internal process. If \ref ERROR_CODE "ERROR_CODE.SUCCESS" is returned, the camera is ready to use. Every other code indicates an error and the program should be stopped.
        /// </returns>
        public ERROR_CODE Open(ref InitParameters initParameters)
        {

            sl_initParameters initP = new sl_initParameters(initParameters); //DLL-friendly version of InitParameters.
            initP.coordinateSystem = initParameters.coordinateSystem; //Left-hand
            int v = dllz_open(CameraID, ref initP, initParameters.serialNumber, 
                new System.Text.StringBuilder(initParameters.pathSVO, initParameters.pathSVO.Length),
                new System.Text.StringBuilder(initParameters.ipStream, initParameters.ipStream.Length),
                initParameters.portStream,
                initParameters.gmslPort,
                new System.Text.StringBuilder(initParameters.sdkVerboseLogFile, initParameters.sdkVerboseLogFile.Length),
                new System.Text.StringBuilder(initParameters.optionalSettingsPath, initParameters.optionalSettingsPath.Length),
                new System.Text.StringBuilder(initParameters.optionalOpencvCalibrationFile, initParameters.optionalOpencvCalibrationFile.Length));

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
        /// Closes the camera.
        ///
        /// Once destroyed, you need to recreate a camera instance to restart again.
        /// </summary>
        public void Close()
        {
            cameraReady = false;
            dllz_close(CameraID);
        }

        /// <summary>
        /// This method will grab the latest images from the camera, rectify them, and compute the
        /// \ref retrieveMeasure() "measurements" based on the \ref RuntimeParameters provided (depth, point cloud, tracking, etc.).
        ///
        /// The grabbing method is typically called in the main loop in a separate thread.
        /// \note For more info, read about the SDK method it calls:
        /// <a href="https://www.stereolabs.com/docs/api/classsl_1_1Camera.html#afa3678a18dd574e162977e97d7cbf67b">grab</a>.
        /// </summary>
        /// <param name="runtimeParameters">A structure containing all the runtime parameters. Default: a preset of RuntimeParameters.</param>
        /// <returns>false if no problem was encountered, true otherwise.</returns>
        public sl.ERROR_CODE Grab(ref sl.RuntimeParameters runtimeParameters)
        {
            sl_RuntimeParameters rt_params = new sl_RuntimeParameters(runtimeParameters);
            return (sl.ERROR_CODE)dllz_grab(CameraID, ref rt_params);
        }

        /// <summary>
        /// Set this camera as a data provider for the Fusion module.
        /// </summary>
        /// <param name="jsonConfigFileName"></param>
        /// <returns>ERROR_CODE "ERROR_CODE.SUCCESS" if everything went fine, \ref ERROR_CODE "ERROR_CODE.FAILURE" otherwise.</returns>
        public ERROR_CODE StartPublishing(ref CommunicationParameters commParams)
        {
            return dllz_start_publishing(CameraID, ref commParams);
        }

        /// <summary>
        /// Set this camera as normal camera(without data providing).
        /// </summary>
        public ERROR_CODE StopPublishing()
        {
            return dllz_stop_publishing(CameraID);
        }

        /// <summary>
        /// Return the sl.INPUT_TYPE currently used.
        /// </summary>
        /// <returns>The current sl.INPUT_TYPE.</returns>
        public sl.INPUT_TYPE GetInputType()
        {
            return (sl.INPUT_TYPE)dllz_get_input_type(CameraID);
        }

        ///@name Video

        /// <summary>
        /// Retrieves an image texture from the ZED SDK and loads it into a sl.Mat.
        ///
        /// Use this to get an individual texture from the last grabbed frame in a human-viewable format. Image textures work for when you want the result to be visible,
        /// such as the direct RGB image from the camera, or a greyscale image of the depth. However it will lose accuracy if used
        /// to show measurements like depth or confidence, unlike measure textures.
        /// \note If you want to access the texture via script, you'll usually want to specify CPU memory. Then you can use
        /// Marshal.Copy to move them into a new byte array, which you can load into a Texture2D. Note that you may need to
        /// change the color space and/or flip the image.
        /// 
        /// \n\note For more info, read about the SDK method it calls:
        /// <a href="https://www.stereolabs.com/docs/api/classsl_1_1Camera.html#a01dce4f0af6f8959a9c974ffaca656b5">retrieveImage</a>.
        /// </summary>
        /// <param name="mat">sl.Mat to fill with the new texture.</param>
        /// <param name="view">Image type (left RGB, right depth map, etc.)</param>
        /// <param name="mem">Whether the image should be on CPU or GPU memory.</param>
        /// <param name="resolution">Resolution of the texture.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveImage(sl.Mat mat, sl.VIEW view, sl.MEM mem = sl.MEM.CPU, sl.Resolution resolution = new sl.Resolution())
        {
            return (sl.ERROR_CODE)(dllz_retrieve_image(CameraID, mat.MatPtr, (int)view, (int)mem, (int)resolution.width, (int)resolution.height, IntPtr.Zero));
        }

        /// <summary>
        /// Returns the InitParameters associated with the Camera object.
        ///
        /// It corresponds to the structure given as argument to \ref Open() method.
        /// </summary>
        /// <returns>InitParameters containing the parameters used to initialize the Camera object.</returns>
        public InitParameters GetInitParameters()
        {
            IntPtr p = dllz_get_init_parameters(CameraID);
            if (p == IntPtr.Zero)
            {
                return new InitParameters();
            }
            sl_initParameters sl_parameters = (sl_initParameters)Marshal.PtrToStructure(p, typeof(sl_initParameters));
            InitParameters parameters = new InitParameters()
            {
                cameraDeviceID = sl_parameters.cameraDeviceID,
                inputType = sl_parameters.inputType,
                resolution = sl_parameters.resolution,
                cameraFPS = sl_parameters.cameraFps,
                svoRealTimeMode = sl_parameters.svoRealTimeMode,
                coordinateUnits = sl_parameters.coordinateUnits,
                coordinateSystem = sl_parameters.coordinateSystem,
                depthMaximumDistance = sl_parameters.depthMaximumDistance,
                depthMinimumDistance = sl_parameters.depthMinimumDistance,
                depthMode = sl_parameters.depthMode,
                cameraImageFlip = (FLIP_MODE)sl_parameters.cameraImageFlip,
                enableImageEnhancement = sl_parameters.enableImageEnhancement,
                enableRightSideMeasure = sl_parameters.enableRightSideMeasure,
                cameraDisableSelfCalib = sl_parameters.cameraDisableSelfCalib,
                sdkGPUId = sl_parameters.sdkGPUId,
                sdkVerbose = sl_parameters.sdkVerbose,
                depthStabilization = sl_parameters.depthStabilization,
                sensorsRequired = sl_parameters.sensorsRequired,
                openTimeoutSec = sl_parameters.openTimeoutSec,
                asyncGrabCameraRecovery = sl_parameters.asyncGrabCameraRecovery,
                grabComputeCappingFPS = sl_parameters.grabComputeCappingFPS,
                enableImageValidityCheck = sl_parameters.enableImageValidityCheck
            };
            return parameters;
        }
        /// <summary>
        /// Returns the RuntimeParameters used.
        ///
        ///  It corresponds to the structure given as argument to the \ref Grab() method.
        /// </summary>
        /// <returns>RuntimeParameters containing the parameters that define the behavior of the \ref Grab method.</returns>
        public RuntimeParameters GetRuntimeParameters()
        {
            IntPtr p = dllz_get_runtime_parameters(CameraID);
            if (p == IntPtr.Zero)
            {
                return new RuntimeParameters();
            }

            sl_RuntimeParameters sl_parameters = (sl_RuntimeParameters)Marshal.PtrToStructure(p, typeof(sl_RuntimeParameters));
            RuntimeParameters parameters = new RuntimeParameters()
            {
                textureConfidenceThreshold = sl_parameters.textureConfidenceThreshold,
                measure3DReferenceFrame = sl_parameters.measure3DReferenceFrame,
                enableDepth = sl_parameters.enableDepth,
                confidenceThreshold = sl_parameters.confidenceThreshold,
                enableFillMode = sl_parameters.enableFillMode,
                removeSaturatedAreas = sl_parameters.removeSaturatedAreas,
            };
            return parameters;
        }

        /// <summary>
        /// Returns the PositionalTrackingParameters used.
        ///
        ///  It corresponds to the structure given as argument to the EnablePositionalTracking() method.
        /// </summary>
        /// <returns>PositionalTrackingParameters containing the parameters used for positional tracking initialization.</returns>
        public PositionalTrackingParameters GetPositionalTrackingParameters()
        {
            IntPtr p = dllz_get_positional_tracking_parameters(CameraID);
            if (p == IntPtr.Zero)
            {
                return new PositionalTrackingParameters();
            }
            sl_PositionalTrackingParameters sl_positionalTracking = (sl_PositionalTrackingParameters)Marshal.PtrToStructure(p, typeof(sl_PositionalTrackingParameters));
            PositionalTrackingParameters trackingParams = new PositionalTrackingParameters()
            {
                initialWorldPosition = sl_positionalTracking.initialWorldPosition,
                initialWorldRotation = sl_positionalTracking.initialWorldRotation,
                enableIMUFusion = sl_positionalTracking.enableIMUFusion,
                enableAreaMemory = sl_positionalTracking.enableAreaMemory,
                enablePoseSmothing = sl_positionalTracking.enablePoseSmothing,
                setAsStatic = sl_positionalTracking.setAsStatic,
                setFloorAsOrigin = sl_positionalTracking.setFloorAsOrigin,
                depthMinRange = sl_positionalTracking.depthMinRange,
                setGravityAsOrigin = sl_positionalTracking.setGravityAsOrigin,
                mode = sl_positionalTracking.mode
            };

            return trackingParams;
        }

        /// <summary>
        /// Gets the corresponding sl.Resolution from an sl.RESOLUTION.
        /// </summary>
        /// <param name="resolution">The wanted sl.RESOLUTION.</param>
        /// <returns>The sl.Resolution corresponding to sl.RESOLUTION given as argument.</returns>
        public static sl.Resolution GetResolution(RESOLUTION resolution)
        {
            sl.Resolution res = new sl.Resolution();
            switch (resolution)
            {
                case RESOLUTION.HD2K: res = new sl.Resolution(2208, 1242); break;
                case RESOLUTION.HD1080: res = new sl.Resolution(1920, 1080); break;
                case RESOLUTION.HD1200: res = new sl.Resolution(1920, 1200); break;
                case RESOLUTION.HD720: res = new sl.Resolution(1280, 720); break;
                case RESOLUTION.VGA: res = new sl.Resolution(672, 376); break;
                case RESOLUTION.HDSVGA: res = new sl.Resolution(960, 600); break;
            }
            return res;
        }

        /// <summary>
        /// Test if the video setting is supported by the camera.
        /// </summary>
        /// <param name="setting">The video setting to test.</param>
        /// <returns>true if the \ref VIDEO_SETTINGS is supported by the camera, false otherwise.</returns>
        public bool IsCameraSettingSupported(VIDEO_SETTINGS setting)
        {
            return dllz_is_camera_setting_supported(CameraID, setting);
        }

        /// <summary>
        /// Sets the min and max range values of the requested \ref VIDEO_SETTINGS "camera setting" (used for settings with a range).
        /// </summary>
        /// <param name="settings">The setting to be set.</param>
        /// <param name="minvalue">The min value of the range to set.</param>
        /// <param name="maxvalue">The min value of the range to set.</param>
        public void SetCameraSettings(VIDEO_SETTINGS settings, int minvalue, int maxvalue)
        {
            AssertCameraIsReady();
            dllz_set_camera_settings_min_max(CameraID, (int)settings, minvalue, maxvalue);
        }

        /// <summary>
        /// Returns the current range of the requested \ref VIDEO_SETTINGS "camera setting".
        /// </summary>
        /// <param name="settings">Setting to be retrieved (setting with range value).</param>
        /// <param name="minvalue">Will be set to the value of the lower bound of the range of the setting.</param>
        /// <param name="maxvalue">Will be set to the value of the higher bound of the range of the setting.</param>
        /// <returns>An sl.ERROR_CODE to indicate if the method was successful.</returns>
        public sl.ERROR_CODE GetCameraSettings(VIDEO_SETTINGS settings, ref int minvalue, ref int maxvalue)
        {
            AssertCameraIsReady();
            int ret = -1;
            return dllz_get_camera_settings_min_max(CameraID, settings, ref minvalue, ref maxvalue);
        }

        /// <summary>
        /// Sets the value of the requested \ref VIDEO_SETTINGS "camera setting" (gain, brightness, hue, exposure, etc.).
        /// </summary>
        /// <param name="settings">The setting to be set.</param>
        /// <param name="value">The value to set. Default: auto mode</param>
        public void SetCameraSettings(VIDEO_SETTINGS settings, int value)
        {
            AssertCameraIsReady();
            dllz_set_camera_settings(CameraID, (int)settings, value);
        }

        /// <summary>
        /// Returns the current value of the requested \ref VIDEO_SETTINGS "camera setting" (gain, brightness, hue, exposure, etc.).
        /// </summary>
        /// <param name="settings">Setting to be retrieved (brightness, contrast, gain, exposure, etc.).</param>
        /// <returns>The value of the requested \ref VIDEO_SETTINGS "camera setting".</returns>
        public int GetCameraSettings(VIDEO_SETTINGS settings)
        {
            AssertCameraIsReady();
            int ret = -1;
            dllz_get_camera_settings(CameraID, settings, ref ret);
            return ret;
        }

        /// <summary>
        /// Overloaded method for VIDEO_SETTINGS.AEC_AGC_ROI which takes a Rect as parameter.
        /// </summary>
        /// <param name="settings"> Must be set at VIDEO_SETTINGS.AEC_AGC_ROI, otherwise the method will have no impact.</param>
        /// <param name="side">sl.SIDE on which to be applied for AEC/AGC computation.</param>
        /// <param name="roi">Rect that defines the target to be applied for AEC/AGC computation. Must be given according to camera resolution.</param>
        /// <param name="reset">Cancel the manual ROI and reset it to the full image.</param>
        /// <returns>An sl.ERROR_CODE to indicate if the method was successful.</returns>
        public ERROR_CODE SetCameraSettings(VIDEO_SETTINGS settings, SIDE side, Rect roi, bool reset = false)
        {
            AssertCameraIsReady();
            if (settings == VIDEO_SETTINGS.AEC_AGC_ROI)
                return dllz_set_roi_for_aec_agc(CameraID, (int)side, roi, reset);
            else
                return ERROR_CODE.FAILURE;
        }

        /// <summary>
        /// Overloaded method for VIDEO_SETTINGS.AEC_AGC_ROI which takes a Rect as parameter.
        /// </summary>
        /// <param name="settings"> Must be set at VIDEO_SETTINGS.AEC_AGC_ROI, otherwise the method will have no impact.</param>
        /// <param name="side">sl.SIDE on which to get the ROI from.</param>
        /// <param name="roi"> Roi that will be filled.</param>
        /// <returns>An sl.ERROR_CODE to indicate if the method was successful.</returns>
        public ERROR_CODE GetCameraSettings(VIDEO_SETTINGS settings, SIDE side, ref Rect roi)
        {
            AssertCameraIsReady();
            if (settings == VIDEO_SETTINGS.AEC_AGC_ROI)
                return dllz_get_roi_for_aec_agc(CameraID, (int)side, ref roi);
            else
                return ERROR_CODE.FAILURE;
        }

        /// <summary>
        /// Reset camera settings to default.
        /// </summary>
        public void ResetCameraSettings()
        {
            AssertCameraIsReady();
            foreach (sl.VIDEO_SETTINGS setting_ in Enum.GetValues(typeof(sl.VIDEO_SETTINGS)))
                SetCameraSettings(setting_, -1);
        }

        /// <summary>
        /// Gets the timestamp at the time the latest grabbed frame was extracted from the USB stream.
        ///
        /// This is the closest timestamp you can get from when the image was taken.
        /// \note Must be called after calling Grab().
        /// </summary>
        /// <returns>Current timestamp in nanoseconds. -1 means it's is not available, such as with an .SVO file without compression.</returns>
        public ulong GetCameraTimeStamp()
        {
            return dllz_get_image_timestamp(CameraID);
        }

        /// <summary>
        /// Gets the current timestamp at the time the method is called.
        ///
        /// Can be compared to the camera timestamp for synchronization, since they have the same reference (the computer's start time).
        /// </summary>
        /// <returns>The timestamp in nanoseconds.</returns>
        public ulong GetCurrentTimeStamp()
        {
            return dllz_get_current_timestamp(CameraID);
        }

        /// <summary>
        /// Returns the current playback position in the SVO file.
        /// </summary>
        /// <returns>The current frame position in the SVO file. -1 if the SDK is not reading an SVO.</returns>
        public int GetSVOPosition()
        {
            return dllz_get_svo_position(CameraID);
        }

        /// <summary>
        /// Retrieves the frame index within the SVO file corresponding to the provided timestamp.
        /// </summary>
        /// <param name="timestamp">The target timestamp for which the frame index is to be determined.</param>
        /// <returns>The frame index within the SVO file that aligns with the given timestamp. Returns -1 if the timestamp falls outside the bounds of the SVO file.</returns>
        public int GetSVOPositionAtTimestamp(ulong timestamp)
        {
            return dllz_get_svo_position_at_timestamp(CameraID, timestamp);
        }

        /// <summary>
        /// Returns the number of frames in the SVO file.
        /// </summary>
        /// <returns>The total number of frames in the SVO file. -1 if the SDK is not reading a SVO.</returns>
        public int GetSVONumberOfFrames()
        {
            return dllz_get_svo_number_of_frames(CameraID);
        }

        /// <summary>
        /// Sets the position of the SVO file currently being read to a desired position.
        /// </summary>
        /// <param name="position">Index of the desired position to be decoded.</param>
        /// <returns>An sl.ERROR_CODE to indicate if the method was successful.</returns>
        public ERROR_CODE SetSVOPosition(int position)
        {
            return dllz_set_svo_position(CameraID, position);
        }

        /// <summary>
        /// Returns the current camera FPS.
        ///
        /// This is limited primarily by resolution but can also be lower due to setting a lower desired resolution in Open() or from USB connection/bandwidth issues.
        /// </summary>
        /// <returns>The current fps</returns>
        public float GetCameraFPS()
        {
            return dllz_get_camera_fps(CameraID);
        }

        /// <summary>
        /// Reports if the camera has been successfully opened.
        /// </summary>
        /// <returns> Returns true if the camera is already setup, otherwise false.</returns>
        public bool IsOpened()
        {
            return dllz_is_opened(CameraID);
        }

        ///@}

        /// <summary>
        /// Return the calibration parameters of the camera.
        /// </summary>
        /// <param name="raw">Whether to return the raw or rectified calibration parameters.</param>
        /// <returns>CalibrationParameters containing the calibration parameters requested.</returns>
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
        /// Gets the camera model (sl.MODEL).
        /// </summary>
        /// <returns>Model of the camera as sl.MODEL.</returns>
        public sl.MODEL GetCameraModel()
        {
            return (sl.MODEL)dllz_get_camera_model(CameraID);
        }

        /// <summary>
        /// Gets the camera firmware version.
        /// </summary>
        /// <returns>The firmware version of the camera.</returns>
        public int GetCameraFirmwareVersion()
        {
            return dllz_get_camera_firmware(CameraID);
        }

        /// <summary>
        /// Gets the sensors firmware version.
        /// </summary>
        /// <returns>The firmware version of the camera.</returns>
        public int GetSensorsFirmwareVersion()
        {
            return dllz_get_sensors_firmware(CameraID);
        }

        /// <summary>
        /// Gets the camera's serial number.
        /// </summary>
        /// <returns>The serial number of the camera.</returns>
        public int GetZEDSerialNumber()
        {
            return dllz_get_zed_serial(CameraID);
        }

        /// <summary>
        /// Returns the camera's vertical field of view in radians.
        /// </summary>
        /// <returns>The vertical field of view.</returns>
        public float GetFOV()
        {
            return GetCalibrationParameters(false).leftCam.vFOV * Deg2Rad;
        }

        /// <summary>
        /// Perform a new self calibration process.
        ///
        /// In some cases, due to temperature changes or strong vibrations, the stereo calibration becomes less accurate.
        /// \n Use this method to update the self-calibration data and get more reliable depth values.
        /// \note The self calibration will occur at the next \ref Grab() call.
        /// New values will then be available in \ref GetCameraInformation(), be sure to get them to still have consistent 2D - 3D conversion.
        /// </summary>
        public void UpdateSelfCalibration()
        {
            dllz_update_self_calibration(CameraID);
        }

        /// <summary>
        /// Gets the number of frames dropped since Grab() was called for the first time.
        ///
        /// It is based on camera timestamps and an FPS comparison.
        /// \note It is similar to the Frame Drop display in the ZED Explorer app.
        /// </summary>
        /// <returns>Frames dropped since first Grab() call.</returns>
        public uint GetFrameDroppedCount()
        {
            return dllz_get_frame_dropped_count(CameraID);
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
        /// Change the coordinate system of a transform matrix.
        /// </summary>
        /// <param name="rotation">[In, Out] : rotation to transform</param>
        /// <param name="translation"> [In, Out] : translation to transform</param>
        /// <param name="coordinateSystemSrc"> The current coordinate system of the translation/rotation</param>
        /// <param name="coordinateSystemDest"> The destination coordinate system for the translation/rotation.</param>
        /// <returns> SUCCESS if everything went well, FAILURE otherwise.</returns>
        public static sl.ERROR_CODE ConvertCoordinateSystem(ref Quaternion rotation, ref Vector3 translation, sl.COORDINATE_SYSTEM coordinateSystemSrc, sl.COORDINATE_SYSTEM coordinateSystemDest)
        {
            return (sl.ERROR_CODE)dllz_convert_coordinate_system(ref rotation, ref translation, coordinateSystemSrc, coordinateSystemDest);
        }
        /// <summary>
        /// Gets the version of the currently installed ZED SDK.
        /// </summary>
        /// <returns>ZED SDK version as a string in the format MAJOR.MINOR.PATCH.</returns>
        public static void GetSDKVersion(ref int major, ref int minor, ref int patch)
        {
            string sdkVersion = PtrToStringUtf8(dllz_get_sdk_version());

            string[] version = sdkVersion.Split('.');

            if (version.Length == 3)
            {
                int.TryParse(version[0], out major);
                int.TryParse(version[1], out minor);
                int.TryParse(version[2], out patch);
            }
        }


        /// <summary>
        /// List all the connected devices with their associated information.
        ///
        /// This method lists all the cameras available and provides their serial number, models and other information.
        /// </summary>
        /// <returns>The device properties for each connected camera.</returns>
        public static sl.DeviceProperties[] GetDeviceList(out int nbDevices)
        {
            sl.DeviceProperties[] deviceList = new sl.DeviceProperties[(int)Constant.MAX_CAMERA_PLUGIN];
            dllz_get_device_list(deviceList, out nbDevices);

            return deviceList;
        }

        /// <summary>
        /// List all the streaming devices with their associated information.
        ///
        /// This method lists all the cameras available and provides their serial number, models and other information.
        /// </summary>
        /// <returns>The device properties for each connected camera.</returns>
        public static sl.StreamingProperties[] GetStreamingDeviceList(out int nbDevices)
        {
            sl.StreamingProperties[] streamingDeviceList = new sl.StreamingProperties[(int)Constant.MAX_CAMERA_PLUGIN];
            dllz_get_streaming_device_list(streamingDeviceList, out nbDevices);

            return streamingDeviceList;
        }

        /// <summary>
        /// Performs a hardware reset of the ZED 2 and the ZED 2i.
        /// </summary>
        /// <param name="serialNumber">Serial number of the camera to reset, or 0 to reset the first camera detected.</param>
        /// <param name="fullReboot">Perform a full reboot (sensors and video modules) if true, otherwise only the video module will be rebooted.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE::CAMERA_NOT_DETECTED if no camera was detected, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public static sl.ERROR_CODE Reboot(int serialNumber, bool fullReboot = true)
        {
            return (sl.ERROR_CODE)dllz_reboot(serialNumber, fullReboot);
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

        ///@{
        /// @name Depth Sensing

        /// <summary>
        /// Retrieves a measure texture from the ZED SDK and loads it into a sl.Mat.
        /// 
        /// Use this to get an individual texture from the last grabbed frame with measurements in every pixel - such as a depth map, confidence map, etc.
        /// Measure textures are not human-viewable but don't lose accuracy, unlike image textures.
        /// \note If you want to access the texture via script, you'll usually want to specify CPU memory. Then you can use
        /// Marshal.Copy to move them into a new byte array, which you can load into a Texture2D.
        /// 
        /// \n\note For more info, read about the SDK method it calls: 
        /// <a href="https://www.stereolabs.com/docs/api/classsl_1_1Camera.html#a9e0773c0c14ce5156c1fa2fde501c13e">retrieveMeasure</a>.
        /// </summary>
        /// <param name="mat">sl.Mat to fill with the new texture.</param>
        /// <param name="measure">Measure type (depth, confidence, xyz, etc.).</param>
        /// <param name="mem">Whether the image should be on CPU or GPU memory.</param>
        /// <param name="resolution">Resolution of the texture.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveMeasure(sl.Mat mat, sl.MEASURE measure, sl.MEM mem = sl.MEM.CPU, sl.Resolution resolution = new sl.Resolution())
        {
            return (sl.ERROR_CODE)(dllz_retrieve_measure(CameraID, mat.MatPtr, (int)measure, (int)mem, (int)resolution.width, (int)resolution.height, IntPtr.Zero));
        }

        /// <summary>
        /// Gets the current confidence threshold value for the disparity map (and by extension the depth map).
        ///
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
        /// Gets the current range of perceived depth.
        /// </summary>
        /// <param name="min">Minimum depth detected (in selected sl.UNIT).</param>
        /// <param name="max">Maximum depth detected (in selected sl.UNIT).</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if values have been extracted. Other sl.ERROR_CODE otherwise.</returns>
        public sl.ERROR_CODE GetCurrentMixMaxDepth(ref float min, ref float max)
        {
            return (sl.ERROR_CODE)dllz_get_current_min_max_depth(CameraID, ref min, ref max);
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
        /// Initializes and starts the positional tracking processes.
        /// </summary>
        /// <param name="positionalTrackingParameters">A structure containing all the specific parameters for the positional tracking. Default: a preset of PositionalTrackingParameters.</param>
        /// <returns>sl.ERROR_CODE.FAILURE if the <b>area_file_path</b> file wasn't found, sl.ERROR_CODE.SUCCESS otherwise.</returns>
        public sl.ERROR_CODE EnablePositionalTracking(ref PositionalTrackingParameters positionalTrackingParameters)
        {
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;

            sl_PositionalTrackingParameters sl_tracking_params = new sl_PositionalTrackingParameters();
            sl_tracking_params.enableAreaMemory = positionalTrackingParameters.enableAreaMemory;
            sl_tracking_params.enableIMUFusion = positionalTrackingParameters.enableIMUFusion;
            sl_tracking_params.enablePoseSmothing = positionalTrackingParameters.enablePoseSmothing;
            sl_tracking_params.initialWorldPosition = positionalTrackingParameters.initialWorldPosition;
            sl_tracking_params.initialWorldRotation = positionalTrackingParameters.initialWorldRotation;
            sl_tracking_params.setAsStatic = positionalTrackingParameters.setAsStatic;
            sl_tracking_params.setFloorAsOrigin = positionalTrackingParameters.setFloorAsOrigin;
            sl_tracking_params.depthMinRange = positionalTrackingParameters.depthMinRange;
            sl_tracking_params.setGravityAsOrigin = positionalTrackingParameters.setGravityAsOrigin;
            sl_tracking_params.mode = positionalTrackingParameters.mode;

            trackingStatus = (sl.ERROR_CODE)dllz_enable_tracking(CameraID, ref sl_tracking_params, new System.Text.StringBuilder(positionalTrackingParameters.areaFilePath, positionalTrackingParameters.areaFilePath.Length));
            return trackingStatus;
        }

        /// <summary>
        ///  Disables the positional tracking.
        /// </summary>
        /// <param name="path">
        /// If set, saves the spatial memory into an '.area' file. Default: (empty)
        /// \n <b>path</b> is the name and path of the database, e.g. <i>path/to/file/myArea1.area"</i>.
        /// </param>
        public void DisablePositionalTracking(string path = "")
        {
            dllz_disable_tracking(CameraID, new System.Text.StringBuilder(path, path.Length));
        }

        /// <summary>
        /// Tells if the tracking module is enabled.
        /// </summary>
        public bool IsPositionalTrackingEnabled()
        {
            return dllz_is_positional_tracking_enabled(CameraID);
        }


        /// <summary>
        /// Saves the current area learning file.
        ///
        /// The file will contain spatial memory data generated by the tracking.
        /// </summary>
        /// <param name="areaFilePath">Path of an '.area' file to save the spatial memory database in.</param>
        public ERROR_CODE SaveAreaMap(string areaFilePath)
        {
            return (ERROR_CODE)dllz_save_area_map(CameraID, new System.Text.StringBuilder(areaFilePath, areaFilePath.Length));
        }

        /// <summary>
        /// Returns the state of the spatial memory export process.
        /// </summary>
        /// <returns> The current \ref AREA_EXPORTING_STATE "state" of the spatial memory export process.</returns>
        public AREA_EXPORT_STATE GetAreaExportState()
        {
            return (AREA_EXPORT_STATE)dllz_get_area_export_state(CameraID);
        }

        /// <summary>
        /// Resets the tracking, and re-initializes the position with the given translation vector and rotation quaternion.
        /// </summary>
        /// <param name="rotation">Rotation to set the positional tracking to.</param>
        /// <param name="translation">Translation to set the positional tracking to.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if the tracking has been reset, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE ResetPositionalTracking(Quaternion rotation, Vector3 translation)
        {
            sl.ERROR_CODE trackingStatus = sl.ERROR_CODE.CAMERA_NOT_DETECTED;
            trackingStatus = (sl.ERROR_CODE)dllz_reset_tracking(CameraID, rotation, translation);
            return trackingStatus;
        }

        /// <summary>
        /// Returns the sensor configuration of the camera.
        /// </summary>
        /// <returns> SensorsConfiguration containing the sensor calibration information of the camera.</returns>
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
        /// Returns the CameraInformation associated the camera being used.
        ///
        /// To ensure accurate calibration, it is possible to specify a custom resolution as a parameter when obtaining scaled information, as calibration parameters are resolution-dependent.
        /// \n When reading an SVO file, the parameters will correspond to the camera used for recording.
        /// </summary>
        /// <returns> CameraInformation containing the calibration parameters of the camera, as well as serial number and firmware version.</returns>
        public CameraInformation GetCameraInformation(Resolution resolution = new Resolution())
        {
            IntPtr p = dllz_get_camera_information(CameraID, (int)resolution.width, (int)resolution.height);

            if (p == IntPtr.Zero)
            {
                return new CameraInformation();
            }
            CameraInformation cameraInformation = (CameraInformation)Marshal.PtrToStructure(p, typeof(CameraInformation));

            return cameraInformation;
        }

        /// <summary>
        /// Gets the position of the camera and the current state of the camera Tracking.
        /// </summary>
        /// <param name="rotation">Quaternion filled with the current rotation of the camera depending on its reference frame.</param>
        /// <param name="position">Vector filled with the current position of the camera depending on its reference frame.</param>
        /// <param name="referenceType">Reference frame for setting the rotation/position. REFERENCE_FRAME.CAMERA gives movement relative to the last pose.
        /// REFERENCE_FRAME.WORLD gives cumulative movements since tracking started.</param>
        /// <returns>The current \ref POSITIONAL_TRACKING_STATE "state" of the tracking process.</returns>
        public POSITIONAL_TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 position, REFERENCE_FRAME referenceType = REFERENCE_FRAME.WORLD)
        {
            return (POSITIONAL_TRACKING_STATE)dllz_get_position(CameraID, ref rotation, ref position, (int)referenceType);
        }

        /// <summary>
        /// Returns the current status of positional tracking module.
        /// </summary>
        /// <returns> The current status of positional tracking module. </returns>
        public PositionalTrackingStatus GetPositionalTrackingStatus()
        {
            IntPtr p = dllz_get_positional_tracking_status(CameraID);
            if (p == IntPtr.Zero)
            {
                return new PositionalTrackingStatus();
            }

            PositionalTrackingStatus positionalTrackingStatus = (PositionalTrackingStatus)Marshal.PtrToStructure(p, typeof(PositionalTrackingStatus));
            return positionalTrackingStatus;
        }

        /// <summary>
        /// Get the current positional tracking landmarks.
        /// </summary>
        /// <param name="landmarks">Array of presents landmarks.</param>
        /// <returns>ERROR_CODE that indicate if the function succeed or not.</returns>
        public sl.ERROR_CODE GetPositionalTrackingLandmarks(ref List<Landmark> landmarks)
        {
            IntPtr landmarkArrayPtr = IntPtr.Zero;
            int count = 0;

            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_get_positional_tracking_landmarks(CameraID, ref landmarkArrayPtr, ref count);
            if (landmarkArrayPtr == IntPtr.Zero)
            {
                return sl.ERROR_CODE.FAILURE;
            }

            int structSize = Marshal.SizeOf(typeof(Landmark));
            // Read the array of Landmark* (individual struct pointers)
            for (int i = 0; i < count; i++)
            {
                IntPtr landmarkPtr = IntPtr.Add(landmarkArrayPtr, i * structSize);
                if (landmarkPtr == IntPtr.Zero)
                {
                    return sl.ERROR_CODE.FAILURE;
                }
                landmarks.Add(Marshal.PtrToStructure<Landmark>(landmarkPtr));
            }
            return err;
        }

        /// <summary>
        /// Get the current positional tracking landmarks 2d.
        /// </summary>
        /// <param name="landmarks">Array of presents landmarks.</param>
        /// <returns>ERROR_CODE that indicate if the function succeed or not.</returns>
        public sl.ERROR_CODE GetPositionalTrackingLandmarks2D(ref List<Landmark2D> landmarks)
        {
            IntPtr landmarkArrayPtr = IntPtr.Zero;
            int count = 0;
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_get_positional_tracking_landmarks_2d(CameraID, ref landmarkArrayPtr, ref count);
            int structSize = Marshal.SizeOf(typeof(Landmark2D));
            // Read the array of Landmark* (individual struct pointers)
            for (int i = 0; i < count; i++)
            {
                IntPtr landmarkPtr = IntPtr.Add(landmarkArrayPtr, i * structSize);
                landmarks.Add(Marshal.PtrToStructure<Landmark2D>(landmarkPtr));
            }
            return err;
        }

        /// <summary>
        /// Gets the current position of the camera and state of the tracking, with an optional offset to the tracking frame.
        /// </summary>
        /// <param name="rotation">Quaternion filled with the current rotation of the camera depending on its reference frame.</param>
        /// <param name="position">Vector filled with the current position of the camera depending on its reference frame.</param>
        /// <param name="targetQuaternion">Rotational offset applied to the tracking frame.</param>
        /// <param name="targetTranslation">Positional offset applied to the tracking frame.</param>
        /// <param name="referenceFrame">Reference frame for setting the rotation/position. REFERENCE_FRAME.CAMERA gives movement relative to the last pose.
        /// REFERENCE_FRAME.WORLD gives cumulative movements since tracking started.</param>
        /// <returns>The current \ref POSITIONAL_TRACKING_STATE "state" of the tracking process.</returns>
        public POSITIONAL_TRACKING_STATE GetPosition(ref Quaternion rotation, ref Vector3 translation, ref Quaternion targetQuaternion, ref Vector3 targetTranslation, REFERENCE_FRAME referenceFrame = REFERENCE_FRAME.WORLD)
        {
            return (POSITIONAL_TRACKING_STATE)dllz_get_position_at_target_frame(CameraID, ref rotation, ref translation, ref targetQuaternion, ref targetTranslation, (int)referenceFrame);
        }

        /// <summary>
        /// Gets the current position of the camera and state of the tracking, filling a Pose struct useful for AR pass-through.
        /// </summary>
        /// <param name="pose">Current pose.</param>
        /// <param name="referenceType">Reference frame for setting the rotation/position. REFERENCE_FRAME.CAMERA gives movement relative to the last pose.
        /// REFERENCE_FRAME.WORLD gives cumulative movements since tracking started.</param>
        /// <returns>The current \ref POSITIONAL_TRACKING_STATE "state" of the tracking process.</returns>
        public POSITIONAL_TRACKING_STATE GetPosition(ref Pose pose, REFERENCE_FRAME referenceType = REFERENCE_FRAME.WORLD)
        {
            return (POSITIONAL_TRACKING_STATE)dllz_get_position_data(CameraID, ref pose, (int)referenceType);
        }

        /// <summary>
        /// Sets a prior to the IMU orientation (not for \ref MODEL "MODEL.ZED").
        ///
        /// Prior must come from a external IMU, such as the HMD orientation and should be in a time frame
        /// that's as close as possible to the camera.
        /// </summary>
        /// <returns>An sl.ERROR_CODE status.</returns>
        /// <param name="rotation">Prior rotation.</param>
        public ERROR_CODE SetIMUOrientationPrior(ref Quaternion rotation)
        {
            return (sl.ERROR_CODE)dllz_set_imu_prior_orientation(CameraID, rotation);
        }

        /// <summary>
        /// Gets the rotation given by the IMU.
        /// \note This method will return ERROR_CODE.INVALID_FUNCTION_CALL with a MODEL.ZED which does not contains internal sensors.
        /// </summary>
        /// <returns>An sl.ERROR_CODE status.</returns>
        /// <param name="rotation">Rotation from the IMU.</param>
        public ERROR_CODE GetIMUOrientation(ref Quaternion rotation, TIME_REFERENCE referenceTime = TIME_REFERENCE.IMAGE)
        {
            return (sl.ERROR_CODE)dllz_get_internal_imu_orientation(CameraID, ref rotation, (int)referenceTime);
        }

        /// <summary>
        /// Retrieves the SensorsData (IMU, magnetometer, barometer) at a specific time reference.
        /// \note This method will return ERROR_CODE.INVALID_FUNCTION_CALL with a MODEL.ZED which does not contains internal sensors.
        /// </summary>
        /// <param name="data">The SensorsData variable to store the data.</param>
        /// <param name="referenceTime">Defines the reference from which you want the data to be expressed. Default: REFERENCE_FRAME.WORLD.</param>
        /// <returns>An sl.ERROR_CODE status.</returns>
        public ERROR_CODE GetSensorsData(ref SensorsData data, TIME_REFERENCE referenceTime = TIME_REFERENCE.IMAGE)
        {
            return (sl.ERROR_CODE)dllz_get_internal_sensors_data(CameraID, ref data, (int)referenceTime);
        }

        /// <summary>
        /// Retrieves all SensorsData associated to most recent grabbed frame in the specified \ref COORDINATE_SYSTEM of InitParameters.
        /// </summary>
        /// <param name="data">SensorsData array that store the data batch.</param>
        /// <returns></returns>
        public ERROR_CODE GetSensorsDataBatch(out List<SensorsData> data)
        {
            data = new List<SensorsData>();
            ERROR_CODE err = (ERROR_CODE)dllz_get_sensors_data_batch_count(CameraID, out int count);

            if (err == ERROR_CODE.SUCCESS)
            {
                SensorsData[] sensorsDataArray = new SensorsData[count];
                err = (ERROR_CODE)dllz_get_sensors_data_batch(CameraID, ref sensorsDataArray); // Get the size of the data

                data.AddRange(sensorsDataArray);
            }
            return err;
        }

        /// <summary>
        /// Defines a region of interest to focus on for all the SDK, discarding other parts.
        /// </summary>
        /// <param name="roiMask"> The Mat defining the requested region of interest, pixels lower than 127 will be discarded from all modules: depth, positional tracking, etc.
        /// If empty, set all pixels as valid. The mask can be either at lower or higher resolution than the current images.</param>
        /// <param name="module"> Apply the ROI to a list of SDK module, all by default. Must of size sl.MODULE.LAST. 
        /// The Mat defining the requested region of interest, pixels lower than 127 will be discarded from all modules: depth, positional tracking, etc.
        /// If empty, set all pixels as valid. The mask can be either at lower or higher resolution than the current images.
        /// </param>
        /// <returns>An sl.ERROR_CODE if something went wrong.</returns>
        public ERROR_CODE SetRegionOfInterest(sl.Mat roiMask, bool[] module)
        {
            if (module.Length != (int)MODULE.LAST) return sl.ERROR_CODE.FAILURE;

            return (sl.ERROR_CODE)dllz_set_region_of_interest(CameraID, roiMask.GetPtr(), module);
        }

        /// <summary>
        /// Get the previously set or computed region of interest.
        /// </summary>
        /// <param name="roiMask">The \ref Mat returned</param>
        /// <param name="resolution">The optional size of the returned mask</param>
        /// <param name="module"> Specifies the module from which the ROI is to be obtained. </param>
        /// <returns>An sl.ERROR_CODE if something went wrong.</returns>
        public ERROR_CODE GetRegionOfInterest(sl.Mat roiMask, sl.Resolution resolution = new sl.Resolution(), MODULE module = MODULE.ALL)
        {
            return (sl.ERROR_CODE)dllz_get_region_of_interest(CameraID, roiMask.MatPtr, (int)resolution.width, (int)resolution.height, module);
        }

        /// <summary>
        /// Start the auto detection of a region of interest to focus on for all the SDK, discarding other parts.
        /// This detection is based on the general motion of the camera combined with the motion in the scene.
        /// The camera must move for this process, an internal motion detector is used, based on the Positional Tracking module.
        /// It requires a few hundreds frames of motion to compute the mask.
        ///  \note This module is expecting a static portion, typically a fairly close vehicle hood at the bottom of the image.
        /// This module may not work correctly or detect incorrect background area, especially with slow motion, if there's no static element.
        /// This module work asynchronously, the status can be obtained using \ref GetRegionOfInterestAutoDetectionStatus(), the result is either auto applied,
        /// or can be retrieve using \ref GetRegionOfInterest function.
        /// </summary>
        /// <param name="roiParams"></param>
        /// <returns>An sl.ERROR_CODE if something went wrong.</returns>
        public ERROR_CODE StartRegionOfInterestAutoDetection(RegionOfInterestParameters roiParams)
        {
            return (sl.ERROR_CODE)dllz_start_region_of_interest_auto_detection(CameraID, ref roiParams);
        }

        /// <summary>
        ///  Return the status of the automatic Region of Interest Detection.
        ///  The automatic Region of Interest Detection is enabled by using \ref StartRegionOfInterestAutoDetection
        /// </summary>
        /// <returns>An sl.ERROR_CODE if something went wrong.</returns>
        public REGION_OF_INTEREST_AUTO_DETECTION_STATE GetRegionOfInterestAutoDetectionStatus()
        {
            return (REGION_OF_INTEREST_AUTO_DETECTION_STATE)dllz_get_region_of_interest_auto_detection_status(CameraID);
        }

        ///@}

        ///@{
        /// @name Spatial Mapping


        ///////////////////////////// SPATIAL MAPPING ////////////////////////////////

        /// <summary>
        /// Initializes and begins the spatial mapping processes.
        /// </summary>
        /// <param name="spatialMappingParameters">Spatial mapping parameters.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE EnableSpatialMapping(ref SpatialMappingParameters spatialMappingParameters)
        {
            sl_SpatialMappingParameters map_params = new sl_SpatialMappingParameters();
            map_params.rangeMeter = spatialMappingParameters.rangeMeter;
            map_params.resolutionMeter = spatialMappingParameters.resolutionMeter;
            map_params.saveTexture = spatialMappingParameters.saveTexture;
            map_params.mapType = spatialMappingParameters.map_type;
            map_params.maxMemoryUsage = 4096;
            map_params.useChunkOnly = spatialMappingParameters.useChunkOnly; //spatialMappingParameters.map_type == SPATIAL_MAP_TYPE.MESH ? true : false;
            map_params.reverseVertexOrder = spatialMappingParameters.reverseVertexOrder;
            map_params.stabilityCounter = spatialMappingParameters.stabilityCounter;

            sl.ERROR_CODE spatialMappingStatus = ERROR_CODE.FAILURE;
            spatialMappingStatus = (sl.ERROR_CODE)dllz_enable_spatial_mapping(CameraID, ref map_params);
            return spatialMappingStatus;
        }
        /// <summary>
        /// Initializes and begins the spatial mapping processes.
        /// </summary>
        /// <param name="resolutionMeter">Spatial mapping resolution in meters.</param>
        /// <param name="maxRangeMeter">Maximum scanning range in meters.</param>
        /// <param name="saveTexture">True to scan surface textures in addition to geometry.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE EnableSpatialMapping(SPATIAL_MAP_TYPE type = SPATIAL_MAP_TYPE.MESH, MAPPING_RESOLUTION mappingResolution = MAPPING_RESOLUTION.MEDIUM, MAPPING_RANGE mappingRange = MAPPING_RANGE.MEDIUM, bool saveTexture = false)
        {
            sl_SpatialMappingParameters map_params = new sl_SpatialMappingParameters();
            map_params.rangeMeter = ConvertRangePreset(mappingRange);
            map_params.resolutionMeter = ConvertResolutionPreset(mappingResolution);
            map_params.saveTexture = saveTexture;
            map_params.mapType = type;
            map_params.maxMemoryUsage = 4096;
            map_params.useChunkOnly = type == SPATIAL_MAP_TYPE.MESH ? true : false;

            sl.ERROR_CODE spatialMappingStatus = ERROR_CODE.FAILURE;
            spatialMappingStatus = (sl.ERROR_CODE)dllz_enable_spatial_mapping(CameraID, ref map_params);
            return spatialMappingStatus;
        }

        /// <summary>
        /// Returns the SpatialMappingParameters used.
        /// 
        /// It corresponds to the structure given as argument to the EnableSpatialMapping() method.
        /// </summary>
        /// <returns>SpatialMappingParameters containing the parameters used for spatial mapping initialization.</returns>
        public SpatialMappingParameters GetSpatialMappingParameters()
        {
            IntPtr p = dllz_get_spatial_mapping_parameters(CameraID);
            if (p == IntPtr.Zero)
            {
                return new SpatialMappingParameters();
            }
            sl_SpatialMappingParameters sl_parameters = (sl_SpatialMappingParameters)Marshal.PtrToStructure(p, typeof(sl_SpatialMappingParameters));
            SpatialMappingParameters parameters = new SpatialMappingParameters()
            {
                resolutionMeter = sl_parameters.resolutionMeter,
                rangeMeter = sl_parameters.rangeMeter,
                saveTexture = sl_parameters.saveTexture,
                map_type = sl_parameters.mapType,
                reverseVertexOrder = sl_parameters.reverseVertexOrder,
                useChunkOnly = sl_parameters.useChunkOnly,
                maxMemoryUsage = sl_parameters.maxMemoryUsage,
                stabilityCounter = sl_parameters.stabilityCounter
            };
            return parameters;
        }

        /// <summary>
        /// Disables the spatial mapping process.
        /// </summary>
        public void DisableSpatialMapping()
        {
            dllz_disable_spatial_mapping(CameraID);
        }

        /// <summary>
        /// Updates the internal version of the mesh and returns the sizes of the meshes.
        /// </summary>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each sub-mesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each sub-mesh.</param>
        /// <param name="nbUpdatedSubmesh">Number of updated sub-meshes.</param>
        /// <param name="updatedIndices">List of all sub-meshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all sub-meshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all sub-meshes.</param>
        /// <param name="nbSubmeshMax">Maximum number of sub-meshes that can be handled.</param>
        /// <returns>sl.ERROR_CODE indicating if the update was successful, and why it wasn't otherwise.</returns>
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
        /// <returns>sl.ERROR_CODE indicating if the update was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE UpdateMesh(ref Mesh mesh)
        {
            ERROR_CODE err = UpdateMesh(mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles, (int)Constant.MAX_SUBMESH);

            return err;
        }

        /// <summary>
        /// Retrieves all chunks of the current generated mesh.
        /// \note Call UpdateMesh() before calling this.
        ///
        /// Vertex and triangle arrays must be at least of the sizes returned by UpdateMesh (nbVertices and nbTriangles).
        /// </summary>
        /// <param name="vertices">Vertices of the mesh.</param>
        /// <param name="triangles">Triangles, formatted as the index of each triangle's three vertices in the vertices array.</param>
        /// <param name="colors"> (b, g, r) colors of the vertices.</param>
        /// <param name="nbSubmeshMax">Maximum number of sub-meshes that can be handled.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveMesh(Vector3[] vertices, int[] triangles, byte[] colors, int nbSubmeshMax, Vector2[] uvs, IntPtr textures)
        {
            return (sl.ERROR_CODE)dllz_retrieve_mesh(CameraID, vertices, triangles, colors, uvs, textures, nbSubmeshMax);
        }

        /// <summary>
        /// Retrieves all chunks of the current generated mesh.
        /// \note Call UpdateMesh() before calling this.
        ///
        /// Vertex and triangle arrays must be at least of the sizes returned by UpdateMesh (nbVertices and nbTriangles).
        /// </summary>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveMesh(ref Mesh mesh)
        {
            ERROR_CODE err = RetrieveMesh(mesh.vertices, mesh.triangles, mesh.colors, (int)Constant.MAX_SUBMESH, mesh.uvs, mesh.textures);
            int verticesOffset = 0;
            int trianglesOffset = 0;
            int colorsOffset = 0;

            for (int i = 0; i < mesh.nbUpdatedSubmesh; i++)
            {
                SetMesh(ref mesh, i, ref verticesOffset, ref trianglesOffset, ref colorsOffset);
            }
            return err;
        }
        /// <summary>
        /// Retrieve all chunks of the generated mesh.
        /// </summary>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveChunks(ref Mesh mesh)
        {
            dllz_update_chunks(CameraID, mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles, (int)Constant.MAX_SUBMESH);

            mesh.vertices = new Vector3[mesh.nbVertices];
            mesh.triangles = new int[mesh.nbTriangles * 3];
            mesh.colors = new byte[mesh.nbVertices * 3];

            ERROR_CODE err = (sl.ERROR_CODE)dllz_retrieve_chunks(CameraID, mesh.vertices, mesh.triangles, mesh.colors, mesh.uvs, mesh.textures, (int)Constant.MAX_SUBMESH);
            int verticesOffset = 0;
            int trianglesOffset = 0;
            int colorsOffset = 0;

            for (int i = 0; i < mesh.nbUpdatedSubmesh; i++)
            {
                SetMesh(ref mesh, i, ref verticesOffset, ref trianglesOffset, ref colorsOffset);
            }
            return err;

        }
        /// <summary>
        /// Process data from a sub-mesh retrieved from the ZED SDK into a chunk
        /// </summary>
        /// <param name="mesh"> Mesh data retrieved from the zed sdk</param>
        /// <param name="indexUpdate">Index of the sub-mesh/chunk to be updated.</param>
        /// <param name="verticesOffset">Starting index in the vertices stack.</param>
        /// <param name="trianglesOffset">Starting index in the triangles stack.</param>
        /// <param name="colorsOffset">Starting index in the colors stack.</param>
        /// <param name="uvsOffset">Starting index in the UVs stack.</param>
        private void SetMesh(ref Mesh mesh, int indexUpdate, ref int verticesOffset, ref int trianglesOffset, ref int colorsOffset)
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
            if (subMesh.colors == null || subMesh.colors.Length != 3 * mesh.nbVerticesInSubmesh[indexUpdate])
            {
                subMesh.colors = new byte[3* mesh.nbVerticesInSubmesh[indexUpdate]];
            }


            //Clear the old mesh data.
            Array.Clear(subMesh.vertices, 0, subMesh.vertices.Length);
            Array.Clear(subMesh.triangles, 0, subMesh.triangles.Length);
            Array.Clear(subMesh.colors, 0, subMesh.colors.Length);

            //Copy data retrieved from the ZED SDK into the current chunk.
            System.Array.Copy(mesh.vertices, verticesOffset, subMesh.vertices, 0, mesh.nbVerticesInSubmesh[indexUpdate]);
            verticesOffset += mesh.nbVerticesInSubmesh[indexUpdate];
            System.Buffer.BlockCopy(mesh.triangles, trianglesOffset * sizeof(int), subMesh.triangles, 0, 3 * mesh.nbTrianglesInSubmesh[indexUpdate] * sizeof(int)); //Block copy has better performance than Array.
            trianglesOffset += 3 * mesh.nbTrianglesInSubmesh[indexUpdate];

            System.Buffer.BlockCopy(mesh.colors, colorsOffset, subMesh.colors, 0, 3 *mesh.nbVerticesInSubmesh[indexUpdate]);
            colorsOffset += 3 * mesh.nbVerticesInSubmesh[indexUpdate];

            mesh.chunks[indexUpdate] = subMesh;
        }

        /// <summary>
        /// Retrieves the current generated mesh.
        /// </summary>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveSpatialMap(ref Mesh mesh)
        {
            UpdateMesh(ref mesh);
            //Resize the mesh buffer according to how many vertices are needed.
            mesh.vertices = new Vector3[mesh.nbVertices]; //Allocation is faster than resizing.
            mesh.triangles = new int[mesh.nbTriangles * 3];
            mesh.colors = new byte[mesh.nbVertices * 3];
            return RetrieveMesh(ref mesh);
        }
        /// <summary>
        /// Retrieves the current fused point cloud.
        /// </summary>
        /// <param name="fusedPointCloud">The Fused Point Cloud to be filled with the generated spatial map.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveSpatialMap(ref FusedPointCloud fusedPointCloud)
        {
            int nbVertices = 0;
            UpdateFusedPointCloud(ref nbVertices);
            // Resize the vertices array according to how many vertices are needed.
            fusedPointCloud.vertices = new Vector4[nbVertices];
            if (nbVertices > 0)
                return RetrieveFusedPointCloud(fusedPointCloud.vertices);
            else
                return ERROR_CODE.FAILURE;
        }

        /// <summary>
        /// Updates the fused point cloud (if spatial map type was \ref SPATIAL_MAP_TYPE "FUSED_POINT_CLOUD").
        /// </summary>
        /// <returns>sl.ERROR_CODE indicating if the update was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE UpdateFusedPointCloud(ref int nbVertices)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;
            err = (sl.ERROR_CODE)dllz_update_fused_point_cloud(CameraID, ref nbVertices);
            return err;
        }

        /// <summary>
        /// Retrieves all points of the fused point cloud.
        /// \note Call UpdateFusedPointCloud() before calling this.
        /// 
        /// Vertex arrays must be at least of the sizes returned by UpdateFusedPointCloud().
        /// </summary>
        /// <param name="vertices">Points of the fused point cloud.</param>
        /// <returns>sl.ERROR_CODE indicating if the retrieval was successful, and why it wasn't otherwise.</returns>
        public sl.ERROR_CODE RetrieveFusedPointCloud(Vector4[] vertices)
        {
            return (sl.ERROR_CODE)dllz_retrieve_fused_point_cloud(CameraID, vertices);
        }

        /// <summary>
        /// Extracts the current spatial map from the spatial mapping process.
        ///
        /// If the object to be filled already contains a previous version of the mesh, only changes will be updated, optimizing performance.
        /// </summary>
        /// <returns>sl.ERROR_CODE.SUCCESS if the mesh is filled and available, otherwise sl.ERROR_CODE.FAILURE.</returns>
        /// This is a blocking method. You should either call it in a thread or at the end of the mapping process.
        public ERROR_CODE ExtractWholeSpatialMap()
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;
            err = (sl.ERROR_CODE)dllz_extract_whole_spatial_map(CameraID);
            return err;
        }
        /// <summary>
        /// Starts the mesh generation process in a thread that does not block the spatial mapping process.
        ///
        /// ZEDSpatialMappingHelper calls this each time it has finished applying the last mesh update.
        /// </summary>
        public void RequestSpatialMap()
        {
            dllz_request_mesh_async(CameraID);
        }

        /// <summary>
        /// Pauses or resumes the spatial mapping processes.
        /// </summary>
        /// <param name="status">If true, the integration is paused. If false, the spatial mapping is resumed.</param>
        public void PauseSpatialMapping(bool status)
        {
            dllz_pause_spatial_mapping(CameraID, status);
        }

        /// <summary>
        /// Returns the mesh generation status.
        ///
        /// Useful for knowing when to update and retrieve the mesh.
        /// </summary>
        /// <returns>sl.ERROR_CODE.SUCCESS if the mesh is ready and not yet retrieved, otherwise sl.ERROR_CODE.FAILURE.</returns>
        public sl.ERROR_CODE GetMeshRequestStatus()
        {
            return (sl.ERROR_CODE)dllz_get_mesh_request_status_async(CameraID);
        }

        /// <summary>
        /// Saves the scanned mesh in a specific file format.
        /// </summary>
        /// <param name="filename">Path and filename of the mesh.</param>
        /// <param name="format">File format (extension). Can be .obj, .ply or .bin.</param>
        /// <returns>Has the mesh been saved successfully.</returns>
        public bool SaveMesh(string filename, MESH_FILE_FORMAT format)
        {
            return dllz_save_mesh(CameraID, filename, format);
        }

        /// <summary>
        /// Saves the scanned point cloud in a specific file format.
        /// </summary>
        /// <param name="filename">Path and filename of the point cloud.</param>
        /// <param name="format">File format (extension). Can be .obj, .ply or .bin.</param>
        /// <returns>Has the point cloud been saved successfully.</returns>
        public bool SavePointCloud(string filename, MESH_FILE_FORMAT format)
        {
            return dllz_save_point_cloud(CameraID, filename, format);
        }

        /// <summary>
        /// Loads a saved mesh file.
        ///
        /// ZEDSpatialMapping then configures itself as if the loaded mesh was just scanned.
        /// </summary>
        /// <param name="filename">Path and filename of the mesh. Should include the extension (.obj, .ply or .bin).</param>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each sub-mesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each sub-mesh.</param>
        /// <param name="nbSubmeshes">Number of sub-meshes.</param>
        /// <param name="updatedIndices">List of all sub-meshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all sub-meshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all sub-meshes.</param>
        /// <param name="nbSubmeshMax">Maximum number of sub-meshes that can be handled.</param>
        /// <param name="textureSize">Array containing the sizes of all the textures (width, height) if applicable.</param>
        /// <returns>Has the mesh been loaded successfully.</returns>
        public bool LoadMesh(string filename, int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices,
            ref int nbVertices, ref int nbTriangles, int nbSubmeshMax, int[] textureSize = null)
        {
            return dllz_load_mesh(CameraID, filename, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices,
                ref nbTriangles, textureSize, nbSubmeshMax);
        }

        /// <summary>
        /// Filters a mesh to remove triangles while still preserving its overall shape (though less accurate).
        /// </summary>
        /// <param name="filterParameters">Filter level. Higher settings remove more triangles.</param>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each sub-mesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each sub-mesh.</param>
        /// <param name="nbSubmeshes">Number of sub-meshes.</param>
        /// <param name="updatedIndices">List of all sub-meshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all sub-meshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all sub-meshes.</param>
        /// <param name="nbSubmeshMax">Maximum number of sub-meshes that can be handled.</param>
        /// <returns>Has the mesh been filtered successfully.</returns>
        public bool FilterMesh(MESH_FILTER filterParameters, int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmeshMax)
        {
            return dllz_filter_mesh(CameraID, filterParameters, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, nbSubmeshMax);
        }
        /// <summary>
        /// Filters a mesh to remove triangles while still preserving its overall shape (though less accurate).
        /// </summary>
        /// <param name="filterParameters">Filter level. Higher settings remove more triangles.</param>
        /// <param name="mesh">The mesh to be filled with the generated spatial map.</param>
        /// <returns>Has the mesh been filtered successfully.</returns>
        public bool FilterMesh(MESH_FILTER filterParameters, ref Mesh mesh)
        {
            return dllz_filter_mesh(CameraID, filterParameters, mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles,
                                    (int)Constant.MAX_SUBMESH);
        }

        /// <summary>
        /// Applies the scanned texture onto the internal scanned mesh.
        /// </summary>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each sub-mesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each sub-mesh.</param>
        /// <param name="nbSubmeshes">Number of sub-meshes.</param>
        /// <param name="updatedIndices">List of all sub-meshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all sub-meshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all sub-meshes.</param>
        /// <param name="textureSize"> Vector containing the size of all the texture (width, height). </param>
        /// <param name="nbSubmeshMax">Maximum number of sub-meshes that can be handled.</param>
        /// <returns>Has the texture been applied successfully.</returns>
        public bool ApplyTexture(int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int[] textureSize, int nbSubmeshMax)
        {
            return dllz_apply_texture(CameraID, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, textureSize, nbSubmeshMax);
        }

        /// <summary>
        /// Applies the texture on a mesh.
        /// </summary>
        /// <param name="mesh">Mesh with a texture to apply.</param>
        /// <returns>Has the texture been applied successfully.</returns>
        public bool ApplyTexture(ref Mesh mesh)
        {
            return dllz_apply_texture(CameraID, mesh.nbVerticesInSubmesh, mesh.nbTrianglesInSubmesh, ref mesh.nbUpdatedSubmesh, mesh.updatedIndices, ref mesh.nbVertices, ref mesh.nbTriangles, mesh.texturesSize, (int)Constant.MAX_SUBMESH);
        }

        /// <summary>
        ///  Returns the current spatial mapping state.
        ///
        /// As the spatial mapping runs asynchronously, this method allows you to get reported errors or status info.
        /// </summary>
        /// <returns>The current \ref SPATIAL_MAPPING_STATE "state" of the spatial mapping process.</returns>
        public SPATIAL_MAPPING_STATE GetSpatialMappingState()
        {
            return (sl.SPATIAL_MAPPING_STATE)dllz_get_spatial_mapping_state(CameraID);
        }

        /// <summary>
        /// Gets a vector pointing toward the direction of gravity.
        ///
        /// This is estimated from a 3D scan of the environment, and as such, a scan must be started/finished for this value to be calculated.
        /// If using a camera other than \ref MODEL "MODEL.ZED", this is not required thanks to its IMU.
        /// </summary>
        /// <returns>Vector3 pointing downward.</returns>
        public Vector3 GetGravityEstimate()
        {
            Vector3 v = Vector3.Zero;
            dllz_spatial_mapping_get_gravity_estimation(CameraID, ref v);
            return v;
        }

        /// <summary>
        /// Consolidates the chunks from a scan.
        ///
        /// This is used to turn lots of small meshes (which are efficient for the scanning process)
        /// into several large meshes (which are more convenient to work with).
        /// </summary>
        /// <param name="numberFaces"></param>
        /// <param name="nbVerticesInSubmeshes">Array of the number of vertices in each sub-mesh.</param>
        /// <param name="nbTrianglesInSubmeshes">Array of the number of triangles in each sub-mesh.</param>
        /// <param name="nbSubmeshes">Number of sub-meshes.</param>
        /// <param name="updatedIndices">List of all sub-meshes updated since the last update.</param>
        /// <param name="nbVertices">Total number of updated vertices in all sub-meshes.</param>
        /// <param name="nbTriangles">Total number of updated triangles in all sub-meshes.</param>
        public void MergeChunks(int numberFaces, int[] nbVerticesInSubmeshes, int[] nbTrianglesInSubmeshes, ref int nbSubmeshes, int[] updatedIndices, ref int nbVertices, ref int nbTriangles, int nbSubmesh)
        {
            dllz_spatial_mapping_merge_chunks(CameraID, numberFaces, nbVerticesInSubmeshes, nbTrianglesInSubmeshes, ref nbSubmeshes, updatedIndices, ref nbVertices, ref nbTriangles, nbSubmesh);
        }

        ///@}

        ///@{
        /// @name Plane Detection


        ///////////////////////////// PLANE DETECTION ////////////////////////////////


        /// <summary>
        /// Detect the floor plane of the scene.
        /// 
        /// Use ZEDPlaneDetectionManager.DetectFloorPlane for a higher-level version that turns planes into GameObjects.
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="playerHeight">Height of the camera from the newly-detected floor.</param>
        /// <param name="priorQuat">Prior rotation.</param>
        /// <param name="priorTrans">Prior position.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        [Obsolete("This Method is Deprecated, use FindFloorPlane instead", false)]
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
        /// Detect the floor plane of the scene.
        /// 
        /// Use ZEDPlaneDetectionManager.DetectFloorPlane for a higher-level version that turns planes into GameObjects.
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="playerHeight">Height of the camera from the newly-detected floor.</param>
        /// <param name="priorQuat">Prior rotation.</param>
        /// <param name="priorTrans">Prior position.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE FindFloorPlane(ref PlaneData plane, out float playerHeight, Quaternion priorQuat, Vector3 priorTrans)
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
        /// data needed to make a mesh that represents it.
        ///
        /// These arrays are updated directly from the wrapper.
        /// </summary>
        /// <param name="vertices">Array to be filled with mesh vertices.</param>
        /// <param name="triangles">Array to be filled with mesh triangles, stored as indexes of each triangle's points.</param>
        /// <param name="numVertices">Total vertices in the mesh.</param>
        /// <param name="numTriangles">Total triangle indexes (3x number of triangles).</param>
        /// <returns>0 is the method was successful, 1 otherwise.</returns>
        [Obsolete("This Method is Deprecated, use ConvertFloorPlaneToMesh instead", false)]
        public int convertFloorPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_floorplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
        }

        /// <summary>
        /// Using data from a detected floor plane, updates supplied vertex and triangle arrays with
        /// data needed to make a mesh that represents it.
        ///
        /// These arrays are updated directly from the wrapper.
        /// </summary>
        /// <param name="vertices">Array to be filled with mesh vertices.</param>
        /// <param name="triangles">Array to be filled with mesh triangles, stored as indexes of each triangle's points.</param>
        /// <param name="numVertices">Total vertices in the mesh.</param>
        /// <param name="numTriangles">Total triangle indexes (3x number of triangles).</param>
        /// <returns>0 is the method was successful, 1 otherwise.</returns>
        public int ConvertFloorPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_floorplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
        }

        /// <summary>
        /// Checks the plane at the given left image coordinates.
        /// </summary>
        /// <param name="plane">The detected plane if the method succeeded.</param>
        /// <param name="coord">The image coordinate. The coordinate must be taken from the full-size image.</param>
        /// <param name="parameters">A structure containing all the specific parameters for the plane detection. Default: a preset of PlaneDetectionParameters.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        [Obsolete("This Method is Deprecated, use FindPlaneAtHit instead", false)]
        public sl.ERROR_CODE findPlaneAtHit(ref PlaneData plane, Vector2 coord, ref PlaneDetectionParameters planeDetectionParameters)
        {
            IntPtr p = IntPtr.Zero;
            Quaternion out_quat = Quaternion.Identity;
            Vector3 out_trans = Vector3.Zero;

            sl_PlaneDetectionParameters plane_params = new sl_PlaneDetectionParameters();
            plane_params.maxDistanceThreshold = planeDetectionParameters.maxDistanceThreshold;
            plane_params.normalSimilarityThreshold= planeDetectionParameters.normalSimilarityThreshold;

            p = dllz_find_plane_at_hit(CameraID, coord, ref plane_params, false);
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
        /// Checks the plane at the given left image coordinates.
        /// </summary>
        /// <param name="plane">The detected plane if the method succeeded.</param>
        /// <param name="coord">The image coordinate. The coordinate must be taken from the full-size image.</param>
        /// <param name="parameters">A structure containing all the specific parameters for the plane detection. Default: a preset of PlaneDetectionParameters.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE FindPlaneAtHit(ref PlaneData plane, Vector2 coord, ref PlaneDetectionParameters planeDetectionParameters)
        {
            IntPtr p = IntPtr.Zero;
            Quaternion out_quat = Quaternion.Identity;
            Vector3 out_trans = Vector3.Zero;

            sl_PlaneDetectionParameters plane_params = new sl_PlaneDetectionParameters();
            plane_params.maxDistanceThreshold = planeDetectionParameters.maxDistanceThreshold;
            plane_params.normalSimilarityThreshold = planeDetectionParameters.normalSimilarityThreshold;

            p = dllz_find_plane_at_hit(CameraID, coord, ref plane_params, false);
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
        /// data needed to make a mesh that represents it.
        ///
        /// These arrays are updated directly from the wrapper.
        /// </summary>
        /// <param name="vertices">Array to be filled with mesh vertices.</param>
        /// <param name="triangles">Array to be filled with mesh triangles, stored as indexes of each triangle's points.</param>
        /// <param name="numVertices">Total vertices in the mesh.</param>
        /// <param name="numTriangles">Total triangle indexes (3x number of triangles).</param>
        /// <returns>0 is the method was successful, 1 otherwise.</returns>
        [Obsolete("This Method is Deprecated, use ConvertHitPlaneToMesh instead", false)]
        public int convertHitPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_hitplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
        }

        /// <summary>
        /// Using data from a detected hit plane, updates supplied vertex and triangle arrays with
        /// data needed to make a mesh that represents it.
        ///
        /// These arrays are updated directly from the wrapper.
        /// </summary>
        /// <param name="vertices">Array to be filled with mesh vertices.</param>
        /// <param name="triangles">Array to be filled with mesh triangles, stored as indexes of each triangle's points.</param>
        /// <param name="numVertices">Total vertices in the mesh.</param>
        /// <param name="numTriangles">Total triangle indexes (3x number of triangles).</param>
        /// <returns>0 is the method was successful, 1 otherwise.</returns>
        public int ConvertHitPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
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
        /// Creates an SVO file to be filled by EnableRecording() and DisableRecording().
        /// \note An SVO is Stereolabs' own format designed for the ZED.
        /// It holds the video feed with timestamps as well as info about the camera used to record it.
        /// </summary>
        /// <param name="videoFileName">Filename of the recording. Whether it ends with .svo or .avi defines its file type.</param>
        /// <param name="compressionMode">The compression to use for recording.</param>
        /// <param name="bitrate">Override default bitrate with a custom bitrate (Kbits/s).</param>
        /// <param name="targetFPS">Use another fps than camera FPS. Must respect camera_fps%target_fps == 0.</param>
        /// <param name="transcode">If input is in streaming mode, dump directly into SVO file (transcode=false) or decode/encode (transcode=true).</param>
        /// <returns>An sl.ERROR_CODE that defines if the file was successfully created and can be filled with images.</returns>
        public ERROR_CODE EnableRecording(string videoFileName, SVO_COMPRESSION_MODE compressionMode = SVO_COMPRESSION_MODE.H264_BASED, uint bitrate = 0, int targetFPS = 0, bool transcode = false)
        {
            return (ERROR_CODE)dllz_enable_recording(CameraID, StringUtf8ToByte(videoFileName), (int)compressionMode, bitrate, targetFPS, transcode);
        }

        /// <summary>
        /// Creates an SVO file to be filled by EnableRecording() and DisableRecording().
        /// </summary>
        /// <param name="videoFileName">A structure containing all the specific parameters for the positional tracking. Default: a reset of RecordingParameters.</param>
        /// <returns>An sl.ERROR_CODE that defines if the file was successfully created and can be filled with images.</returns>
        public ERROR_CODE EnableRecording(RecordingParameters recordingParameters)
        {
            return (ERROR_CODE)dllz_enable_recording(CameraID, StringUtf8ToByte(recordingParameters.videoFilename), (int)recordingParameters.compressionMode, recordingParameters.bitrate,
                    recordingParameters.targetFPS, recordingParameters.transcode);
        }

        /// <summary>
        /// Get the recording information.
        /// </summary>
        /// <returns>The recording state structure. For more details, see \ref RecordingStatus.</returns>
        public sl.RecordingStatus GetRecordingStatus()
        {
            IntPtr p = dllz_get_recording_status(CameraID);

            if (p == IntPtr.Zero)
            {
                return new RecordingStatus();
            }
            RecordingStatus parameters = (RecordingStatus)Marshal.PtrToStructure(p, typeof(RecordingStatus));

            return parameters;
        }

        /// <summary>
        /// Returns the RecordingParameters used.
        /// 
        /// It corresponds to the structure given as argument to the EnableRecording() method.
        /// </summary>
        /// <returns>sl.RecordingParameters containing the parameters used for recording initialization.</returns>
        public sl.RecordingParameters GetRecordingParameters()
        {
            IntPtr p = dllz_get_recording_parameters(CameraID);

            if (p == IntPtr.Zero)
            {
                return new RecordingParameters();
            }
            RecordingParameters parameters = (RecordingParameters)Marshal.PtrToStructure(p, typeof(RecordingParameters));

            return parameters;
        }

        /// <summary>
        /// Pauses or resumes the recording.
        /// </summary>
        /// <param name="status">If true, the recording is paused. If false, the recording is resumed.</param>
        public void PauseRecording(bool status)
        {
            dllz_pause_recording(CameraID, status);
        }

        /// <summary>
        /// Disables the recording initiated by EnableRecording() and closes the generated file.
        /// </summary>
        public void DisableRecording()
        {
            dllz_disable_recording(CameraID);
        }

        /// <summary>
        /// Ingest SVOData in a SVO file.
        /// </summary>
        /// <param name="data">Data to ingest in the SVO file..</param>
        /// Note: The method works only if the camera is recording.
        /// <returns></returns>
        public ERROR_CODE IngestDataIntoSVO(ref SVOData data)
        {
            ERROR_CODE err = dllz_ingest_data_into_svo(CameraID, ref data);
            return err;
        }

        /// <summary>
        /// Retrieves SVO data from the SVO file at the given channel key and in the given timestamp range.
        /// </summary>
        /// <param name="key"> The key of the SVOData that is going to be retrieved.</param>
        /// <param name="data"> The map to be filled with SVOData objects, with timestamps as keys.</param>
        /// <param name="tsBegin"> The beginning of the range.</param>
        /// <param name="tsEnd">The end of the range.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS in case of success, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public ERROR_CODE RetrieveSVOData(string key, ref List<SVOData> data, ulong tsBegin = 0, ulong tsEnd = 0)
        {
            ERROR_CODE err = ERROR_CODE.FAILURE;

            int nbData = dllz_get_svo_data_size(CameraID, key, tsBegin, tsEnd);
            if (nbData > 0)
            {
                IntPtr[] dataPtr = new IntPtr[nbData];
                err = dllz_retrieve_svo_data(CameraID, key, nbData, out dataPtr[0], tsBegin, tsEnd);

                SVOData[] data_array = new SVOData[nbData];
                for (int i = 0; i < nbData; i++)
                {
                    if (dataPtr[i] != IntPtr.Zero)
                    {
                        data_array[i] = (SVOData)Marshal.PtrToStructure(dataPtr[i], typeof(SVOData));
                        // Free memory allocated by C (only if allocated in C)
                        Marshal.FreeHGlobal(dataPtr[i]);
                    }
                    else
                    {
                        Console.WriteLine(i + " is null");
                    }
                }

                data = new List<SVOData>(data_array);
            }

            return err;
        }

        /// <summary>
        ///  Gets the external channels that can be retrieved from the SVO file.
        /// </summary>
        /// <returns>List of available keys.</returns>
        public List<string> GetSVODataKeys()
        {
            int nbKeys = dllz_get_svo_data_keys_size(CameraID);

            if (nbKeys > 0)
            {
                // Allocate memory for string pointers
                IntPtr[] keysPtr = new IntPtr[nbKeys];

                dllz_get_svo_data_keys(CameraID, nbKeys, out keysPtr[0]);

                // Retrieve strings
                string[] keys = new string[nbKeys];
                for (int i = 0; i < nbKeys; i++)
                {
                    keys[i] = Marshal.PtrToStringAnsi(keysPtr[i]);

                    // Free memory allocated by C (only if allocated in C)
                    Marshal.FreeHGlobal(keysPtr[i]);
                }

                List<string> list = new List<string>(keys);

                return list;
            }

            return new List<string>();
        }

        ///@}

        ///@{
        /// @name Streaming

        ///////////////////////////// Streaming Module ////////////////////////////////

        /// <summary>
        /// Creates an streaming pipeline.
        /// </summary>
        /// <param name="codec">Defines the codec used for streaming.</param>
        /// <param name="bitrate">Defines the streaming bitrate in Kbits/s.</param>
        /// <param name="port">Defines the port used for streaming.</param>
        /// <param name="gopSize">Defines the gop size in number of frames.</param>
        /// <param name="adaptativeBitrate">Enable/Disable adaptive bitrate.</param>
        /// <param name="chunkSize">Defines a single chunk size.</param>
        /// <param name="targetFPS">Defines the target framerate for the streaming output.</param>
        /// <returns>An sl.ERROR_CODE that defines if the streaming pipe was successfully created.</returns>
        public ERROR_CODE EnableStreaming(STREAMING_CODEC codec = STREAMING_CODEC.H264_BASED, uint bitrate = 8000, ushort port = 30000, int gopSize = -1, bool adaptativeBitrate = false, int chunkSize = 32768, int targetFPS = 0)
        {
            int doAdaptBitrate = adaptativeBitrate ? 1 : 0;
            return (ERROR_CODE)dllz_enable_streaming(CameraID, codec, bitrate, port, gopSize, doAdaptBitrate, chunkSize, targetFPS);
        }

        /// <summary>
        /// Creates an streaming pipeline.
        /// </summary>
        /// <param name="streamingParameters">
        /// A structure containing all the specific parameters for the streaming. Default: a preset of StreamingParameters.
        /// </param>
        /// <returns>An sl.ERROR_CODE that defines if the streaming pipe was successfully created.</returns>
        public ERROR_CODE EnableStreaming(ref StreamingParameters streamingParameters)
        {
            int doAdaptBitrate = streamingParameters.adaptativeBitrate ? 1 : 0;
            return (ERROR_CODE)dllz_enable_streaming(CameraID, streamingParameters.codec, streamingParameters.bitrate, streamingParameters.port, streamingParameters.gopSize, doAdaptBitrate, streamingParameters.chunkSize, streamingParameters.targetFPS);
        }

        /// <summary>
        /// Tells if the streaming is running.
        /// </summary>
        /// <returns>Has the streaming been enabled successfully.</returns>
        public bool IsStreamingEnabled()
        {
            int res = dllz_is_streaming_enabled(CameraID);
            if (res == 1)
                return true;
            else
                return false;
        }

        /// <summary>
        /// Disables the streaming initiated by EnableStreaming().
        /// </summary>
        public void DisableStreaming()
        {
            dllz_disable_streaming(CameraID);
        }

        /// <summary>
        /// Returns the StreamingParameters used.
        /// 
        /// It corresponds to the structure given as argument to the EnableStreaming() method.
        /// </summary>
        /// <returns>sl.StreamingParameters containing the parameters used for streaming initialization.</returns>
        public sl.StreamingParameters GetStreamingParameters()
        {
            IntPtr p = dllz_get_streaming_parameters(CameraID);

            if (p == IntPtr.Zero)
            {
                return new StreamingParameters();
            }
            StreamingParameters parameters = (StreamingParameters)Marshal.PtrToStructure(p, typeof(StreamingParameters));

            return parameters;
        }

        ///@}

        ///////////////////////////// Save utils fct ////////////////////////////////

        /// <summary>
        /// Save current image (specified by view) in a file defined by filename.
        ///
        /// Supported formats are JPEG and PNG. \n Filename must end with either .jpg or .png.
        /// </summary>
        /// <param name="side">sl.SIDE on which to save the image.</param>
        /// <param name="filename"> Filename must end with .jpg or .png.</param>
        /// <returns> An sl.ERROR_CODE that indicates the type of error.</returns>
        public sl.ERROR_CODE SaveCurrentImageInFile(sl.VIEW view, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_image(CameraID, view, filename);
            return err;
        }

        /// <summary>
        /// Save the current depth in a file defined by filename.
        ///
        /// Supported formats are PNG, PFM and PGM.
        /// </summary>
        /// <param name="side">sl.SIDE on which to save the depth.</param>
        /// <param name="filename"> Filename must end with .png, .pfm or .pgm.</param>
        /// <returns> An sl.ERROR_CODE that indicates the type of error.</returns>
        public sl.ERROR_CODE SaveCurrentDepthInFile(SIDE side, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_depth(CameraID, (int)side, filename);
            return err;
        }

        /// <summary>
        /// Save the current point cloud in a file defined by filename.
        ///
        /// Supported formats are PLY, VTK, XYZ and PCD.
        /// </summary>
        /// <param name="side">sl.SIDE on which to save the point cloud.</param>
        /// <param name="filename">Filename must end with .ply, .xyz , .vtk or .pcd.</param>
        /// <returns> An sl.ERROR_CODE that indicates the type of error.</returns>
        public sl.ERROR_CODE SaveCurrentPointCloudInFile(SIDE side, String filename)
        {
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_save_current_point_cloud(CameraID, (int)side, filename);
            return err;
        }

        ///@{
        /// @name Object Detection


        ///////////////////////////// Object detection ////////////////////////////////

        /// <summary>
        /// Check if a corresponding optimized engine is found for the requested model based on your rig configuration.
        /// </summary>
        /// <param name="model"> AI model to check.</param>
        /// <param name="gpu_id">ID of the gpu.</param>
        /// <returns>The \ref AI_Model_status "status" of the AI model.</returns>
        public static AI_Model_status CheckAIModelStatus(AI_MODELS model, int gpu_id = 0)
        {
            IntPtr p = dllz_check_AI_model_status(model, gpu_id);
            if (p == IntPtr.Zero)
            {
                return new AI_Model_status();
            }
            AI_Model_status status = (AI_Model_status)Marshal.PtrToStructure(p, typeof(AI_Model_status));

            return status;
        }

        /// <summary>
        /// Optimize the requested model, possible download if the model is not present on the host.
        /// </summary>
        /// <param name="model">AI model to optimize.</param>
        /// <param name="gpu_id">ID of the gpu to optimize on.</param>
        /// <returns>An sl.ERROR_CODE that indicates the type of error.</returns>
        public static sl.ERROR_CODE OptimizeAIModel(AI_MODELS model, int gpu_id = 0)
        {
            return (sl.ERROR_CODE)dllz_optimize_AI_model(model, gpu_id);
        }

        /// <summary>
        /// Initializes and starts object detection module.
        /// </summary>
        /// <param name="od_params">A structure containing all the specific parameters for the object detection. Default: a preset of ObjectDetectionParameters.</param>
        /// <returns>An sl.ERROR_CODE that indicates the type of error.</returns>
        public sl.ERROR_CODE EnableObjectDetection(ref ObjectDetectionParameters od_params)
        {
            sl.ERROR_CODE objDetectStatus = ERROR_CODE.FAILURE;
            objDetectStatus = (sl.ERROR_CODE)dllz_enable_object_detection(CameraID, ref od_params);

            return objDetectStatus;
        }

        /// <summary>
        /// Initializes and starts body tracking module.
        /// </summary>
        /// <param name="bt_params">A structure containing all the specific parameters for the body tracking. Default: a preset of BodyTrackingParameters.</param>
        /// <returns>An sl.ERROR_CODE that indicates the type of error.</returns>
        public sl.ERROR_CODE EnableBodyTracking(ref BodyTrackingParameters bt_params)
        {
            sl.ERROR_CODE btStatus = ERROR_CODE.FAILURE;
            btStatus = (sl.ERROR_CODE)dllz_enable_body_tracking(CameraID, ref bt_params);

            return btStatus;
        }

        /// <summary>
        /// Disable object detection module and release the resources.
        /// </summary>
        /// <param name="instanceID">Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.</param>
        /// <param name="disableAllInstance">Should disable all instances of the object detection module or just <b>instanceID</b>.</param>
        public void DisableObjectDetection(uint instanceID = 0, bool disableAllInstance = false)
        {
            dllz_disable_object_detection(CameraID, instanceID, disableAllInstance);
        }

        /// <summary>
        /// Disable body tracking module and release the resources.
        /// </summary>
        /// <param name="instanceID">Id of the body tracking module instance. Used when multiple instances of the body tracking module module are enabled at the same time.</param>
        /// <param name="disableAllInstance">Should disable all instances of the body tracking module or just <b>instanceID</b>.</param>
        public void DisableBodyTracking(uint instanceID = 0, bool disableAllInstance = false)
        {
            dllz_disable_body_tracking(CameraID, instanceID, disableAllInstance);
        }

        /// <summary>
        /// Returns the ObjectDetectionParameters used.
        ///
        /// It corresponds to the structure given as argument to the EnableObjectDetection() method.
        /// </summary>
        /// <returns>sl.ObjectDetectionParameters containing the parameters used for object detection initialization.</returns>
        public sl.ObjectDetectionParameters GetObjectDetectionParameters()
        {
            IntPtr p = dllz_get_object_detection_parameters(CameraID);

            if (p == IntPtr.Zero)
            {
                return new ObjectDetectionParameters();
            }
            ObjectDetectionParameters parameters = (ObjectDetectionParameters)Marshal.PtrToStructure(p, typeof(ObjectDetectionParameters));

            return parameters;
        }

        /// <summary>
        /// Returns the BodyTrackingParameters used.
        ///
        /// It corresponds to the structure given as argument to the EnableBodyTracking() method.
        /// </summary>
        /// <returns>sl.BodyTrackingParameters containing the parameters used for body tracking initialization.</returns>
        public sl.BodyTrackingParameters GetBodyTrackingParameters()
        {
            IntPtr p = dllz_get_body_tracking_parameters(CameraID);

            if (p == IntPtr.Zero)
            {
                return new BodyTrackingParameters();
            }
            BodyTrackingParameters parameters = (BodyTrackingParameters)Marshal.PtrToStructure(p, typeof(BodyTrackingParameters));

            return parameters;
        }
        
        /// <summary>
        /// Feed the 3D Object tracking method with your own 2D bounding boxes from your own detection algorithm.
        /// </summary>
        /// <param name="objects_in">List of CustomBoxObjectData to feed the object detection.</param>
        /// <param name="instanceID">Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine.</returns>
        public sl.ERROR_CODE IngestCustomBoxObjects(List<CustomBoxObjectData> objects_in)
        {
            return (sl.ERROR_CODE)dllz_ingest_custom_box_objects(CameraID, objects_in.Count, objects_in.ToArray());
        }

        /// <summary>
        /// Feed the 3D Object tracking function with your own 2D bounding boxes with masks from your own detection algorithm.
        /// </summary>
        /// <param name="masks_in"> Masks</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine.</returns>
        public sl.ERROR_CODE IngestCustomMaskObjects(List<CustomMaskObjectData> masks_in)
        {
            return (sl.ERROR_CODE)dllz_ingest_custom_mask_objects(CameraID, masks_in.Count, masks_in.ToArray());
        }

        /// <summary>
        /// Retrieve objects detected by the object detection module.
        /// </summary>
        /// <param name="objs"> Retrieved objects. </param>
        /// <param name="od_params"> Object detection runtime parameters </param>
        /// <param name="instanceID">Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE RetrieveObjects(ref Objects objs, ref ObjectDetectionRuntimeParameters od_params, uint instanceID = 0)
        {
            IntPtr p = Marshal.AllocHGlobal(System.Runtime.InteropServices.Marshal.SizeOf<sl.Objects>());
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_retrieve_objects_data(CameraID, ref od_params, p, instanceID);

            if (p != IntPtr.Zero)
            {
                objs = (sl.Objects)Marshal.PtrToStructure(p, typeof(sl.Objects));
                Marshal.FreeHGlobal(p);
                return err;
            }
            else
            {
                Marshal.FreeHGlobal(p);
                return sl.ERROR_CODE.FAILURE;
            }
        }

        /// <summary>
        /// Set the object detection module instance runtime parameters
        /// By default the object detection module will use the parameters set in the ObjectDetectionRuntimeParameters constructor.
        /// This can be changed at any time, however since the processing is done in parallel, the parameters will be used for the next inference.
        /// This function can be called only on parameters change, the previous values will be applied during inference.
        /// </summary>
        /// <param name="objectDetectionRuntimeParameters"> </param>
        /// <param name="instanceID"></param>
        /// <returns></returns>
        public sl.ERROR_CODE SetObjectDetectionRuntimeParameters(ObjectDetectionRuntimeParameters objectDetectionRuntimeParameters, uint instanceID = 0)
        {
            return (sl.ERROR_CODE)dllz_set_object_detection_runtime_parameters(CameraID, objectDetectionRuntimeParameters, instanceID);
        }

        /// <summary>
        /// Retrieve objects detected by the custom object detection module.
        /// </summary>
        /// <param name="objs">Custom object detection runtime settings, can be changed at each detection. In async mode, the parameters update is applied on the next iteration.</param>
        /// <param name="od_params">The detected objects will be saved into this object. If the object already contains data from a previous detection, it will be updated, keeping a unique ID for the same person.</param>
        /// <param name="instanceID">Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE RetrieveCustomObjects(ref Objects objs, ref CustomObjectDetectionRuntimeParameters od_params, uint instanceID = 0)
        {
            IntPtr p = Marshal.AllocHGlobal(System.Runtime.InteropServices.Marshal.SizeOf<sl.Objects>());
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_retrieve_custom_objects(CameraID, ref od_params, p, instanceID);

            if (p != IntPtr.Zero)
            {
                objs = (sl.Objects)Marshal.PtrToStructure(p, typeof(sl.Objects));
                Marshal.FreeHGlobal(p);
                return err;
            }
            else
            {
                Marshal.FreeHGlobal(p);
                return sl.ERROR_CODE.FAILURE;
            }
        }

        /// <summary>
        /// Set the Object detection module instance runtime parameters when using the Custom model (OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS and CUSTOM_BOX_OBJECTS::CUSTOM_YOLOLIKE_BOX_OBJECTS)
        /// By default the object detection module will use the parameters set in the CustomObjectDetectionRuntimeParameters constructor.
        /// This can be changed at any time, however since the processing is done in parallel, the parameters will be used for the next inference.
        /// This function can be called only on parameters change, the previous values will be applied during inference.
        /// </summary>
        /// <param name="customObjectDetectionRuntimeParameters"> </param>
        /// <param name="instanceID"></param>
        /// <returns></returns>
        public sl.ERROR_CODE SetCustomObjectDetectionRuntimeParameters(CustomObjectDetectionRuntimeParameters customObjectDetectionRuntimeParameters, uint instanceID = 0)
        {
            return (sl.ERROR_CODE)dllz_set_custom_object_detection_runtime_parameters(CameraID, customObjectDetectionRuntimeParameters, instanceID);
        }

        /// <summary>
        /// Retrieve bodies detected by the body tracking module.
        /// </summary>
        /// <param name="objs"> Retrieved bodies. </param>
        /// <param name="bt_params"> Body tracking runtime parameters </param>
        /// <param name="instanceID">Id of the body tracking instance. Used when multiple instances of the body tracking module are enabled at the same time.</param>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went fine, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE RetrieveBodies(ref Bodies bodies, ref BodyTrackingRuntimeParameters bt_params, uint instanceID = 0)
        {
            IntPtr p = Marshal.AllocHGlobal(System.Runtime.InteropServices.Marshal.SizeOf<sl.Bodies>());
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_retrieve_bodies_data(CameraID, ref bt_params, p, instanceID);

            if (p != IntPtr.Zero)
            {
                bodies = (sl.Bodies)Marshal.PtrToStructure(p, typeof(sl.Bodies));
                Marshal.FreeHGlobal(p);
                return err;
            }
            else
            {
                Marshal.FreeHGlobal(p);
                return sl.ERROR_CODE.FAILURE;
            }
        }

        /// <summary>
        /// Set the Body tracking module instance runtime parameters
        /// By default the Body tracking module will use the parameters set in the BodyTrackingRuntimeParameters constructor.
        /// This can be changed at any time, however since the processing is done in parallel, the parameters will be used for the next inference.
        /// This function can be called only on parameters change, the previous values will be applied during inference.
        /// </summary>
        /// <param name="bodyTrackingRuntimeParameters"> </param>
        /// <param name="instanceID"></param>
        /// <returns></returns>
        public sl.ERROR_CODE SetBodyTrackingRuntimeParameters(BodyTrackingRuntimeParameters bodyTrackingRuntimeParameters, uint instanceID = 0)
        {
            return (sl.ERROR_CODE)dllz_set_body_tracking_runtime_parameters(CameraID, bodyTrackingRuntimeParameters, instanceID);
        }

        /// <summary>
        /// Update the batch trajectories and retrieve the number of batches.
        /// </summary>
        /// <param name="nbBatches"> Numbers of batches. </param>
        /// <returns>An sl.ERROR_CODE that indicates the type of error.</returns>
        public sl.ERROR_CODE UpdateObjectsBatch(out int nbBatches)
        {
            return (sl.ERROR_CODE)dllz_update_objects_batch(CameraID, out nbBatches);
        }
        /// <summary>
        /// Retrieve a batch of objects.
        /// \note This method need to be called after RetrieveObjects(), otherwise trajectories will be empty.
        /// \note It also needs to be called after UpdateObjectsBatch() in order to retrieve the number of batch trajectories.
        /// \note To retrieve all the objects' batches, you need to iterate from 0 to nbBatches (retrieved from UpdateObjectsBatch()).
        /// </summary>
        /// <param name="batch_index"> Index of the batch retrieved.</param>
        /// <param name="objectsBatch"> Trajectory that will be filled by the batching queue process.</param>
        /// <returns>An sl.ERROR_CODE that indicates the type of error.</returns>
        public sl.ERROR_CODE GetObjectsBatch(int batch_index, ref ObjectsBatch objectsBatch)
        {
            return (sl.ERROR_CODE)dllz_get_objects_batch_data(CameraID, batch_index, ref objectsBatch.numData, ref objectsBatch.id , ref objectsBatch.label, ref objectsBatch.sublabel,
                ref objectsBatch.trackingState, objectsBatch.positions, objectsBatch.positionCovariances, objectsBatch.velocities, objectsBatch.timestamps, objectsBatch.boundingBoxes2D,
                objectsBatch.boundingBoxes, objectsBatch.confidences, objectsBatch.actionStates, objectsBatch.headBoundingBoxes2D,
                objectsBatch.headBoundingBoxes, objectsBatch.headPositions);
        }
        ///@}

    }
}
