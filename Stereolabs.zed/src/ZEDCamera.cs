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

        private const float Deg2Rad = 0.0174532924F;
        private const float Rad2Deg = 57.29578F;

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
        /// Current ZED resolution setting. Set at initialization.
        /// </summary>
        private RESOLUTION currentResolution;

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
        private static extern int dllz_open(int cameraID, ref sl_initParameters parameters, uint serialNumber, System.Text.StringBuilder svoPath, System.Text.StringBuilder ipStream, int portStream, System.Text.StringBuilder output, System.Text.StringBuilder opt_settings_path, System.Text.StringBuilder opencv_calib_path);

        [DllImport(nameDll, EntryPoint = "sl_start_publishing")]
        private static extern void dllz_start_publishing(int cameraID, System.Text.StringBuilder jsonConfigFileName);

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

        /*
         * Camera control functions.
         */

        [DllImport(nameDll, EntryPoint = "sl_set_camera_settings")]
        private static extern ERROR_CODE dllz_set_camera_settings(int id, int mode, int value);

        [DllImport(nameDll, EntryPoint = "sl_get_camera_settings")]
        private static extern ERROR_CODE dllz_get_camera_settings(int id, VIDEO_SETTINGS settingToRetrieve, ref int value);

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
        private static extern void dllz_set_svo_position(int cameraID, int frame);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_number_of_frames")]
        private static extern int dllz_get_svo_number_of_frames(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_get_svo_position")]
        private static extern int dllz_get_svo_position(int cameraID);


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

        [DllImport(nameDll, EntryPoint = "sl_get_area_export_state")]
        private static extern int dllz_get_area_export_state(int cameraID);

        [DllImport(nameDll, EntryPoint = "sl_set_region_of_interest")]
        private static extern int dllz_sl_set_region_of_interest(int cameraID, IntPtr roiMask);

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
        private static extern int dllz_retrieve_mesh(int cameraID, [In, Out] Vector3[] vertices, int[] triangles, [In, Out] Vector2[] uvs, IntPtr texture, int nbSubmesh);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_chunks")]
        private static extern int dllz_retrieve_chunks(int cameraID, [In, Out] Vector3[] vertices, int[] triangles, [In, Out] Vector2[] uvs, IntPtr texture, int maxSubmesh);

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
        private static extern IntPtr dllz_find_plane_at_hit(int cameraID, Vector2 HitPixel, bool refine);

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

        [DllImport(nameDll, EntryPoint = "sl_pause_body_tracking")]
        private static extern IntPtr dllz_pause_body_tracking(int cameraID, bool status, uint instanceID);

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

        [DllImport(nameDll, EntryPoint = "sl_disable_object_detection")]
        private static extern void dllz_disable_object_detection(int cameraID, uint instanceID, bool forceDisableAllInstances);

        [DllImport(nameDll, EntryPoint = "sl_pause_object_detection")]
        private static extern void dllz_pause_object_detection(int cameraID, bool status, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_ingest_custom_box_objects")]
        private static extern int dllz_ingest_custom_box_objects(int cameraID, int nb_objects, CustomBoxObjectData[] objects_in);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_objects")]
        private static extern int dllz_retrieve_objects_data(int cameraID, ref ObjectDetectionRuntimeParameters od_params, IntPtr objs, uint instanceID);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_bodies")]
        private static extern int dllz_retrieve_bodies_data(int cameraID, ref BodyTrackingRuntimeParameters bt_params, IntPtr objs, uint instanceID);

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
        private static extern int dllz_retrieve_measure(int cameraID, System.IntPtr ptr, int type, int mem, int width, int height);

        [DllImport(nameDll, EntryPoint = "sl_retrieve_image")]
        private static extern int dllz_retrieve_image(int cameraID, System.IntPtr ptr, int type, int mem, int width, int height);

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
        private static uint GetFpsForResolution(RESOLUTION reso)
        {
            if (reso == RESOLUTION.HD1080) return 30;
            else if (reso == RESOLUTION.HD2K) return 15;
            else if (reso == RESOLUTION.HD720) return 60;
            else if (reso == RESOLUTION.VGA) return 100;
            return 30;
        }

        /// <summary>
        /// Generate a UUID like unique ID to help identify and track AI detections.
        /// </summary>
        /// <returns></returns>
        public static string GenerateUniqueID()
        {
            byte[] array = new byte[37];
            int size = dllz_generate_unique_id(array);

            return new string(System.Text.Encoding.ASCII.GetChars(array));
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

            sl_initParameters initP = new sl_initParameters(initParameters); //DLL-friendly version of InitParameters.
            initP.coordinateSystem = initParameters.coordinateSystem; //Left-hand
            int v = dllz_open(CameraID, ref initP, GetCameraInformation().serialNumber , 
                new System.Text.StringBuilder(initParameters.pathSVO, initParameters.pathSVO.Length),
                new System.Text.StringBuilder(initParameters.ipStream, initParameters.ipStream.Length),
                initParameters.portStream,
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
        /// Once destroyed, you need to recreate a camera instance to restart again.
        /// </summary>
        public void Close()
        {
            cameraReady = false;
            dllz_close(CameraID);
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
            sl_RuntimeParameters rt_params = new sl_RuntimeParameters(runtimeParameters);
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
            return (sl.ERROR_CODE)(dllz_retrieve_image(CameraID, mat.MatPtr, (int)view, (int)mem, (int)resolution.width, (int)resolution.height));
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
                asyncGrabCameraRecovery = sl_parameters.asyncGrabCameraRecovery
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
                setGravityAsOrigin = sl_positionalTracking.setGravityAsOrigin
            };

            return trackingParams;
        }

        /// <summary>
        /// Get sl.Resolion from a RESOLUTION enum
        /// </summary>
        /// <param name="resolution"></param>
        /// <returns></returns>
        public static sl.Resolution GetResolution(RESOLUTION resolution)
        {
            sl.Resolution res = new sl.Resolution();
            switch (resolution)
            {
                case RESOLUTION.HD2K: res = new sl.Resolution(2208, 1242); break;
                case RESOLUTION.HD1080: res = new sl.Resolution(1920, 1080); break;
                case RESOLUTION.HD720: res = new sl.Resolution(1280, 720); break;
                case RESOLUTION.VGA: res = new sl.Resolution(672, 376); break;
            }
            return res;
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
            dllz_set_camera_settings(CameraID, (int)settings, value);
        }

        /// <summary>
        /// Gets the value of a given setting from the ZED camera.
        /// </summary>
        /// <param name="settings">Setting to be retrieved (brightness, contrast, gain, exposure, etc.)</param>
        /// <returns>The current value for the corresponding setting. Returns -1 if encounters an error.</returns>
        public int GetCameraSettings(VIDEO_SETTINGS settings)
        {
            AssertCameraIsReady();
            int ret = -1;
            dllz_get_camera_settings(CameraID, settings, ref ret);
            return ret;
        }

        /// <summary>
        /// Overloaded function for CAMERA_SETTINGS.AEC_AGC_ROI (requires Rect as input)
        /// </summary>
        /// <param name="settings"> Must be set to CAMERA_SETTINGS.AEC_AGC_ROI. Otherwise will return ERROR_CODE.FAILURE.</param>
        /// <param name="side"> defines left=0 or right=1 or both=2 sensor target</param>
        /// <param name="roi">the roi defined as a sl.Rect</param>
        /// <param name="reset">Defines if the target must be reset to full sensor</param>
        /// <returns>ERROR_CODE.SUCCESS if ROI has been applied. Other ERROR_CODE otherwise.</returns>
        public ERROR_CODE SetCameraSettings(VIDEO_SETTINGS settings, SIDE side, Rect roi, bool reset = false)
        {
            AssertCameraIsReady();
            if (settings == VIDEO_SETTINGS.AEC_AGC_ROI)
                return dllz_set_roi_for_aec_agc(CameraID, (int)side, roi, reset);
            else
                return ERROR_CODE.FAILURE;
        }

        /// <summary>
        /// Overloaded function for CAMERA_SETTINGS.AEC_AGC_ROI (requires Rect as input)
        /// </summary>
        /// <param name="settings"> Must be set to CAMERA_SETTINGS.AEC_AGC_ROI. Otherwise will return ERROR_CODE.FAILURE.</param>
        /// <param name="side"> defines left=0 or right=1 or both=2 sensor target.</param>
        /// <param name="roi"> Roi that will be filled.</param>
        /// <returns> ERROR_CODE.SUCCESS if ROI has been applied. Other ERROR_CODE otherwise.</returns>
        public ERROR_CODE GetCameraSettings(VIDEO_SETTINGS settings, SIDE side, ref Rect roi)
        {
            AssertCameraIsReady();
            if (settings == VIDEO_SETTINGS.AEC_AGC_ROI)
                return dllz_get_roi_for_aec_agc(CameraID, (int)side, ref roi);
            else
                return ERROR_CODE.FAILURE;
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
            SetCameraSettings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1);
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
            return dllz_get_image_timestamp(CameraID);
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
        /// Reports if the camera has been successfully opened.
        /// </summary>
        /// <returns> Returns true if the ZED is already setup, otherwise false.</returns>
        public bool IsOpened()
        {
            return dllz_is_opened(CameraID);
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
        /// Perform a new self calibration process.
        /// In some cases, due to temperature changes or strong vibrations, the stereo calibration becomes less accurate.
        /// Use this function to update the self-calibration data and get more reliable depth values.
        /// <remarks>The self calibration will occur at the next \ref grab() call.</remarks>
        /// New values will then be available in \ref getCameraInformation(), be sure to get them to still have consistent 2D - 3D conversion.
        /// </summary>
        /// <param name="cameraID"></param>
        /// <returns></returns>
        public void UpdateSelfCalibration()
        {
            dllz_update_self_calibration(CameraID);
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
        /// List all the streaming devices with their associated information.
        /// This function lists all the cameras available and provides their serial number, models and other information.
        /// </summary>
        /// <returns>The device properties for each connected camera</returns>
        public static sl.DeviceProperties[] GetDeviceList(out int nbDevices)
        {
            sl.DeviceProperties[] deviceList = new sl.DeviceProperties[(int)Constant.MAX_CAMERA_PLUGIN];
            dllz_get_device_list(deviceList, out nbDevices);

            return deviceList;
        }

        /// <summary>
        /// List all the connected devices with their associated information.
        /// This function lists all the cameras available and provides their serial number, models and other information.
        /// </summary>
        /// <returns>The device properties for each connected camera</returns>
        public static sl.StreamingProperties[] GetStreamingDeviceList(out int nbDevices)
        {
            sl.StreamingProperties[] streamingDeviceList = new sl.StreamingProperties[(int)Constant.MAX_CAMERA_PLUGIN];
            dllz_get_streaming_device_list(streamingDeviceList, out nbDevices);

            return streamingDeviceList;
        }

        /// <summary>
        /// Performs an hardware reset of the ZED 2.
        /// </summary>
        /// <param name="serialNumber">Serial number of the camera</param>
        /// <param name="fullReboot"> Perform a full reboot (Sensors and Video modules)</param>
        /// <returns>SUCCESS if everything went fine, ERROR_CODE::CAMERA_NOT_DETECTED if no camera was detected, ERROR_CODE.FAILURE otherwise.</returns>
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
            return (sl.ERROR_CODE)(dllz_retrieve_measure(CameraID, mat.MatPtr, (int)measure, (int)mem, (int)resolution.width, (int)resolution.height));
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
        /// Gets the current range of perceived depth.
        /// </summary>
        /// <param name="min">Minimum depth detected (in selected sl.UNIT)</param>
        /// <param name="max">Maximum depth detected (in selected sl.UNIT)</param>
        /// <returns>SUCCESS if values have been extracted. Other ERROR_CODE otherwise.</returns>
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

        ///@}

        ///@{
        /// @name Positional Tracking
        ///
        /// <summary>
        /// Initialize and Start the tracking functions
        /// </summary>
        /// <param name="positionalTrackingParameters"> struct that contains all positional tracking parameters</param>
        /// <returns>ERROR_CODE.FAILURE if the area_file_path file wasn't found, SUCCESS otherwise.</returns>
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

            trackingStatus = (sl.ERROR_CODE)dllz_enable_tracking(CameraID, ref sl_tracking_params, new System.Text.StringBuilder(positionalTrackingParameters.areaFilePath, positionalTrackingParameters.areaFilePath.Length));
            return trackingStatus;
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
        /// Gets the current position of the camera and state of the tracking, with an optional offset to the tracking frame.
        /// </summary>
        /// <returns> true if the tracking module is enabled</returns>
        public bool IsPositionalTrackingEnabled()
        {
            return dllz_is_positional_tracking_enabled(CameraID);
        }


        /// <summary>
        /// Saves the current area learning file. The file will contain spatial memory data generated by the tracking.
        /// </summary>
        /// <param name="path"></param>
        /// <param name=""></param>
        public ERROR_CODE SaveAreaMap(string areaFilePath)
        {
            return (ERROR_CODE)dllz_save_area_map(CameraID, new System.Text.StringBuilder(areaFilePath, areaFilePath.Length));
        }

        /// <summary>
        /// Returns the state of the spatial memory export process.
        /// </summary>
        /// <returns> The current \ref AREA_EXPORTING_STATE "state" of the spatial memory export process</returns>
        public AREA_EXPORT_STATE GetAreaExportState()
        {
            return (AREA_EXPORT_STATE)dllz_get_area_export_state(CameraID);
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
        /// Returns the calibration parameters, serial number and other information about the camera being used.
        /// </summary>
        /// <returns> CameraInformation containing the calibration parameters of the ZED, as well as serial number and firmware version.</returns>
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
        /// Sets a prior to the IMU orientation (not for ZED1).
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

        /// <summary>
        /// Defines a region of interest to focus on for all the SDK, discarding other parts.
        /// </summary>
        /// <param name="roiMask">the Mat defining the requested region of interest, all pixel set to 0 will be discard. If empty, set all pixels as valid, otherwise should fit the resolution of the current instance and its type should be U8_C1.</param>
        /// <returns></returns>
        public ERROR_CODE SetRegionOfInterest(sl.Mat roiMask)
        {
            sl.ERROR_CODE err = sl.ERROR_CODE.FAILURE;

            err = (sl.ERROR_CODE)dllz_sl_set_region_of_interest(CameraID, roiMask.GetPtr());
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
            sl_SpatialMappingParameters map_params = new sl_SpatialMappingParameters();
            map_params.rangeMeter = spatialMappingParameters.rangeMeter;
            map_params.resolutionMeter = spatialMappingParameters.resolutionMeter;
            map_params.saveTexture = spatialMappingParameters.saveTexture;
            map_params.mapType = spatialMappingParameters.map_type;
            map_params.maxMemoryUsage = 4096;
            map_params.useChunkOnly = spatialMappingParameters.useChunkOnly; //spatialMappingParameters.map_type == SPATIAL_MAP_TYPE.MESH ? true : false;
            map_params.reverseVertexOrder = spatialMappingParameters.reverseVertexOrder;

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
        /// <returns>SUCCES if everything went fine, FAILURE otherwise</returns>
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
        /// Returns the spatial mapping parameters used. Correspond to the structure send when the \ref enableSpatialMapping() function was called.
        /// </summary>
        /// <returns>SpatialMappingParameters containing the parameters used for spatial mapping intialization.</returns>
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
                maxMemoryUsage = sl_parameters.maxMemoryUsage
            };
            return parameters;
        }

        /// <summary>
        /// Disables the Spatial Mapping process.
        /// </summary>
        public void DisableSpatialMapping()
        {
            dllz_disable_spatial_mapping(CameraID);
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
            return (sl.ERROR_CODE)dllz_retrieve_mesh(CameraID, vertices, triangles, uvs, textures, nbSubmeshMax);
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

            ERROR_CODE err = (sl.ERROR_CODE)dllz_retrieve_chunks(CameraID, mesh.vertices, mesh.triangles, mesh.uvs, mesh.textures, (int)Constant.MAX_SUBMESH);
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
                ref nbTriangles, textureSize, nbSubmeshMax);
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
        /// Looks for a plane in the visible area that is likely to represent the floor.
        /// Use ZEDPlaneDetectionManager.DetectFloorPlane for a higher-level version that turns planes into GameObjects.
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="playerHeight">Height of the camera from the newly-detected floor.</param>
        /// <param name="priorQuat">Prior rotation.</param>
        /// <param name="priorTrans">Prior position.</param>
        /// <returns></returns>
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
        /// data needed to make a mesh that represents it. These arrays are updated directly from the wrapper.
        /// </summary>
        /// <param name="vertices">Array to be filled with mesh vertices.</param>
        /// <param name="triangles">Array to be filled with mesh triangles, stored as indexes of each triangle's points.</param>
        /// <param name="numVertices">Total vertices in the mesh.</param>
        /// <param name="numTriangles">Total triangle indexes (3x number of triangles).</param>
        /// <returns></returns>
        [Obsolete("This Method is Deprecated, use ConvertFloorPlaneToMesh instead", false)]
        public int convertFloorPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_floorplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
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
        public int ConvertFloorPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_floorplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
        }

        /// <summary>
        /// Checks for a plane in the real world at given screen-space coordinates.
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="screenPos">Point on the ZED image to check for a plane.</param>
        /// <returns></returns>
        [Obsolete("This Method is Deprecated, use FindPlaneAtHit instead", false)]
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
        /// Checks for a plane in the real world at given screen-space coordinates.
        /// </summary>
        /// <param name="plane">Data on the detected plane.</param>
        /// <param name="screenPos">Point on the ZED image to check for a plane.</param>
        /// <returns></returns>
        public sl.ERROR_CODE FindPlaneAtHit(ref PlaneData plane, Vector2 coord)
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
        [Obsolete("This Method is Deprecated, use ConvertHitPlaneToMesh instead", false)]
        public int convertHitPlaneToMesh(Vector3[] vertices, int[] triangles, out int numVertices, out int numTriangles)
        {
            return dllz_convert_hitplane_to_mesh(CameraID, vertices, triangles, out numVertices, out numTriangles);
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
        ///  Get the recording information
        /// </summary>
        /// <returns></returns>
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
        ///  Get the recording parameters
        /// </summary>
        /// <returns></returns>
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
        /// <param name="status"> if true, the recording is paused. If false, the recording is resumed. </param>
        /// <returns></returns>
        public void PauseRecording(bool status)
        {
            dllz_pause_recording(CameraID, status);
        }

        /// <summary>
        /// Stops recording to an SVO/AVI, if applicable, and closes the file.
        /// </summary>
        public void DisableRecording()
        {
            dllz_disable_recording(CameraID);
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
        /// <returns>An ERROR_CODE that defines if the streaming pipe was successfully created</returns>
        public ERROR_CODE EnableStreaming(STREAMING_CODEC codec = STREAMING_CODEC.H264_BASED, uint bitrate = 8000, ushort port = 30000, int gopSize = -1, bool adaptativeBitrate = false, int chunkSize = 32768, int targetFPS = 0)
        {
            int doAdaptBitrate = adaptativeBitrate ? 1 : 0;
            return (ERROR_CODE)dllz_enable_streaming(CameraID, codec, bitrate, port, gopSize, doAdaptBitrate, chunkSize, targetFPS);
        }

        /// <summary>
        /// Creates an streaming pipeline.
        /// </summary>
        /// <params>
        /// Streaming parameters: See sl.StreamingParameters for more informations.
        /// </params>
        /// <returns>An ERROR_CODE that defines if the streaming pipe was successfully created</returns>
        public ERROR_CODE EnableStreaming(ref StreamingParameters streamingParameters)
        {
            int doAdaptBitrate = streamingParameters.adaptativeBitrate ? 1 : 0;
            return (ERROR_CODE)dllz_enable_streaming(CameraID, streamingParameters.codec, streamingParameters.bitrate, streamingParameters.port, streamingParameters.gopSize, doAdaptBitrate, streamingParameters.chunkSize, streamingParameters.targetFPS);
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

        /// <summary>
        ///  Get the streaming parameters
        /// </summary>
        /// <returns></returns>
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
        /// Check if a corresponding optimized engine is found for the requested Model based on your rig configuration.
        /// </summary>
        /// <param name="model"> AI model to check.</param>
        /// <param name="gpu_id">ID of the gpu.</param>
        /// <returns></returns>
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
        /// <returns></returns>
        public static sl.ERROR_CODE OptimizeAIModel(AI_MODELS model, int gpu_id = 0)
        {
            return (sl.ERROR_CODE)dllz_optimize_AI_model(model, gpu_id);
        }

        /// <summary>
        /// Enable object detection module
        /// </summary>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE EnableObjectDetection(ref ObjectDetectionParameters od_params)
        {
            sl.ERROR_CODE objDetectStatus = ERROR_CODE.FAILURE;
            objDetectStatus = (sl.ERROR_CODE)dllz_enable_object_detection(CameraID, ref od_params);

            return objDetectStatus;
        }

        /// <summary>
        /// Enable body tracking module
        /// </summary>
        /// <param name="bt_params">Body Tracking parameters</param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE EnableBodyTracking(ref BodyTrackingParameters bt_params)
        {
            sl.ERROR_CODE btStatus = ERROR_CODE.FAILURE;
            btStatus = (sl.ERROR_CODE)dllz_enable_body_tracking(CameraID, ref bt_params);

            return btStatus;
        }

        /// <summary>
        /// Disable object detection module and release the resources.
        /// instanceID : Id of the object detection instance. Used when multiple instances of the OD module are enabled at the same time.
        /// disableAllInstance : should disable all instances of the object detection module or just instanceID.
        /// </summary>
        public void DisableObjectDetection(uint instanceID = 0, bool disableAllInstance = false)
        {
            dllz_disable_object_detection(CameraID, instanceID, disableAllInstance);
        }

        /// <summary>
        /// Disable body tracking module and release the resources.
        /// instanceID : Id of the body tracking instance. Used when multiple instances of the BT module are enabled at the same time.
        /// disableAllInstance : should disable all instances of the body tracking module or just instanceID.
        /// </summary>
        public void DisableBodyTracking(uint instanceID = 0, bool disableAllInstance = false)
        {
            dllz_disable_body_tracking(CameraID, instanceID, disableAllInstance);
        }

        /// <summary>
        ///  Get the object detections parameters
        /// </summary>
        /// <returns></returns>
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
        ///  Get the body tracking parameters
        /// </summary>
        /// <returns></returns>
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
        /// Pause or Unpause the object detection.
        /// The retrieveObjects function will keep on returning the last objects detected while in pause.
        /// </summary>
        /// <param name="status">True : Pause the OD. False : Unpause the OD.</param>
        /// <param name="instanceID">Id of the Object detection instance. Used when multiple instances of the OD module are enabled at the same time.</param>
        public void PauseObjectDetection(bool status, uint instanceID = 0)
        {
            dllz_pause_object_detection(CameraID, status, instanceID);
        }

        /// <summary>
        /// Pause or Unpause the body tracking.
        /// The RetrieveBodies function will keep on returning the last bodies detected while in pause.
        /// </summary>
        /// <param name="status">True : Pause the BT. False : Unpause the BT.</param>
        /// <param name="instanceID">Id of the Body Tracking instance. Used when multiple instances of the BT module are enabled at the same time.</param>
        public void PauseBodyTracking(bool status, uint instanceID = 0)
        {
            dllz_pause_body_tracking(CameraID, status, instanceID);
        }

        public sl.ERROR_CODE IngestCustomBoxObjects(List<CustomBoxObjectData> objects_in)
        {
            return (sl.ERROR_CODE)dllz_ingest_custom_box_objects(CameraID, objects_in.Count, objects_in.ToArray());
        }

        /// <summary>
        /// Retrieve objects detected by the object detection module. To retrieve Body Tracking data use RetrieveBodies.
        /// </summary>
        /// <param name="objs"> Retrieved objects. </param>
        /// <param name="od_params"> Object detection runtime parameters </param>
        /// <param name="instanceID">Id of the Object detection instance. Used when multiple instances of the OD module are enabled at the same time.</param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
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
        /// Retrieve bodies detected by the Body Tracking module. To retrieve Body Tracking data use RetrieveBodies.
        /// </summary>
        /// <param name="objs"> Retrieved bodies. </param>
        /// <param name="bt_params"> Body Tracking runtime parameters </param>
        /// <param name="instanceID">Id of the Body Tracking instance. Used when multiple instances of the BT module are enabled at the same time.</param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE RetrieveBodies(ref Bodies objs, ref BodyTrackingRuntimeParameters bt_params, uint instanceID = 0)
        {
            IntPtr p = Marshal.AllocHGlobal(System.Runtime.InteropServices.Marshal.SizeOf<sl.Bodies>());
            sl.ERROR_CODE err = (sl.ERROR_CODE)dllz_retrieve_bodies_data(CameraID, ref bt_params, p, instanceID);

            if (p != IntPtr.Zero)
            {
                objs = (sl.Bodies)Marshal.PtrToStructure(p, typeof(sl.Bodies));
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
        /// Update the batch trajectories and retrieve the number of batches.
        /// </summary>
        /// <param name="nbBatches"> numbers of batches </param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
        public sl.ERROR_CODE UpdateObjectsBatch(out int nbBatches)
        {
            return (sl.ERROR_CODE)dllz_update_objects_batch(CameraID, out nbBatches);
        }
        /// <summary>
        /// Retrieve a batch of objects.
        /// This function need to be called after RetrieveObjects, otherwise trajectories will be empty.
        /// If also needs to be called after UpdateOBjectsBatch in order to retrieve the number of batch trajectories.
        /// </summary>
        /// <remarks> To retrieve all the objectsbatches, you need to iterate from 0 to nbBatches (retrieved from UpdateObjectBatches) </remarks>
        /// <param name="batch_index"> index of the batch retrieved. </param>
        /// <param name="objectsBatch"> trajectory that will be filled by the batching queue process </param>
        /// <returns> returns an ERROR_CODE that indicates the type of error </returns>
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
