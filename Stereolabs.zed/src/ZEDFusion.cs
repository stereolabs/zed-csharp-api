//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============
using System;
using System.Numerics;
using System.Runtime.InteropServices;

namespace sl
{ 
    public class Fusion
    {
        /// <summary>
        /// DLL name, used for extern calls to the wrapper.
        /// </summary>
        const string nameDll = sl.ZEDCommon.NameDLL;

        #region DLL Calls

        /************************************************************************
         * Fusion API Initialisation
         ************************************************************************/

        [DllImport(nameDll, EntryPoint = "sl_fusion_init")]
        private static extern FUSION_ERROR_CODE dllz_fusion_init(ref InitFusionParameters initFusionParams);

        [DllImport(nameDll, EntryPoint = "sl_fusion_process")]
        private static extern FUSION_ERROR_CODE dllz_fusion_process();

        [DllImport(nameDll, EntryPoint = "sl_fusion_subscribe")]
        private static extern FUSION_ERROR_CODE dllz_fusion_subscribe(ref CameraIdentifier uuid, System.Text.StringBuilder jsonConfigFileName, ref Vector3 poseTranslation, ref Quaternion poseRotation);

        [DllImport(nameDll, EntryPoint = "sl_fusion_update_pose")]
        private static extern FUSION_ERROR_CODE dllz_fusion_update_pose(ref CameraIdentifier uuid, ref Vector3 poseTranslation, ref Quaternion poseRotation);

        /************************************************************************
         * Body Tracking Fusion
         ************************************************************************/

        [DllImport(nameDll, EntryPoint = "sl_fusion_enable_body_tracking")]
        private static extern FUSION_ERROR_CODE dllz_fusion_enable_body_tracking(ref BodyTrackingFusionParameters btfParams);

        [DllImport(nameDll, EntryPoint = "sl_fusion_disable_body_tracking")]
        private static extern void dllz_fusion_disable_body_tracking();

        [DllImport(nameDll, EntryPoint = "sl_fusion_retrieve_bodies")]
        private static extern FUSION_ERROR_CODE dllz_fusion_retrieve_bodies(ref Bodies bodies, ref BodyTrackingFusionRuntimeParameters rtparams, CameraIdentifier uuid);

        [DllImport(nameDll, EntryPoint = "sl_fusion_get_process_metrics")]
        private static extern FUSION_ERROR_CODE dllz_fusion_get_process_metrics(ref FusionMetrics metrics);

        /************************************************************************
         * Positional Tracking
         ************************************************************************/

        [DllImport(nameDll, EntryPoint = "sl_fusion_enable_positional_tracking")]
        private static extern FUSION_ERROR_CODE dllz_fusion_enable_positional_tracking(ref PositionalTrackingFusionParameters ptfParams);

        [DllImport(nameDll, EntryPoint = "sl_fusion_get_position")]
        private static extern POSITIONAL_TRACKING_STATE dllz_fusion_get_position(ref Pose pose, REFERENCE_FRAME referenceFrame, COORDINATE_SYSTEM coordinateSystem, UNIT unit, ref CameraIdentifier uuid, POSITION_TYPE retrieveType);

        [DllImport(nameDll, EntryPoint = "sl_fusion_disable_positional_tracking")]
        private static extern void dllz_fusion_disable_positional_tracking();

        [DllImport(nameDll, EntryPoint = "sl_fusion_close")]
        private static extern void dllz_fusion_close();

        /************************************************************************
         * GNSS Fusion
         ************************************************************************/

        [DllImport(nameDll, EntryPoint = "sl_fusion_ingest_gnss_data")]
        private static extern void dllz_fusion_ingest_gnss_data(ref GNSSData data);

        [DllImport(nameDll, EntryPoint = "sl_fusion_get_current_gnss_data")]
        private static extern POSITIONAL_TRACKING_STATE dllz_fusion_get_current_gnss_data(ref GNSSData data);

        [DllImport(nameDll, EntryPoint = "sl_fusion_get_geo_pose")]
        private static extern POSITIONAL_TRACKING_STATE dllz_fusion_get_geo_pose(ref GeoPose pose);

        [DllImport(nameDll, EntryPoint = "sl_fusion_geo_to_camera")]
        private static extern POSITIONAL_TRACKING_STATE dllz_fusion_geo_to_camera(ref LatLng inLatLng, out Pose outPose);

        [DllImport(nameDll, EntryPoint = "sl_fusion_camera_to_geo")]
        private static extern POSITIONAL_TRACKING_STATE dllz_fusion_camera_to_geo(ref Pose inPose, out GeoPose outGeoPose);

        #endregion

        ~Fusion()
        {
            CloseFusion();
        }

        public void CloseFusion()
        {
            dllz_fusion_close();
        }

        public FUSION_ERROR_CODE Init(ref InitFusionParameters initFusionParameters)
        {
            return dllz_fusion_init(ref initFusionParameters);
        }

        public FUSION_ERROR_CODE Process()
        {
            return dllz_fusion_process();
        }

        public FUSION_ERROR_CODE Subscribe(ref CameraIdentifier uuid, string jsonConfigFileName, ref Vector3 poseTranslation, ref Quaternion poseRotation)
        {
            return dllz_fusion_subscribe(ref uuid, new System.Text.StringBuilder(jsonConfigFileName, jsonConfigFileName.Length), ref poseTranslation, ref poseRotation);
        }

        public FUSION_ERROR_CODE UpdatePose(ref CameraIdentifier uuid, ref Vector3 poseTranslation, ref Quaternion poseRotation)
        {
            return dllz_fusion_update_pose(ref uuid, ref poseTranslation, ref poseRotation);
        }

        /************************************************************************
         * Body Tracking Fusion
         ************************************************************************/

        public FUSION_ERROR_CODE EnableBodyTrackingFusion(ref BodyTrackingFusionParameters btfParams)
        {
            return dllz_fusion_enable_body_tracking(ref btfParams);
        }

        public void DisableBodyTrackingFusion()
        {
            dllz_fusion_disable_body_tracking();
        }

        public FUSION_ERROR_CODE RetrieveBodiesFusion(ref Bodies bodies, ref BodyTrackingFusionRuntimeParameters rtparams, CameraIdentifier uuid)
        {
            return dllz_fusion_retrieve_bodies(ref bodies, ref rtparams, uuid);
        }

        public FUSION_ERROR_CODE GetProcessMetricsFusion(ref FusionMetrics metrics)
        {
            return dllz_fusion_get_process_metrics(ref metrics);
        }

        /************************************************************************
         * Positional Tracking
         ************************************************************************/

        public FUSION_ERROR_CODE EnablePositionalTrackingFusion(ref PositionalTrackingFusionParameters ptfParams)
        {
            return dllz_fusion_enable_positional_tracking(ref ptfParams);
        }

        public POSITIONAL_TRACKING_STATE GetPositionFusion(ref Pose pose, REFERENCE_FRAME referenceFrame, COORDINATE_SYSTEM coordinateSystem, UNIT unit, ref CameraIdentifier uuid, POSITION_TYPE retrieveType)
        {
            return dllz_fusion_get_position(ref pose, referenceFrame, coordinateSystem, unit, ref uuid, retrieveType);
        }

        public void DisablePositionalTrackingFusion()
        {
            dllz_fusion_disable_positional_tracking();
        }

        /************************************************************************
         * GNSS Fusion
         ************************************************************************/

        public void IngestGNSSDataFusion(ref GNSSData data)
        {
            dllz_fusion_ingest_gnss_data(ref data);
        }

        public POSITIONAL_TRACKING_STATE GetCurrentGNSSDataFusion(ref GNSSData data)
        {
            return dllz_fusion_get_current_gnss_data(ref data);
        }

        public POSITIONAL_TRACKING_STATE GetGeoPoseFusion(ref GeoPose pose)
        {
            return dllz_fusion_get_geo_pose(ref pose);
        }

        public POSITIONAL_TRACKING_STATE GeoToCameraFusion(ref LatLng inLatLng, out Pose outPose)
        {
            return dllz_fusion_geo_to_camera(ref inLatLng, out outPose);
        }

        public POSITIONAL_TRACKING_STATE CameraToGeoFusion(ref Pose inPose, out GeoPose outGeoPose)
        {
            return dllz_fusion_camera_to_geo(ref inPose, out outGeoPose);
        }
    }
}