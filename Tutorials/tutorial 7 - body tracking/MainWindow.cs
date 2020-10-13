using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Numerics;
using System.Threading.Tasks;
using OpenGL;
using OpenGL.CoreUI;

namespace sl
{
    class MainWindow
    {
        GLViewer viewer;
        ZEDCamera zedCamera;
        dll_ObjectDetectionRuntimeParameters obj_runtime_parameters;
        RuntimeParameters runtimeParameters;
        ZEDMat zedMat;
        ObjectsFrameSDK object_frame;

        public MainWindow()
        {
            // Set configuration parameters
            InitParameters init_params = new InitParameters();
            init_params.resolution = RESOLUTION.HD720;
            init_params.cameraFPS = 60;
            init_params.depthMode = DEPTH_MODE.ULTRA;
            init_params.coordinateUnit = UNIT.METER;
            init_params.coordinateSystem = COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP;
            init_params.depthMaximumDistance = 15f;
            
            // Open the camera
            zedCamera = new ZEDCamera(0);
            ERROR_CODE err = zedCamera.Init(ref init_params);

            if (err != ERROR_CODE.SUCCESS)
                Environment.Exit(-1);

            if (zedCamera.CameraModel != sl.MODEL.ZED2)
            {
                Console.WriteLine(" ERROR : Use ZED2 Camera only");
                return;
            }

            // Enable tracking (mandatory for object detection)
            Quaternion quat = Quaternion.Identity;
            Vector3 vec = Vector3.Zero;
            zedCamera.EnableTracking(ref quat, ref vec);

            runtimeParameters = new RuntimeParameters();

            // Enable the Objects detection module
            dll_ObjectDetectionParameters obj_det_params = new dll_ObjectDetectionParameters();
            obj_det_params.enableObjectTracking = true; // the object detection will track objects across multiple images, instead of an image-by-image basis
            obj_det_params.enable2DMask = false;
            obj_det_params.enable_body_fitting = true; // smooth skeletons moves
            obj_det_params.imageSync = true; // the object detection is synchronized to the image
            obj_det_params.detectionModel = sl.DETECTION_MODEL.HUMAN_BODY_ACCURATE;

            zedCamera.EnableObjectsDetection(ref obj_det_params);

            // Create ZED Objects filled in the main loop
            object_frame = new ObjectsFrameSDK();
            zedMat = new ZEDMat();
            int Height = zedCamera.ImageHeight;
            int Width = zedCamera.ImageWidth;

            Resolution res = new Resolution((uint)Width, (uint)Height);
            zedMat.Create(res, MAT_TYPE.MAT_8U_C4, MEM.MEM_CPU);

            // Create OpenGL Viewer
            viewer = new GLViewer(new Resolution((uint)Width, (uint)Height));

            // Configure object detection runtime parameters
            obj_runtime_parameters = new dll_ObjectDetectionRuntimeParameters();
            obj_runtime_parameters.detectionConfidenceThreshold = 20;
            obj_runtime_parameters.objectClassFilter = new int[(int)OBJECT_CLASS.LAST];
            obj_runtime_parameters.objectClassFilter[(int)sl.OBJECT_CLASS.PERSON] = Convert.ToInt32(true);

            // To set a specific threshold
            obj_runtime_parameters.object_confidence_threshold = new int[(int)OBJECT_CLASS.LAST];
            obj_runtime_parameters.object_confidence_threshold[(int)sl.OBJECT_CLASS.PERSON] = 50;

            // Create OpenGL window
            CreateWindow();
        }

        // Create Window
        public void CreateWindow()
        {
            using (NativeWindow nativeWindow = NativeWindow.Create())
            {
                nativeWindow.ContextCreated += NativeWindow_ContextCreated;
                nativeWindow.Render += NativeWindow_Render;
                nativeWindow.KeyDown += (object obj, NativeWindowKeyEventArgs e) =>
                {
                    switch (e.Key)
                    {
                        case KeyCode.Escape:
                            close();
                            nativeWindow.Stop();
                            break;

                        case KeyCode.F:
                            nativeWindow.Fullscreen = !nativeWindow.Fullscreen;
                            break;
                    }
                };

                nativeWindow.Animation = false;
                nativeWindow.MultisampleBits = 4;
                nativeWindow.Create((int)(zedCamera.ImageWidth * 0.05f), (int)(zedCamera.ImageHeight * 0.05f), (uint)(zedCamera.ImageWidth), (uint)(zedCamera.ImageHeight), NativeWindowStyle.Resizeable);
                nativeWindow.Show();
                nativeWindow.Run();
            }
        }

        // Init Window
        private void NativeWindow_ContextCreated(object sender, NativeWindowEventArgs e)
        {
            NativeWindow nativeWindow = (NativeWindow)sender;

            Gl.ReadBuffer(ReadBufferMode.Back);
            Gl.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);

            Gl.Enable(EnableCap.Blend);
            Gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

            Gl.Enable(EnableCap.LineSmooth);
            Gl.Hint(HintTarget.LineSmoothHint, HintMode.Nicest);

            viewer.init(zedCamera.GetCalibrationParameters().leftCam);
        }

        // Render loop
        private void NativeWindow_Render(object sender, NativeWindowEventArgs e)
        {
            NativeWindow nativeWindow = (NativeWindow)sender;
            Gl.Viewport(0, 0, (int)nativeWindow.Width, (int)nativeWindow.Height);
            Gl.Clear(ClearBufferMask.ColorBufferBit);

            ERROR_CODE err = ERROR_CODE.FAILURE;
            if (zedCamera.Grab(ref runtimeParameters) == ERROR_CODE.SUCCESS)
            {
                if (zedMat.IsInit())
                {
                    // Retrieve left image
                    err = zedCamera.RetrieveImage(zedMat, sl.VIEW.LEFT, sl.MEM.MEM_CPU);
                    if (err == ERROR_CODE.SUCCESS)
                    {
                        // Retrieve Objects
                        zedCamera.RetrieveObjectsDetectionData(ref obj_runtime_parameters, ref object_frame);

                        //Update GL View
                        viewer.update(zedMat, object_frame);
                        viewer.render();


                    }
                }
            }
        }

        private void close()
        {
            zedCamera.DisableTracking();
            zedCamera.DisableObjectsDetection();
            zedCamera.Close();
            viewer.exit();
        }
    }
}
