using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Threading;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using sl;


namespace Image_capture
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private CancellationTokenSource tokenSource = new CancellationTokenSource();
        private VIEW view = VIEW.LEFT;
        private RuntimeParameters runtimeParameters = new RuntimeParameters();
        private ZEDCamera zedCamera = new ZEDCamera();
        bool isRunning = false;

        private void control_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            tokenSource.Cancel();
            // Make sure it's not grabbing
            while (isRunning) { Thread.Sleep(20); }
            zedCamera.Close();
        }

        public MainWindow()
        {
            InitializeComponent();

     
            // Set configuration parameters
            InitParameters init_params = new InitParameters();
            init_params.depthMode = DEPTH_MODE.ULTRA;
            init_params.resolution = RESOLUTION.HD1080;
 
            // Open the camera
            ERROR_CODE err = zedCamera.Init(ref init_params);
            if (err != ERROR_CODE.SUCCESS)
                Environment.Exit(-1);

        
            // Create a ZEDMat to handle the images
            ZEDMat zedMat = new ZEDMat();
            int Height = zedCamera.ImageHeight;
            int Width = zedCamera.ImageWidth;

            Resolution res = new Resolution((uint)Width, (uint)Height);
            zedMat.Create(res, MAT_TYPE.MAT_8U_C4, MEM.MEM_CPU);

            // Create a writeable bitmap to handle the image we need to show
            WriteableBitmap writeableBitmap = new WriteableBitmap(Width, Height, 96, 96, PixelFormats.Bgra32, null);
            Image.Source = writeableBitmap;

            int stride = (Width * writeableBitmap.Format.BitsPerPixel + 7) / 8;
            int bufferSize = Height * stride;
            byte[] Pixels;
            Int32Rect rect = new Int32Rect(0, 0, Width, Height);

            Task.Factory.StartNew(() =>
            {
               
                while (!tokenSource.Token.IsCancellationRequested)
                {
                    isRunning = true;
                    // Grab an image           
                    if (zedCamera.Grab(ref runtimeParameters) == ERROR_CODE.SUCCESS)
                    {
                        // A new image is available if grab() returns ERROR_CODE::SUCCESS                        
                        if (zedMat.IsInit())
                            err = zedCamera.RetrieveImage(zedMat, view, MEM.MEM_CPU, res);
                         
                        if (err == ERROR_CODE.SUCCESS)
                        {
                            // Convert IntPtr buffer into byte array
                            Pixels = ZEDMat2ByteArray(zedMat);

                            // Draw on Renderer
                            Dispatcher.BeginInvoke(new Action(() =>
                            {
                                writeableBitmap.Lock();
                                writeableBitmap.WritePixels(rect, Pixels, stride, 0);
                                writeableBitmap.AddDirtyRect(rect);
                                writeableBitmap.Unlock();
                            }), DispatcherPriority.Render);
                        }
                    }
                    isRunning = false;
                }


             }, tokenSource.Token);
        }

        byte[] ZEDMat2ByteArray(ZEDMat zedMat)
        {
            int size = zedMat.GetWidth() * zedMat.GetHeight() * 4;
            byte[] Pixels = new byte[size];
            Marshal.Copy(zedMat.GetPtr(),Pixels, 0, size);
            return Pixels;
        }

        private void OnLeftButtonClick(object sender, RoutedEventArgs e)
        {
            view = sl.VIEW.LEFT;
        }

        private void OnDepthButtonClick(object sender, RoutedEventArgs e)
        {
            view = sl.VIEW.DEPTH;
        }

        private void OnNormalsButtonClick(object sender, RoutedEventArgs e)
        {
            view = sl.VIEW.NORMALS;
        }

        private void OnConfidenceValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            int newVal = (int)e.NewValue;
            runtimeParameters.confidenceThreshold = newVal;
        }

        private void OnTextureConfidenceValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            int newVal = (int)e.NewValue;
            runtimeParameters.textureConfidenceThreshold = newVal;
        }

        private void cmbSensingMode_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            runtimeParameters.sensingMode = (sl.SENSING_MODE)SensingModeCombo.SelectedIndex;
        }


    }
}
