//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============
using System;
using System.Runtime.InteropServices;

namespace sl
{
    /// <summary>
    /// Represents a 2D vector of uchars for use on both the CPU and GPU. 
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct char2
    {
        public byte r;
        public byte g;
    }

    /// <summary>
    /// Represents a 3D vector of uchars for use on both the CPU and GPU. 
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct char3
    {
        public byte r;
        public byte g;
        public byte b;
    }

    /// <summary>
    /// Represents a 4D vector of uchars for use on both the CPU and GPU. 
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct char4
    {
        [MarshalAs(UnmanagedType.U1)]
        public byte r;
        [MarshalAs(UnmanagedType.U1)]
        public byte g;
        [MarshalAs(UnmanagedType.U1)]
        public byte b;
        [MarshalAs(UnmanagedType.U1)]
        public byte a;
    }

    /// <summary>
    /// Represents a 2D vector of floats for use on both the CPU and GPU. 
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct float2
    {
        /// <summary>
        /// the x component of the float2.
        /// </summary>
        public float x;
        /// <summary>
        /// the y component of the float2.
        /// </summary>
        public float y;
        /// <summary>
        /// Constructor : Creates a float2 whose elements have the specified values.
        /// </summary>
        /// <param name="m_x">value to assign to the x field</param>
        /// <param name="m_y">value to assign to the y field</param>
        public float2(float m_x, float m_y)
        {
            x = m_x;
            y = m_y;
        }

        public float2 add(float2 b)
        {
            return new float2(x + b.x, y + b.y);
        }

        public float2 sub(float2 b)
        {
            return new float2(x - b.x, y - b.y);
        }
    }
    /// <summary>
    /// Represents a 3D vector of floats for use on both the CPU and GPU. 
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct float3
    {
        /// <summary>
        /// The x component of the float3.
        /// </summary>
        public float x;
        /// <summary>
        /// the y component of the float3.
        /// </summary>
        public float y;
        /// <summary>
        /// the z component of the float3.
        /// </summary>
        public float z;

        /// <summary>
        /// Constructor : Creates a float3 whose elements have the specified values.
        /// </summary>
        /// <param name="m_x">value to assign to the x field.</param>
        /// <param name="m_y">value to assign to the y field.</param>
        /// <param name="m_z">value to assign to the z field</param>
        public float3(float m_x, float m_y, float m_z)
        {
            x = m_x;
            y = m_y;
            z = m_z;
        }

        /// <summary>
        /// Returns the addition of two float3
        /// </summary>
        /// <param name="b"></param>
        /// <returns>The second vector to add.</returns>
        public float3 add(float3 b)
        {
            return new float3(x + b.x, y + b.y, z + b.z);
        }
        /// <summary>
        /// Returns the substraction of two float3
        /// </summary>
        /// <param name="b">The second vector.</param>
        /// <returns></returns>
        public float3 sub(float3 b)
        {
            return new float3(x - b.x, y - b.y, z - b.z);
        }
        /// <summary>
        /// Divides the float3 by a specified scalar value
        /// </summary>
        /// <param name="a">The scalar value</param>
        public void divide(float a)
        {
            x /= a;
            y /= a;
            z /= a;
        }
        /// <summary>
        /// Multiplies the float3 by a specified scalar value
        /// </summary>
        /// <param name="a">The scalar value</param>
        /// <returns></returns>
        public float3 multiply(float a)
        {
            return new float3(x * a, y * a, z * a);
        }
        /// <summary>
        /// Returns the length of the float3 
        /// </summary>
        /// <returns></returns>
        public float norm() { return ((float)Math.Sqrt(x * x + y * y + z * z)); }
        /// <summary>
        /// Returns the dot product of two vectors.
        /// </summary>
        /// <param name="b">The second vector</param>
        /// <returns></returns>
        public float dot(float3 b)
        {
            return (x * b.x + y * b.y + z * b.z);
        }
        /// <summary>
        /// Returns the cross product of two vectors
        /// </summary>
        /// <param name="b">The second vector</param>
        /// <returns></returns>
        public float3 cross(float3 b)
        {
            float3 result = new float3();
            result.x = y * b.z - z * b.y;
            result.y = z * b.x - x * b.z;
            result.z = x * b.y - y * b.x;

            return result;
        }
    }
    /// <summary>
    /// Represents a 4D vector of floats for use on both the CPU and GPU. 
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct float4
    {
        /// <summary>
        /// The x component of the float4.
        /// </summary>
        public float x;
        /// <summary>
        /// the y component of the float4.
        /// </summary>
        public float y;
        /// <summary>
        /// the z component of the float4.
        /// </summary>
        public float z;
        /// <summary>
        /// The w component of the float4.
        /// </summary>
        public float w;
    }


    /// \ingroup Core_group
    /// <summary>
    /// Lists available sl.Mat formats.
    /// \note sl.Mat type depends on image or measure type.
    /// \note For the dependencies, see sl.VIEW and sl.MEASURE.
    /// </summary>
    public enum MAT_TYPE
    {
        /// <summary>
        /// 1-channel matrix of float
        /// </summary>
        MAT_32F_C1,
        /// <summary>
        /// 2-channel matrix of float
        /// </summary>
        MAT_32F_C2,
        /// <summary>
        /// 3-channel matrix of float
        /// </summary>
        MAT_32F_C3,
        /// <summary>
        /// 4-channel matrix of float
        /// </summary>
        MAT_32F_C4,
        /// <summary>
        /// 1-channel matrix of unsigned char
        /// </summary>
        MAT_8U_C1,
        /// <summary>
        /// 2-channel matrix of unsigned char
        /// </summary>
        MAT_8U_C2,
        /// <summary>
        /// 3-channel matrix of unsigned char
        /// </summary>
        MAT_8U_C3,
        /// <summary>
        /// 4-channel matrix of unsigned char
        /// </summary>
        MAT_8U_C4,
        /// <summary>
        /// 1-channel matrix of unsigned short
        /// </summary>
        MAT_16U_C1,
        /// <summary>
        /// 4-channel matrix of signed char
        /// </summary>
        MAT_S8_C4
    };

    /// \ingroup Core_group
    /// <summary>
    /// Lists available copy operation on sl.Mat.
    /// </summary>
    public enum COPY_TYPE
    {
        /// <summary>
        /// Copy data from CPU to CPU.
        /// </summary>
        CPU_CPU,
        /// <summary>
        /// Copy data from CPU to GPU.
        /// </summary>
        CPU_GPU,
        /// <summary>
        /// Copy data from GPU to GPU.
        /// </summary>
        GPU_GPU,
        /// <summary>
        /// Copy data from GPU to CPU.
        /// </summary>
        GPU_CPU
    };

    /// \ingroup Core_group
    /// <summary>
    /// Lists available memory type.
    /// </summary>
    public enum MEM
    {
        /// <summary>
        /// Data will be stored on the CPU (processor side).
        /// </summary>
        CPU = 0,
        /// <summary>
        /// Data will be stored on the GPU (graphic card side).
        /// </summary>
        GPU = 1,
        /// <summary>
        /// Data will be stored on both the CPU and GPU
        /// </summary>
        BOTH = 2

    };

    /// \ingroup Core_group
    /// <summary>
    /// Class representing 1 to 4-channel matrix of float or uchar, stored on CPU and/or GPU side.
    /// </summary>
    /// This class is defined in a row-major order, meaning that for an image buffer, the rows are stored consecutively from top to bottom.
    /// \note For more info, read about the ZED SDK C++ class it mirrors:
    /// <a href="https://www.stereolabs.com/docs/api/classsl_1_1Mat.html">Mat</a>
    public class Mat
    {
        #region DLL Calls
        const string nameDll = sl.ZEDCommon.NameDLL;

        [DllImport(nameDll, EntryPoint = "sl_mat_create_new")]
        private static extern IntPtr dllz_mat_create_new(int width, int height, int type, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_create_new_empty")]
        private static extern IntPtr dllz_mat_create_new_empty();


        [DllImport(nameDll, EntryPoint = "sl_mat_is_init")]
        private static extern bool dllz_mat_is_init(System.IntPtr ptr);
        [DllImport(nameDll, EntryPoint = "sl_mat_free")]
        private static extern bool dllz_mat_free(System.IntPtr ptr, int type);
        [DllImport(nameDll, EntryPoint = "sl_mat_get_infos")]
        private static extern bool dllz_mat_get_infos(System.IntPtr ptr, byte[] buffer);


        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_float")]
        private static extern int dllz_mat_get_value_float(System.IntPtr ptr, int x, int y, out float value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_float2")]
        private static extern int dllz_mat_get_value_float2(System.IntPtr ptr, int x, int y, out float2 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_float3")]
        private static extern int dllz_mat_get_value_float3(System.IntPtr ptr, int x, int y, out float3 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_float4")]
        private static extern int dllz_mat_get_value_float4(System.IntPtr ptr, int x, int y, out float4 value, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_uchar")]
        private static extern int dllz_mat_get_value_uchar(System.IntPtr ptr, int x, int y, out byte value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_uchar2")]
        private static extern int dllz_mat_get_value_uchar2(System.IntPtr ptr, int x, int y, out char2 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_uchar3")]
        private static extern int dllz_mat_get_value_uchar3(System.IntPtr ptr, int x, int y, out char3 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_get_value_uchar4")]
        private static extern int dllz_mat_get_value_uchar4(System.IntPtr ptr, int x, int y, out char4 value, int mem);


        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_float")]
        private static extern int dllz_mat_set_value_float(System.IntPtr ptr, int x, int y, ref float value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_float2")]
        private static extern int dllz_mat_set_value_float2(System.IntPtr ptr, int x, int y, ref float2 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_float3")]
        private static extern int dllz_mat_set_value_float3(System.IntPtr ptr, int x, int y, ref float3 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_float4")]
        private static extern int dllz_mat_set_value_float4(System.IntPtr ptr, int x, int y, ref float4 value, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_uchar")]
        private static extern int dllz_mat_set_value_uchar(System.IntPtr ptr, int x, int y, ref byte value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_uchar2")]
        private static extern int dllz_mat_set_value_uchar2(System.IntPtr ptr, int x, int y, ref char2 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_uchar3")]
        private static extern int dllz_mat_set_value_uchar3(System.IntPtr ptr, int x, int y, ref char3 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_value_uchar4")]
        private static extern int dllz_mat_set_value_uchar4(System.IntPtr ptr, int x, int y, ref char4 value, int mem);


        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_float")]
        private static extern int dllz_mat_set_to_float(System.IntPtr ptr, ref float value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_float2")]
        private static extern int dllz_mat_set_to_float2(System.IntPtr ptr, ref float2 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_float3")]
        private static extern int dllz_mat_set_to_float3(System.IntPtr ptr, ref float3 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_float4")]
        private static extern int dllz_mat_set_to_float4(System.IntPtr ptr, ref float4 value, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_uchar")]
        private static extern int dllz_mat_set_to_uchar(System.IntPtr ptr,  ref byte value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_uchar2")]          
        private static extern int dllz_mat_set_to_uchar2(System.IntPtr ptr, ref char2 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_uchar3")]          
        private static extern int dllz_mat_set_to_uchar3(System.IntPtr ptr, ref char3 value, int mem);
        [DllImport(nameDll, EntryPoint = "sl_mat_set_to_uchar4")]
        private static extern int dllz_mat_set_to_uchar4(System.IntPtr ptr, ref char4 value, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_update_cpu_from_gpu")]
        private static extern int dllz_mat_update_cpu_from_gpu(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_update_gpu_from_cpu")]
        private static extern int dllz_mat_update_gpu_from_cpu(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_read")]
        private static extern int dllz_mat_read(System.IntPtr ptr, string filePath);

        [DllImport(nameDll, EntryPoint = "sl_mat_write")]
        private static extern int dllz_mat_write(System.IntPtr ptr, string filePath,int compression_level);

        [DllImport(nameDll, EntryPoint = "sl_mat_copy_to")]
        private static extern int dllz_mat_copy_to(System.IntPtr ptr, System.IntPtr dest, int cpyType);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_width")]
        private static extern int dllz_mat_get_width(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_height")]
        private static extern int dllz_mat_get_height(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_channels")]
        private static extern int dllz_mat_get_channels(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_memory_type")]
        private static extern int dllz_mat_get_memory_type(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_pixel_bytes")]
        private static extern int dllz_mat_get_pixel_bytes(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_step")]
        private static extern int dllz_mat_get_step(System.IntPtr ptr, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_step_bytes")]
        private static extern int dllz_mat_get_step_bytes(System.IntPtr ptr, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_width_bytes")]
        private static extern int dllz_mat_get_width_bytes(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_is_memory_owner")]
        private static extern bool dllz_mat_is_memory_owner(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_resolution")]
        private static extern sl.Resolution dllz_mat_get_resolution(System.IntPtr ptr);

        [DllImport(nameDll, EntryPoint = "sl_mat_alloc")]
        private static extern void dllz_mat_alloc(System.IntPtr ptr, int width, int height, int type, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_set_from")]
        private static extern int dllz_mat_set_from(System.IntPtr ptr, System.IntPtr source, int copyType);

        [DllImport(nameDll, EntryPoint = "sl_mat_get_ptr")]
        private static extern System.IntPtr dllz_mat_get_ptr(System.IntPtr ptr, int mem);

        [DllImport(nameDll, EntryPoint = "sl_mat_clone")]
        private static extern void dllz_mat_clone(System.IntPtr ptr, System.IntPtr ptrSource);

        [DllImport(nameDll, EntryPoint = "sl_mat_convert_color")]
        private static extern int dllz_mat_convert_color(System.IntPtr ptr, int mem, bool swapRBChannels, int cudaStream = 0);

        [DllImport(nameDll, EntryPoint = "sl_convert_color")]
        private static extern int dllz_convert_color(System.IntPtr ptr1, System.IntPtr ptr2, bool swapRBChannels, bool removeAlphaChannel, int mem, int cudaStream = 0);

        [DllImport(nameDll, EntryPoint = "sl_blob_from_image")]
        private static extern int dllz_blob_from_image(System.IntPtr image_in, System.IntPtr tensor_out, sl.Resolution resolution,
                                    float scaleFactor, float3 mean, float3 stdDev, bool keepAspectRatio, bool swapRBChannels,
                                    int cudaStream = 0);

        [DllImport(nameDll, EntryPoint = "sl_blob_from_images")]
        private static extern int dllz_blob_from_images(System.IntPtr[] images_in, int nbImages, System.IntPtr tensor_out, sl.Resolution resolution,
                            float scaleFactor, float3 mean, float3 stdDev, bool keepAspectRatio, bool swapRBChannels,
                            int cudaStream = 0);

        #endregion

        /// <summary>
        /// Returns the internal ptr of a Mat. 
        /// </summary>
        private System.IntPtr _matInternalPtr;
        /// <summary>
        /// Returns the internal ptr of a Mat.
        /// </summary>
        public IntPtr MatPtr
        {
            get { return _matInternalPtr; }
        }

        /// <summary>
        /// Default constructor.
        /// </summary>
        /// Creates an empty sl.Mat.
        public Mat()
        {
            _matInternalPtr = IntPtr.Zero;
        }

        /// <summary>
        /// Constructor.
        /// </summary>
        /// Creates a sl.Mat from an existing internal pointer.
        /// <param name="ptr">Internal pointer to create the sl.Mat with.</param>
        public Mat(System.IntPtr ptr) 
        {
            if(ptr == IntPtr.Zero)
            {
                throw new Exception("ZED Mat not initialized.");
            }
            _matInternalPtr = ptr;
        }

        /// <summary>
        /// Constructor.
        /// </summary>
        /// Creates a sl.Mat with a given sl.Resolution.
        /// <param name="resolution">Size of the matrix in pixels.</param>
        /// <param name="type">Type of the matrix. Depends on texture type (see sl.VIEW and sl.MEASURE).</param>
        /// <param name="mem">Where the buffer will be stored (CPU or GPU memory).
        /// \n Choose depending on where you'll need to access it from.</param>
        public Mat(sl.Resolution resolution, MAT_TYPE type, MEM mem = MEM.CPU)
        {
            _matInternalPtr = dllz_mat_create_new((int)resolution.width, (int)resolution.height, (int)(type), (int)(mem));
        }

        /// <summary>
        /// Creates a sl.Mat with a given sl.Resolution.
        /// </summary>
        /// <param name="resolution">Size of the matrix in pixels.</param>
        /// <param name="type">Type of the matrix. Depends on texture type (see sl.VIEW and sl.MEASURE).</param>
        /// <param name="mem">Where the buffer will be stored (CPU or GPU memory).
        /// \n Choose depending on where you'll need to access it from.</param>
        public void Create(sl.Resolution resolution, MAT_TYPE type, MEM mem = MEM.CPU)
        {
            _matInternalPtr = dllz_mat_create_new((int)resolution.width, (int)resolution.height, (int)(type), (int)(mem));
        }

        /// <summary>
        /// Creates a Mat with a given width and height.
        /// </summary>
        /// <param name="width">Width of the matrix in pixels.</param>
        /// <param name="height">Height of the matrix in pixels..</param>
        /// <param name="type">Type of the matrix. Depends on texture type (see sl.VIEW and sl.MEASURE).</param>
        /// <param name="mem">Where the buffer will be stored (CPU or GPU memory).
        /// \n Choose depending on where you'll need to access it from.</param>
        public void Create(uint width, uint height, MAT_TYPE type, MEM mem = MEM.CPU)
        {
            _matInternalPtr = dllz_mat_create_new((int)width, (int)height, (int)(type), (int)(mem));
        }

        /// <summary>
        /// Whether the sl.Mat has been initialized.
        /// </summary>
        public bool IsInit()
        {
            return dllz_mat_is_init(_matInternalPtr);
        }

        /// <summary>
        /// Frees the memory of the sl.Mat.
        /// </summary>
        /// <param name="mem">Whether the sl.Mat is on CPU or GPU memory.</param>
        public void Free(MEM mem = MEM.BOTH)
        {
            dllz_mat_free(_matInternalPtr, (int)mem);
            _matInternalPtr = IntPtr.Zero;
        }

        /// <summary>
        /// Copies data from the GPU to the CPU, if possible.
        /// </summary>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went well, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE UpdateCPUFromGPU()
        {
            return (sl.ERROR_CODE)dllz_mat_update_cpu_from_gpu(_matInternalPtr);
        }

        /// <summary>
        /// Copies data from the CPU to the GPU, if possible.
        /// </summary>
        /// <returns>sl.ERROR_CODE.SUCCESS if everything went well, sl.ERROR_CODE.FAILURE otherwise.</returns>
        public sl.ERROR_CODE UpdateGPUFromCPU()
        {
            return (sl.ERROR_CODE)dllz_mat_update_gpu_from_cpu(_matInternalPtr);
        }

        /// <summary>
        /// Returns the information about the sl::Mat into a string.
        /// </summary>
        /// <returns>String containing the sl::Mat information.</returns>
        public string GetInfos()
        {
            byte[] buf = new byte[300];
            dllz_mat_get_infos(_matInternalPtr, buf);
            return System.Text.Encoding.ASCII.GetString(buf);
        }

        /// <summary>
        /// Copies data from this sl.Mat to another sl.Mat (deep copy).
        /// </summary>
        /// <param name="dest">sl.Mat that the data will be copied to.</param>
        /// <param name="copyType">The to and from memory types.</param>
        /// <returns>sl.ERROR_CODE indicating if the copy was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE CopyTo(sl.Mat dest, sl.COPY_TYPE copyType = COPY_TYPE.CPU_CPU)
        {
            return (sl.ERROR_CODE)dllz_mat_copy_to(_matInternalPtr, dest._matInternalPtr, (int)(copyType));
        }
        
        /// <summary>
        /// Reads an image from a file.
        /// </summary>
        /// Supports .png and .jpeg.
        /// <param name="filePath">Path of the file to read (including the name and extension).</param>
        /// <returns>sl.ERROR_CODE indicating if the copy was successful, or why it wasn't.</returns>
        /// \note Only works if sl.Mat has access to sl.MEM.CPU.
        public sl.ERROR_CODE Read(string filePath)
        {
            return (sl.ERROR_CODE)dllz_mat_read(_matInternalPtr, filePath);
        }

        /// <summary>
        /// Writes the sl.Mat into a file as an image.
        /// </summary>
        /// <param name="filePath">Path of the file to write in (including the name and extension).</param>
        /// <param name="compression_level"> Compression level used. Highest value means highest compression (smaller size). Range  [0 - 100].</param>
        /// <returns>sl.ERROR_CODE indicating if the copy was successful, or why it wasn't.</returns>
        /// \note Only works if sl.Mat has access to sl.MEM.CPU.
        public sl.ERROR_CODE Write(string filePath,int compressionLevel = -1)
        {
            return (sl.ERROR_CODE)dllz_mat_write(_matInternalPtr, filePath, compressionLevel);
        }

        /// <summary>
        /// Returns the width of the matrix.
        /// </summary>
        public int GetWidth()
        {
            return dllz_mat_get_width(_matInternalPtr);
        }

        /// <summary>
        /// Returns the height of the matrix.
        /// </summary>
        /// <returns></returns>
        public int GetHeight()
        {
            return dllz_mat_get_height(_matInternalPtr);
        }

        /// <summary>
        /// Returns the number of values stored in one pixel.
        /// </summary>
        public int GetChannels()
        {
            return dllz_mat_get_channels(_matInternalPtr);
        }

        /// <summary>
        /// Returns the size of one pixel in bytes.
        /// </summary>
        public int GetPixelBytes()
        {
            return dllz_mat_get_pixel_bytes(_matInternalPtr);
        }

        /// <summary>
        /// Returns the memory step in number of elements (size in one pixel row).
        /// </summary>
        public int GetStep(sl.MEM mem = sl.MEM.CPU)
        {
            return dllz_mat_get_step(_matInternalPtr, (int)mem);
        }

        /// <summary>
        /// Returns the memory step in bytes (size of one pixel row).
        /// </summary>
        /// <returns></returns>
        public int GetStepBytes(sl.MEM mem = sl.MEM.CPU)
        {
            return dllz_mat_get_step_bytes(_matInternalPtr, (int)mem);
        }

        /// <summary>
        /// Returns the size of a row in bytes.
        /// </summary>
        /// <returns></returns>
        public int GetWidthBytes()
        {
            return dllz_mat_get_width_bytes(_matInternalPtr);
        }

        /// <summary>
        /// Returns the type of memory (CPU and/or GPU).
        /// </summary>
        /// <returns></returns>
        public MEM GetMemoryType()
        {
            return (MEM)dllz_mat_get_memory_type(_matInternalPtr);
        }

        /// <summary>
        /// Returns whether the sl.Mat is the owner of the memory it accesses.
        /// </summary>
        /// <returns></returns>
        public bool IsMemoryOwner()
        {
            return dllz_mat_is_memory_owner(_matInternalPtr);
        }

        /// <summary>
        /// Returns the resolution (width and height) of the matrix.
        /// </summary>
        /// <returns></returns>
        public sl.Resolution GetResolution()
        {
            return dllz_mat_get_resolution(_matInternalPtr);
        }

        /// <summary>
        /// Allocates the sl.Mat memory.
        /// </summary>
        /// <param name="width">Width of the image/matrix in pixels.</param>
        /// <param name="height">Height of the image/matrix in pixels.</param>
        /// <param name="matType">Type of matrix (data type and channels - see sl.MAT_TYPE)</param>
        /// <param name="mem">Where the buffer will be stored - CPU memory or GPU memory.</param>
        public void Alloc(uint width, uint height, MAT_TYPE matType, MEM mem = MEM.CPU)
        {
            dllz_mat_alloc(_matInternalPtr, (int)width, (int)height, (int)matType, (int)mem);
        }

        /// <summary>
        /// Allocates the sl.Mat memory.
        /// </summary>
        /// <param name="resolution">Size of the image/matrix in pixels.</param>
        /// <param name="matType">Type of matrix (data type and channels - see sl.MAT_TYPE)</param>
        /// <param name="mem">Where the buffer will be stored - CPU memory or GPU memory.</param>
        public void Alloc(sl.Resolution resolution, MAT_TYPE matType, MEM mem = MEM.CPU)
        {
            dllz_mat_alloc(_matInternalPtr, (int)resolution.width, (int)resolution.height, (int)matType, (int)mem);
        }

        /// <summary>
        /// Copies data from another sl.Mat into this one (deep copy).
        /// </summary>
        /// <param name="src">sl.Mat where the data will be copied from.</param>
        /// <param name="copyType">Specifies the memory that will be used for the copy.</param>
        /// <returns>sl.ERROR_CODE (as an int) indicating if the copy was successful, or why it wasn't.</returns>
        public int SetFrom(Mat src, COPY_TYPE copyType = COPY_TYPE.CPU_CPU)
        {
            return dllz_mat_set_from(_matInternalPtr, src._matInternalPtr, (int)copyType);
        }

        /// <summary>
        /// Returns the CPU or GPU data pointer.
        /// </summary>
        /// <param name="mem">Specifies whether you want sl.MEM.CPU or sl.MEM.GPU.</param>
        public System.IntPtr GetPtr(MEM mem = MEM.CPU)
        {
            return dllz_mat_get_ptr(_matInternalPtr, (int)mem);
        }

        /// <summary>
        /// Duplicates a sl.Mat by copying all its data into a new one (deep copy).
        /// </summary>
        /// <param name="source">sl.Mat to clone.</param>
        public void Clone(Mat source)
        {
            dllz_mat_clone(_matInternalPtr, source._matInternalPtr);
        }

        /************ GET VALUES *********************/
        //Cannot send values by template due to a covariant issue with an out needed.

        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C1.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out float value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_float(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C2.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out float2 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_float2(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C3.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out float3 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_float3(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C4.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out float4 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_float4(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C1.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out byte value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_uchar(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C2.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out char2 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_uchar2(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C3.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out char3 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_uchar3(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /// <summary>
        /// Returns the value of a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C4.
        /// </summary>
        /// <param name="x">Column of the point to get the value from.</param>
        /// <param name="y">Row of the point to get the value from.</param>
        /// <param name="value">Gets filled with the current value.</param>
        /// <param name="mem">Which memory should be read.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE GetValue(int x, int y, out char4 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_get_value_uchar4(_matInternalPtr, x, y, out value, (int)(mem)));
        }
        /***************************************************************************************/
        /************ SET VALUES *********************/
        //Cannot send values by template due to a covariant issue with an out needed.

        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C1.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, ref float value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_float(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C2.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, ref float2 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_float2(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C3.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, ref float3 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_float3(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_32F_C4.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, float4 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_float4(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C1.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, ref byte value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_uchar(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C2.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, ref char2 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_uchar2(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C3.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, ref char3 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_uchar3(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /// <summary>
        /// Sets a value to a specific point in the matrix of type sl.MAT_TYPE.MAT_8U_C4.
        /// </summary>
        /// <param name="x">Column of the point to set the value.</param>
        /// <param name="y">Row of the point to set the value.</param>
        /// <param name="value">Value to which the point will be set.</param>
        /// <param name="mem">Which memory will be updated.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetValue(int x, int y, ref char4 value, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_value_uchar4(_matInternalPtr, x, y, ref value, (int)(mem)));
        }
        /***************************************************************************************/

        /************ SET TO *********************/
        //Cannot send values by template due to a covariant issue with an out needed.

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_32F_C1 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo(ref float value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_float(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_32F_C2 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo(ref float2 value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_float2(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_32F_C3 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo(ref float3 value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_float3(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_32F_C4 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo(ref float4 value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_float4(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_8U_C1 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo(ref byte value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_uchar(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_8U_C2 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo(ref char2 value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_uchar2(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_8U_C3 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo(ref char3 value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_uchar3(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Fills the entire sl.Mat of type sl.MAT_TYPE.MAT_8U_C4 with the given value.
        /// </summary>
        /// <param name="value">Value to be copied all over the matrix.</param>
        /// <param name="mem">Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE SetTo( ref char4 value, sl.MEM mem)
        {
            return (sl.ERROR_CODE)(dllz_mat_set_to_uchar4(_matInternalPtr, ref value, (int)(mem)));
        }

        /// <summary>
        /// Convert the color channels of the Mat (RGB to BGR or RGBA to BGRA).
        /// This methods works only on 8U_C4 or 8U_C3.
        /// </summary>
        /// <param name="mem"> Which buffer to fill, CPU and/or GPU.</param>
        /// <param name="swapRBChannels">Swap blue and red channels.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public sl.ERROR_CODE ConvertColor(sl.MEM mem = sl.MEM.CPU, bool swapRBChannels = false)
        {
            return (sl.ERROR_CODE)dllz_mat_convert_color(_matInternalPtr, (int)mem, swapRBChannels);
        }

        /// <summary>
        /// Convert the color channels of the Mat into another Mat
        /// This methods works only on 8U_C4 if removeAlphaChannel is enabled, or 8U_C4 and 8U_C3 if swapRBChannels is enabled
        /// </summary>
        /// <param name="src">Source image.</param>
        /// <param name="dst">Destination image.</param>
        /// <param name="swapRBChannels">Swap blue and red channels.</param>
        /// <param name="removeAlphaChannel"> Remove alpha channel.</param>
        /// <param name="mem"> Which buffer to fill, CPU and/or GPU.</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public static sl.ERROR_CODE ConvertColor(sl.Mat src, sl.Mat dst, bool swapRBChannels = false, bool removeAlphaChannel = false, sl.MEM mem = sl.MEM.CPU)
        {
            return (sl.ERROR_CODE)dllz_convert_color(src._matInternalPtr, dst._matInternalPtr, swapRBChannels, removeAlphaChannel, (int)mem);
        }

        /// <summary>
        /// Convert an image into a GPU Tensor in planar channel configuration (NCHW), ready to use for deep learning model
        /// </summary>
        /// <param name="image_in">input image to convert</param>
        /// <param name="tensor_out">output GPU tensor</param>
        /// <param name="resolution">resolution of the output image, generally square, although not mandatory</param>
        /// <param name="scaleFactor">Scale factor applied to each pixel value, typically to convert the char value into [0-1] float</param>
        /// <param name="mean"> mean, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the mean would be sl::float3(0.485, 0.456, 0.406)</param>
        /// <param name="stdDev">standard deviation, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the standard deviation would be sl::float3(0.229, 0.224, 0.225)</param>
        /// <param name="keepAspectRatio">indicates if the original width and height ratio should be kept using padding (sometimes refer to as letterboxing) or if the image should be stretched</param>
        /// <param name="swapRBChannels">indicates if the Red and Blue channels should be swapped (RGB-BGR or RGBA-BGRA)</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public static sl.ERROR_CODE BlobFromImage(sl.Mat image_in, sl.Mat tensor_out, sl.Resolution resolution,
                                               float scaleFactor, float3 mean, float3 stdDev, bool keepAspectRatio, 
                                               bool swapRBChannels)
        {
            return (sl.ERROR_CODE)dllz_blob_from_image(image_in._matInternalPtr, tensor_out._matInternalPtr, resolution, scaleFactor, mean, stdDev, keepAspectRatio, swapRBChannels, 0);
        }

        /// <summary>
        /// Convert an image array into a GPU Tensor in planar channel configuration (NCHW), ready to use for deep learning model
        /// </summary>
        /// <param name="image_in">input images to convert</param>
        /// <param name="tensor_out">output GPU tensor</param>
        /// <param name="resolution">resolution of the output image, generally square, although not mandatory</param>
        /// <param name="scaleFactor">Scale factor applied to each pixel value, typically to convert the char value into [0-1] float</param>
        /// <param name="mean"> mean, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the mean would be sl::float3(0.485, 0.456, 0.406)</param>
        /// <param name="stdDev">standard deviation, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the standard deviation would be sl::float3(0.229, 0.224, 0.225)</param>
        /// <param name="keepAspectRatio">indicates if the original width and height ratio should be kept using padding (sometimes refer to as letterboxing) or if the image should be stretched</param>
        /// <param name="swapRBChannels">indicates if the Red and Blue channels should be swapped (RGB-BGR or RGBA-BGRA)</param>
        /// <returns>sl.ERROR_CODE indicating if the get was successful, or why it wasn't.</returns>
        public static sl.ERROR_CODE BlobFromImages(sl.Mat[] images_in, sl.Mat tensor_out, sl.Resolution resolution,
                                       float scaleFactor, float3 mean, float3 stdDev, bool keepAspectRatio, bool swapRBChannels)
        {
            IntPtr[] ptrArray = new IntPtr[images_in.Length];

            for (int i = 0; i < images_in.Length; i++)
            {
                ptrArray[i] = images_in[i]._matInternalPtr;
            }

            return (sl.ERROR_CODE) dllz_blob_from_images(ptrArray, images_in.Length, tensor_out._matInternalPtr, resolution, scaleFactor, mean, stdDev, keepAspectRatio, swapRBChannels, 0);
        }
    }

}
