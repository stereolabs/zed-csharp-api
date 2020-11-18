//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============
using System;
using System.Runtime.InteropServices;
using System.Numerics;


namespace Image_capture
{
    class Program
    {

        [STAThread]
        static void Main(string[] args)
        {
            var w = new MainWindow();
            w.ShowDialog();
        }
       
    }
}