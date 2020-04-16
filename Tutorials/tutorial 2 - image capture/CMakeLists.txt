cmake_minimum_required( VERSION 3.8.0 )

project(Image_Capture CSharp)

add_executable(${PROJECT_NAME}
  	Program.cs
	App.config
	packages.config
	Properties/AssemblyInfo.cs
	Properties/Resources.Designer.cs
	Properties/Settings.Designer.cs
	App.xaml
	App.xaml.cs
	MainWindow.xaml.cs
	MainWindow.xaml
)
 
# Set the target platform to x64, since ZED SDK does not support 32-bits arch
target_compile_options(${PROJECT_NAME} PRIVATE "/platform:x64" )

# Set the .NET Framework version for the target.
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.6.1")

# Set the C# language version, otherwise default 3.0 is taken
set(CMAKE_CSharp_FLAGS "/langversion:7")

SET(DIR_ZED_SDK_BINAIRIES $ENV{ZED_SDK_ROOT_DIR}/bin)	 
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DOTNET_REFERENCE_Stereolabs.zed "${DIR_ZED_SDK_BINAIRIES}/Stereolabs.zed.dll")
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DOTNET_REFERENCE_sl_zed_interface "${DIR_ZED_SDK_BINAIRIES}/sl_zed_interface.dll")



set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DOTNET_REFERENCES
        "Microsoft.CSharp"
        "PresentationCore"
        "PresentationFramework"
        "System"
		"System.Xaml"
		"System.Data"
		"System.Linq"
		"System.Windows"
        "System.Numerics"
		"WindowsBase"
)
 