cmake_minimum_required(VERSION 3.8)
project(Stereolabs.zed CSharp)


# Add source to project
add_library(${PROJECT_NAME} SHARED
    src/ZEDCamera.cs
    src/ZEDCameraSettings.cs
    src/ZEDCommon.cs
    src/ZEDMat.cs
)

# Set the target platform to x64, since ZED SDK does not support 32-bits arch
target_compile_options(${PROJECT_NAME} PRIVATE "/platform:x64" )

# Set the .NET Framework version for the target.
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.6.1")

# Set the C# language version, otherwise default 3.0 is taken
set(CMAKE_CSharp_FLAGS "/langversion:7")


# Set configuration type as Class Library
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_CONFIGURATION_TYPE ClassLibrary)

# Add DotNet references
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DOTNET_REFERENCES
        "Microsoft.CSharp"
        "PresentationCore"
        "PresentationFramework"
        "System"
        "System.Numerics"
)


## INSTALL into resources folder

if (WIN32)
 	SET(DIR_ZED_SDK_BINAIRIES $ENV{ZED_SDK_ROOT_DIR}/bin)	 	
	## Install Stereolabs.zed.dll in C:/Program Files/ZED SDK/bin/
	INSTALL(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${DIR_ZED_SDK_BINAIRIES} COMPONENT library)
	## Install C interface sl_zed_interface.dll in C:/Program Files/ZED SDK/bin/
	INSTALL(FILES  "${CMAKE_CURRENT_SOURCE_DIR}/../Resources/sl_zed_interface.dll" DESTINATION ${DIR_ZED_SDK_BINAIRIES})
endif(WIN32)



