cmake_minimum_required(VERSION 3.23)
project(Stereolabs.zed CSharp)


# Add source to project
add_library(${PROJECT_NAME} SHARED
    src/ZEDCamera.cs
    src/ZEDCommon.cs
    src/ZEDMat.cs
)

# Set the target platform to x64, since ZED SDK does not support 32-bits arch
target_compile_options(${PROJECT_NAME} PRIVATE "/platform:x64" )

# Set the .NET Framework version for the target.
set_target_properties(${PROJECT_NAME} PROPERTIES
    DOTNET_SDK "Microsoft.NET.Sdk"
    DOTNET_TARGET_FRAMEWORK "net6.0")

# Set the C# language version, otherwise default 3.0 is taken
set(CMAKE_CSharp_FLAGS "/langversion:7")

# Set configuration type as Class Library
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_CONFIGURATION_TYPE ClassLibrary)

# Add DotNet references
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DOTNET_REFERENCES
        "Microsoft.CSharp"
        "System.Core"
        "System"
        "System.Numerics"
)

