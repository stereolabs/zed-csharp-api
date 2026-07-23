cmake_minimum_required(VERSION 3.8)
project(Stereolabs.zed CSharp)

set(ZED_CS_SOURCES
    src/ZEDCamera.cs
    src/ZEDCommon.cs
    src/ZEDMat.cs
    src/ZEDFusion.cs
)

# Set the C# language version, otherwise default 3.0 is taken
set(CMAKE_CSharp_FLAGS "/langversion:7")

# Give each target its own intermediate directory so NuGet restore doesn't collide
# Enable deterministic builds, embedded PDB and Source Link
file(WRITE "${CMAKE_BINARY_DIR}/Directory.Build.props"
[=[<Project>
  <PropertyGroup>
    <BaseIntermediateOutputPath>$(MSBuildProjectDirectory)\obj\$(MSBuildProjectName)\</BaseIntermediateOutputPath>
    <Deterministic>true</Deterministic>
    <ContinuousIntegrationBuild Condition="'$(CI)' == 'true'">true</ContinuousIntegrationBuild>
    <PublishRepositoryUrl>true</PublishRepositoryUrl>
    <EmbedUntrackedSources>true</EmbedUntrackedSources>
  </PropertyGroup>
  <ItemGroup Condition="'$(MSBuildProjectExtension)' == '.csproj'">
    <PackageReference Include="Microsoft.SourceLink.GitHub" Version="8.*" PrivateAssets="All" />
  </ItemGroup>
</Project>
]=])

# Append target framework to output path so net8.0 and net461 don't overwrite each other
# Override DebugType to embed PDB inside the DLL
file(WRITE "${CMAKE_BINARY_DIR}/Directory.Build.targets"
[=[<Project>
  <PropertyGroup>
    <OutDir>$(OutputPath)$(TargetFramework)\</OutDir>
    <DebugType>embedded</DebugType>
  </PropertyGroup>
</Project>
]=])

##############
## .NET 8.0 ##
##############
add_library(${PROJECT_NAME}_net8.0 SHARED ${ZED_CS_SOURCES})
target_compile_options(${PROJECT_NAME}_net8.0 PRIVATE "/platform:x64")
set_target_properties(${PROJECT_NAME}_net8.0 PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}
    DOTNET_SDK "Microsoft.NET.Sdk"
    DOTNET_TARGET_FRAMEWORK "net8.0"
    VS_CONFIGURATION_TYPE ClassLibrary
)
set_property(TARGET ${PROJECT_NAME}_net8.0 PROPERTY VS_DOTNET_REFERENCES
        "Microsoft.CSharp"
        "System.Core"
        "System"
        "System.Numerics"
)
set_property(TARGET ${PROJECT_NAME}_net8.0 PROPERTY VS_DOTNET_DOCUMENTATION_FILE
        "${PROJECT_NAME}.xml"
)

######################
## .NET Framework 4.6.1 ##
######################
add_library(${PROJECT_NAME}_net461 SHARED ${ZED_CS_SOURCES})
target_compile_options(${PROJECT_NAME}_net461 PRIVATE "/platform:x64")
set_target_properties(${PROJECT_NAME}_net461 PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}
    DOTNET_SDK "Microsoft.NET.Sdk"
    DOTNET_TARGET_FRAMEWORK "net461"
    VS_CONFIGURATION_TYPE ClassLibrary
)
set_property(TARGET ${PROJECT_NAME}_net461 PROPERTY VS_DOTNET_REFERENCES
        "Microsoft.CSharp"
        "PresentationCore"
        "PresentationFramework"
        "System"
        "System.Numerics"
)
set_property(TARGET ${PROJECT_NAME}_net461 PROPERTY VS_DOTNET_DOCUMENTATION_FILE
        "${PROJECT_NAME}.xml"
)
