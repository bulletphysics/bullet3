# Microsoft Developer Studio Project File - Name="libcolladadom" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libcolladadom - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libcolladadom.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libcolladadom.mak" CFG="libcolladadom - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libcolladadom - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libcolladadom - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libcolladadom - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libcolladadom\"
# PROP Intermediate_Dir "..\..\out\release6\build\libcolladadom\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\Demos\OpenGL" /I "..\..\Extras\COLLADA_DOM\include" /I "..\..\Extras\COLLADA_DOM\include\1.4" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libcolladadom\libcolladadom.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\Demos\OpenGL" /i "..\..\Extras\COLLADA_DOM\include" /i "..\..\Extras\COLLADA_DOM\include\1.4" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libcolladadom.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libcolladadom - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libcolladadom\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libcolladadom\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\Demos\OpenGL" /I "..\..\Extras\COLLADA_DOM\include" /I "..\..\Extras\COLLADA_DOM\include\1.4" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libcolladadom\libcolladadom.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\Demos\OpenGL" /i "..\..\Extras\COLLADA_DOM\include" /i "..\..\Extras\COLLADA_DOM\include\1.4" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libcolladadom_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "libcolladadom - Win32 Release"
# Name "libcolladadom - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domAccessor.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domAnimation.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domAnimation_clip.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domAsset.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domBind_material.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domBool_array.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domBox.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCamera.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCapsule.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_connect_param.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_newarray_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_newparam.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_param_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_sampler1D.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_sampler2D.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_sampler3D.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_samplerCUBE.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_samplerDEPTH.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_samplerRECT.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_setarray_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_setparam.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_setparam_simple.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_setuser_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCg_surface_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domChannel.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCOLLADA.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCommon_color_or_texture_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCommon_float_or_param_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCommon_newparam_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCommon_transparent_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domConstants.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domController.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domConvex_mesh.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domCylinder.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domEffect.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domEllipsoid.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domExtra.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFloat_array.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domForce_field.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_annotate_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_annotate_type_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_basic_type_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_clearcolor_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_cleardepth_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_clearstencil_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_code_profile.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_colortarget_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_depthtarget_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_include_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_newparam_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_profile_abstract.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_sampler1D_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_sampler2D_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_sampler3D_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_samplerCUBE_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_samplerDEPTH_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_samplerRECT_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_stenciltarget_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_surface_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_surface_format_hint_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_surface_init_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_surface_init_cube_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_surface_init_from_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_surface_init_planar_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domFx_surface_init_volume_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGeometry.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_hook_abstract.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_pipeline_settings.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_sampler1D.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_sampler2D.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_sampler3D.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_samplerCUBE.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_samplerDEPTH.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGl_samplerRECT.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_basic_type_common.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_newparam.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_pipeline_settings.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_sampler_state.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texcombiner_argumentAlpha_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texcombiner_argumentRGB_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texcombiner_command_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texcombiner_commandAlpha_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texcombiner_commandRGB_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texenv_command_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texture_constant_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texture_pipeline.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGles_texture_unit.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGlsl_newarray_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGlsl_newparam.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGlsl_param_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGlsl_setarray_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGlsl_setparam.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGlsl_setparam_simple.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domGlsl_surface_type.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domIDREF_array.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domImage.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInputGlobal.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInputLocal.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInputLocalOffset.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_camera.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_controller.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_effect.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_force_field.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_geometry.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_light.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_material.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_node.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_physics_material.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_physics_model.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_rigid_body.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstance_rigid_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInstanceWithExtra.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domInt_array.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_animation_clips.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_animations.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_cameras.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_controllers.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_effects.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_force_fields.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_geometries.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_images.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_lights.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_materials.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_nodes.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_physics_materials.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_physics_models.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_physics_scenes.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLibrary_visual_scenes.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLight.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLines.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLinestrips.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domLookat.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domMaterial.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domMatrix.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domMesh.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domMorph.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domName_array.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domNode.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domP.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domParam.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domPhysics_material.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domPhysics_model.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domPhysics_scene.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domPlane.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domPolygons.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domPolylist.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domProfile_CG.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domProfile_COMMON.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domProfile_GLES.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domProfile_GLSL.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domRigid_body.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domRigid_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domRotate.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domSampler.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domScale.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domSkew.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domSkin.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domSource.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domSphere.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domSpline.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTapered_capsule.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTapered_cylinder.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTargetableFloat.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTargetableFloat3.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTechnique.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTranslate.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTriangles.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTrifans.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTristrips.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domTypes.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domVertices.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\1.4\dom\domVisual_scene.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\dae.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeArray.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeAtomicType.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeDocument.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeDom.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeElement.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeError.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeErrorHandler.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeIDRef.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMemorySystem.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaAny.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaAttribute.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaChoice.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaCMPolicy.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaElement.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaElementAttribute.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaGroup.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeMetaSequence.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeSIDResolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeStringRef.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeStringTable.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\daeURI.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\dae\domAny.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\modules\LIBXMLPlugin\daeLIBXMLPlugin.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\modules\LIBXMLPlugin\daeLIBXMLResolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\modules\stdErrPlugin\stdErrPlugin.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\COLLADA_DOM\src\modules\STLDatabase\daeSTLDatabase.cpp
# End Source File
# End Group
# End Target
# End Project
