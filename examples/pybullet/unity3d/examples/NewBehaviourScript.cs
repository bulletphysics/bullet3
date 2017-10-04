using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System;

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]


public class NewBehaviourScript : MonoBehaviour {

      
    Text text;
    IntPtr sharedAPI;
    IntPtr pybullet;

	// Use this for initialization
	void Start () {
        text = GetComponent<Text>();


        
        pybullet = NativeMethods.b3ConnectPhysicsDirect();// NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY);
        IntPtr cmd = NativeMethods.b3InitResetSimulationCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        int numBodies = NativeMethods.b3GetNumBodies(pybullet);
        cmd = NativeMethods.b3LoadUrdfCommandInit(pybullet, "plane.urdf");
        status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status);
        if (statusType == (EnumSharedMemoryServerStatus)EnumSharedMemoryServerStatus.CMD_URDF_LOADING_COMPLETED)
        {
            numBodies = NativeMethods.b3GetNumBodies(pybullet);
            text.text = numBodies.ToString();
            cmd = NativeMethods.b3InitRequestVisualShapeInformation(pybullet, 0);
            status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
            statusType = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(status);

            if (statusType == (EnumSharedMemoryServerStatus)EnumSharedMemoryServerStatus.CMD_VISUAL_SHAPE_INFO_COMPLETED)
            {
                b3VisualShapeInformation visuals = new b3VisualShapeInformation();
                NativeMethods.b3GetVisualShapeInformation(pybullet, ref visuals);
                System.Console.WriteLine("visuals.m_numVisualShapes=" + visuals.m_numVisualShapes);
                System.IntPtr visualPtr = visuals.m_visualShapeData;

                for (int s = 0; s < visuals.m_numVisualShapes; s++)
                {
                    b3VisualShapeData visual = (b3VisualShapeData)Marshal.PtrToStructure(visualPtr, typeof(b3VisualShapeData));
                    visualPtr = new IntPtr(visualPtr.ToInt64()+(Marshal.SizeOf(typeof(b3VisualShapeData))));
                    double x = visual.m_dimensions[0];
                    double y = visual.m_dimensions[1];
                    double z = visual.m_dimensions[2];
                    System.Console.WriteLine("visual.m_visualGeometryType = " + visual.m_visualGeometryType);
                    System.Console.WriteLine("visual.m_dimensions" + x + "," + y + "," + z);
                    if (visual.m_visualGeometryType == (int)eUrdfGeomTypes.GEOM_MESH)
                    {
                        System.Console.WriteLine("visual.m_meshAssetFileName=" + visual.m_meshAssetFileName);
                        text.text = visual.m_meshAssetFileName;
                    }
                }
            }
            if (numBodies > 0)
            {

                b3BodyInfo info=new b3BodyInfo();
                NativeMethods.b3GetBodyInfo(pybullet, 0, ref  info );

                text.text = info.m_baseName;
            }
            

        }
        
    }

    // Update is called once per frame
    void Update () {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitStepSimulationCommand(pybullet);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
    }

    void OnDestroy()
    {
        if (pybullet != IntPtr.Zero)
        {
            NativeMethods.b3DisconnectSharedMemory(pybullet);
        }
       
    }
}
