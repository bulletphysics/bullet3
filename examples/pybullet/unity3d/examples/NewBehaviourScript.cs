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
    int m_cubeUid;

	// Use this for initialization
	void Start () {
        text = GetComponent<Text>();


        pybullet = NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY);
        if (NativeMethods.b3CanSubmitCommand(pybullet)==0)
        {
            pybullet = NativeMethods.b3ConnectPhysicsDirect();
        }

        

        IntPtr cmd = NativeMethods.b3InitResetSimulationCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        {
            IntPtr command = NativeMethods.b3InitPhysicsParamCommand(pybullet);
            NativeMethods.b3PhysicsParamSetGravity(command, 0, -10, 0);
            IntPtr statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, command);
        }

        int numBodies = NativeMethods.b3GetNumBodies(pybullet);
        {
            cmd = NativeMethods.b3LoadUrdfCommandInit(pybullet, "plane.urdf");
            Quaternion qq = Quaternion.Euler(-90, 0, 0);
            NativeMethods.b3LoadUrdfCommandSetStartOrientation(cmd, qq.x, qq.y, qq.z, qq.w);
            status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }

        
        cmd = NativeMethods.b3LoadUrdfCommandInit(pybullet, "cube.urdf");
        NativeMethods.b3LoadUrdfCommandSetStartPosition(cmd, 0, 20, 0);
        Quaternion q = Quaternion.Euler(35, 0, 0);
        NativeMethods.b3LoadUrdfCommandSetStartOrientation(cmd, q.x, q.y, q.z, q.w);
        status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        m_cubeUid = NativeMethods.b3GetStatusBodyIndex(status);
        
        EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status);
        if (statusType == (EnumSharedMemoryServerStatus)EnumSharedMemoryServerStatus.CMD_URDF_LOADING_COMPLETED)
        {
            numBodies = NativeMethods.b3GetNumBodies(pybullet);
            text.text = numBodies.ToString();
            cmd = NativeMethods.b3InitRequestVisualShapeInformation(pybullet, m_cubeUid);
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
    public struct MyPos
    {
        public double x, y, z;
        public double qx, qy, qz, qw;
    }
    // Update is called once per frame
    void Update () {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitStepSimulationCommand(pybullet);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
        if (m_cubeUid>=0)
        {
            IntPtr cmd_handle =
                NativeMethods.b3RequestActualStateCommandInit(pybullet, m_cubeUid);
            IntPtr status_handle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd_handle);

            EnumSharedMemoryServerStatus status_type = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status_handle);

            if (status_type == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
            {
                IntPtr p = IntPtr.Zero;
                int objUid = 0;
                int numDofQ = 0;
                int numDofU = 0;
                IntPtr inertialFrame = IntPtr.Zero;
                IntPtr actualStateQ = IntPtr.Zero;
                IntPtr actualStateQdot = IntPtr.Zero;
                IntPtr joint_reaction_forces = IntPtr.Zero;

                NativeMethods.b3GetStatusActualState(
                status_handle, ref objUid, ref numDofQ , ref numDofU,
                ref inertialFrame, ref actualStateQ,
                ref actualStateQdot , ref joint_reaction_forces);
                //Debug.Log("objUid=" + objUid.ToString());
                //Debug.Log("numDofQ=" + numDofQ.ToString());
                //Debug.Log("numDofU=" + numDofU.ToString());
                
                MyPos mpos = (MyPos)Marshal.PtrToStructure(actualStateQ, typeof(MyPos));
                //Debug.Log("pos=(" + mpos.x.ToString()+","+ mpos.y.ToString()+ "," + mpos.z.ToString()+")");
                //Debug.Log("orn=(" + mpos.qx.ToString() + "," + mpos.qy.ToString() + "," + mpos.qz.ToString() + mpos.qw.ToString() + ")");
                Vector3 pos = new Vector3((float)mpos.x, (float)mpos.y, (float)mpos.z);
                Quaternion orn = new Quaternion((float)mpos.qx, (float)mpos.qy, (float)mpos.qz, (float)mpos.qw);
                Vector3 dimensions = new Vector3(1, 1, 1);
                CjLib.DebugUtil.DrawBox(pos, orn, dimensions, Color.black);
            }


        }
        
        {
            CjLib.DebugUtil.DrawLine(new Vector3(-1, 0, 0), new Vector3(1, 0, 0), Color.red);
            CjLib.DebugUtil.DrawLine(new Vector3(0, -1, 0), new Vector3(0, 1, 0), Color.green);
            CjLib.DebugUtil.DrawLine(new Vector3(0, 0, -1), new Vector3(0, 0, 1), Color.blue);
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
