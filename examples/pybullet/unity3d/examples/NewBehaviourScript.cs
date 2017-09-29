using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System;

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]


public class NewBehaviourScript : MonoBehaviour {

                                 
   

    [DllImport("TestCppPlug.dll")]
    static extern int Add(int a, int b);

    [DllImport("TestCppPlug.dll")]
    static extern int CallMe(Action<int> action);

    [DllImport("TestCppPlug.dll")]
    static extern IntPtr CreateSharedAPI(int id);
    
    [DllImport("TestCppPlug.dll")]
    static extern int GetMyIdPlusTen(IntPtr api);

    [DllImport("TestCppPlug.dll")]
    static extern void DeleteSharedAPI(IntPtr api);

    private void IWasCalled(int value)
    {
        text.text = value.ToString();
    }

    Text text;
    IntPtr sharedAPI;
    IntPtr pybullet;

	// Use this for initialization
	void Start () {
        text = GetComponent<Text>();
        CallMe(IWasCalled);
        sharedAPI = CreateSharedAPI(30);

        pybullet = NativeMethods.b3ConnectPhysicsDirect();// SharedMemory(12347);
        IntPtr cmd = NativeMethods.b3InitResetSimulationCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        int numBodies = NativeMethods.b3GetNumBodies(pybullet);
        cmd = NativeMethods.b3LoadUrdfCommandInit(pybullet, "plane.urdf");
        status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status);
        if (statusType == EnumSharedMemoryServerStatus.CMD_URDF_LOADING_COMPLETED)
        {
            numBodies = NativeMethods.b3GetNumBodies(pybullet);
            text.text = numBodies.ToString();
            if (numBodies > 0)
            {

                //b3BodyInfo info=new b3BodyInfo();
                //NativeMethods.b3GetBodyInfo(pybullet, 0, ref  info );
                //text.text = info.m_baseName;
            }
        }
        
        //IntPtr clientPtr = b3CreateInProcessPhysicsServerAndConnect(0, ref ptr);
    }

    // Update is called once per frame
    void Update () {
        //if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitStepSimulationCommand(pybullet);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

            //text.text = System.IO.Directory.GetCurrentDirectory().ToString();
            //text.text = Add(4, 5).ToString();
            //text.text = UnityEngine.Random.Range(0f, 1f).ToString();// GetMyIdPlusTen(sharedAPI).ToString();
        }
    }

    void OnDestroy()
    {
        NativeMethods.b3DisconnectSharedMemory(pybullet);
       
    }
}
