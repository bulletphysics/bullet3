using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System;

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]


public class NewBehaviourScript : MonoBehaviour {

    [System.Runtime.InteropServices.DllImportAttribute("pybullet_vs2010_x64_release.dll", EntryPoint = "b3ConnectSharedMemory")]
    public static extern System.IntPtr b3ConnectSharedMemory(int key);

    [System.Runtime.InteropServices.DllImportAttribute("pybullet_vs2010_x64_release.dll", EntryPoint = "b3CreateInProcessPhysicsServerAndConnect")]
    public static extern System.IntPtr b3CreateInProcessPhysicsServerAndConnect(int argc, ref System.IntPtr argv);

    [System.Runtime.InteropServices.DllImportAttribute("pybullet_vs2010_x64_release.dll", EntryPoint = "b3InitStepSimulationCommand")]
    public static extern System.IntPtr b3InitStepSimulationCommand(IntPtr  physClient);

    
    [System.Runtime.InteropServices.DllImportAttribute("pybullet_vs2010_x64_release.dll", EntryPoint = "b3LoadUrdfCommandInit")]
    public static extern System.IntPtr b3LoadUrdfCommandInit(IntPtr physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string urdfFileName);

    [System.Runtime.InteropServices.DllImportAttribute("pybullet_vs2010_x64_release.dll", EntryPoint = "b3InitResetSimulationCommand")]
    public static extern System.IntPtr b3InitResetSimulationCommand(IntPtr physClient);

    [System.Runtime.InteropServices.DllImportAttribute("pybullet_vs2010_x64_release.dll", EntryPoint = "b3SubmitClientCommandAndWaitStatus")]
    public static extern System.IntPtr b3SubmitClientCommandAndWaitStatus(IntPtr physClient, IntPtr commandHandle);


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

        IntPtr pybullet = b3ConnectSharedMemory(12347);
        IntPtr cmd = b3InitResetSimulationCommand(pybullet);
        IntPtr status = b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        cmd = b3LoadUrdfCommandInit(pybullet, "plane.urdf");
        status = b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        //IntPtr clientPtr = b3CreateInProcessPhysicsServerAndConnect(0, ref ptr);
    }
	
	// Update is called once per frame
	void Update () {
        IntPtr cmd = b3InitStepSimulationCommand(pybullet);
        IntPtr status = b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        //System.IO.Directory.GetCurrentDirectory().ToString();//
        //text.text = Add(4, 5).ToString();
        text.text = UnityEngine.Random.Range(0f, 1f).ToString();// GetMyIdPlusTen(sharedAPI).ToString();
    }

    void OnDestroy()
    {

        DeleteSharedAPI(sharedAPI);
    }
}
