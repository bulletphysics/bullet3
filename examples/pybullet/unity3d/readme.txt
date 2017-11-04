Quick prototype to connect Unity 3D to pybullet

Generate C# Native Methods using the Microsoft PInvoke Signature Toolkit:

sigimp.exe  /lang:cs e:\develop\bullet3\examples\SharedMemory\PhysicsClientC_API.h

Add some #define B3_SHARED_API __declspec(dllexport) to the exported methods,
replace [3], [4], [16] by [] to get sigimp.exe working.

This generates autogen/NativeMethods.cs

Then put pybullet.dll in the right location, so Unity finds it.

NewBehaviourScript.cs is a 1 evening prototype that works within Unity 3D:
Create a connection to pybullet, reset the world, load a urdf at startup.
Step the simulation each Update.

Now the real work can start, converting Unity objects to pybullet,
pybullet robots to Unity, synchronizing the transforms each Update.

void Start () {
	IntPtr pybullet = b3ConnectSharedMemory(12347);
	IntPtr cmd = b3InitResetSimulationCommand(pybullet);
	IntPtr status = b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
	cmd = b3LoadUrdfCommandInit(pybullet, "plane.urdf");
	status = b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
}

void Update () 
{
	IntPtr cmd = b3InitStepSimulationCommand(pybullet);
	IntPtr status = b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
}
