
/// Return Type: void
///param0: VirtContext->VirtContextS*
///param1: void*
public delegate void VirtuoseCallbackFn(System.IntPtr param0, System.IntPtr param1);

public partial class NativeMethods {
    
    /// Return Type: int
    ///major: int*
    ///minor: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtAPIVersion")]
public static extern  int virtAPIVersion(ref int major, ref int minor) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///mass: float
    ///mxmymz: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtAttachVO")]
public static extern  int virtAttachVO(System.IntPtr VC, float mass, float[] mxmymz) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///Ks: float[]
    ///Bs: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtAttachQSVO")]
public static extern  int virtAttachQSVO(System.IntPtr VC, float[] Ks, float[] Bs) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///mass: float
    ///mxmymz: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtAttachVOAvatar")]
public static extern  int virtAttachVOAvatar(System.IntPtr VC, float mass, float[] mxmymz) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtClose")]
public static extern  int virtClose(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtDetachVO")]
public static extern  int virtDetachVO(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtDetachVOAvatar")]
public static extern  int virtDetachVOAvatar(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///avatar: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetAvatarFrame")]
public static extern  int virtGetAvatarFrame(System.IntPtr VC, float[] avatar) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///frame: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetBaseFrame")]
public static extern  int virtGetBaseFrame(System.IntPtr VC, float[] frame) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///button_number: int
    ///state: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetButton")]
public static extern  int virtGetButton(System.IntPtr VC, int button_number, ref int state) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///type: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetCommandType")]
public static extern  int virtGetCommandType(System.IntPtr VC, ref int type) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///dead_man: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetDeadMan")]
public static extern  int virtGetDeadMan(System.IntPtr VC, ref int dead_man) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///emergency_stop: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetEmergencyStop")]
public static extern  int virtGetEmergencyStop(System.IntPtr VC, ref int emergency_stop) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///error: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetError")]
public static extern  int virtGetError(System.IntPtr VC, ref int error) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetErrorCode")]
public static extern  int virtGetErrorCode(System.IntPtr VC) ;

    
    /// Return Type: char*
    ///code: int
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetErrorMessage")]
public static extern  System.IntPtr virtGetErrorMessage(int code) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///force: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetForce")]
public static extern  int virtGetForce(System.IntPtr VC, float[] force) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///force_factor: float*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetForceFactor")]
public static extern  int virtGetForceFactor(System.IntPtr VC, ref float force_factor) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///torque: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetLimitTorque")]
public static extern  int virtGetLimitTorque(System.IntPtr VC, float[] torque) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///obs: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetObservationFrame")]
public static extern  int virtGetObservationFrame(System.IntPtr VC, float[] obs) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///pos: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetPosition")]
public static extern  int virtGetPosition(System.IntPtr VC, float[] pos) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///power: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetPowerOn")]
public static extern  int virtGetPowerOn(System.IntPtr VC, ref int power) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///speed: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetSpeed")]
public static extern  int virtGetSpeed(System.IntPtr VC, float[] speed) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///speed_factor: float*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetSpeedFactor")]
public static extern  int virtGetSpeedFactor(System.IntPtr VC, ref float speed_factor) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///time: unsigned int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetTimeLastUpdate")]
public static extern  int virtGetTimeLastUpdate(System.IntPtr VC, ref uint time) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///time_value: float*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetTimeoutValue")]
public static extern  int virtGetTimeoutValue(System.IntPtr VC, ref float time_value) ;

    
    /// Return Type: VirtContext->VirtContextS*
    ///nom: char*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtOpen")]
public static extern  System.IntPtr virtOpen([System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string nom) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///avatar: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetAvatarFrame")]
public static extern  int virtSetAvatarFrame(System.IntPtr VC, float[] avatar) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///frame: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetBaseFrame")]
public static extern  int virtSetBaseFrame(System.IntPtr VC, float[] frame) ;


    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///type: unsigned short
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint = "virtSetCommandType")]
public static extern int virtSetCommandType(System.IntPtr VC, ushort type);


    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///flag: unsigned short
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetDebugFlags")]
public static extern  int virtSetDebugFlags(System.IntPtr VC, ushort flag) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///force: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetForce")]
public static extern  int virtSetForce(System.IntPtr VC, float[] force) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///force_factor: float
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetForceFactor")]
public static extern  int virtSetForceFactor(System.IntPtr VC, float force_factor) ;


    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///mode: unsigned short
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint = "virtSetIndexingMode")]
public static extern int virtSetIndexingMode(System.IntPtr VC, ushort mode);


/// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///torque: float
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetLimitTorque")]
public static extern  int virtSetLimitTorque(System.IntPtr VC, float torque) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///obs: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetObservationFrame")]
public static extern  int virtSetObservationFrame(System.IntPtr VC, float[] obs) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///speed: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetObservationFrameSpeed")]
public static extern  int virtSetObservationFrameSpeed(System.IntPtr VC, float[] speed) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///name: char*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetOutputFile")]
public static extern  int virtSetOutputFile(System.IntPtr VC, System.IntPtr name) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///fn: VirtuoseCallbackFn
    ///period: float*
    ///arg: void*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetPeriodicFunction")]
public static extern  int virtSetPeriodicFunction(System.IntPtr VC, VirtuoseCallbackFn fn, ref float period, System.IntPtr arg) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///pos: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetPosition")]
public static extern  int virtSetPosition(System.IntPtr VC, float[] pos) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///power: int
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetPowerOn")]
public static extern  int virtSetPowerOn(System.IntPtr VC, int power) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///speed: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetSpeed")]
public static extern  int virtSetSpeed(System.IntPtr VC, float[] speed) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///speed_factor: float
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetSpeedFactor")]
public static extern  int virtSetSpeedFactor(System.IntPtr VC, float speed_factor) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///position: float[]
    ///intensite: float[]
    ///reinit: int
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetTexture")]
public static extern  int virtSetTexture(System.IntPtr VC, float[] position, float[] intensite, int reinit) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///texture_force: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetTextureForce")]
public static extern  int virtSetTextureForce(System.IntPtr VC, float[] texture_force) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///step: float
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetTimeStep")]
public static extern  int virtSetTimeStep(System.IntPtr VC, float step) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///time_value: float
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSetTimeoutValue")]
public static extern  int virtSetTimeoutValue(System.IntPtr VC, float time_value) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtStartLoop")]
public static extern  int virtStartLoop(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtStopLoop")]
public static extern  int virtStopLoop(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtWaitForSynch")]
public static extern  int virtWaitForSynch(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtTrajRecordStart")]
public static extern  int virtTrajRecordStart(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtTrajRecordStop")]
public static extern  int virtTrajRecordStop(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///timeStep: float
    ///recordTime: unsigned int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtTrajSetSamplingTimeStep")]
public static extern  int virtTrajSetSamplingTimeStep(System.IntPtr VC, float timeStep, ref uint recordTime) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///nbSamples: unsigned int
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmStartTrajSampling")]
public static extern  int virtVmStartTrajSampling(System.IntPtr VC, uint nbSamples) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///samples: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmGetTrajSamples")]
public static extern  int virtVmGetTrajSamples(System.IntPtr VC, float[] samples) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmActivate")]
public static extern  int virtVmActivate(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmDeactivate")]
public static extern  int virtVmDeactivate(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///frame: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSetBaseFrame")]
public static extern  int virtVmSetBaseFrame(System.IntPtr VC, float[] frame) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///bounds: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSetMaxArtiBounds")]
public static extern  int virtVmSetMaxArtiBounds(System.IntPtr VC, float[] bounds) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///bounds: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSetMinArtiBounds")]
public static extern  int virtVmSetMinArtiBounds(System.IntPtr VC, float[] bounds) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///pos: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetPhysicalPosition")]
public static extern  int virtGetPhysicalPosition(System.IntPtr VC, float[] pos) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///pos: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetAvatarPosition")]
public static extern  int virtGetAvatarPosition(System.IntPtr VC, float[] pos) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///forceThreshold: float
    ///momentThreshold: float
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtSaturateTorque")]
public static extern  int virtSaturateTorque(System.IntPtr VC, float forceThreshold, float momentThreshold) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSetDefaultToTransparentMode")]
public static extern  int virtVmSetDefaultToTransparentMode(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSetDefaultToCartesianPosition")]
public static extern  int virtVmSetDefaultToCartesianPosition(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSetBaseFrameToCurrentFrame")]
public static extern  int virtVmSetBaseFrameToCurrentFrame(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///rgb: float[]
    ///gray: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtConvertRGBToGrayscale")]
public static extern  int virtConvertRGBToGrayscale(System.IntPtr VC, float[] rgb, float[] gray) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///frame: float[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmGetBaseFrame")]
public static extern  int virtVmGetBaseFrame(System.IntPtr VC, float[] frame) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///button_number: int
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtWaitPressButton")]
public static extern  int virtWaitPressButton(System.IntPtr VC, int button_number) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///step: float*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetTimeStep")]
public static extern  int virtGetTimeStep(System.IntPtr VC, ref float step) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///OnOff: int
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSetRobotMode")]
public static extern  int virtVmSetRobotMode(System.IntPtr VC, int OnOff) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///file_name: char*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmSaveCurrentSpline")]
public static extern  int virtVmSaveCurrentSpline(System.IntPtr VC, System.IntPtr file_name) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///file_name: char*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmLoadSpline")]
public static extern  int virtVmLoadSpline(System.IntPtr VC, System.IntPtr file_name) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///file_name: char*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmDeleteSpline")]
public static extern  int virtVmDeleteSpline(System.IntPtr VC, System.IntPtr file_name) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtVmWaitUpperBound")]
public static extern  int virtVmWaitUpperBound(System.IntPtr VC) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///disable: int
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtDisableControlConnexion")]
public static extern  int virtDisableControlConnexion(System.IntPtr VC, int disable) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///bounds: unsigned int[]
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtIsInBounds")]
public static extern  int virtIsInBounds(System.IntPtr VC, uint[] bounds) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///alarm: unsigned int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetAlarm")]
public static extern  int virtGetAlarm(System.IntPtr VC, ref uint alarm) ;

    
    /// Return Type: int
    ///VC: VirtContext->VirtContextS*
    ///x: int*
    ///y: int*
    [System.Runtime.InteropServices.DllImportAttribute("VirtuoseAPI.dll", EntryPoint="virtGetTrackball")]
public static extern  int virtGetTrackball(System.IntPtr VC, ref int x, ref int y) ;

}
