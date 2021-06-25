#include "vivetracking.h"

// Destructor for the LighthouseTracking object
ViveTracking::~ViveTracking()
{
    if (vr_pointer != NULL)
    {
        // VR Shutdown: https://github.com/ValveSoftware/openvr/wiki/API-Documentation#initialization-and-cleanup
        VR_Shutdown();
        vr_pointer = NULL;
    }

    this->trackingFile.close();
}

// Constructor for the LighthouseTracking object
ViveTracking::ViveTracking(InitFlags f)
{
    flags = f;
    coordsBuf = new char[1024];
    trackBuf = new char[1024];
    rotBuf = new char[1024];
    trackers = new TrackerData[16];

    // Create new File for Tracking
    createNewTrackingFile();

    // Definition of the init error
    EVRInitError eError = VRInitError_None;

    /*
    VR_Init (
        arg1: Pointer to EVRInitError type (enum defined in openvr.h)
        arg2: Must be of type EVRApplicationType

            The type of VR Applicaion.  This example uses the SteamVR instance that is already running.
            Because of this, the init function will fail if SteamVR is not already running.

            Other EVRApplicationTypes include:
                * VRApplication_Scene - "A 3D application that will be drawing an environment.""
                * VRApplication_Overlay - "An application that only interacts with overlays or the dashboard.""
                * VRApplication_Utility
    */

    vr_pointer = VR_Init(&eError, VRApplication_Background);

    // If the init failed because of an error
    if (eError != VRInitError_None)
    {
        vr_pointer = NULL;
        printf("Unable to init VR runtime.\n");
        exit(EXIT_FAILURE);
    }
}

bool ViveTracking::RunProcedure()
{
    // Define a VREvent
    VREvent_t event;
    if(vr_pointer->PollNextEvent(&event, sizeof(event)))
    {
        /*
            ProcessVREvent is a function defined in this module.  It returns false if
            the function determines the type of error to be fatal or signal some kind of quit.
        */
        if (!ProcessVREvent(event))
        {
            // If ProcessVREvent determined that OpenVR quit, print quit message
            printf("\nEVENT--(OpenVR) service quit");
            return false;
        }
    }
    // ParseTrackingFrame() is where the tracking and vibration code starts
    ParseTrackingFrame();
    return true;
}

bool ViveTracking::ProcessVREvent(const VREvent_t & event)
{
    char* buf = new char[100];
    bool ret = true;
    switch (event.eventType)
    {
        case VREvent_TrackedDeviceActivated:
             sprintf(buf, "\nEVENT--(OpenVR) Device : %d attached", event.trackedDeviceIndex);
        break;

        case VREvent_TrackedDeviceDeactivated:
            sprintf(buf, "\nEVENT--(OpenVR) Device : %d detached", event.trackedDeviceIndex);
        break;

        case VREvent_TrackedDeviceUpdated:
            sprintf(buf, "\nEVENT--(OpenVR) Device : %d updated", event.trackedDeviceIndex);
        break;

        case VREvent_DashboardActivated:
            sprintf(buf, "\nEVENT--(OpenVR) Dashboard activated");
        break;

        case VREvent_DashboardDeactivated:
            sprintf(buf, "\nEVENT--(OpenVR) Dashboard deactivated");
        break;

        case VREvent_ChaperoneDataHasChanged:
            sprintf(buf, "\nEVENT--(OpenVR) Chaperone data has changed");
        break;

        case VREvent_ChaperoneSettingsHaveChanged:
            sprintf(buf, "\nEVENT--(OpenVR) Chaperone settings have changed");
        break;

        case VREvent_ChaperoneUniverseHasChanged:
            sprintf(buf, "\nEVENT--(OpenVR) Chaperone universe has changed");
        break;

        case VREvent_Quit:
        {
            sprintf(buf, "\nEVENT--(OpenVR) Received SteamVR Quit (%d%s", VREvent_Quit, ")");
            ret =  false;
        }
        break;

        case VREvent_ProcessQuit:
        {
            sprintf(buf, "\nEVENT--(OpenVR) SteamVR Quit Process (%d%s", VREvent_ProcessQuit, ")");
            ret = false;
        }
        break;

        case VREvent_QuitAcknowledged:
        {
            sprintf(buf, "\nEVENT--(OpenVR) SteamVR Quit Acknowledged (%d%s", VREvent_QuitAcknowledged, ")");
            ret = false;
        }
        break;

        case VREvent_TrackedDeviceRoleChanged:
            sprintf(buf, "\nEVENT--(OpenVR) TrackedDeviceRoleChanged: %d", event.trackedDeviceIndex);
        break;

        case VREvent_TrackedDeviceUserInteractionStarted:
            sprintf(buf, "\nEVENT--(OpenVR) TrackedDeviceUserInteractionStarted: %d", event.trackedDeviceIndex);
        break;

    }
    if(flags.printEvents)
        printf("%s",buf);
    return ret;
}

HmdVector3_t ViveTracking::GetPosition(HmdMatrix34_t matrix)
{
    HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

HmdQuaternion_t ViveTracking::GetRotation(HmdMatrix34_t matrix)
{
    HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return q;
}

HmdQuaternion_t ViveTracking::ProcessRotation(HmdQuaternion_t quat)
{
    HmdQuaternion_t out;
    out.w = 2 * acos(quat.w);
    out.x = quat.x / sin(out.w/2);
    out.y = quat.y / sin(out.w/2);
    out.z = quat.z / sin(out.w/2);

    printf("\nPROCESSED w:%.3f x:%.3f y:%.3f z:%.3f",out.w,out.x,out.y,out.z);
    return out;
}

void ViveTracking::iterateAssignIds()
{
    //Un-assigns the deviceIds and hands of controllers. If they are truely connected, will be re-assigned later in this function
    controllers[0].deviceId = -1;
    controllers[1].deviceId = -1;
    controllers[0].hand = -1;
    controllers[1].hand = -1;

    int numTrackersInitialized = 0;
    int numControllersInitialized = 0;

    for (unsigned int i = 0; i < k_unMaxTrackedDeviceCount; i++)  // Iterates across all of the potential device indicies
    {
        if (!vr_pointer->IsTrackedDeviceConnected(i))
            continue; //Doesn't use the id if the device isn't connected

        //vr_pointer points to the VRSystem that was in init'ed in the constructor.
        ETrackedDeviceClass trackedDeviceClass = vr_pointer->GetTrackedDeviceClass(i);

        //Finding the type of device
        if (trackedDeviceClass == ETrackedDeviceClass::TrackedDeviceClass_HMD)
        {
            hmdDeviceId = i;
            if(flags.printSetIds)
                printf("\nSETID--Assigned hmdDeviceId=%d",hmdDeviceId);
        }
        else if (trackedDeviceClass == ETrackedDeviceClass::TrackedDeviceClass_Controller && numControllersInitialized < 2)
        {
            ControllerData* pC = &(controllers[numControllersInitialized]);

            int sHand = -1;

            ETrackedControllerRole role = vr_pointer->GetControllerRoleForTrackedDeviceIndex(i);
            if (role == TrackedControllerRole_Invalid) //Invalid hand is actually very common, always need to test for invalid hand (lighthouses have lost tracking)
                sHand = 0;
            else if (role == TrackedControllerRole_LeftHand)
                sHand = 1;
            else if (role == TrackedControllerRole_RightHand)
                sHand = 2;
            pC->hand = sHand;
            pC->deviceId = i;


            //Used to get/store property ids for the xy of the pad and the analog reading of the trigger
            for(int x=0; x<k_unControllerStateAxisCount; x++ )
            {
                int prop = vr_pointer->GetInt32TrackedDeviceProperty(pC->deviceId,
                    (ETrackedDeviceProperty)(Prop_Axis0Type_Int32 + x));

                if( prop==k_eControllerAxis_Trigger )
                    pC->idtrigger = x;
                else if( prop==k_eControllerAxis_TrackPad )
                    pC->idpad = x;
            }
            if(flags.printSetIds)
                printf("\nSETID--Assigned controllers[%d] .hand=%d .deviceId=%d .idtrigger=%d .idpad=%d",numControllersInitialized,sHand, i , pC->idtrigger, pC->idpad);
            numControllersInitialized++; //Increment this count so that the other controller gets initialized after initializing this one
        }
        else if(trackedDeviceClass == ETrackedDeviceClass::TrackedDeviceClass_GenericTracker)
        {
            TrackerData* pT = &(trackers[numTrackersInitialized]);
            pT->deviceId = i;
            if(flags.printSetIds)
                printf("\nSETID--Assigned tracker[%d] .deviceId=%d",numTrackersInitialized,pT->deviceId);
            numTrackersInitialized++;
        }

    }
}

void ViveTracking::ParseTrackingFrame()
{
    //Runs the iterateAssignIds() method if...
    if(hmdDeviceId < 0 ||                     // HMD id not yet initialized
        controllers[0].deviceId < 0 ||       // One of the controllers not yet initialized
        controllers[1].deviceId < 0 ||
        controllers[0].deviceId == controllers[1].deviceId ||  //Both controllerData structs store the same deviceId
        controllers[0].hand == controllers[1].hand)          //Both controllerData structs are the same hand
    {
        iterateAssignIds();
    }

    TrackerCoords();

    std::string coords = coordsBuf;
    std::string rot = rotBuf;

    if(flags.printCoords)
        std::cout << coordsBuf << std::endl;

    if(flags.printRotation)
        std::cout << rotBuf << std::endl;

//    if(flags.printTrack)
//        printf("\nTRACK-- %s",trackBuf);

    std::cout << "--------------------------------------------------------" << std::endl;

     addLineToFile(coords + rot);
}

void ViveTracking::TrackerCoords()
{
    TrackedDevicePose_t trackedDevicePose;
    VRControllerState_t controllerState;
    HmdQuaternion_t rot;
    sprintf(coordsBuf,"");
    sprintf(trackBuf,"");
    sprintf(rotBuf,"");

     for(int i = 0; i < 16; i++)
    {
        TrackerData* pT = &(trackers[i]);

        if (i == 0 && pT->deviceId == -1){
            std::string errMsg = "Es konnten Keine HTC VIVE Tracker gefunden werden.\n";
            throw std::runtime_error(errMsg);
        }

        if (pT->deviceId < 0 || !vr_pointer->IsTrackedDeviceConnected(pT->deviceId) )
            continue;

        vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, pT->deviceId, &controllerState, sizeof(controllerState), &trackedDevicePose);
        pT->pos = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
        rot = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
        pT->isValid =trackedDevicePose.bPoseIsValid;

        sprintf(coordsBuf,"%sT%d, %s",coordsBuf, i, getPoseXYZString(trackedDevicePose,0));

        //qw, qx, qy, qz
        sprintf(rotBuf,"%sRot%d, %.2f, %.2f, %.2f, %.2f, ",rotBuf, i,rot.w,rot.x,rot.y,rot.z);

//        sprintf(trackBuf,"%s T%d: %-25.25s %-7.7s" , trackBuf, i, getEnglishTrackingResultForPose(trackedDevicePose), getEnglishPoseValidity(trackedDevicePose));
    }
}

char* ViveTracking::getEnglishTrackingResultForPose(TrackedDevicePose_t pose)
{
    char* buf = new char[50];
    switch (pose.eTrackingResult)
    {
        case vr::ETrackingResult::TrackingResult_Uninitialized:
                sprintf(buf, "Invalid tracking result");
                break;
        case vr::ETrackingResult::TrackingResult_Calibrating_InProgress:
                sprintf(buf, "Calibrating in progress");
                break;
        case vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange:
                sprintf(buf, "Calibrating Out of range");
                break;
        case vr::ETrackingResult::TrackingResult_Running_OK:
                sprintf(buf, "Running OK");
                break;
        case vr::ETrackingResult::TrackingResult_Running_OutOfRange:
                sprintf(buf, "WARNING: Running Out of Range");
                break;
        default:
                sprintf(buf, "Default");
                break;
    }
    return buf;
}

char* ViveTracking::getEnglishPoseValidity(TrackedDevicePose_t pose)
{
    char* buf = new char[50];
    if(pose.bPoseIsValid)
        sprintf(buf, "Valid");
    else
        sprintf(buf, "Invalid");
    return buf;
}

char* ViveTracking::getPoseXYZString(TrackedDevicePose_t pose, int hand)
{
    HmdVector3_t pos = GetPosition(pose.mDeviceToAbsoluteTracking);
    char* cB = new char[50];
    if(pose.bPoseIsValid)
        //x, y, z
        sprintf(cB, "%.3f, %.3f, %.3f, ",pos.v[0], pos.v[1], pos.v[2]);
    else
        sprintf(cB, "            INVALID");
    if(flags.pipeCoords)
        for(int i = 0; i < 3; i++)
            if(pose.bPoseIsValid)
                printf("%.5f\n",pos.v[i]);
            else
                printf("invalid\n",pos.v[i]);
    return cB;
}

void ViveTracking::createNewTrackingFile(){
    std::string fileName = DirectoryManager().getCurrentDateAsString();
    std::string fileType = ".txt";
    this->trackingFile.open("/home/daniel/ZAR/trackingFiles/" + fileName + fileType, std::ios_base::app);
}

void ViveTracking::addLineToFile(std::string lineText){
    this->trackingFile << lineText << std::endl;
}
