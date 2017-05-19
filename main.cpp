#include <sixense.h>
#include <sixense_math.hpp>
#include <openvr.h>

#include <algorithm>
#include <deque>
#include <thread>
#include <mutex>
#include <chrono>

#define SIXENSE_MAX_HISTORY 50
#define M_RAD_45 0.78539816339744830961566084581988

typedef struct _sixenseControllerDataOld {
    float pos[3];
    float rot_mat[3][3];
    unsigned char joystick_x;
    unsigned char joystick_y;
    unsigned char trigger;
    unsigned int buttons;
    unsigned char sequence_number;
    float rot_quat[4];
    unsigned short firmware_revision;
    unsigned short hardware_revision;
    unsigned short packet_type;
    unsigned short magnetic_frequency;
    int enabled;
    int controller_index;
    unsigned char is_docked;
    unsigned char which_hand;
} sixenseControllerDataOld;

typedef struct _sixenseAllControllerDataOld {
    sixenseControllerDataOld controllers[4];
} sixenseAllControllerDataOld;

#ifdef SIXENSE_LEGACY
typedef sixenseAllControllerDataOld compatAllControllerData;
typedef sixenseControllerDataOld compatControllerData;
#else
typedef sixenseAllControllerData compatAllControllerData;
typedef sixenseControllerData compatControllerData;
#endif

bool g_running = false;
std::thread g_thread;
std::mutex g_data_mutex;
std::deque<compatAllControllerData> g_controller_data;

void sixenseThreadFunc()
{
    // We know the sixense SDK thread is running at "60 FPS"
    auto interval = std::chrono::milliseconds(16);

    uint64_t hw_rev = vr::VRSystem()->GetUint64TrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_HardwareRevision_Uint64);
    uint64_t fw_rev = vr::VRSystem()->GetUint64TrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_FirmwareVersion_Uint64);

    sixenseMath::Vector3 axis(1.0f, 0.0f, 0.0f);
    sixenseMath::Matrix3 offset = sixenseMath::Matrix3::rotation((float)-M_RAD_45, axis);

    for (uint8_t sequence = 0; g_running; sequence++)
    {
        vr::TrackedDeviceIndex_t devices[SIXENSE_MAX_CONTROLLERS];
        memset(devices, vr::k_unTrackedDeviceIndexInvalid, sizeof(devices));
        vr::VRSystem()->GetSortedTrackedDeviceIndicesOfClass(vr::TrackedDeviceClass_Controller, devices, SIXENSE_MAX_CONTROLLERS);

        vr::TrackedDevicePose_t poses[SIXENSE_MAX_CONTROLLERS];
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, 0.0f, poses, SIXENSE_MAX_CONTROLLERS);

        compatAllControllerData all_data = {};
        for (int i = 0; i < SIXENSE_MAX_CONTROLLERS; i++)
        {
            if (devices[i] == vr::k_unTrackedDeviceIndexInvalid)
                break;

            compatControllerData& data = all_data.controllers[i];
            vr::HmdMatrix34_t& pose = poses[devices[i]].mDeviceToAbsoluteTracking;

            // Sixense SDK is column-major instead of row-major
            float matData[3][3];
            for (int row = 0; row < 3; row++)
                for (int col = 0; col < 3; col++)
                    matData[col][row] = pose.m[row][col];
            sixenseMath::Matrix3 mat = offset * sixenseMath::Matrix3(matData);

            vr::VRControllerState_t state;
            if (!vr::VRSystem()->GetControllerState(devices[i], &state, sizeof(state)))
                break;

            // Sixense SDK uses millimeters instead of meters
            data.pos[0] = pose.m[0][3] * 1000.0f;
            data.pos[1] = pose.m[1][3] * 1000.0f;
            data.pos[2] = pose.m[2][3] * 1000.0f;

            // Sixense SDK is column-major instead of row-major
            for (int row = 0; row < 3; row++)
                for (int col = 0; col < 3; col++)
                    data.rot_mat[col][row] = mat[col][row];

            // TODO: Buttons
#ifdef SIXENSE_LEGACY
            data.joystick_x = (uint8_t)((state.rAxis[0].x + 1.0f) * 127.5f);
            data.joystick_y = (uint8_t)((state.rAxis[0].y + 1.0f) * 127.5f);
            data.trigger = (uint8_t)(state.rAxis[1].x * 255.0f);
#else
            data.joystick_x = state.rAxis[0].x;
            data.joystick_y = state.rAxis[0].y;
            data.trigger = state.rAxis[1].x;
#endif

            data.sequence_number = sequence;

            sixenseMath::Quat quat(mat);
            data.rot_quat[0] = quat[0];
            data.rot_quat[1] = quat[1];
            data.rot_quat[2] = quat[2];
            data.rot_quat[3] = quat[3];

            data.firmware_revision = (uint16_t)fw_rev;
            data.hardware_revision = (uint16_t)hw_rev;

            data.packet_type = 1;
            data.magnetic_frequency = 0;
            data.enabled = 1;
            data.controller_index = i;
            data.is_docked = vr::VRSystem()->GetBoolTrackedDeviceProperty(devices[i], vr::Prop_DeviceIsCharging_Bool);
            data.which_hand = vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(devices[i]);
#ifndef SIXENSE_LEGACY
            data.hemi_tracking_enabled = 0;
#endif
        }

        {
            std::lock_guard<std::mutex> lk(g_data_mutex);
            g_controller_data.push_front(all_data);
            g_controller_data.pop_back();
        }

        std::this_thread::sleep_for(interval);
    }
}

SIXENSE_EXPORT int sixenseInit(void)
{
    vr::EVRInitError err;
    vr::VR_Init(&err, vr::VRApplication_Other);
    
    if (err == vr::VRInitError_None)
    {
        for (int i = 0; i < SIXENSE_MAX_HISTORY; i++)
        {
            compatAllControllerData all_data = {};
            g_controller_data.push_back(all_data);
        }
        g_running = true;
        g_thread = std::thread(sixenseThreadFunc);
        return SIXENSE_SUCCESS;
    }
    return SIXENSE_FAILURE;
}
SIXENSE_EXPORT int sixenseExit(void)
{
    g_running = false;
    g_thread.join();

    g_controller_data.clear();
    vr::VR_Shutdown();
    return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseGetMaxBases() { return 1; }
SIXENSE_EXPORT int sixenseSetActiveBase(int i) { return SIXENSE_SUCCESS; }
SIXENSE_EXPORT int sixenseIsBaseConnected(int i) { return 1; }

SIXENSE_EXPORT int sixenseGetMaxControllers(void) { return SIXENSE_MAX_CONTROLLERS; }
SIXENSE_EXPORT int sixenseIsControllerEnabled(int which)
{
    if (which >= 4)
        return SIXENSE_FAILURE;

    vr::TrackedDeviceIndex_t devices[SIXENSE_MAX_CONTROLLERS];
    memset(devices, vr::k_unTrackedDeviceIndexInvalid, sizeof(devices));
    vr::VRSystem()->GetSortedTrackedDeviceIndicesOfClass(vr::TrackedDeviceClass_Controller, devices, SIXENSE_MAX_CONTROLLERS);
    return vr::VRSystem()->IsTrackedDeviceConnected(devices[which]);
}
SIXENSE_EXPORT int sixenseGetNumActiveControllers() { return std::min((int)vr::VRSystem()->GetSortedTrackedDeviceIndicesOfClass(vr::TrackedDeviceClass_Controller, nullptr, 0), SIXENSE_MAX_CONTROLLERS); }

SIXENSE_EXPORT int sixenseGetHistorySize() { return SIXENSE_MAX_HISTORY; }

SIXENSE_EXPORT int sixenseGetData(int which, int index_back, sixenseControllerData *data)
{
    if (!data || index_back >= SIXENSE_MAX_HISTORY)
        return SIXENSE_FAILURE;

    std::lock_guard<std::mutex> lk(g_data_mutex);
    *(compatControllerData*)data = g_controller_data[index_back].controllers[which];
    return data->enabled ? SIXENSE_SUCCESS : SIXENSE_FAILURE;
}

SIXENSE_EXPORT int sixenseGetAllData(int index_back, sixenseAllControllerData *data)
{
    if (!data || index_back >= SIXENSE_MAX_HISTORY)
        return SIXENSE_FAILURE;

    std::lock_guard<std::mutex> lk(g_data_mutex);
    *(compatAllControllerData*)data = g_controller_data[index_back];
    return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseGetNewestData(int which, sixenseControllerData *data)
{
    if (!data)
        return SIXENSE_FAILURE;

    std::lock_guard<std::mutex> lk(g_data_mutex);
    *(compatControllerData*)data = g_controller_data[0].controllers[which];
    return data->enabled ? SIXENSE_SUCCESS : SIXENSE_FAILURE;
}

SIXENSE_EXPORT int sixenseGetAllNewestData(sixenseAllControllerData *data)
{
    if (!data)
        return SIXENSE_FAILURE;

    std::lock_guard<std::mutex> lk(g_data_mutex);
    *(compatAllControllerData*)data = g_controller_data[0];
    return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseSetHemisphereTrackingMode(int which_controller, int state) { return SIXENSE_SUCCESS; }
SIXENSE_EXPORT int sixenseGetHemisphereTrackingMode(int which_controller, int *state) { return SIXENSE_SUCCESS; }

SIXENSE_EXPORT int sixenseAutoEnableHemisphereTracking(int which_controller) { return SIXENSE_SUCCESS; }

SIXENSE_EXPORT int sixenseSetHighPriorityBindingEnabled(int on_or_off) { return SIXENSE_SUCCESS; }
SIXENSE_EXPORT int sixenseGetHighPriorityBindingEnabled(int *on_or_off) { return SIXENSE_SUCCESS; }

SIXENSE_EXPORT int sixenseTriggerVibration(int controller_id, int duration_100ms, int pattern_id)
{
    if (controller_id >= 4)
        return SIXENSE_FAILURE;

    /* TODO: OpenVR doesn't allow such long duration vibrations and this function is unsupported anyway.
    vr::TrackedDeviceIndex_t devices[SIXENSE_MAX_CONTROLLERS];
    memset(devices, vr::k_unTrackedDeviceIndexInvalid, sizeof(devices));
    vr::VRSystem()->GetSortedTrackedDeviceIndicesOfClass(vr::TrackedDeviceClass_Controller, devices, SIXENSE_MAX_CONTROLLERS);

    vr::VRSystem()->TriggerHapticPulse(devices[controller_id], 0, duration_100ms * 100000);
    */
    return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseSetFilterEnabled(int on_or_off) { return SIXENSE_SUCCESS; }
SIXENSE_EXPORT int sixenseGetFilterEnabled(int *on_or_off) { return SIXENSE_SUCCESS; }

SIXENSE_EXPORT int sixenseSetFilterParams(float near_range, float near_val, float far_range, float far_val) { return SIXENSE_SUCCESS; }
SIXENSE_EXPORT int sixenseGetFilterParams(float *near_range, float *near_val, float *far_range, float *far_val) { return SIXENSE_SUCCESS; }

SIXENSE_EXPORT int sixenseSetBaseColor(unsigned char red, unsigned char green, unsigned char blue) { return SIXENSE_SUCCESS; }
SIXENSE_EXPORT int sixenseGetBaseColor(unsigned char *red, unsigned char *green, unsigned char *blue) { return SIXENSE_SUCCESS; }

// Private APIS

extern "C"
{
    SIXENSE_EXPORT int sixenseSetDebugParam() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetDebugParam() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseSetCalibrationEnabled() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetCalibrationEnabled() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseSetHemisphereVector() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetHemisphereVector() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetRawData() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetRawDataSingle() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetSignalMatrix() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetSignalQuality() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseSetTestMode() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseGetTestMode() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixensePlaybackLogFile() { return SIXENSE_SUCCESS; }
    SIXENSE_EXPORT int sixenseSendTestCommand() { return SIXENSE_SUCCESS; }
}
