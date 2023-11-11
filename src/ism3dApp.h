#ifndef ISM3DAPP_H_
#define ISM3DAPP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "devices/ism3d.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Ism3dApp : public App
    {
    public:
        Ism3dApp(void);
        ~Ism3dApp(void);
        void connectSignals(Device& device) override;
        void disconnectSignals(Device& device) override;
        void doTask(int_t key, const std::string& path) override;

        Slot<Ahrs&, uint64_t, const Quaternion&, real_t, real_t> slotAhrsData{ this, & Ism3dApp::callbackAhrs };
        Slot<GyroSensor&, const Vector3&> slotGyroData{ this, & Ism3dApp::callbackGyroData };
        Slot<AccelSensor&, const Vector3&> slotAccelData{ this, & Ism3dApp::callbackAccelData };
        Slot<MagSensor&, const Vector3&> slotMagData{ this, & Ism3dApp::callbackMagData };
        Slot<AccelSensor&, Vector3::Axis, const Vector3&, uint_t> slotAccelCal{ this, & Ism3dApp::callbackAccelCal };

        Slot<Ism3d&> slotScriptDataReceived{ this, & Ism3dApp::callbackScriptDataReceived };
        Slot<Ism3d&, bool_t> slotSettingsUpdated{ this, & Ism3dApp::callbackSettingsUpdated };

    private:
        void callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount);
        void callbackGyroData(GyroSensor& gyro, const Vector3& v);
        void callbackAccelData(AccelSensor& accel, const Vector3& v);
        void callbackMagData(MagSensor& mag, const Vector3& v);
        void callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress);

        void callbackScriptDataReceived(Ism3d& ism3d);
        void callbackSettingsUpdated(Ism3d& ism3d, bool_t ok);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
