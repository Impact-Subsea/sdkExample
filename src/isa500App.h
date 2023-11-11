#ifndef ISA500APP_H_
#define ISA500APP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "devices/isa500.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Isa500App : public App
    {
    public:
        Isa500App(void);
        ~Isa500App(void);
        void connectSignals(Device& device) override;
        void disconnectSignals(Device& device) override;
        void doTask(int_t key, const std::string& path) override;

        Slot<Ahrs&, uint64_t, const Quaternion&, real_t, real_t> slotAhrsData{ this, &Isa500App::callbackAhrs };
        Slot<GyroSensor&, const Vector3&> slotGyroData{ this, &Isa500App::callbackGyroData };
        Slot<AccelSensor&, const Vector3&> slotAccelData{ this, &Isa500App::callbackAccelData };
        Slot<MagSensor&, const Vector3&> slotMagData{ this, &Isa500App::callbackMagData };
        Slot<AccelSensor&, Vector3::Axis, const Vector3&, uint_t> slotAccelCal{ this, &Isa500App::callbackAccelCal };

        Slot<Isa500&, uint64_t, uint_t, uint_t, const std::vector<Isa500::Echo>&> slotEchoData{ this, &Isa500App::callbackEchoData };
        Slot<Isa500&, const std::vector<uint8_t>&> slotPingData{ this, &Isa500App::callbackEchogramData };
        Slot<Isa500&, real_t> slotTemperatureData{ this, &Isa500App::callbackTemperatureData };
        Slot<Isa500&, real_t> slotVoltageData{ this, &Isa500App::callbackVoltageData };
        Slot<Isa500&, bool_t> slotTriggerData{ this, &Isa500App::callbackTriggerData };
        Slot<Isa500&> slotScriptDataReceived{ this, &Isa500App::callbackScriptDataReceived };
        Slot<Isa500&, bool_t> slotSettingsUpdated{ this, &Isa500App::callbackSettingsUpdated };

    private:
        void connectEvent(Device& device);
        void callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount);
        void callbackGyroData(GyroSensor& gyro, const Vector3& v);
        void callbackAccelData(AccelSensor& accel, const Vector3& v);
        void callbackMagData(MagSensor& mag, const Vector3& v);
        void callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress);

        void callbackEchoData(Isa500& isa500, uint64_t timeUs, uint_t selectedIdx, uint_t totalEchoCount, const std::vector<Isa500::Echo>& echoes);
        void callbackEchogramData(Isa500& isa500, const std::vector<uint8_t>& data);
        void callbackTemperatureData(Isa500& isa500, real_t temperatureC);
        void callbackVoltageData(Isa500& isa500, real_t voltage12);
        void callbackTriggerData(Isa500& isa500, bool_t risingEdge);
        void callbackScriptDataReceived(Isa500& isa500);
        void callbackSettingsUpdated(Isa500& isa500, bool_t ok);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
