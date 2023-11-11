#ifndef ISD4000APP_H_
#define ISD4000APP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "devices/isd4000.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Isd4000App : public App
    {
    public:
        Isd4000App(void);
        ~Isd4000App(void);
        void connectSignals(Device& device) override;
        void disconnectSignals(Device& device) override;
        void doTask(int_t key, const std::string& path) override;

        Slot<Ahrs&, uint64_t, const Quaternion&, real_t, real_t> slotAhrsData{ this, & Isd4000App::callbackAhrs };
        Slot<GyroSensor&, const Vector3&> slotGyroData{ this, & Isd4000App::callbackGyroData };
        Slot<AccelSensor&, const Vector3&> slotAccelData{ this, & Isd4000App::callbackAccelData };
        Slot<MagSensor&, const Vector3&> slotMagData{ this, & Isd4000App::callbackMagData };
        Slot<AccelSensor&, Vector3::Axis, const Vector3&, uint_t> slotAccelCal{ this, & Isd4000App::callbackAccelCal };

        Slot<Isd4000&, uint64_t, real_t, real_t, real_t> slotPressure{ this, & Isd4000App::callbackPressureData };
        Slot<Isd4000&, real_t, real_t> slotTemperature{ this, & Isd4000App::callbackTemperatureData };
        Slot<Isd4000&> slotScriptDataReceived{ this, & Isd4000App::callbackScriptDataReceived };
        Slot<Isd4000&, bool_t> slotSettingsUpdated{ this, & Isd4000App::callbackSettingsUpdated };
        Slot<Isd4000&, const Isd4000::PressureCal&> slotPressureCalCert{ this, & Isd4000App::callbackPressureCal };
        Slot<Isd4000&, const Isd4000::TemperatureCal&> slotTemperatureCalCert{ this, & Isd4000App::callbackTemperatureCal };

    private:
        void callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount);
        void callbackGyroData(GyroSensor& gyro, const Vector3& v);
        void callbackAccelData(AccelSensor& accel, const Vector3& v);
        void callbackMagData(MagSensor& mag, const Vector3& v);
        void callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress);

        void callbackPressureData(Isd4000& isd4000, uint64_t timeUs, real_t pressureBar, real_t depthM, real_t pressureBarRaw);
        void callbackTemperatureData(Isd4000& isd4000, real_t temperatureC, real_t temperatureRawC);
        void callbackScriptDataReceived(Isd4000& isd4000);
        void callbackSettingsUpdated(Isd4000& isd4000, bool_t ok);
        void callbackPressureCal(Isd4000& isd4000, const Isd4000::PressureCal& cal);
        void callbackTemperatureCal(Isd4000& isd4000, const Isd4000::TemperatureCal& cal);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
