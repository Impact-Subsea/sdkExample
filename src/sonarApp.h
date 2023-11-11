#ifndef SONARAPP_H_
#define SONARAPP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "devices/sonar.h"
#include "helpers/sonarDataStore.h"
#include "helpers/sonarImage.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SonarApp : public App
    {
    public:
        SonarApp(void);
        ~SonarApp(void);
        void renderPalette(const std::string& path);
        void connectSignals(Device& device) override;
        void disconnectSignals(Device& device) override;
        void doTask(int_t key, const std::string& path) override;

        Slot<Ahrs&, uint64_t, const Quaternion&, real_t, real_t> slotAhrsData{ this, & SonarApp::callbackAhrs };
        Slot<GyroSensor&, const Vector3&> slotGyroData{ this, & SonarApp::callbackGyroData };
        Slot<AccelSensor&, const Vector3&> slotAccelData{ this, & SonarApp::callbackAccelData };
        Slot<MagSensor&, const Vector3&> slotMagData{ this, & SonarApp::callbackMagData };
        Slot<AccelSensor&, Vector3::Axis, const Vector3&, uint_t> slotAccelCal{ this, & SonarApp::callbackAccelCal };

        Slot<Sonar&, bool_t, Sonar::Settings::Type> slotSettingsUpdated{ this, &SonarApp::callbackSettingsUpdated };
        Slot<Sonar&, const Sonar::HeadHome&> slotHeadHome{ this, & SonarApp::callbackHeadHome };
        Slot<Sonar&, const Sonar::Ping&> slotPingData{ this, & SonarApp::callbackPingData };
        Slot<Sonar&, const Sonar::Echos&> slotEchoData{ this, &SonarApp::callbackEchoData };
        Slot<Sonar&, const Sonar::CpuPowerTemp& > slotPwrAndTemp{ this, & SonarApp::callbackPwrAndTemp };;
        

    private:
        Palette m_palette;
        SonarImage m_circular;
        SonarImage m_texture;
        uint_t m_pingCount;
        SonarDataStore sonarDataStore;
        virtual void connectEvent(Device& device);
        void callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount);
        void callbackGyroData(GyroSensor& gyro, const Vector3& v);
        void callbackAccelData(AccelSensor& accel, const Vector3& v);
        void callbackMagData(MagSensor& mag, const Vector3& v);
        void callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress);

        void callbackSettingsUpdated(Sonar& iss360, bool_t ok, Sonar::Settings::Type settingsType);
        void callbackHeadHome(Sonar& iss360, const Sonar::HeadHome& data);
        void callbackEchoData(Sonar& iss360, const Sonar::Echos& data);
        void callbackPingData(Sonar& iss360, const Sonar::Ping& data);
        void callbackPwrAndTemp(Sonar& iss360, const Sonar::CpuPowerTemp& data);

        
    };
}

//--------------------------------------------------------------------------------------------------
#endif
