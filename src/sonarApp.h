#ifndef SONARAPP_H_
#define SONARAPP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "devices/sonar.h"
#include "imuManager.h"
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

        Slot<Sonar&, bool_t, Sonar::Settings::Type> slotSettingsUpdated{ this, &SonarApp::callbackSettingsUpdated };
        Slot<Sonar&, const Sonar::HeadIndexes&> slotHeadIndexesAcquired{ this, &SonarApp::callbackHeadIndexesAcquired };
        Slot<Sonar&, const Sonar::Ping&> slotPingData{ this, &SonarApp::callbackPingData };
        Slot<Sonar&, const Sonar::Echos&> slotEchoData{ this, &SonarApp::callbackEchoData };
        Slot<Sonar&, const Sonar::CpuPowerTemp& > slotPwrAndTemp{ this, &SonarApp::callbackPwrAndTemp };
        Slot<Sonar&> slotMotorSlip { this, &SonarApp::callbackMotorSlip };
        Slot<Sonar&, bool_t> slotMotorMoveComplete { this, &SonarApp::callbackMotorMoveComplete };
        
    private:
        AhrsManager ahrs;
        GyroManager gyro;
        AccelManager accel;

        Palette m_palette;
        SonarImage m_circular;
        SonarImage m_texture;
        uint_t m_pingCount;
        SonarDataStore sonarDataStore;
        virtual void connectEvent(Device& device);
       
        void callbackSettingsUpdated(Sonar& sonar, bool_t ok, Sonar::Settings::Type settingsType);
        void callbackHeadIndexesAcquired(Sonar& sonar, const Sonar::HeadIndexes& data);
        void callbackEchoData(Sonar& sonar, const Sonar::Echos& data);
        void callbackPingData(Sonar& sonar, const Sonar::Ping& data);
        void callbackPwrAndTemp(Sonar& sonar, const Sonar::CpuPowerTemp& data);
        void callbackMotorSlip(Sonar& sonar);
        void callbackMotorMoveComplete(Sonar& sonar, bool_t ok);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
