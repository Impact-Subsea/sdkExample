//------------------------------------------ Includes ----------------------------------------------

#include "sonarApp.h"
#include "maths/maths.h"
#include "platform/debug.h"
#include "files/bmpFile.h"
#include "utils/utils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
SonarApp::SonarApp(void) : App("SonarApp"), m_pingCount(0)
{
    Debug::log(Debug::Severity::Notice, name.c_str(), "created" NEW_LINE
                                                      "d -> Set settings to defualt" NEW_LINE
                                                      "s -> Save settings to file" NEW_LINE
                                                      "r -> Start scaning" NEW_LINE
                                                      "R -> Stop scaning" NEW_LINE
                                                      "p -> Save palette" NEW_LINE
                                                      "t -> Save sonar texture" NEW_LINE
                                                      "i -> Save sonar image" NEW_LINE);
}
//--------------------------------------------------------------------------------------------------
SonarApp::~SonarApp(void)
{
}
//--------------------------------------------------------------------------------------------------
void SonarApp::renderPalette(const std::string& path)
{
    const uint_t w = 100, h = 1000;

    std::unique_ptr<Palette> p = std::make_unique<Palette>();
    std::vector<uint32_t> buf(w * h);

    p->render(&buf[0], w, h, false);
    BmpFile::save(path + "palette.bmp", &buf[0], 32, w, h);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::connectSignals(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    sonar.ahrs.onData.connect(slotAhrsData);                // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //sonar.gyro.onData.connect(slotGyroData);              // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //sonar.accel.onData.connect(slotAccelData);            // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //sonar.mag.onData.connect(slotMagData);                // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    sonar.accel.onCalProgress.connect(slotAccelCal);

    sonar.onSettingsUpdated.connect(slotSettingsUpdated);
    sonar.onHeadHomed.connect(slotHeadHome);
    sonar.onPingData.connect(slotPingData);                 // Subscribing to this event causes ping data to be sent
    sonar.onEchoData.connect(slotEchoData);                 // Subscribing to this event causes profiling data to be sent
    sonar.onPwrAndTemp.connect(slotPwrAndTemp);             // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()

    

    Sonar::SensorRates rates;
    rates.ahrs = 100;
    rates.gyro = 100;
    rates.accel = 100;
    rates.mag = 100;
    rates.voltageAndTemp = 1000;
    sonar.setSensorRates(rates);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::disconnectSignals(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    sonar.ahrs.onData.disconnect(slotAhrsData);
    sonar.gyro.onData.disconnect(slotGyroData);
    sonar.accel.onData.disconnect(slotAccelData);
    sonar.mag.onData.disconnect(slotMagData);
    sonar.accel.onCalProgress.disconnect(slotAccelCal);

    sonar.onSettingsUpdated.disconnect(slotSettingsUpdated);
    sonar.onHeadHomed.disconnect(slotHeadHome);
    sonar.onPingData.disconnect(slotPingData);
    sonar.onEchoData.disconnect(slotEchoData);
    sonar.onPwrAndTemp.disconnect(slotPwrAndTemp);

    
}
//--------------------------------------------------------------------------------------------------
void SonarApp::doTask(int_t key, const std::string& path)
{
    if (m_device)
    {
        Sonar& sonar = reinterpret_cast<Sonar&>(*m_device);

        switch (key)
        {
        case 'd':
            sonar.setSystemSettings(Sonar::System(), true);
            sonar.setAcousticSettings(Sonar::Acoustic(), true);
            sonar.setSetupSettings(Sonar::Setup(), true);
            break;

        case 's':
            sonar.saveConfig(path + m_device->info.pnSnAsStr() + " settings.xml");
            break;

        case 'r':
            sonar.startScanning();
            break;

        case 'R':
            sonar.stopScanning();
            break;

        case 'p':
            renderPalette(path);
            break;

        case 'i':
            m_circular.render(sonarDataStore, m_palette, true);
            BmpFile::save(path + "texture.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
            break;

        case 't':
            m_texture.renderTexture(sonarDataStore, m_palette, false);
            BmpFile::save(path + "sonar.bmp", reinterpret_cast<const uint32_t*>(&m_texture.buf[0]), 32, m_texture.width, m_texture.height);
            break;

        default:
            break;
        }

        App::doTask(key, path);
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::connectEvent(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    m_circular.setBuffer(1000, 1000, true);
    m_circular.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);
    m_circular.useBilinerInterpolation = true;

    // Optimal texture size to pass to the GPU - each pixel represents a data point. The GPU can then map this texture to circle (triangle fan)
    m_texture.useBilinerInterpolation = false;
    m_texture.setBuffer(sonar.settings.setup.imageDataPoint, Sonar::maxAngle / Math::abs(sonar.settings.setup.stepSize), true);
    m_texture.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount)
{
    EulerAngles euler = q.toEulerAngles(0);

    Debug::log(Debug::Severity::Info, name.c_str(), "H:%.1f    P:%.2f    R%.2f", Math::radToDeg(euler.heading), Math::radToDeg(euler.pitch), Math::radToDeg(euler.roll));
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackGyroData(GyroSensor& gyro, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Gyro x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackAccelData(AccelSensor& accel, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Accel x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackMagData(MagSensor& mag, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Mag x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress)
{
    const char* lable[] = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };

    Debug::log(Debug::Severity::Info, name.c_str(), "accel %s   %.2f, %.2f, %.2f,   0x%02x", lable[static_cast<uint_t>(axis)], v.x, v.y, v.z, progress);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackSettingsUpdated(Sonar& iss360, bool_t ok, Sonar::Settings::Type settingsType)
{
    const char* settingsTypeStr[] = { "System", "Acostic", "Setup" };

    if (ok)
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "%s settings updated", settingsTypeStr[static_cast<uint_t>(settingsType)]);

        if (settingsType == Sonar::Settings::Type::Setup)
        {
            m_circular.setSectorArea(0, iss360.settings.setup.maxRangeMm, iss360.settings.setup.sectorStart, iss360.settings.setup.sectorSize);

            // Optimal texture size to pass to the GPU - each pixel represents a data point. The GPU can then map this texture to circle (triangle fan)
            m_texture.setBuffer(iss360.settings.setup.imageDataPoint, Sonar::maxAngle / Math::abs(iss360.settings.setup.stepSize), true);
            m_texture.setSectorArea(0, iss360.settings.setup.maxRangeMm, iss360.settings.setup.sectorStart, iss360.settings.setup.sectorSize);
        }
    }
    else
    {
        Debug::log(Debug::Severity::Warning, name.c_str(), "%s settings failed to update", settingsTypeStr[static_cast<uint_t>(settingsType)]);
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackHeadHome(Sonar& iss360, const Sonar::HeadHome& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Head Homed");
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackPingData(Sonar& iss360, const Sonar::Ping& ping)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Ping data");

    uint_t txPulseLengthMm = static_cast<uint_t>(iss360.settings.setup.speedOfSound * iss360.settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
    txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);

    sonarDataStore.add(ping, txPulseLengthMm);

    m_pingCount++;

    if (m_pingCount % (Sonar::maxAngle / iss360.settings.setup.stepSize) == 0)
    {
        m_pingCount = 0;
        /*
        m_texture.renderTexture(sonarDataStore, m_palette, false);
        BmpFile::save("snrTex.bmp", reinterpret_cast<const uint32_t*>(&m_texture.buf[0]), 32, m_texture.width, m_texture.height);

        m_circular.render(sonarDataStore, m_palette, true);
        BmpFile::save("snrCi.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
        */
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackEchoData(Sonar& iss360, const Sonar::Echos& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Echo data, size:%u", data.data.size());
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackPwrAndTemp(Sonar& iss360, const Sonar::CpuPowerTemp& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "CPU:%.1f, SYS:%.1f, 1.0V:%.2f, 1.8V:%.2f, 1.35V:%.2f", data.cpuTemperature, data.auxTemperature, data.core1V0, data.aux1V8, data.ddr1V35);
}
//--------------------------------------------------------------------------------------------------

