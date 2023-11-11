//------------------------------------------ Includes ----------------------------------------------

#include "ism3dApp.h"
#include "maths/maths.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Ism3dApp::Ism3dApp(void) : App("Ism3dApp")
{
    Debug::log(Debug::Severity::Notice, name.c_str(), "created" NEW_LINE
                                                      "d -> Set settings to defualt" NEW_LINE
                                                      "s -> Save settings to file" NEW_LINE);
}
//--------------------------------------------------------------------------------------------------
Ism3dApp::~Ism3dApp(void)
{
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::connectSignals(Device& device)
{
    Ism3d& ism3d = reinterpret_cast<Ism3d&>(device);

    ism3d.ahrs.onData.connect(slotAhrsData);                    // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //ism3d.gyro.onData.connect(slotGyroData);                  // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //ism3d.gyroSec.onData.connect(slotGyroData);               // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //ism3d.accel.onData.connect(slotAccelData);                // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //ism3d.accelSec.onData.connect(slotAccelData);             // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //ism3d.mag.onData.connect(slotMagData);                    // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    ism3d.accel.onCalProgress.connect(slotAccelCal);

    ism3d.onScriptDataReceived.connect(slotScriptDataReceived);
    ism3d.onSettingsUpdated.connect(slotSettingsUpdated);

    Ism3d::SensorRates rates;
    rates.ahrs = 100;
    rates.gyro = 100;
    rates.accel = 100;
    rates.mag = 100;
    ism3d.setSensorRates(rates);
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::disconnectSignals(Device& device)
{
    Ism3d& ism3d = reinterpret_cast<Ism3d&>(device);

    ism3d.ahrs.onData.disconnect(slotAhrsData);
    ism3d.gyro.onData.disconnect(slotGyroData);
    ism3d.gyroSec.onData.disconnect(slotGyroData);
    ism3d.accel.onData.disconnect(slotAccelData);
    ism3d.accelSec.onData.disconnect(slotAccelData);
    ism3d.mag.onData.disconnect(slotMagData);
    ism3d.accel.onCalProgress.disconnect(slotAccelCal);

    ism3d.onScriptDataReceived.disconnect(slotScriptDataReceived);
    ism3d.onSettingsUpdated.disconnect(slotSettingsUpdated);
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::doTask(int_t key, const std::string& path)
{
    if (m_device)
    {
        Ism3d& ism3d = reinterpret_cast<Ism3d&>(*m_device);

        switch (key)
        {
        case 'd':
            ism3d.setSettings(Ism3d::Settings(), true);
            break;

        case 's':
            ism3d.saveConfig(path + m_device->info.pnSnAsStr() + " settings.xml");
            break;

        default:
            break;
        }

        App::doTask(key, path);
    }
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount)
{
    EulerAngles euler = q.toEulerAngles(0);

    Debug::log(Debug::Severity::Info, name.c_str(), "H:%.1f    P:%.2f    R%.2f", Math::radToDeg(euler.heading), Math::radToDeg(euler.pitch), Math::radToDeg(euler.roll));
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::callbackGyroData(GyroSensor& gyro, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Gyro:%u x:%.2f, y:%.2f, z:%.2f", gyro.sensorNumber, v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::callbackAccelData(AccelSensor& accel, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Accel:%u x:%.2f, y:%.2f, z:%.2f", accel.sensorNumber, v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::callbackMagData(MagSensor& mag, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Mag x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress)
{
    const char* lable[] = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };

    Debug::log(Debug::Severity::Info, name.c_str(), "accel %s   %.2f, %.2f, %.2f,   0x%02x", lable[static_cast<uint_t>(axis)], v.x, v.y, v.z, progress);
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::callbackScriptDataReceived(Ism3d& ism3d)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Script data received");
}
//--------------------------------------------------------------------------------------------------
void Ism3dApp::callbackSettingsUpdated(Ism3d& ism3d, bool_t ok)
{
    if (ok)
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "Settings updated ok");
    }
    else
    {
        Debug::log(Debug::Severity::Warning, name.c_str(), "Settings failed to update");
    }
}
//--------------------------------------------------------------------------------------------------
