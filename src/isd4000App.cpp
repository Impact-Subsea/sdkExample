//------------------------------------------ Includes ----------------------------------------------

#include "isd4000App.h"
#include "maths/maths.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Isd4000App::Isd4000App(void) : App("Isd4000App")
{
    Debug::log(Debug::Severity::Notice, name.c_str(), "created" NEW_LINE
                                                      "d -> Set settings to defualt" NEW_LINE
                                                      "s -> Save settings to file" NEW_LINE);
}
//--------------------------------------------------------------------------------------------------
Isd4000App::~Isd4000App(void)
{
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::connectSignals(Device & device)
{
    Isd4000& isd4000 = reinterpret_cast<Isd4000&>(device);

    isd4000.ahrs.onData.connect(slotAhrsData);                  // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //isd4000.gyro.onData.connect(slotGyroData);                // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //isd4000.accel.onData.connect(slotAccelData);              // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //isd4000.mag.onData.connect(slotMagData);                  // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    isd4000.accel.onCalProgress.connect(slotAccelCal);

    isd4000.onPressure.connect(slotPressure);                   // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    isd4000.onTemperature.connect(slotTemperature);             // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    isd4000.onScriptDataReceived.connect(slotScriptDataReceived);
    isd4000.onSettingsUpdated.connect(slotSettingsUpdated);
    isd4000.onPressureCalCert.connect(slotPressureCalCert);
    isd4000.onTemperatureCalCert.connect(slotTemperatureCalCert);

    Isd4000::SensorRates rates;
    rates.pressure = 100;
    rates.ahrs = 100;
    rates.gyro = 100;
    rates.accel = 100;
    rates.mag = 100;
    rates.temperature = 200;
    isd4000.setSensorRates(rates);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::disconnectSignals(Device& device)
{
    Isd4000& isd4000 = reinterpret_cast<Isd4000&>(device);

    isd4000.ahrs.onData.disconnect(slotAhrsData);
    isd4000.gyro.onData.disconnect(slotGyroData);
    isd4000.accel.onData.disconnect(slotAccelData);
    isd4000.mag.onData.disconnect(slotMagData);
    isd4000.accel.onCalProgress.disconnect(slotAccelCal);

    isd4000.onPressure.disconnect(slotPressure);
    isd4000.onTemperature.disconnect(slotTemperature);
    isd4000.onScriptDataReceived.disconnect(slotScriptDataReceived);
    isd4000.onSettingsUpdated.disconnect(slotSettingsUpdated);
    isd4000.onPressureCalCert.disconnect(slotPressureCalCert);
    isd4000.onTemperatureCalCert.disconnect(slotTemperatureCalCert);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::doTask(int_t key, const std::string& path)
{
    if (m_device)
    {
        Isd4000& isd4000 = reinterpret_cast<Isd4000&>(*m_device);
        switch (key)
        {
        case 'd':
            isd4000.setSettings(Isd4000::Settings(), true);
            break;

        case 's':
            isd4000.saveConfig(path + m_device->info.pnSnAsStr() + " settings.xml");
            break;

        default:
            break;
        }
        App::doTask(key, path);
    }
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount)
{
    EulerAngles euler = q.toEulerAngles(0);

    Debug::log(Debug::Severity::Info, name.c_str(), "H:%.1f    P:%.2f    R%.2f", Math::radToDeg(euler.heading), Math::radToDeg(euler.pitch), Math::radToDeg(euler.roll));
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackGyroData(GyroSensor& gyro, const Vector3& v)
{
    Debug::log(Debug::Severity::Info,name.c_str(), "Gyro x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackAccelData(AccelSensor& accel, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Accel x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackMagData(MagSensor& mag, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Mag x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress)
{
    const char* lable[] = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };

    Debug::log(Debug::Severity::Info,name.c_str(), "accel %s   %.2f, %.2f, %.2f,   0x%02x", lable[static_cast<uint_t>(axis)], v.x, v.y, v.z, progress);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackPressureData(Isd4000& isd4000, uint64_t timeUs, real_t pressureBar, real_t depthM, real_t pressureBarRaw)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Pressure %.5f Bar, Depth %.3f Meters", pressureBar, depthM);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackTemperatureData(Isd4000& isd4000, real_t temperatureC, real_t temperatureRawC)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Temperature %.2fC", temperatureRawC);
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackScriptDataReceived(Isd4000& isd4000)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Script data received");
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackSettingsUpdated(Isd4000& isd4000, bool_t ok)
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
void Isd4000App::callbackPressureCal(Isd4000& isd4000, const Isd4000::PressureCal& cal)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Pressure cal received");
}
//--------------------------------------------------------------------------------------------------
void Isd4000App::callbackTemperatureCal(Isd4000& isd4000, const Isd4000::TemperatureCal& cal)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Temperature cal received");
}
//--------------------------------------------------------------------------------------------------
