//------------------------------------------ Includes ----------------------------------------------

#include "isa500App.h"
#include "maths/maths.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Isa500App::Isa500App(void) : App("Isa500App")
{
    Debug::log(Debug::Severity::Notice, name.c_str(), "created" NEW_LINE
                                                      "d -> Set settings to defualt" NEW_LINE
                                                      "s -> Save settings to file" NEW_LINE
                                                      "p -> Ping now" NEW_LINE);
}
//--------------------------------------------------------------------------------------------------
Isa500App::~Isa500App(void)
{
}
//--------------------------------------------------------------------------------------------------
void Isa500App::connectSignals(Device& device)
{
    Isa500& isa500 = reinterpret_cast<Isa500&>(device);

    isa500.ahrs.onData.connect(slotAhrsData);                   // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //isa500.gyro.onData.connect(slotGyroData);                 // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //isa500.accel.onData.connect(slotAccelData);               // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //isa500.mag.onData.connect(slotMagData);                   // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    isa500.accel.onCalProgress.connect(slotAccelCal);

    isa500.onEcho.connect(slotEchoData);                        // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    isa500.onEchogramData.connect(slotPingData);
    //isa500.onTemperature.connect(slotTemperatureData);        // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //isa500.onVoltage.connect(slotVoltageData);                // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    isa500.onTrigger.connect(slotTriggerData);
    isa500.onScriptDataReceived.connect(slotScriptDataReceived);
    isa500.onSettingsUpdated.connect(slotSettingsUpdated);

    Isa500::SensorRates rates;
    rates.ping = 0;
    rates.ahrs = 100;
    rates.gyro = 100;
    rates.accel = 100;
    rates.mag = 100;
    rates.temperature = 1000;
    rates.voltage = 1000;
    isa500.setSensorRates(rates);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::disconnectSignals(Device& device)
{
    Isa500& isa500 = reinterpret_cast<Isa500&>(device);

    isa500.ahrs.onData.disconnect(slotAhrsData);
    isa500.gyro.onData.disconnect(slotGyroData);
    isa500.accel.onData.disconnect(slotAccelData);
    isa500.mag.onData.disconnect(slotMagData);
    isa500.accel.onCalProgress.disconnect(slotAccelCal);

    isa500.onEcho.disconnect(slotEchoData);
    isa500.onEchogramData.disconnect(slotPingData);
    isa500.onTemperature.disconnect(slotTemperatureData);
    isa500.onVoltage.disconnect(slotVoltageData);
    isa500.onTrigger.disconnect(slotTriggerData);
    isa500.onScriptDataReceived.disconnect(slotScriptDataReceived);
    isa500.onSettingsUpdated.disconnect(slotSettingsUpdated);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::doTask(int_t key, const std::string& path)
{
    if (m_device)
    {
        Isa500& isa500 = reinterpret_cast<Isa500&>(*m_device);

        switch (key)
        {
        case 'd':
            isa500.setSettings(Isa500::Settings(), true);
            break;

        case 's':
            isa500.saveConfig(path + m_device->info.pnSnAsStr() + " settings.xml");
            break;

        case 'p':
            isa500.pingNow();
            break;

        default:
            break;
        }

        App::doTask(key, path);
    }
}
//--------------------------------------------------------------------------------------------------
void Isa500App::connectEvent(Device& device)
{
    Isa500& isa500 = reinterpret_cast<Isa500&>(device);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount)
{
    EulerAngles euler = q.toEulerAngles(0);

    Debug::log(Debug::Severity::Info, name.c_str(), "H:%.1f    P:%.2f    R%.2f", Math::radToDeg(euler.heading), Math::radToDeg(euler.pitch), Math::radToDeg(euler.roll));
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackGyroData(GyroSensor& gyro, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Gyro x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackAccelData(AccelSensor& accel, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Accel x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackMagData(MagSensor& mag, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Mag x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress)
{
    const char* lable[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

    Debug::log(Debug::Severity::Info, name.c_str(), "accel %s   %.2f, %.2f, %.2f,   0x%02x", lable[static_cast<uint_t>(axis)], v.x, v.y, v.z, progress);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackEchoData(Isa500& isa500, uint64_t timeUs, uint_t selectedIdx, uint_t totalEchoCount, const std::vector<Isa500::Echo>& echoes)
{
    if (echoes.size())
    {
        // echoes.size() is limited to isa500.settings.multiEchoLimit
        // totalEchoCount is the number of received echoes and has nothing to do with the length of echoes array
        Debug::log(Debug::Severity::Info, name.c_str(), "Echo received, range %.3f meters. Total Echoes: %u", echoes[selectedIdx].totalTof * isa500.settings.speedOfSound * 0.5, totalEchoCount);
    }
    else
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "No echoes received");
    }
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackEchogramData(Isa500& isa500, const std::vector<uint8_t>& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Echogram data size: %u bytes", data.size());

    /*for (size_t i = 0; i < data.size(); i++)
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "%u", FMT_U(data[i]));
    }*/
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackTemperatureData(Isa500& isa500, real_t temperatureC)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Temperature %.2f", temperatureC);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackVoltageData(Isa500& isa500, real_t voltage12)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Voltage %.2fV", voltage12);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackTriggerData(Isa500& isa500, bool_t risingEdge)
{
    const char* lable[] = { "falling", "rising" };

    Debug::log(Debug::Severity::Info, name.c_str(), "Trigger, %s edge", lable[risingEdge]);
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackScriptDataReceived(Isa500& isa500)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Script data received");
}
//--------------------------------------------------------------------------------------------------
void Isa500App::callbackSettingsUpdated(Isa500& isa500, bool_t ok)
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

