#include "PowerFSM.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include "main.h"
#include "power.h"
#include "mqtt/MQTT.h"
#include "NodeDB.h"

#include <Adafruit_LIS3DH.h>
#include <Adafruit_MPU6050.h>

#define ACCELEROMETER_CHECK_INTERVAL_MS 100
#define ACCELEROMETER_CLICK_THRESHOLD 40

namespace concurrency
{
class AccelerometerThread : public concurrency::OSThread
{
  public:
    AccelerometerThread(ScanI2C::DeviceType type = ScanI2C::DeviceType::NONE) : OSThread("AccelerometerThread")
    {
        if (accelerometer_found.port == ScanI2C::I2CPort::NO_I2C) {
            LOG_DEBUG("AccelerometerThread disabling due to no sensors found\n");
            disable();
            return;
        }

        if (!config.display.wake_on_tap_or_motion && !config.device.double_tap_as_button_press) {
            LOG_DEBUG("AccelerometerThread disabling due to no interested configurations\n");
            disable();
            return;
        }

        accelerometer_type = type;
        LOG_DEBUG("AccelerometerThread initializing\n");

        if (accelerometer_type == ScanI2C::DeviceType::MPU6050 &&
            mpu.begin(accelerometer_found.address, &Wire1)) {
            LOG_DEBUG("MPU6050 initializing\n");
            // setup motion detection
            mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
            mpu.setMotionDetectionThreshold(1);
            mpu.setMotionDetectionDuration(20);
            mpu.setInterruptPinLatch(
                true); // Keep it latched.  Will turn off when reinitialized.
            mpu.setInterruptPinPolarity(true);
        } else if (accelerometer_type == ScanI2C::DeviceType::LIS3DH && lis.begin(accelerometer_found.address)) {
            LOG_DEBUG("LIS3DH initializing\n");
            lis.setRange(LIS3DH_RANGE_2_G);
            // Adjust threshhold, higher numbers are less sensitive
            lis.setClick(config.device.double_tap_as_button_press ? 2 : 1, ACCELEROMETER_CLICK_THRESHOLD);
        }
    }

  protected:
    int32_t runOnce() override
    {
        canSleep = true; // Assume we should not keep the board awake

        if (accelerometer_type == ScanI2C::DeviceType::MPU6050) {
            if (mpu.getMotionInterruptStatus()) {
              wakeScreen();
            }
            reportMpu6050();
        } else if (accelerometer_type == ScanI2C::DeviceType::LIS3DH &&
                   lis.getClick() > 0) {
            uint8_t click = lis.getClick();
            if (!config.device.double_tap_as_button_press) {
                wakeScreen();
            }

            if (config.device.double_tap_as_button_press && (click & 0x20)) {
                buttonPress();
                return 500;
            }
        }
        return ACCELEROMETER_CHECK_INTERVAL_MS;
    }

  private:
    void wakeScreen()
    {
        if (powerFSM.getState() == &stateDARK) {
            LOG_INFO("Tap or motion detected. Turning on screen\n");
            powerFSM.trigger(EVENT_INPUT);
        }
    }

    void buttonPress()
    {
        LOG_DEBUG("Double-tap detected. Firing button press\n");
        powerFSM.trigger(EVENT_PRESS);
    }

    void reportMpu6050() {
        // TODO: for now this is called each time the accelerometer thread
        // spins, however it might be better to collect min/max/avg values
        // during some period (during several accel thread loops) and report
        // only occasionally, say, once per second or per ten seconds.

        using std::string;
        using std::to_string;

        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        const bool ok = mpu.getEvent(&accel, &gyro, &temp);
        if (not ok) {
            LOG_WARN("Can't get MPU6050 accelerometer event\n");
            return;
        }

        string timestamp;
        const uint64_t rtc_sec = getValidTime(RTCQuality::RTCQualityDevice);
        static constexpr uint64_t millis_in_second{1000};
        if (rtc_sec > 0) {
            timestamp = to_string(rtc_sec * millis_in_second + millis());
        } else {
            timestamp = "\"undefined\"";
        }

        const string device_name{
            (owner.short_name[0] != '\0') ? owner.short_name : "undefined"};

        // example:
        // {"device":"theo","time":1684925523038,"x":1.046266,"y":7.283162,"z":-5.889736,"t":28.012352}
        string accel_message{"{"};
        accel_message += "\"device\":\"" + device_name + "\"";
        accel_message += ",\"time\":" + timestamp;
        accel_message += ",\"x\":";
        accel_message += to_string(accel.acceleration.x);
        accel_message += ",\"y\":";
        accel_message += to_string(accel.acceleration.y);
        accel_message += ",\"z\":";
        accel_message += to_string(accel.acceleration.z);
        accel_message += ",\"t\":";
        accel_message += to_string(temp.temperature);
        accel_message += "}";

        LOG_INFO("%s", accel_message.c_str());

        if (mqtt && mqtt->connected()) {
            string topic;
            if (*moduleConfig.mqtt.root) {
                topic += moduleConfig.mqtt.root;
            }
            // TODO: decide on a better topic name or a configurable topic name
            // if necessary
            topic += "/accelerometer";
            mqtt->debugPublish(topic.c_str(), accel_message.c_str());
        } else {
            LOG_DEBUG("MQTT is not connected\n");
        }
    }

    ScanI2C::DeviceType accelerometer_type;
    Adafruit_MPU6050 mpu;
    Adafruit_LIS3DH lis;
};

} // namespace concurrency
