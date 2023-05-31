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

// TODO: move into a separate class file
#include <algorithm>
#include <list>
#include <numeric>
template <typename T> class Filter {
public:
  Filter(int size) : size_(size) {}
  void update(T value) {
    values_.push_back(value);
    if (values_.size() > size_) {
      values_.pop_front();
    }
  }
  void reset() { values_.clear(); }
  T min() const { return *std::min_element(values_.begin(), values_.end()); }
  T max() const { return *std::max_element(values_.begin(), values_.end()); }
  T avg() const {
    const T sum =
        std::accumulate(values_.begin(), values_.end(), T(0), std::plus<T>());
    const T avg = sum / static_cast<T>(std::max(values_.size(), size_t(1)));
    return avg;
  }

private:
  std::list<T> values_;
  size_t size_{0};
};

enum class MotionState { Moving, Still, LongStill, Unknown };

class Mpu6050DataHandler {
  using DataType = float;

public:
  const int sliding_window_length_{20};
  const DataType min_acceleration_diff_when_moving{0.08};

  void update_accelerations(DataType acceleration_x, DataType acceleration_y,
                            DataType acceleration_z) {
    // TODO: expand with angular momentums?
    x_.update(acceleration_x);
    y_.update(acceleration_y);
    z_.update(acceleration_z);
  }

  void compute_motion_state() {
    static constexpr DataType default_accel{0.0};
    const DataType avg_x_accel = x_.avg();
    const DataType avg_y_accel = y_.avg();
    const DataType avg_z_accel = z_.avg();

    if (avg_x_accel == default_accel and avg_y_accel == default_accel and
        last_avg_z_accel_ == default_accel) {
      // A true acceleration values for all axes can never be exactly zero,
      // but if it is, it means the sensor is probably not working correctly.
      motion_state_ = MotionState::Unknown;
      return;
    }

    const DataType x_diff = std::abs(avg_x_accel - last_avg_x_accel_);
    const DataType y_diff = std::abs(avg_y_accel - last_avg_y_accel_);
    const DataType z_diff = std::abs(avg_z_accel - last_avg_z_accel_);

    if (x_diff >= min_acceleration_diff_when_moving or
        y_diff >= min_acceleration_diff_when_moving or
        z_diff >= min_acceleration_diff_when_moving) {
      motion_state_ = MotionState::Moving;
      still_time_ = 0;
    } else {
      still_time_++;
      if (still_time_ < long_still_time_threshold) {
        motion_state_ = MotionState::Still;
      } else {
        motion_state_ = MotionState::LongStill;
      }
    }

    last_avg_x_accel_ = avg_x_accel;
    last_avg_y_accel_ = avg_y_accel;
    last_avg_z_accel_ = avg_z_accel;
  };

  // Getters for retrieveing avg, min, max values per axis
  const Filter<DataType> &x() { return x_; };
  const Filter<DataType> &y() { return y_; };
  const Filter<DataType> &z() { return z_; };

  std::string motion_state_as_string() {
    return motion_state_ == MotionState::Still       ? "still"
           : motion_state_ == MotionState::Moving    ? "moving"
           : motion_state_ == MotionState::LongStill ? "long_still"
                                                     : "unknown";
  }

private:
  Filter<DataType> x_{sliding_window_length_};
  Filter<DataType> y_{sliding_window_length_};
  Filter<DataType> z_{sliding_window_length_};

  DataType last_avg_x_accel_{0};
  DataType last_avg_y_accel_{0};
  DataType last_avg_z_accel_{0};

  MotionState motion_state_{MotionState::Unknown};
  int still_time_{0};
  const int long_still_time_threshold{10};
};

namespace concurrency
{
class AccelerometerThread : public concurrency::OSThread
{
  public:
    AccelerometerThread(ScanI2C::DeviceType type = ScanI2C::DeviceType::NONE) : OSThread("AccelerometerThread")
    {
        TwoWire *accelerometer_port_ptr = nullptr;
        if (accelerometer_found.port == ScanI2C::I2CPort::NO_I2C) {
            LOG_DEBUG("AccelerometerThread disabling due to no sensors found\n");
            disable();
            return;
        } else if (accelerometer_found.port == ScanI2C::I2CPort::WIRE) {
            accelerometer_port_ptr = &Wire;
        } else {
            accelerometer_port_ptr = &Wire1;
        }

        if (!config.display.wake_on_tap_or_motion && !config.device.double_tap_as_button_press) {
            LOG_DEBUG("AccelerometerThread disabling due to no interested configurations\n");
            disable();
            return;
        }

        accelerometer_type = type;
        LOG_DEBUG("AccelerometerThread initializing\n");

        if (accelerometer_type == ScanI2C::DeviceType::MPU6050 &&
            mpu.begin(accelerometer_found.address, accelerometer_port_ptr)) {
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

        mpu6050_data_handler_.update_accelerations(
            accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

        if (report_counter_++ < reporting_period_) {
            return;
        }
        report_counter_ = 0;

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

        mpu6050_data_handler_.compute_motion_state();

        // example:
        // {"device":"b034","time":1685533673943,"x":-9.234076,"y":1.325669,"z":1.327226,
        //  "t":28.577059,"motion":"still"}
        string accel_message{"{"};
        accel_message += "\"device\":\"" + device_name + "\"";
        accel_message += ",\"time\":" + timestamp;
        accel_message += ",\"x\":";
        accel_message += to_string(mpu6050_data_handler_.x().avg());
        accel_message += ",\"y\":";
        accel_message += to_string(mpu6050_data_handler_.y().avg());
        accel_message += ",\"z\":";
        accel_message += to_string(mpu6050_data_handler_.z().avg());
        accel_message += ",\"t\":";
        accel_message += to_string(temp.temperature);
        accel_message += ",\"motion\":\"" +
                         mpu6050_data_handler_.motion_state_as_string() + "\"";
        accel_message += "}";

        LOG_INFO("%s\n", accel_message.c_str());

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

    int report_counter_{0};
    const int reporting_period_{10};
    Mpu6050DataHandler mpu6050_data_handler_;
};

} // namespace concurrency
