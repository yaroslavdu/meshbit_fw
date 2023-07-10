#include "SDCardLogger.h"
#include "RTC.h"

#ifdef HAS_SDCARD
#include <FS.h>
#include <FSCommon.h>
#include <SD.h>
#endif

SDCardLogger *sd_card_log_writer{nullptr};

void sdCardLogWriterInit() {
  // Must be dynamically allocated because we are now inheriting from thread
#ifdef HAS_SDCARD
  new SDCardLogger();
#endif
}

SDCardLogger::SDCardLogger() : concurrency::OSThread("SDCardLogger") {
  assert(!sd_card_log_writer);
  sd_card_log_writer = this;

  setupSDCard();

  const auto path = "/default.txt";
  // current_log_file_path().c_str();
  if (SD.exists(path)) {
    auto f = SD.open(path, FILE_READ, true);
    if (f) {
      LOG_DEBUG("Log file at %s exists, size: %d\n", path, f.available());
    }
    f.close();
  }
}

int32_t SDCardLogger::runOnce() {
  if (cache_.size() < cache_limit) {
    return 0;
  }
  const auto path = current_log_file_path().c_str();
  auto logfile = SD.open(path, FILE_WRITE, true);
  if (logfile) {
    logfile.write(cache_.data(), cache_.size());
    logfile.close();
    LOG_DEBUG("Dumped console log to SD card.\n");
  } else {
    LOG_WARN("Can't open log file %s on SD card!\n", path);
  }

  cache_.clear();

  return 0;
}

const std::string &SDCardLogger::current_log_file_path() const {
  static constexpr uint32_t seconds_in_day{86400};
  static constexpr uint32_t seconds_in_hour{3600};
  static constexpr uint32_t seconds_in_minute{60};
  static constexpr uint32_t new_log_file_period{seconds_in_hour};

  const uint32_t rtc_sec = getValidTime(RTCQuality::RTCQualityDevice);

  if (rtc_sec > (last_file_timestamp_ + new_log_file_period)) {
    last_file_timestamp_ = rtc_sec;
    current_path_ =
        log_folder + std::to_string(last_file_timestamp_) + log_file_extension;
  }

  return current_path_;
}

void SDCardLogger::append(uint8_t c) { cache_.emplace_back(c); }

SDCardLogger::~SDCardLogger() {}