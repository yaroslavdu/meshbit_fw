#include "SDCardLogger.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "configuration.h"

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

  if (SD.exists(default_log_filename)) {
    auto f = SD.open(default_log_filename, FILE_READ);
    if (f) {
      LOG_DEBUG("Default log file exists, size: %d\n", f.available());
    }
    f.close();
  }
}

int32_t SDCardLogger::runOnce() {
  if (cache_.size() < cache_limit) {
    return 0;
  }

  auto logfile = SD.open(default_log_filename, FILE_APPEND);
  if (logfile) {
    logfile.write(cache_.data(), cache_.size());
  }
  logfile.close();

  cache_.clear();

  LOG_DEBUG("Dumped console log to SD card.\n");
  return 0;
}

void SDCardLogger::append(uint8_t c) { cache_.emplace_back(c); }

SDCardLogger::~SDCardLogger() {}