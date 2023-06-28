#pragma once

#include <vector>

#include "concurrency/OSThread.h"

namespace fs {
class File;
}

void sdCardLogWriterInit();

class SDCardLogger : public concurrency::OSThread {
public:
  SDCardLogger();
  ~SDCardLogger();

  int32_t runOnce() override;

  void append(uint8_t c);

  static constexpr size_t cache_limit{1024};
  const char *default_log_filename = "/log.txt";

private:
  std::vector<uint8_t> cache_;
};

extern SDCardLogger *sd_card_log_writer;
