#pragma once

#include <vector>
#include <string>

#include "concurrency/OSThread.h"

namespace fs {
class File;
}

void sdCardLogWriterInit();

using std::string;

class SDCardLogger : public concurrency::OSThread {

public:
  SDCardLogger();
  ~SDCardLogger();

  int32_t runOnce() override;

  void append(uint8_t c);

  const string& current_log_file_path() const;

  static constexpr size_t cache_limit{1024};
  const string log_file_extension = ".log";
  const string log_folder = "/lifelink/logs/";
  const string default_name = "default";

private:
  std::vector<uint8_t> cache_;
  mutable uint32_t last_file_timestamp_{0};
  mutable string current_path_{log_folder + default_name + log_file_extension};
};

extern SDCardLogger *sd_card_log_writer;
