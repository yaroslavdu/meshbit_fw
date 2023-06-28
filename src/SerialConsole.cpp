#include "SerialConsole.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "configuration.h"

#ifdef HAS_SDCARD
#include <SD.h>
#include <FS.h>
#include <FSCommon.h>
#endif

#include "SDCardLogger.h"

#define Port Serial
// Defaulting to the formerly removed phone_timeout_secs value of 15 minutes
#define SERIAL_CONNECTION_TIMEOUT (15 * 60) * 1000UL

SerialConsole *console;

void consoleInit()
{
  // Must be dynamically allocated because we are now inheriting from thread
#ifdef HAS_SDCARD
  new SerialAndFileConsole();
#else
  new SerialConsole();
#endif
}

void consolePrintf(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    console->vprintf(format, arg);
    va_end(arg);
    console->flush();
}

SerialConsole::SerialConsole() : StreamAPI(&Port), RedirectablePrint(&Port), concurrency::OSThread("SerialConsole")
{
    assert(!console);
    console = this;
    canWrite = false; // We don't send packets to our port until it has talked to us first
                      // setDestination(&noopPrint); for testing, try turning off 'all' debug output and see what leaks

    Port.begin(SERIAL_BAUD);
#if defined(ARCH_NRF52) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
    time_t timeout = millis();
    while (!Port) {
        if ((millis() - timeout) < 5000) {
            delay(100);
        } else {
            break;
        }
    }
#endif
    emitRebooted();
}

int32_t SerialConsole::runOnce()
{
    return runOncePart();
}

void SerialConsole::flush()
{
    Port.flush();
}

// For the serial port we can't really detect if any client is on the other side, so instead just look for recent messages
bool SerialConsole::checkIsConnected()
{
    uint32_t now = millis();
    return (now - lastContactMsec) < SERIAL_CONNECTION_TIMEOUT;
}

/**
 * we override this to notice when we've received a protobuf over the serial
 * stream.  Then we shunt off debug serial output.
 */
bool SerialConsole::handleToRadio(const uint8_t *buf, size_t len)
{
    // only talk to the API once the configuration has been loaded and we're sure the serial port is not disabled.
    if (config.has_lora && config.device.serial_enabled) {
        // Turn off debug serial printing once the API is activated, because other threads could print and corrupt packets
        if (!config.device.debug_log_enabled)
            setDestination(&noopPrint);
        canWrite = true;

        return StreamAPI::handleToRadio(buf, len);
    } else {
        return false;
    }
}

SerialAndFileConsole::SerialAndFileConsole() : SerialConsole() {}

bool SerialAndFileConsole::tryStartingLogInFile(const char* filepath) {
    logfile_ = new fs::File();
    *logfile_ = SD.open(filepath, FILE_APPEND);
    if (*logfile_) {
        logfile_->write('#');
        logfile_->flush();
        return true;
    } else {
        delete logfile_;
        logfile_ = nullptr;
    }

    return false;
}

void SerialAndFileConsole::writeToFileWithCaching(uint8_t c) {
    /*cache_.emplace_back(c);
    if (cache_.size() < cache_limit_) {
        return;
    }
    if (not logfile_ or not *logfile_) {
        SerialConsole::write('\n');
        SerialConsole::write('\n');
        if (tryStartingLogInFile("/default_log.txt"))
            SerialConsole::write('!');
        else
            SerialConsole::write('?');
        SerialConsole::write('\n');
        SerialConsole::write('\n');
    } 
    //auto logfile = SD.open("/default_log.txt", FILE_WRITE, true);
    if (logfile_ and *logfile_) {
        logfile_->write(cache_.data(), cache_.size());
        //logfile_->flush();
        //logfile.close();
    }
    SerialConsole::write('\n');
    SerialConsole::write('\n');
    SerialConsole::write('#');
    SerialConsole::write('\n');
    
    
    cache_.clear();*/
    if (sd_card_log_writer) {
        sd_card_log_writer->append(c);
    }
}

SerialAndFileConsole::~SerialAndFileConsole() {
    // TODO: ensure closing the file handle correctly
    if (logfile_ and *logfile_) {
        logfile_->close();
        delete logfile_;
        logfile_ = nullptr;
    }
}