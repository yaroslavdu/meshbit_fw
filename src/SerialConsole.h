#pragma once

#include "RedirectablePrint.h"
#include "StreamAPI.h"

#include <vector>

/**
 * Provides both debug printing and, if the client starts sending protobufs to us, switches to send/receive protobufs
 * (and starts dropping debug printing - FIXME, eventually those prints should be encapsulated in protobufs).
 */
class SerialConsole : public StreamAPI, public RedirectablePrint, private concurrency::OSThread
{
  public:
    SerialConsole();

    /**
     * we override this to notice when we've received a protobuf over the serial stream.  Then we shunt off
     * debug serial output.
     */
    virtual bool handleToRadio(const uint8_t *buf, size_t len) override;

    virtual size_t write(uint8_t c) override
    {
        if (c == '\n') // prefix any newlines with carriage return
            RedirectablePrint::write('\r');
        // TODO: impl. writing to the file handle
        return RedirectablePrint::write(c);
    }

    virtual int32_t runOnce() override;

    void flush();

  protected:
    /// Check the current underlying physical link to see if the client is currently connected
    virtual bool checkIsConnected() override;
};

// A simple wrapper to allow non class aware code write to the console
void consolePrintf(const char *format, ...);
void consoleInit();

namespace fs {
class File;
}

/**
 * Provides both debug printing and, if the client starts sending protobufs to us, switches to send/receive protobufs
 * (and starts dropping debug printing - FIXME, eventually those prints should be encapsulated in protobufs).
 */
class SerialAndFileConsole : public SerialConsole {
  public:
    using SerialConsole::checkIsConnected;
    using SerialConsole::flush;
    using SerialConsole::handleToRadio;
    using SerialConsole::runOnce;

    SerialAndFileConsole();

    virtual ~SerialAndFileConsole() override;

    bool tryStartingLogInFile(const char* filepath);

    virtual size_t write(uint8_t c) override {
        // Write to the card here just as well.
        writeToFileWithCaching(c);
        return SerialConsole::write(c);
    }

  private:
    // TODO: file handle
    fs::File* logfile_{nullptr};
    std::vector<uint8_t> cache_;
    static constexpr size_t cache_limit_{300};
    void writeToFileWithCaching(uint8_t c);
};

extern SerialConsole *console;
