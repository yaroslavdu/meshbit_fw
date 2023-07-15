#include "FSCommon.h"
#include "configuration.h"

#ifdef HAS_SDCARD
#include <SD.h>
#include <SPI.h>

#ifdef SDCARD_USE_SPI1
SPIClass SPI1(HSPI);
#define SDHandler SPI1
#endif

#ifdef SDCARD_USE_SPI2
SPIClass SPI2(HSPI);
#define SDHandler SPI2
#endif

#endif // HAS_SDCARD

bool copyFile(const char *from, const char *to)
{
#ifdef FSCom
    unsigned char cbuffer[16];

    File f1 = FSCom.open(from, FILE_O_READ);
    if (!f1) {
        LOG_ERROR("Failed to open source file %s\n", from);
        return false;
    }

    File f2 = FSCom.open(to, FILE_O_WRITE);
    if (!f2) {
        LOG_ERROR("Failed to open destination file %s\n", to);
        return false;
    }

    while (f1.available() > 0) {
        byte i = f1.read(cbuffer, 16);
        f2.write(cbuffer, i);
    }

    f2.flush();
    f2.close();
    f1.close();
    return true;
#endif
}

bool renameFile(const char *pathFrom, const char *pathTo)
{
#ifdef FSCom
#ifdef ARCH_ESP32
    // rename was fixed for ESP32 IDF LittleFS in April
    return FSCom.rename(pathFrom, pathTo);
#else
    if (copyFile(pathFrom, pathTo) && FSCom.remove(pathFrom)) {
        return true;
    } else {
        return false;
    }
#endif
#endif
}

void listDir(const char *dirname, uint8_t levels, boolean del = false)
{
#ifdef FSCom
#if (defined(ARCH_ESP32) || defined(ARCH_RP2040) || defined(ARCH_PORTDUINO))
    char buffer[255];
#endif
    File root = FSCom.open(dirname, FILE_O_READ);
    if (!root) {
        return;
    }
    if (!root.isDirectory()) {
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory() && !String(file.name()).endsWith(".")) {
            if (levels) {
#ifdef ARCH_ESP32
                listDir(file.path(), levels - 1, del);
                if (del) {
                    LOG_DEBUG("Removing %s\n", file.path());
                    strncpy(buffer, file.path(), sizeof(buffer));
                    file.close();
                    FSCom.rmdir(buffer);
                } else {
                    file.close();
                }
#elif (defined(ARCH_RP2040) || defined(ARCH_PORTDUINO))
                listDir(file.name(), levels - 1, del);
                if (del) {
                    LOG_DEBUG("Removing %s\n", file.name());
                    strncpy(buffer, file.name(), sizeof(buffer));
                    file.close();
                    FSCom.rmdir(buffer);
                } else {
                    file.close();
                }
#else
                listDir(file.name(), levels - 1, del);
                file.close();
#endif
            }
        } else {
#ifdef ARCH_ESP32
            if (del) {
                LOG_DEBUG("Deleting %s\n", file.path());
                strncpy(buffer, file.path(), sizeof(buffer));
                file.close();
                FSCom.remove(buffer);
            } else {
                LOG_DEBUG(" %s (%i Bytes)\n", file.path(), file.size());
                file.close();
            }
#elif (defined(ARCH_RP2040) || defined(ARCH_PORTDUINO))
            if (del) {
                LOG_DEBUG("Deleting %s\n", file.name());
                strncpy(buffer, file.name(), sizeof(buffer));
                file.close();
                FSCom.remove(buffer);
            } else {
                LOG_DEBUG(" %s (%i Bytes)\n", file.name(), file.size());
                file.close();
            }
#else
            LOG_DEBUG(" %s (%i Bytes)\n", file.name(), file.size());
            file.close();
#endif
        }
        file = root.openNextFile();
    }
#ifdef ARCH_ESP32
    if (del) {
        LOG_DEBUG("Removing %s\n", root.path());
        strncpy(buffer, root.path(), sizeof(buffer));
        root.close();
        FSCom.rmdir(buffer);
    } else {
        root.close();
    }
#elif (defined(ARCH_RP2040) || defined(ARCH_PORTDUINO))
    if (del) {
        LOG_DEBUG("Removing %s\n", root.name());
        strncpy(buffer, root.name(), sizeof(buffer));
        root.close();
        FSCom.rmdir(buffer);
    } else {
        root.close();
    }
#else
    root.close();
#endif
#endif
}

void rmDir(const char *dirname)
{
#ifdef FSCom
#if (defined(ARCH_ESP32) || defined(ARCH_RP2040) || defined(ARCH_PORTDUINO))
    listDir(dirname, 10, true);
#elif defined(ARCH_NRF52)
    // nRF52 implementation of LittleFS has a recursive delete function
    FSCom.rmdir_r(dirname);
#endif
#endif
}

void fsInit()
{
#ifdef FSCom
    if (!FSBegin()) {
        LOG_ERROR("Filesystem mount Failed.\n");
        // assert(0); This auto-formats the partition, so no need to fail here.
    }
#ifdef ARCH_ESP32
    LOG_DEBUG("Filesystem files (%d/%d Bytes):\n", FSCom.usedBytes(), FSCom.totalBytes());
#else
    LOG_DEBUG("Filesystem files:\n");
#endif
    listDir("/", 10);
#endif
}

void setupSDCard()
{
#ifdef HAS_SDCARD
    SDHandler.begin(SD_SPI_SCLK, SD_SPI_MISO, SD_SPI_MOSI, SD_SPI_CS);
    pinMode(SD_SPI_CS, OUTPUT);

    if (!SD.begin(SD_SPI_CS, SDHandler)) {
        LOG_DEBUG("No SD_MMC card detected\n");
        return;
    }
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        LOG_DEBUG("No SD_MMC card attached\n");
        return;
    }
    LOG_DEBUG("SD_MMC Card Type: ");
    if (cardType == CARD_MMC) {
        LOG_DEBUG("MMC\n");
    } else if (cardType == CARD_SD) {
        LOG_DEBUG("SDSC\n");
    } else if (cardType == CARD_SDHC) {
        LOG_DEBUG("SDHC\n");
    } else {
        LOG_DEBUG("UNKNOWN\n");
    }

    static constexpr uint64_t bytes_in_mb{uint64_t(1024) * 1024};
    const uint32_t cardSizeMB = static_cast<uint32_t>(SD.cardSize() / bytes_in_mb);
    const uint32_t totalMegaBytes = static_cast<uint32_t>(SD.totalBytes() / bytes_in_mb);
    const uint32_t usedMegaBytes = static_cast<uint32_t>(SD.usedBytes() / bytes_in_mb);
    LOG_DEBUG("SD Card Size: %lu MB\n", cardSizeMB);
    LOG_DEBUG("Total space: %lu MB\n", totalMegaBytes);
    LOG_DEBUG("Used space: %lu MB\n", usedMegaBytes);
#endif
}
