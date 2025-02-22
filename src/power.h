#pragma once
#include "PowerStatus.h"
#include "concurrency/OSThread.h"
#ifdef ARCH_ESP32
#include <esp_adc_cal.h>
#include <soc/adc_channel.h>
#endif
/**
 * Per @spattinson
 * MIN_BAT_MILLIVOLTS seems high. Typical 18650 are different chemistry to LiPo, even for LiPos that chart seems a bit off, other
 * charts put 3690mV at about 30% for a lipo, for 18650 i think 10% remaining iis in the region of 3.2-3.3V. Reference 1st graph
 * in [this test report](https://lygte-info.dk/review/batteries2012/Samsung%20INR18650-30Q%203000mAh%20%28Pink%29%20UK.html)
 * looking at the red line - discharge at 0.2A - he gets a capacity of 2900mah, 90% of 2900 = 2610, that point in the graph looks
 * to be a shade above 3.2V
 */
#define MIN_BAT_MILLIVOLTS 3250 // millivolts. 10% per https://blog.ampow.com/lipo-voltage-chart/

#define BAT_MILLIVOLTS_FULL 4100
#define BAT_MILLIVOLTS_EMPTY 3500
#ifdef BAT_MEASURE_ADC_UNIT
extern RTC_NOINIT_ATTR uint64_t RTC_reg_b;
#include "soc/sens_reg.h" // needed for adc pin reset
#endif

class Power : private concurrency::OSThread
{

  public:
    Observable<const meshtastic::PowerStatus *> newStatus;

    Power();

    void shutdown();
    void readPowerStatus();
    virtual bool setup();
    virtual int32_t runOnce() override;
    void setStatusHandler(meshtastic::PowerStatus *handler) { statusHandler = handler; }

  protected:
    meshtastic::PowerStatus *statusHandler;

    /// Setup a xpowers chip axp192/axp2101, return true if found
    bool axpChipInit();

    /// Setup a simple ADC input based battery sensor
    bool analogInit();

  private:
    uint8_t low_voltage_counter;
#ifdef DEBUG_HEAP
    uint32_t lastheap;
#endif
};

extern Power *power;