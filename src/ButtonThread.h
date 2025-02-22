#include "PowerFSM.h"
#include "RadioLibInterface.h"
#include "buzz.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include "graphics/Screen.h"
#include "power.h"
#include <OneButton.h>

namespace concurrency
{
/**
 * Watch a GPIO and if we get an IRQ, wake the main thread.
 * Use to add wake on button press
 */
void wakeOnIrq(int irq, int mode)
{
    attachInterrupt(
        irq,
        [] {
            BaseType_t higherWake = 0;
            mainDelay.interruptFromISR(&higherWake);
        },
        FALLING);
}

class ButtonThread : public concurrency::OSThread
{
// Prepare for button presses
#ifdef BUTTON_PIN
    OneButton userButton;
#endif
#ifdef BUTTON_PIN_ALT
    OneButton userButtonAlt;
#endif
#ifdef BUTTON_PIN_TOUCH
    OneButton userButtonTouch;
#endif
    static bool shutdown_on_long_stop;

  public:
    static uint32_t longPressTime;

    // callback returns the period for the next callback invocation (or 0 if we should no longer be called)
    ButtonThread() : OSThread("Button")
    {
#ifdef BUTTON_PIN
        userButton = OneButton(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN, true, true);
#ifdef INPUT_PULLUP_SENSE
        // Some platforms (nrf52) have a SENSE variant which allows wake from sleep - override what OneButton did
        pinMode(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN, INPUT_PULLUP_SENSE);
#endif
        userButton.attachClick(userButtonPressed);
        userButton.setClickTicks(300);
        userButton.attachDuringLongPress(userButtonPressedLong);
        userButton.attachDoubleClick(userButtonDoublePressed);
        userButton.attachMultiClick(userButtonMultiPressed);
        userButton.attachLongPressStart(userButtonPressedLongStart);
        userButton.attachLongPressStop(userButtonPressedLongStop);
        wakeOnIrq(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN, FALLING);
#endif
#ifdef BUTTON_PIN_ALT
        userButtonAlt = OneButton(BUTTON_PIN_ALT, true, true);
#ifdef INPUT_PULLUP_SENSE
        // Some platforms (nrf52) have a SENSE variant which allows wake from sleep - override what OneButton did
        pinMode(BUTTON_PIN_ALT, INPUT_PULLUP_SENSE);
#endif
        userButtonAlt.attachClick(userButtonPressed);
        userButtonAlt.attachDuringLongPress(userButtonPressedLong);
        userButtonAlt.attachDoubleClick(userButtonDoublePressed);
        userButtonAlt.attachLongPressStart(userButtonPressedLongStart);
        userButtonAlt.attachLongPressStop(userButtonPressedLongStop);
        wakeOnIrq(BUTTON_PIN_ALT, FALLING);
#endif

#ifdef BUTTON_PIN_TOUCH
        userButtonTouch = OneButton(BUTTON_PIN_TOUCH, true, true);
        userButtonTouch.attachClick(touchPressed);
        wakeOnIrq(BUTTON_PIN_TOUCH, FALLING);
#endif
    }

  protected:
    /// If the button is pressed we suppress CPU sleep until release
    int32_t runOnce() override
    {
        canSleep = true; // Assume we should not keep the board awake

#ifdef BUTTON_PIN
        userButton.tick();
        canSleep &= userButton.isIdle();
#endif
#ifdef BUTTON_PIN_ALT
        userButtonAlt.tick();
        canSleep &= userButtonAlt.isIdle();
#endif
#ifdef BUTTON_PIN_TOUCH
        userButtonTouch.tick();
        canSleep &= userButtonTouch.isIdle();
#endif
        // if (!canSleep) LOG_DEBUG("Supressing sleep!\n");
        // else LOG_DEBUG("sleep ok\n");

        return 5;
    }

  private:
    static void touchPressed()
    {
        screen->forceDisplay();
        LOG_DEBUG("touch press!\n");
    }

    static void userButtonPressed()
    {
        // LOG_DEBUG("press!\n");
#ifdef BUTTON_PIN
        if (((config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN) !=
             moduleConfig.canned_message.inputbroker_pin_press) ||
            !moduleConfig.canned_message.enabled) {
            powerFSM.trigger(EVENT_PRESS);
        }
#endif
    }
    static void userButtonPressedLong()
    {
        static constexpr int thirty_seconds{30000};
        static constexpr int three_seconds{3000};
        static constexpr int millis_inaccuracy{50};

        if (longPressTime <= 0) {
            return;
        }
        const int elapsed = millis() - longPressTime;

        // If user button is held down for 3 seconds, send SOS signal to mesh
        // and continue sending each 3 seconds while the button is held down.
        if ((elapsed > three_seconds) and
            (elapsed % three_seconds) < millis_inaccuracy) {
            sendSosToMesh();
        }

        // If user button is held down for 30 seconds, shutdown the device.
        if (elapsed > thirty_seconds) {
#if defined(ARCH_NRF52) || defined(ARCH_ESP32)
            // Do actual shutdown when button released, otherwise the button release
            // may wake the board immediatedly.
            if ((!shutdown_on_long_stop) && (millis() > 30 * 1000)) {
                screen->startShutdownScreen();
                LOG_INFO("Shutdown from long press");
                playBeep();
#ifdef PIN_LED1
                ledOff(PIN_LED1);
#endif
#ifdef PIN_LED2
                ledOff(PIN_LED2);
#endif
#ifdef PIN_LED3
                ledOff(PIN_LED3);
#endif
                shutdown_on_long_stop = true;
            }
#endif
        }
    }

    static void userButtonDoublePressed()
    {
#if defined(USE_EINK) && defined(PIN_EINK_EN)
        digitalWrite(PIN_EINK_EN, digitalRead(PIN_EINK_EN) == LOW);
#endif
        screen->print("Sent ad-hoc ping\n");
        service.refreshMyNodeInfo();
        service.sendNetworkPing(NODENUM_BROADCAST, true);
    }

    static void userButtonMultiPressed()
    {
#if defined(GPS_POWER_TOGGLE)
        if (config.position.gps_enabled) {
            LOG_DEBUG("Flag set to false for gps power\n");
        } else {
            LOG_DEBUG("Flag set to true to restore power\n");
        }
        config.position.gps_enabled = !(config.position.gps_enabled);
        doGPSpowersave(config.position.gps_enabled);
#endif
        sendSosToMesh();
    }

    static void userButtonPressedLongStart()
    {
        if (millis() > 30 * 1000) {
            LOG_DEBUG("Long press start!\n");
            longPressTime = millis();
        }
    }

    static void userButtonPressedLongStop()
    {
        if (millis() > 30 * 1000) {
            LOG_DEBUG("Long press stop!\n");
            longPressTime = 0;
            if (shutdown_on_long_stop) {
                playShutdownMelody();
                delay(3000);
                power->shutdown();
            }
        }
    }

    static void sendSosToMesh() {
        static const std::string device_name{
            (owner.short_name[0] != '\0') ? owner.short_name : "undefined"};
        static const std::string message{/*device_name + */"SOS"};

        LOG_DEBUG("Triple or long button click --> SOS sending to mesh requested.\n");

        screen->blink();

        meshtastic_MeshPacket *p = router->allocForSending();

        p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        p->channel = SOS_SIGNAL_MESHTASTIC_CHANNEL_NUM;

        p->decoded.payload.size = message.size();
        memcpy(p->decoded.payload.bytes, message.c_str(),
               p->decoded.payload.size);

        service.sendToMesh(p);
    }
};

} // namespace concurrency
