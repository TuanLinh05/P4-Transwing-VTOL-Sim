/*
 * P4 iBUS -> USB CDC bridge, Arduino MKR WiFi 1010 (RC-only)
 *
 * Wiring:
 *   FS-iA6B iBUS-SERVO signal -> MKR D13 (Serial1 RX)
 *   FS-iA6B GND               -> MKR GND
 *   MKR USB                    -> Windows PC
 *
 * The SENS port, D0 and D1 are intentionally unused. This sketch only
 * forwards the ten FlySky channels to the PC. It never sends MAVLink and
 * never changes flight modes; the P4 Control Station owns that authority.
 *
 * USB protocol (100 Hz):
 *   P4RC,1,<seq>,<millis>,<ch1>..<ch10>,<flags>,<CRC16HEX>
 *
 * flags bit 0: fresh valid iBUS frame
 * flags bit 1: iBUS timeout; safe channel values are substituted
 */

#include <Arduino.h>

namespace {

constexpr uint32_t IBUS_BAUD = 115200;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint8_t IBUS_FRAME_SIZE = 32;
constexpr uint8_t IBUS_CHANNEL_COUNT = 14;
constexpr uint8_t EXPORTED_CHANNEL_COUNT = 10;
constexpr uint32_t USB_SEND_PERIOD_MS = 10;  // 100 Hz
constexpr uint32_t IBUS_TIMEOUT_MS = 150;

constexpr uint16_t RC_SAFE_CENTER = 1500;
constexpr uint16_t RC_SAFE_THROTTLE = 1000;
constexpr uint16_t RC_MIN_VALID = 750;
constexpr uint16_t RC_MAX_VALID = 2250;

constexpr uint8_t FLAG_IBUS_VALID = 0x01;
constexpr uint8_t FLAG_IBUS_TIMEOUT = 0x02;

struct RcState {
    uint16_t channel[IBUS_CHANNEL_COUNT];
    bool valid;
    uint32_t lastValidFrameMs;
    uint32_t sequence;
};

RcState rc;
uint8_t ibusBuffer[IBUS_FRAME_SIZE];
uint8_t ibusIndex = 0;
uint8_t ibusExpectedLength = 0;
uint32_t lastUsbSendMs = 0;

void setSafeRcState() {
    for (uint8_t i = 0; i < IBUS_CHANNEL_COUNT; ++i) {
        rc.channel[i] = RC_SAFE_CENTER;
    }
    rc.channel[2] = RC_SAFE_THROTTLE;
    rc.valid = false;
    rc.lastValidFrameMs = 0;
    rc.sequence = 0;
}

uint16_t crc16CcittFalse(const char *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(static_cast<uint8_t>(data[i])) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000)
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool ibusFrameIsValid(const uint8_t *frame) {
    if (frame[0] != 0x20 || frame[1] != 0x40) {
        return false;
    }
    uint16_t checksum = 0xFFFF;
    for (uint8_t i = 0; i < 30; ++i) {
        checksum = static_cast<uint16_t>(checksum - frame[i]);
    }
    const uint16_t received = static_cast<uint16_t>(frame[30])
        | (static_cast<uint16_t>(frame[31]) << 8);
    return checksum == received;
}

void decodeIbusFrame(const uint8_t *frame) {
    uint16_t decoded[EXPORTED_CHANNEL_COUNT];
    for (uint8_t i = 0; i < EXPORTED_CHANNEL_COUNT; ++i) {
        const uint8_t offset = 2 + (2 * i);
        decoded[i] = static_cast<uint16_t>(frame[offset])
            | (static_cast<uint16_t>(frame[offset + 1]) << 8);
        // CH11-CH14 may be empty on the 10-channel transmitter, so only
        // validate the ten channels which the Control Station forwards.
        if (decoded[i] < RC_MIN_VALID || decoded[i] > RC_MAX_VALID) {
            return;
        }
    }
    for (uint8_t i = 0; i < EXPORTED_CHANNEL_COUNT; ++i) {
        rc.channel[i] = decoded[i];
    }
    rc.valid = true;
    rc.lastValidFrameMs = millis();
    ++rc.sequence;
}

void consumeIbusByte(uint8_t byteIn) {
    if (ibusIndex == 0) {
        if (byteIn == 0x20) {
            ibusExpectedLength = IBUS_FRAME_SIZE;
            ibusBuffer[ibusIndex++] = byteIn;
        }
        return;
    }
    if (ibusIndex == 1 && byteIn != 0x40) {
        ibusIndex = byteIn == 0x20 ? 1 : 0;
        if (ibusIndex == 1) {
            ibusBuffer[0] = byteIn;
            ibusExpectedLength = IBUS_FRAME_SIZE;
        }
        return;
    }
    ibusBuffer[ibusIndex++] = byteIn;
    if (ibusIndex < ibusExpectedLength) {
        return;
    }
    if (ibusFrameIsValid(ibusBuffer)) {
        decodeIbusFrame(ibusBuffer);
    }
    ibusIndex = 0;
    ibusExpectedLength = 0;
}

void pollIbus() {
    while (Serial1.available() > 0) {
        consumeIbusByte(static_cast<uint8_t>(Serial1.read()));
    }
}

bool ibusIsFresh(uint32_t nowMs) {
    return rc.valid && (nowMs - rc.lastValidFrameMs <= IBUS_TIMEOUT_MS);
}

void printHex16(uint16_t value) {
    const char hex[] = "0123456789ABCDEF";
    Serial.print(hex[(value >> 12) & 0x0F]);
    Serial.print(hex[(value >> 8) & 0x0F]);
    Serial.print(hex[(value >> 4) & 0x0F]);
    Serial.print(hex[value & 0x0F]);
}

void sendUsbFrame(uint32_t nowMs) {
    const bool fresh = ibusIsFresh(nowMs);
    const uint16_t ch1 = fresh ? rc.channel[0] : RC_SAFE_CENTER;
    const uint16_t ch2 = fresh ? rc.channel[1] : RC_SAFE_CENTER;
    const uint16_t ch3 = fresh ? rc.channel[2] : RC_SAFE_THROTTLE;
    const uint16_t ch4 = fresh ? rc.channel[3] : RC_SAFE_CENTER;
    const uint16_t ch5 = fresh ? rc.channel[4] : RC_SAFE_CENTER;
    const uint16_t ch6 = fresh ? rc.channel[5] : RC_SAFE_CENTER;
    const uint16_t ch7 = fresh ? rc.channel[6] : RC_SAFE_CENTER;
    const uint16_t ch8 = fresh ? rc.channel[7] : RC_SAFE_CENTER;
    const uint16_t ch9 = fresh ? rc.channel[8] : RC_SAFE_CENTER;
    const uint16_t ch10 = fresh ? rc.channel[9] : RC_SAFE_CENTER;
    const uint8_t flags = fresh ? FLAG_IBUS_VALID : FLAG_IBUS_TIMEOUT;

    char payload[160];
    const int count = snprintf(
        payload, sizeof(payload),
        "P4RC,1,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        static_cast<unsigned long>(rc.sequence),
        static_cast<unsigned long>(nowMs),
        ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10,
        static_cast<unsigned>(flags));
    if (count <= 0 || count >= static_cast<int>(sizeof(payload))) {
        return;
    }
    Serial.print(payload);
    Serial.print(',');
    printHex16(crc16CcittFalse(payload, static_cast<size_t>(count)));
    Serial.print('\n');
}

}  // namespace

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    setSafeRcState();
    Serial.begin(USB_BAUD);
    Serial1.begin(IBUS_BAUD);  // D13 = iBUS-SERVO receive
}

void loop() {
    pollIbus();
    const uint32_t nowMs = millis();
    digitalWrite(LED_BUILTIN, ibusIsFresh(nowMs) ? HIGH : LOW);
    if (nowMs - lastUsbSendMs >= USB_SEND_PERIOD_MS) {
        lastUsbSendMs = nowMs;
        sendUsbFrame(nowMs);
    }
}
