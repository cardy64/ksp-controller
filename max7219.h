
// Pins for control:
static const int DATA_PIN = 2;
static const int LOAD_PIN = 3;
static const int CLOCK_PIN = 4;

// Number of displays:
static const int DISPLAY_COUNT = 4;

// MAX7219 registers:
static const byte MAX7219_REG_NOOP         = 0x00;
static const byte MAX7219_REG_DIGIT0       = 0x01; // Right-most digit.
static const byte MAX7219_REG_DIGIT1       = 0x02;
static const byte MAX7219_REG_DIGIT2       = 0x03;
static const byte MAX7219_REG_DIGIT3       = 0x04;
static const byte MAX7219_REG_DIGIT4       = 0x05;
static const byte MAX7219_REG_DIGIT5       = 0x06;
static const byte MAX7219_REG_DIGIT6       = 0x07;
static const byte MAX7219_REG_DIGIT7       = 0x08; // Left-most digit.
static const byte MAX7219_REG_DECODE_MODE  = 0x09;
static const byte MAX7219_REG_INTENSITY    = 0x0A;
static const byte MAX7219_REG_SCAN_LIMIT   = 0x0B;
static const byte MAX7219_REG_SHUTDOWN     = 0x0C;
static const byte MAX7219_REG_DISPLAY_TEST = 0x0F;

// BCD values (0 through 9 are themselves).
static const byte BCD_HYPHEN    = 0x0A;
static const byte BCD_E         = 0x0B;
static const byte BCD_H         = 0x0C;
static const byte BCD_L         = 0x0D;
static const byte BCD_P         = 0x0E;
static const byte BCD_BLANK     = 0x0F;

// Bit to turn on dot.
static const byte BCD_DOT       = 0x80;

static void writeByte(byte data) {
    for (int i = 0; i < 8; i++) {
        digitalWrite(CLOCK_PIN, LOW);
        digitalWrite(DATA_PIN, data & 0x80 ? HIGH : LOW);
        digitalWrite(CLOCK_PIN, HIGH);

        data <<= 1;
    }
}

static void setRegister(byte reg, byte value, int display_index = -1) {
    digitalWrite(LOAD_PIN, LOW);

    for (int i = 0; i < DISPLAY_COUNT; i++) {
        if (i == display_index || display_index == -1) {
            writeByte(reg);
            writeByte(value);
        } else {
            writeByte(MAX7219_REG_NOOP);
            writeByte(0);
        }
    }

    digitalWrite(LOAD_PIN, HIGH);
}

void setupLeds() {
    // Initialize Arduino.
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LOAD_PIN, OUTPUT);

    // Initialize the MAX7219.
    // Scan all 8 digits:
    setRegister(MAX7219_REG_SCAN_LIMIT, 0x07);

    // All digits are in BCD mode:
    setRegister(MAX7219_REG_DECODE_MODE, 0xFF);

    // Not in shutdown mode:
    setRegister(MAX7219_REG_SHUTDOWN, 0x01);

    // Not in display test (all segments on):
    setRegister(MAX7219_REG_DISPLAY_TEST, 0x00);

    // All digits blank:
    for (int i = 0; i < 8; i++) {
        setRegister(MAX7219_REG_DIGIT0 + i, BCD_BLANK);
    }

    // 0x00 to 0x0F:
    setRegister(MAX7219_REG_INTENSITY, 0x0F);
}

int dot = 0;

void loopLeds() {
    for (int i = 0; i < 8; i++) {
        setRegister(MAX7219_REG_DIGIT0 + i, (8 - i) | (dot == 7 - i ? BCD_DOT : 0));
    }
    dot = dot == 7 ? 0 : dot + 1;

    delay(100);
}
