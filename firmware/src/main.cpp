#define SERIAL_RX_BUFFER_SIZE 256

#include <Arduino.h>

static const float DAC_VREF = 3.3f;
static const uint16_t DAC_MAX = 4095;
static const float MAX_AMPLITUDE = 1.65f;
static const float MAX_DC = 3.3f;
static const float MAX_FREQ = 5000.0f;
static const int MIN_SR = 100;
static const int MAX_SR = 100000;
static const int CMD_BUF_SIZE = 64;
static const float ABS_GAIN_MAX = 20.0f;
static const uint8_t NUM_DACS = 2;

// TCRT5000 (аналоговый вход, см. tcrt5000_test.ino в корне репозитория)
static const uint8_t IR_PIN = A0;            // ADC0/PA16 — не пересекается с DAC0/DAC1 (PB15/PB16)
static const uint8_t IR_AVG_N = 8;           // окно усреднения
static const uint16_t IR_ADC_MAX = 4095;
static const float IR_VREF = 3.3f;
static const uint16_t IR_TH_DEFAULT = 1500;  // raw < порога => есть отражение
static const uint16_t IR_HY_DEFAULT = 200;   // гистерезис, чтобы состояние не дребезжало

volatile float g_amp1 = 0.5f;
volatile float g_phase1 = 0.0f;
volatile float g_amp2 = 0.5f;
volatile float g_phase2 = 0.0f;
volatile float g_dc = 2.0f;
volatile float g_gainMax = 4.0f;

volatile float g_freq1[NUM_DACS] = {100.0f, 100.0f};
volatile float g_freq2[NUM_DACS] = {200.0f, 200.0f};
volatile float g_pacc1[NUM_DACS] = {0.0f, 0.0f};
volatile float g_pacc2[NUM_DACS] = {0.0f, 0.0f};
volatile float g_paccEnv[NUM_DACS] = {(float)HALF_PI, (float)HALF_PI};
volatile float g_inc1[NUM_DACS] = {0.0f, 0.0f};
volatile float g_inc2[NUM_DACS] = {0.0f, 0.0f};
volatile float g_incEnv[NUM_DACS] = {0.0f, 0.0f};

volatile int g_sampleRate = 10000;
volatile bool g_running = false;

volatile bool g_dbgEnabled = false;
volatile float g_dbgLastV[NUM_DACS] = {0.0f, 0.0f};
volatile float g_dbgEnvelope[NUM_DACS] = {0.0f, 0.0f};
volatile float g_dbgPeakV[NUM_DACS] = {0.0f, 0.0f};
volatile float g_dbgTroughV[NUM_DACS] = {DAC_VREF, DAC_VREF};
volatile uint32_t g_dbgClipHigh[NUM_DACS] = {0, 0};
volatile uint32_t g_dbgClipLow[NUM_DACS] = {0, 0};
volatile uint32_t g_dbgSamples = 0;
volatile uint32_t g_isrUs = 0;

volatile bool g_irEnabled = false;
volatile uint16_t g_irRaw = 0;
volatile uint16_t g_irMin = IR_ADC_MAX;
volatile uint16_t g_irMax = 0;
volatile bool g_irReflect = false;
volatile uint16_t g_irTh = IR_TH_DEFAULT;
volatile uint16_t g_irHyst = IR_HY_DEFAULT;

uint16_t g_irBuf[IR_AVG_N] = {0};
uint32_t g_irSum = 0;
uint8_t g_irIdx = 0;
uint8_t g_irFilled = 0;

float clampAmplitude(float a) { return constrain(a, 0.0f, MAX_AMPLITUDE); }
float clampDC(float dc) { return constrain(dc, 0.0f, MAX_DC); }
float clampFreq(float f) { return constrain(f, 0.0f, MAX_FREQ); }
float degToRad(float d) { return d * (float)PI / 180.0f; }

static inline void wrapPhase(volatile float &phase) {
    if (phase > TWO_PI) phase -= TWO_PI;
    if (phase < -TWO_PI) phase += TWO_PI;
}

static inline uint16_t voltageToDac(float v) {
    uint16_t dac = (uint16_t)((v / DAC_VREF) * (float)DAC_MAX + 0.5f);
    if (dac > DAC_MAX) dac = DAC_MAX;
    return dac;
}

static inline uint32_t packTaggedPair(uint16_t dac0, uint16_t dac1) {
    uint32_t word0 = (uint32_t)(dac0 & 0x0FFFu);
    uint32_t word1 = ((uint32_t)(dac1 & 0x0FFFu)) << 16;
    word1 |= (1u << 28);
    return word0 | word1;
}

void writeDacPair(uint16_t dac0, uint16_t dac1) {
    while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
    DACC->DACC_CDR = packTaggedPair(dac0, dac1);
}

void resetDebugStats() {
    for (uint8_t ch = 0; ch < NUM_DACS; ++ch) {
        g_dbgLastV[ch] = 0.0f;
        g_dbgEnvelope[ch] = 0.0f;
        g_dbgPeakV[ch] = 0.0f;
        g_dbgTroughV[ch] = DAC_VREF;
        g_dbgClipHigh[ch] = 0;
        g_dbgClipLow[ch] = 0;
    }
    g_dbgSamples = 0;
    g_isrUs = 0;
}

void recomputeIncrements(uint8_t channel) {
    float sr = (float)g_sampleRate;
    g_inc1[channel] = TWO_PI * g_freq1[channel] / sr;
    g_inc2[channel] = TWO_PI * g_freq2[channel] / sr;
    g_incEnv[channel] = (float)PI * (g_freq2[channel] - g_freq1[channel]) / sr;
    g_pacc1[channel] = g_phase1;
    g_pacc2[channel] = g_phase2;
    g_paccEnv[channel] = (float)HALF_PI;
}

void recomputeAllIncrements() {
    for (uint8_t ch = 0; ch < NUM_DACS; ++ch) {
        recomputeIncrements(ch);
    }
}

void TC6_Handler(void) {
    TC_GetStatus(TC2, 0);

    if (!g_running) {
        writeDacPair(0, 0);
        return;
    }

    uint32_t t0 = micros();
    uint16_t dacValues[NUM_DACS] = {0, 0};

    for (uint8_t ch = 0; ch < NUM_DACS; ++ch) {
        g_pacc1[ch] += g_inc1[ch];
        g_pacc2[ch] += g_inc2[ch];
        g_paccEnv[ch] += g_incEnv[ch];

        wrapPhase(g_pacc1[ch]);
        wrapPhase(g_pacc2[ch]);
        wrapPhase(g_paccEnv[ch]);

        float denom = sinf(g_paccEnv[ch]);
        float envelope;
        if (fabsf(denom) < 1e-6f) {
            envelope = g_gainMax;
        } else {
            envelope = fabsf(1.0f / denom);
            if (envelope > g_gainMax) envelope = g_gainMax;
        }

        float sines = g_amp1 * sinf(g_pacc1[ch]) + g_amp2 * sinf(g_pacc2[ch]);
        float v = g_dc + envelope * sines;

        if (v < 0.0f) {
            v = 0.0f;
            g_dbgClipLow[ch]++;
        }
        if (v > DAC_VREF) {
            v = DAC_VREF;
            g_dbgClipHigh[ch]++;
        }

        dacValues[ch] = voltageToDac(v);
        g_dbgLastV[ch] = v;
        g_dbgEnvelope[ch] = envelope;
        if (v > g_dbgPeakV[ch]) g_dbgPeakV[ch] = v;
        if (v < g_dbgTroughV[ch]) g_dbgTroughV[ch] = v;
    }

    writeDacPair(dacValues[0], dacValues[1]);
    g_dbgSamples++;
    g_isrUs = micros() - t0;
}

void setupTimer(int sampleRate) {
    sampleRate = constrain(sampleRate, MIN_SR, MAX_SR);
    g_sampleRate = sampleRate;

    pmc_set_writeprotect(false);
    pmc_enable_periph_clk(TC6_IRQn);

    TC_Configure(
        TC2,
        0,
        TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1
    );

    uint32_t rc = 42000000UL / (uint32_t)sampleRate;
    TC_SetRC(TC2, 0, rc);

    TC2->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

    NVIC_EnableIRQ(TC6_IRQn);
    TC_Start(TC2, 0);
}

void setupDACC() {
    pmc_enable_periph_clk(ID_DACC);
    DACC->DACC_CR = DACC_CR_SWRST;
    DACC->DACC_MR = DACC_MR_TRGEN_DIS
                  | DACC_MR_WORD_WORD
                  | DACC_MR_TAG_EN
                  | DACC_MR_REFRESH(1)
                  | DACC_MR_STARTUP_8
                  | DACC_MR_MAXS;
    dacc_enable_channel(DACC, 0);
    dacc_enable_channel(DACC, 1);
    writeDacPair(0, 0);
}

static uint32_t g_lastDbgMs = 0;

void printDebug() {
    if (!g_dbgEnabled) return;
    uint32_t now = millis();
    if (now - g_lastDbgMs < 100) return;
    g_lastDbgMs = now;

    float lastV0 = g_dbgLastV[0];
    float lastV1 = g_dbgLastV[1];
    float env0 = g_dbgEnvelope[0];
    float env1 = g_dbgEnvelope[1];
    float peak0 = g_dbgPeakV[0];
    float peak1 = g_dbgPeakV[1];
    float trough0 = g_dbgTroughV[0];
    float trough1 = g_dbgTroughV[1];
    uint32_t clipHi0 = g_dbgClipHigh[0];
    uint32_t clipHi1 = g_dbgClipHigh[1];
    uint32_t clipLo0 = g_dbgClipLow[0];
    uint32_t clipLo1 = g_dbgClipLow[1];
    uint32_t samps = g_dbgSamples;
    uint32_t isrUs = g_isrUs;

    g_dbgPeakV[0] = 0.0f;
    g_dbgPeakV[1] = 0.0f;
    g_dbgTroughV[0] = DAC_VREF;
    g_dbgTroughV[1] = DAC_VREF;
    g_dbgClipHigh[0] = 0;
    g_dbgClipHigh[1] = 0;
    g_dbgClipLow[0] = 0;
    g_dbgClipLow[1] = 0;

    Serial.print("{\"dbg\":1");
    Serial.print(",\"v0\":"); Serial.print(lastV0, 4);
    Serial.print(",\"v1\":"); Serial.print(lastV1, 4);
    Serial.print(",\"env0\":"); Serial.print(env0, 4);
    Serial.print(",\"env1\":"); Serial.print(env1, 4);
    Serial.print(",\"peak0\":"); Serial.print(peak0, 4);
    Serial.print(",\"peak1\":"); Serial.print(peak1, 4);
    Serial.print(",\"trough0\":"); Serial.print(trough0, 4);
    Serial.print(",\"trough1\":"); Serial.print(trough1, 4);
    Serial.print(",\"clipHi0\":"); Serial.print(clipHi0);
    Serial.print(",\"clipHi1\":"); Serial.print(clipHi1);
    Serial.print(",\"clipLo0\":"); Serial.print(clipLo0);
    Serial.print(",\"clipLo1\":"); Serial.print(clipLo1);
    Serial.print(",\"samps\":"); Serial.print(samps);
    Serial.print(",\"isrUs\":"); Serial.print(isrUs);
    Serial.print(",\"run\":"); Serial.print(g_running ? 1 : 0);
    Serial.println("}");
}

static uint32_t g_lastIrMs = 0;

void pollIR() {
    uint16_t s = (uint16_t)analogRead(IR_PIN);

    g_irSum -= g_irBuf[g_irIdx];
    g_irBuf[g_irIdx] = s;
    g_irSum += s;
    g_irIdx = (uint8_t)((g_irIdx + 1) % IR_AVG_N);
    if (g_irFilled < IR_AVG_N) g_irFilled++;

    uint16_t avg = (uint16_t)(g_irSum / g_irFilled);
    g_irRaw = avg;

    if (s < g_irMin) g_irMin = s;
    if (s > g_irMax) g_irMax = s;

    if (!g_irReflect && avg < g_irTh) {
        g_irReflect = true;
    } else if (g_irReflect && avg > (uint16_t)(g_irTh + g_irHyst)) {
        g_irReflect = false;
    }
}

void emitIR() {
    uint16_t raw = g_irRaw;
    uint16_t mn = g_irMin;
    uint16_t mx = g_irMax;
    bool refl = g_irReflect;

    g_irMin = IR_ADC_MAX;
    g_irMax = 0;

    float volts = raw * IR_VREF / (float)IR_ADC_MAX;

    Serial.print("{\"ir\":1");
    Serial.print(",\"raw\":"); Serial.print(raw);
    Serial.print(",\"v\":"); Serial.print(volts, 3);
    Serial.print(",\"min\":"); Serial.print(mn);
    Serial.print(",\"max\":"); Serial.print(mx);
    Serial.print(",\"ref\":"); Serial.print(refl ? 1 : 0);
    Serial.print(",\"th\":"); Serial.print(g_irTh);
    Serial.println("}");
}

void printIR() {
    if (!g_irEnabled) return;
    uint32_t now = millis();
    if (now - g_lastIrMs < 100) return;
    g_lastIrMs = now;
    emitIR();
}

char g_cmdBuf[CMD_BUF_SIZE];
int g_cmdIdx = 0;

void setChannelFrequency(uint8_t channel, uint8_t slot, float value) {
    bool wasRunning = g_running;
    g_running = false;
    if (slot == 0) {
        g_freq1[channel] = clampFreq(value);
    } else {
        g_freq2[channel] = clampFreq(value);
    }
    recomputeIncrements(channel);
    g_running = wasRunning;
}

void processCommand(const char *cmd) {
    char key[8];
    float val;

    if (strncmp(cmd, "START", 5) == 0) {
        g_running = false;
        resetDebugStats();
        recomputeAllIncrements();
        g_running = true;
        Serial.println("OK START");
        return;
    }

    if (strncmp(cmd, "STOP", 4) == 0) {
        g_running = false;
        writeDacPair(0, 0);
        Serial.println("OK STOP");
        return;
    }

    if (strncmp(cmd, "STATUS", 6) == 0) {
        Serial.print("{\"status\":1");
        Serial.print(",\"a1\":"); Serial.print(g_amp1, 4);
        Serial.print(",\"p1\":"); Serial.print(g_phase1 * 180.0f / PI, 4);
        Serial.print(",\"a2\":"); Serial.print(g_amp2, 4);
        Serial.print(",\"p2\":"); Serial.print(g_phase2 * 180.0f / PI, 4);
        Serial.print(",\"dac0_f1\":"); Serial.print(g_freq1[0], 4);
        Serial.print(",\"dac0_f2\":"); Serial.print(g_freq2[0], 4);
        Serial.print(",\"dac1_f1\":"); Serial.print(g_freq1[1], 4);
        Serial.print(",\"dac1_f2\":"); Serial.print(g_freq2[1], 4);
        Serial.print(",\"dc\":"); Serial.print(g_dc, 4);
        Serial.print(",\"gm\":"); Serial.print(g_gainMax, 4);
        Serial.print(",\"sr\":"); Serial.print(g_sampleRate);
        Serial.print(",\"run\":"); Serial.print(g_running ? 1 : 0);
        Serial.println("}");
        return;
    }

    if (strncmp(cmd, "DBG ", 4) == 0) {
        const char *arg = cmd + 4;
        if (strncmp(arg, "ON", 2) == 0) {
            g_dbgEnabled = true;
            Serial.println("OK DBG ON");
        } else if (strncmp(arg, "OFF", 3) == 0) {
            g_dbgEnabled = false;
            Serial.println("OK DBG OFF");
        } else {
            Serial.println("ERR DBG ON|OFF");
        }
        return;
    }

    if (strncmp(cmd, "IR?", 3) == 0) {
        emitIR();
        return;
    }

    if (strncmp(cmd, "IR ", 3) == 0) {
        const char *arg = cmd + 3;
        if (strncmp(arg, "ON", 2) == 0) {
            g_irEnabled = true;
            Serial.println("OK IR ON");
        } else if (strncmp(arg, "OFF", 3) == 0) {
            g_irEnabled = false;
            Serial.println("OK IR OFF");
        } else {
            Serial.println("ERR IR ON|OFF");
        }
        return;
    }

    if (sscanf(cmd, "SET %7s %f", key, &val) == 2) {
        if (strcmp(key, "A1") == 0) {
            g_amp1 = clampAmplitude(val);
            Serial.print("OK A1="); Serial.println(g_amp1, 4);
        } else if (strcmp(key, "A2") == 0) {
            g_amp2 = clampAmplitude(val);
            Serial.print("OK A2="); Serial.println(g_amp2, 4);
        } else if (strcmp(key, "P1") == 0) {
            bool wasRunning = g_running;
            g_running = false;
            g_phase1 = degToRad(val);
            recomputeAllIncrements();
            g_running = wasRunning;
            Serial.print("OK P1="); Serial.println(val, 4);
        } else if (strcmp(key, "P2") == 0) {
            bool wasRunning = g_running;
            g_running = false;
            g_phase2 = degToRad(val);
            recomputeAllIncrements();
            g_running = wasRunning;
            Serial.print("OK P2="); Serial.println(val, 4);
        } else if (strcmp(key, "DC") == 0) {
            g_dc = clampDC(val);
            Serial.print("OK DC="); Serial.println(g_dc, 4);
        } else if (strcmp(key, "GM") == 0) {
            if (val < 1.0f) val = 1.0f;
            if (val > ABS_GAIN_MAX) val = ABS_GAIN_MAX;
            g_gainMax = val;
            Serial.print("OK GM="); Serial.println(g_gainMax, 4);
        } else if (strcmp(key, "IRTH") == 0) {
            g_irTh = (uint16_t)constrain(val, 0.0f, (float)IR_ADC_MAX);
            Serial.print("OK IRTH="); Serial.println(g_irTh);
        } else if (strcmp(key, "IRHY") == 0) {
            g_irHyst = (uint16_t)constrain(val, 0.0f, 1000.0f);
            Serial.print("OK IRHY="); Serial.println(g_irHyst);
        } else if (strcmp(key, "SR") == 0) {
            bool wasRunning = g_running;
            g_running = false;
            delay(2);
            setupTimer((int)val);
            recomputeAllIncrements();
            g_running = wasRunning;
            Serial.print("OK SR="); Serial.println(g_sampleRate);
        } else if (strcmp(key, "F1") == 0 || strcmp(key, "D0F1") == 0) {
            setChannelFrequency(0, 0, val);
            Serial.print("OK D0F1="); Serial.println(g_freq1[0], 4);
        } else if (strcmp(key, "F2") == 0 || strcmp(key, "D0F2") == 0) {
            setChannelFrequency(0, 1, val);
            Serial.print("OK D0F2="); Serial.println(g_freq2[0], 4);
        } else if (strcmp(key, "D1F1") == 0) {
            setChannelFrequency(1, 0, val);
            Serial.print("OK D1F1="); Serial.println(g_freq1[1], 4);
        } else if (strcmp(key, "D1F2") == 0) {
            setChannelFrequency(1, 1, val);
            Serial.print("OK D1F2="); Serial.println(g_freq2[1], 4);
        } else {
            Serial.print("ERR unknown key: "); Serial.println(key);
        }
        return;
    }

    Serial.print("ERR bad command: "); Serial.println(cmd);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    setupDACC();
    resetDebugStats();
    recomputeAllIncrements();
    setupTimer(g_sampleRate);

    analogReadResolution(12);
    pinMode(IR_PIN, INPUT);

    NVIC_SetPriority(UART_IRQn, 0);
    NVIC_SetPriority(TC6_IRQn, 1);

    Serial.println("READY dual_dac_sine v4.1 (ir tcrt5000)");
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_cmdIdx > 0) {
                g_cmdBuf[g_cmdIdx] = '\0';
                processCommand(g_cmdBuf);
                g_cmdIdx = 0;
            }
        } else if (g_cmdIdx < CMD_BUF_SIZE - 1) {
            g_cmdBuf[g_cmdIdx++] = c;
        }
    }

    printDebug();
    pollIR();
    printIR();
}
