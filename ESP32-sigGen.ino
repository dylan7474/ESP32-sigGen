// ESP32 SPI Signal Generator with AM, MCW, and CW Morse (Sol edition)
// Arduino-ESP32 core 2.x

#include <Arduino.h>
#include <math.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"

// -------- Pins --------
static const int PIN_SCK       = 18;
static const int PIN_MOSI      = 23;
static const int PIN_HEARTBEAT = 2;

// -------- SPI presets --------
static const uint32_t F_LIST[] = {
  80000000u, 60000000u, 48000000u, 40000000u, 32000000u, 26000000u, 20000000u
};
static const size_t F_COUNT = sizeof(F_LIST)/sizeof(F_LIST[0]);

// -------- DMA / buffering --------
static const size_t BUF_BYTES   = 2048; // smaller -> faster updates (better high tone)
static const int    QUEUE_DEPTH = 8;
static const int    DMA_CH      = 1;

// Stripe granularity inside each DMA buffer (modulation resolution)
static const int STRIPE = 16; // 8..64; smaller -> smoother envelope, more CPU

spi_device_handle_t spi = nullptr;
uint8_t *bufs[QUEUE_DEPTH];
spi_transaction_t trans[QUEUE_DEPTH];

volatile uint32_t tx_done_count = 0;
volatile uint32_t last_done_us  = 0;
volatile uint32_t max_gap_us    = 0;

uint32_t bytes_pushed_this_sec = 0;
uint32_t trans_pushed_this_sec = 0;

uint32_t current_spi_hz = F_LIST[0];
int      cpu_mhz_cache  = 240;

// -------- Modulation modes --------
enum ModMode { MODE_AM = 0, MODE_MCW = 1, MODE_CW = 2 };
static ModMode MOD_MODE = MODE_AM;

// -------- AM/MCW parameters --------
static float  TONE_HZ   = 1000.0f; // tone for AM & MCW
static float  MOD_INDEX = 0.8f;    // depth 0..1
static float  BASELINE  = 0.2f;    // floor amplitude when "off" (set 0 for true off)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Tone phase (cycles); we step *within* each buffer per stripe
static double tone_phase = 0.0; // cycles
// Precomputed increments per time quantum (filled at rebuild)
static double cycles_per_buffer = 0.0;
static double cycles_per_stripe = 0.0;

// -------- Morse (CW/MCW) --------
static String MORSE_MSG = "CQ CQ DX ";
static int    MORSE_WPM = 20;

// Timing derived from WPM:
// unit = 1200 / WPM (ms). dit=1, dah=3, elem_gap=1, char_gap=3, word_gap=7 units.
static double morse_unit_us = 60000.0; // recomputed in recalc_morse()

// A compact Morse table (dit='.', dah='-')
struct MorseMap { char c; const char* pat; };
static const MorseMap MORSE_TABLE[] = {
  {'a',".-"},   {'b',"-..."}, {'c',"-.-."}, {'d',"-.."},  {'e',"."},
  {'f',"..-."}, {'g',"--."},  {'h',"...."}, {'i',".."},   {'j',".---"},
  {'k',"-.-"},  {'l',".-.."}, {'m',"--"},   {'n',"-."},   {'o',"---"},
  {'p',".--."}, {'q',"--.-"}, {'r',".-."},  {'s',"..."},  {'t',"-"},
  {'u',"..-"},  {'v',"...-"}, {'w',".--"},  {'x',"-..-"}, {'y',"-.--"},
  {'z',"--.."},
  {'0',"-----"},{'1',".----"},{'2',"..---"},{'3',"...--"},{'4',"....-"},
  {'5',"....."},{'6',"-...."},{'7',"--..."},{'8',"---.."},{'9',"----."},
  {'?', "..--.."},{'/', "-..-."},{'.', ".-.-.-"},{',', "--..--"},
  {'=', "-...-"},{'+', ".-.-."},{'-', "-....-"}
};
static const size_t MORSE_TABLE_N = sizeof(MORSE_TABLE)/sizeof(MORSE_TABLE[0]);

// Morse state machine
enum KeyState { KEY_OFF=0, KEY_ON=1 };

static size_t morse_msg_index = 0;  // index into MORSE_MSG
static size_t morse_pat_index = 0;  // index into current char pattern
static KeyState morse_key = KEY_OFF;
static double morse_time_left_us = 0.0;
static String current_pat = "";
static bool between_elements = false; // tracking 1-unit intra-character gaps

// Utilities
inline size_t sz_min(size_t a, size_t b) { return a < b ? a : b; }

const char* lookup_morse(char ch) {
  if (ch == ' ') return nullptr; // word gap handled specially
  ch = tolower((unsigned char)ch);
  for (size_t i=0;i<MORSE_TABLE_N;i++)
    if (MORSE_TABLE[i].c == ch) return MORSE_TABLE[i].pat;
  return nullptr;
}

void recalc_rates() {
  const double bits_per_buf   = (double)BUF_BYTES * 8.0;
  const double us_per_buf     = 1e6 * bits_per_buf / (double)current_spi_hz;
  cycles_per_buffer           = TONE_HZ * (us_per_buf / 1e6);
  const double bits_per_stripe = (double)STRIPE * 8.0;
  const double us_per_stripe   = 1e6 * bits_per_stripe / (double)current_spi_hz;
  cycles_per_stripe           = TONE_HZ * (us_per_stripe / 1e6);
}

void recalc_morse() {
  morse_unit_us = (1200.0 / (double)MORSE_WPM) * 1000.0; // ms -> us
}

// Advance Morse state by "slice_us" microseconds; return instantaneous key (on/off)
KeyState morse_advance(double slice_us) {
  // If no current timing, (re)load next element
  while (morse_time_left_us <= 0.0) {
    // Need a new element/gap
    if (current_pat.length() == 0) {
      // Move to next message character
      if (morse_msg_index >= MORSE_MSG.length()) {
        morse_msg_index = 0; // loop
      }
      char ch = MORSE_MSG.charAt(morse_msg_index++);
      if (ch == ' ') {
        morse_key = KEY_OFF;
        morse_time_left_us = 7.0 * morse_unit_us; // word gap
        return morse_key;
      }
      const char* pat = lookup_morse(ch);
      if (!pat) {
        // Unknown char -> treat as inter-word gap
        morse_key = KEY_OFF;
        morse_time_left_us = 7.0 * morse_unit_us;
        return morse_key;
      }
      current_pat = String(pat);
      morse_pat_index = 0;
      between_elements = false;
    }

    // Between elements inside a character?
    if (between_elements) {
      morse_key = KEY_OFF;
      morse_time_left_us = 1.0 * morse_unit_us; // intra-character gap
      between_elements = false;
      return morse_key;
    }

    // Start next element (dit/dah) or finish char
    if (morse_pat_index < current_pat.length()) {
      char e = current_pat.charAt(morse_pat_index++);
      morse_key = KEY_ON;
      morse_time_left_us = (e == '-') ? (3.0 * morse_unit_us) : (1.0 * morse_unit_us);
      // After this element ends, we owe a 1-unit gap unless this was the last element
      between_elements = (morse_pat_index < current_pat.length());
      return morse_key;
    } else {
      // End of character -> 3-unit gap
      current_pat = "";
      morse_key = KEY_OFF;
      morse_time_left_us = 3.0 * morse_unit_us;
      return morse_key;
    }
  }

  // Consume time
  morse_time_left_us -= slice_us;
  return morse_key;
}

// ---------------- Diagnostics ----------------
void IRAM_ATTR on_tx_done() {
  uint32_t now = micros();
  uint32_t prev = last_done_us;
  last_done_us = now;
  if (prev != 0) {
    uint32_t gap = now - prev;
    if (gap > max_gap_us) max_gap_us = gap;
  }
  tx_done_count++;
  digitalWrite(PIN_HEARTBEAT, HIGH);
  digitalWrite(PIN_HEARTBEAT, LOW);
}

// ------------- Modulated buffer fill -------------
// Writes a buffer in STRIPE-sized chunks. Within each stripe we lay down either
// carrier-on bytes (0xAA) or off bytes (0x00). For AM/MCW we compute a duty based
// on the *instantaneous* sine envelope; for CW we only key on/off.
void fill_buffer_modulated(uint8_t* dst, size_t nbytes) {
  // Time per stripe (us)
  const double bits_per_stripe = (double)STRIPE * 8.0;
  const double stripe_us = 1e6 * bits_per_stripe / (double)current_spi_hz;

  size_t written = 0;
  while (written < nbytes) {
    size_t blk = sz_min((size_t)STRIPE, nbytes - written);

    // Determine desired amplitude for this stripe
    float duty = 0.0f;

    if (MOD_MODE == MODE_AM) {
      // Continuous AM
      float s = sinf(2.0f * (float)M_PI * (float)tone_phase); // [-1..+1]
      float a = 0.5f * (1.0f + MOD_INDEX * s);
      a = BASELINE + (1.0f - BASELINE) * a;
      if (a < 0.0f) a = 0.0f; if (a > 1.0f) a = 1.0f;
      duty = a;
    } else if (MOD_MODE == MODE_MCW) {
      // Tone bursts keyed by Morse
      KeyState k = morse_advance(stripe_us);
      if (k == KEY_ON) {
        float s = sinf(2.0f * (float)M_PI * (float)tone_phase);
        float a = 0.5f * (1.0f + MOD_INDEX * s);
        a = BASELINE + (1.0f - BASELINE) * a; // set BASELINE=0 for silent gaps
        if (a < 0.0f) a = 0.0f; if (a > 1.0f) a = 1.0f;
        duty = a;
      } else {
        duty = BASELINE; // 0.0 recommended for true key-up silence
      }
    } else { // MODE_CW
      // Pure on/off carrier (A1A)
      KeyState k = morse_advance(stripe_us);
      duty = (k == KEY_ON) ? 1.0f : BASELINE; // set BASELINE=0 for true silence
    }

    // Write this stripe: 'on' part is 0xAA, 'off' is 0x00
    int on_bytes = (int)lroundf(duty * (float)blk);
    if (on_bytes < 0) on_bytes = 0; if (on_bytes > (int)blk) on_bytes = blk;
    memset(dst + written, 0xAA, on_bytes);
    if ((int)blk - on_bytes > 0) memset(dst + written + on_bytes, 0x00, blk - on_bytes);

    // Advance tone phase for next stripe
    tone_phase += cycles_per_stripe;
    if (tone_phase >= 1.0) tone_phase -= floor(tone_phase);

    written += blk;
  }
}

// ---------------- SPI setup ----------------
void queue_all_transfers() {
  for (int i = 0; i < QUEUE_DEPTH; ++i) {
    if (spi_device_queue_trans(spi, &trans[i], 0) != ESP_OK) break;
  }
}

void rebuild_device(uint32_t hz) {
  if (spi) { spi_bus_remove_device(spi); spi = nullptr; }

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = (int)hz;
  devcfg.mode           = 0;
  devcfg.spics_io_num   = -1;
  devcfg.queue_size     = QUEUE_DEPTH;
  devcfg.flags          = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi));

  current_spi_hz = hz;

  // reset stats
  tx_done_count = 0; bytes_pushed_this_sec = 0; trans_pushed_this_sec = 0;
  last_done_us = 0; max_gap_us = 0;

  // recompute rates
  recalc_rates();
  recalc_morse();

  // Initial fill
  for (int i = 0; i < QUEUE_DEPTH; ++i) {
    fill_buffer_modulated(bufs[i], BUF_BYTES);
    memset(&trans[i], 0, sizeof(spi_transaction_t));
    trans[i].length    = BUF_BYTES * 8;
    trans[i].tx_buffer = bufs[i];
  }
  queue_all_transfers();
}

void setup_spi_bus() {
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num     = PIN_MOSI;
  buscfg.miso_io_num     = -1;
  buscfg.sclk_io_num     = PIN_SCK;
  buscfg.quadwp_io_num   = -1;
  buscfg.quadhd_io_num   = -1;
  buscfg.max_transfer_sz = BUF_BYTES;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, DMA_CH));
}

// ---------------- Serial controls ----------------
String cmd;

void handle_serial_line(const String& line) {
  if (line.length() < 1) return;
  String s = line;
  s.trim();
  if (s.startsWith("t ")) {
    float v = s.substring(2).toFloat();
    if (v > 0.0f && v <= 20000.0f) {
      TONE_HZ = v;
      recalc_rates();
      Serial.printf("Tone set to %.1f Hz\n", TONE_HZ);
    } else Serial.println("Tone out of range (0..20 kHz).");
  } else if (s.startsWith("m ")) {
    float v = s.substring(2).toFloat();
    if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f;
    MOD_INDEX = v;
    Serial.printf("Mod index set to %.2f\n", MOD_INDEX);
  } else if (s.startsWith("b ")) {
    float v = s.substring(2).toFloat();
    if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f;
    BASELINE = v;
    Serial.printf("Baseline set to %.2f\n", BASELINE);
  } else if (s.startsWith("w ")) {
    int v = s.substring(2).toInt();
    if (v >= 5 && v <= 40) {
      MORSE_WPM = v;
      recalc_morse();
      Serial.printf("WPM set to %d\n", MORSE_WPM);
    } else Serial.println("WPM range 5..40.");
  } else if (s.startsWith("msg ")) {
    MORSE_MSG = s.substring(4);
    if (MORSE_MSG.length() == 0) MORSE_MSG = "CQ CQ DX ";
    // reset Morse state
    morse_msg_index = 0; morse_pat_index = 0; current_pat = ""; morse_time_left_us = 0;
    Serial.printf("Message set to: %s\n", MORSE_MSG.c_str());
  } else if (s.startsWith("mode ")) {
    String m = s.substring(5); m.toLowerCase();
    if (m == "am") MOD_MODE = MODE_AM;
    else if (m == "mcw") MOD_MODE = MODE_MCW;
    else if (m == "cw") MOD_MODE = MODE_CW;
    else { Serial.println("Modes: am | mcw | cw"); return; }
    // reset Morse state when entering mcw/cw
    morse_msg_index = 0; morse_pat_index = 0; current_pat = ""; morse_time_left_us = 0;
    Serial.printf("Mode set to %s\n", (MOD_MODE==MODE_AM?"AM":(MOD_MODE==MODE_MCW?"MCW":"CW")));
  } else if (s.startsWith("f ")) {
    int idx = s.substring(2).toInt() - 1;
    if (idx >= 0 && (size_t)idx < F_COUNT) {
      rebuild_device(F_LIST[idx]);
      Serial.printf("SPI=%u Hz (carrier ≈ %.3f MHz)\n",
                    current_spi_hz, current_spi_hz/2.0/1e6);
    } else Serial.println("Clock index 1..7");
  } else {
    Serial.println("Commands: mode am|mcw|cw | msg <text> | w <wpm> | t <Hz> | m <0..1> | b <0..1> | f <1..7>");
  }
}

void maybe_handle_serial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { handle_serial_line(cmd); cmd = ""; }
    else if (isPrintable(ch) || ch==' ') cmd += ch;
  }
}

// ---------------- Arduino setup/loop ----------------
void setup() {
  pinMode(PIN_HEARTBEAT, OUTPUT);
  digitalWrite(PIN_HEARTBEAT, LOW);

  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(F("ESP32 SPI Signal Generator with AM / MCW / CW + Diagnostics"));
  Serial.println(F("Pins: SCK=18, MOSI=23, Heartbeat=2"));
  Serial.println(F("Hotkeys: f 1..7 for SPI 80..20 MHz (carrier ≈ SPI/2)"));
  Serial.println(F("Commands: mode am|mcw|cw | msg <text> | w <wpm> | t <Hz> | m <0..1> | b <0..1> | f <1..7>"));
  Serial.println(F("Defaults: mode=mcw, msg=\"CQ CQ DX \", w=20, tone=1000 Hz"));

  cpu_mhz_cache = getCpuFrequencyMhz();

  // Default to MCW per your ask
  MOD_MODE = MODE_MCW;
  MORSE_MSG = "CQ CQ DX ";
  MORSE_WPM = 20;
  recalc_morse();

  // Allocate DMA buffers
  for (int i = 0; i < QUEUE_DEPTH; ++i) {
    bufs[i] = (uint8_t*)heap_caps_malloc(BUF_BYTES, MALLOC_CAP_DMA);
    if (!bufs[i]) { Serial.println(F("DMA buffer alloc failed")); while(1){delay(1000);} }
  }

  setup_spi_bus();
  rebuild_device(current_spi_hz);
}

void loop() {
  // Drain completed transactions, refill, requeue
  spi_transaction_t *done;
  while (spi_device_get_trans_result(spi, &done, 0) == ESP_OK) {
    on_tx_done();
    fill_buffer_modulated((uint8_t*)done->tx_buffer, BUF_BYTES);
    esp_err_t e = spi_device_queue_trans(spi, done, 0);
    if (e != ESP_OK) { /* will retry next loop */ }
    bytes_pushed_this_sec += BUF_BYTES;
    trans_pushed_this_sec++;
  }

  // 1 Hz stats
  static uint32_t last_print_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_print_ms >= 1000) {
    last_print_ms = now_ms;
    float f_out_mhz = current_spi_hz / 2.0f / 1e6f;
    uint32_t free_heap = esp_get_free_heap_size();
    Serial.printf(
      "SPI=%2.0f MHz | carrier≈%6.3f MHz | bytes=%7u/s | tx=%4u/s | max_gap=%3u us | heap=%6u | CPU=%d | mode=%s | tone=%.1f Hz | WPM=%d\n",
      current_spi_hz/1e6f, f_out_mhz,
      bytes_pushed_this_sec, trans_pushed_this_sec,
      max_gap_us, free_heap, cpu_mhz_cache,
      (MOD_MODE==MODE_AM?"AM":(MOD_MODE==MODE_MCW?"MCW":"CW")),
      TONE_HZ, MORSE_WPM
    );
    bytes_pushed_this_sec = 0; trans_pushed_this_sec = 0; max_gap_us = 0;
  }

  maybe_handle_serial();
}
