#include "ssl_app.h"

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>

#include "xdrive_kin.h"
#include "Comm/COMM.h"

extern UART_HandleTypeDef huart1;

static char rx_buf[256];
static uint16_t rx_len = 0;
uint32_t g_last_cmd_tick_ms = 0;

/* Configuração básica do link NRF24 (endereços e canal podem ser ajustados). */
#define SSL_RF_CHANNEL   120
/* Robot escuta em A1..A5 e fala com a base em B1..B5 (conforme código validado). */
static const uint8_t SSL_MY_ADDR[5]   = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
static const uint8_t SSL_PEER_ADDR[5] = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5};
static const uint8_t SSL_ROBOT_ID     = 1;

/* Limites de bateria (mV) com histerese para evitar liga/desliga rápido. */
#define BATTERY_MIN_MV       0//10200
#define BATTERY_RESTORE_MV   0//10500
#define BATTERY_QUERY_MS      2000

#define BATTERY_WARN_MV       0 //10800
#define BATTERY_CRIT_MV       0//10200

#define MOTOR_TYPE            2
#define MOTOR_PULSE_PHASE     20
#define MOTOR_PULSE_LINE      13
#define MOTOR_WHEEL_DIAMETER_X1000 48000
#define MOTOR_DEADZONE        1600
/* 1 = envia $flash_reset# no boot */
#define MOTOR_FLASH_RESET_ON_BOOT 0
/* PID padrão (x1000). Ajuste via Live Expressions alterando as variáveis globais. */
#define MOTOR_PID_P_X1000     0
#define MOTOR_PID_I_X1000     0
#define MOTOR_PID_D_X1000     0
/* 0: none, 1: total encoder, 2: 10ms encoder, 3: speed mm/s */
#define MOTOR_UPLOAD_DATA     3

#define BEACON_FLASH_ON_MS       70
#define BEACON_FLASH_GAP_MS      90
#define BEACON_BETWEEN_PAIR_MS  650

#define BEACON_CRIT_SCALE_NUM    1
#define BEACON_CRIT_SCALE_DEN    2


static xdrive_t kin;
volatile float ssl_vx = 0.0f;
volatile float ssl_vy = 0.0f;
volatile float ssl_w  = 0.0f;
volatile float ssl_m_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile int32_t ssl_pid_p_x1000 = MOTOR_PID_P_X1000;
volatile int32_t ssl_pid_i_x1000 = MOTOR_PID_I_X1000;
volatile int32_t ssl_pid_d_x1000 = MOTOR_PID_D_X1000;
volatile uint8_t ssl_read_flash_req = 0;
volatile uint16_t ssl_last_rx_len = 0;
volatile char ssl_last_rx[256];
static bool comm_ready = false;
static int32_t battery_mV = -1;
static bool battery_low = false;
static bool battery_halted = false;
static int debug_usb = 1;

static int32_t parse_battery_mV(const char *msg);
static void battery_update(int32_t mv);
static void battery_try_request(void);
static void battery_beacon_update(uint32_t now_ms);
static void led_set(int on);


static inline uint8_t usb_ready(void) {
  extern USBD_HandleTypeDef hUsbDeviceFS;
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

static inline void led_set(int on)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Padrão: "double flash" (ON, gap, ON, pausa longa) repetindo.
   - Em WARN: padrão normal
   - Em CRIT: padrão mais rápido */
static void battery_beacon_update(uint32_t now_ms)
{
  static uint32_t t0 = 0;
  static uint8_t state = 0;

  /* Só pisca se estiver em low. Se não, garante LED desligado (ou deixa teu toggle de cmd).
     Se quiser manter o "toggle de comando recebido", então aqui NÃO force OFF quando ok. */
  if (!battery_low) {
    // led_set(0);   // descomenta se quiser LED sempre apagado quando bateria ok
    state = 0;
    t0 = now_ms;
    return;
  }

  /* Decide se é WARN ou CRIT */
  bool crit = (battery_mV > 0 && battery_mV <= BATTERY_CRIT_MV);

  uint32_t on_ms      = BEACON_FLASH_ON_MS;
  uint32_t gap_ms     = BEACON_FLASH_GAP_MS;
  uint32_t between_ms = BEACON_BETWEEN_PAIR_MS;

  if (crit) {
    on_ms      = (on_ms      * BEACON_CRIT_SCALE_NUM) / BEACON_CRIT_SCALE_DEN;
    gap_ms     = (gap_ms     * BEACON_CRIT_SCALE_NUM) / BEACON_CRIT_SCALE_DEN;
    between_ms = (between_ms * BEACON_CRIT_SCALE_NUM) / BEACON_CRIT_SCALE_DEN;
    if (on_ms < 30) on_ms = 30; // evita sumir
  }

  switch (state) {
    case 0: // inicia 1º flash
      led_set(1);
      t0 = now_ms;
      state = 1;
      break;

    case 1: // fim 1º flash
      if ((now_ms - t0) >= on_ms) {
        led_set(0);
        t0 = now_ms;
        state = 2;
      }
      break;

    case 2: // gap curto
      if ((now_ms - t0) >= gap_ms) {
        led_set(1);
        t0 = now_ms;
        state = 3;
      }
      break;

    case 3: // fim 2º flash
      if ((now_ms - t0) >= on_ms) {
        led_set(0);
        t0 = now_ms;
        state = 4;
      }
      break;

    case 4: // pausa longa entre pares
    default:
      if ((now_ms - t0) >= between_ms) {
        state = 0;
      }
      break;
  }
}

int _write(int file, char *ptr, int len) {
  (void)file;
  if (!usb_ready()) return len;

  int sent = 0;
  while (sent < len) {
    uint16_t n = (uint16_t)((len - sent) > 64 ? 64 : (len - sent));
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t*)(ptr + sent), n) == USBD_BUSY) {
      if ((HAL_GetTick() - t0) > 10) return len;
    }
    sent += n;
  }
  return len;
}

static void uart_send_str(const char *s) {
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 100);
}

static void send_cmd(const char *fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uart_send_str(buf);
}

static void uart_poll_rx_parse(void) {
  uint8_t ch;

  while (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK) {

    if (ch == '$') {
      rx_len = 0;
      rx_buf[rx_len++] = '$';
      continue;
    }

    if (rx_len == 0) continue;

    if (rx_len < sizeof(rx_buf) - 1) {
      rx_buf[rx_len++] = (char)ch;
    } else {
      rx_len = 0;
      continue;
    }

    if (ch == '#') {
      rx_buf[rx_len] = 0;
      size_t n = rx_len + 1;
      if (n > sizeof(ssl_last_rx)) n = sizeof(ssl_last_rx);
      memcpy((void*)ssl_last_rx, rx_buf, n);
      ssl_last_rx_len = (uint16_t)(n ? (n - 1) : 0);
      printf("RX: %s\r\n", rx_buf);
      int32_t mv = parse_battery_mV(rx_buf);
      if (mv > 0) battery_update(mv);
      rx_len = 0;
    }
  }
}

static inline void set_motor_type(int t)      { send_cmd("$mtype:%d#", t); }
static inline void set_pluse_phase(int x)     { send_cmd("$mphase:%d#", x); }
static inline void set_pluse_line(int x)      { send_cmd("$mline:%d#", x); }
static inline void flash_reset(void)          { uart_send_str("$flash_reset#"); }
static inline void read_flash(void)           { uart_send_str("$read_flash#"); }
static inline void set_pid_x1000(int32_t p_x1000, int32_t i_x1000, int32_t d_x1000)
{
  int32_t vals[3] = { p_x1000, i_x1000, d_x1000 };
  const char *signs[3];
  uint32_t ints[3];
  uint32_t fracs[3];

  for (int i = 0; i < 3; i++) {
    int64_t v = (int64_t)vals[i];
    if (v < 0) {
      v = -v;
      signs[i] = "-";
    } else {
      signs[i] = "";
    }
    ints[i] = (uint32_t)(v / 1000);
    fracs[i] = (uint32_t)(v % 1000);
  }

  send_cmd("$mpid:%s%lu.%03lu,%s%lu.%03lu,%s%lu.%03lu#",
           signs[0], (unsigned long)ints[0], (unsigned long)fracs[0],
           signs[1], (unsigned long)ints[1], (unsigned long)fracs[1],
           signs[2], (unsigned long)ints[2], (unsigned long)fracs[2]);
}
static inline void set_wheel_diameter_x1000(uint32_t mm_x1000)
{
  uint32_t i = mm_x1000 / 1000U;
  uint32_t f = mm_x1000 % 1000U;
  send_cmd("$wdiameter:%lu.%03lu#", (unsigned long)i, (unsigned long)f);
}
static inline void set_deadzone(int x)        { send_cmd("$deadzone:%d#", x); }
static inline void send_upload_data(int all, int ten, int speed)
{
  send_cmd("$upload:%d,%d,%d#", all, ten, speed);
}
static inline void upload_speed_on(void)      { send_upload_data(0, 0, 1); }
static inline void request_battery(void)      { uart_send_str("$read_vol#"); }
static inline void control_speed(int m1,int m2,int m3,int m4){
  send_cmd("$spd:%d,%d,%d,%d#", m1,m2,m3,m4);
}
static inline void stop_pwm(void)             { uart_send_str("$pwm:0,0,0,0#"); }


static inline float fclamp(float x, float lo, float hi){
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static const char* strcasestr_local(const char *haystack, const char *needle) {
  if (!haystack || !needle) return NULL;
  size_t nlen = strlen(needle);
  if (nlen == 0) return haystack;
  for (const char *p = haystack; *p; ++p) {
    if (strncasecmp(p, needle, nlen) == 0) {
      return p;
    }
  }
  return NULL;
}

static void scale_to_i16_4(const float m[4], int16_t out[4], int16_t limit, int16_t deadband)
{
  float mx = 0.0f;
  for (int i = 0; i < 4; i++) {
    float a = fabsf(m[i]);
    if (a > mx) mx = a;
  }
  float s = 1.0f;
  if (mx > (float)limit && mx > 0.0f) s = (float)limit / mx;

  for (int i = 0; i < 4; i++) {
    float v = m[i] * s;
    int32_t iv = (int32_t)lroundf(fclamp(v, -(float)limit, (float)limit));
    if (iv < deadband && iv > -deadband) iv = 0;
    out[i] = (int16_t)iv;
  }
}

static int32_t parse_battery_mV(const char *msg) {
  if (msg == NULL) return -1;

  /* Procura por dígitos na mensagem (ex: "$vol:11800#"). */
  const char *p = msg;
  /* Se existir "vol" antes, pula até depois dele para priorizar essa leitura. */
  const char *vol = strcasestr_local(msg, "vol");
  if (vol) {
    p = vol;
  }

  while (*p && !isdigit((unsigned char)*p)) p++;
  if (!*p) return -1;

  long v = strtol(p, NULL, 10);
  if (v <= 0 || v > 50000) return -1; /* sanity */
  return (int32_t)v;
}

static void battery_update(int32_t mv) {
  if (mv <= 0) return;
  battery_mV = mv;

  /* Estado WARN (beacon) */
  bool was_low = battery_low;
  if (mv < BATTERY_WARN_MV) {
    battery_low = true;
  } else if (mv > BATTERY_RESTORE_MV) {
    battery_low = false;
  }

  /* Estado CRIT (corta motor) */
  bool was_halted = battery_halted;
  if (mv < BATTERY_MIN_MV) {
    battery_halted = true;
  } else if (mv > BATTERY_RESTORE_MV) {
    battery_halted = false;
  }

  if (battery_halted && !was_halted) {
    ssl_vx = ssl_vy = ssl_w = 0.0f;
    stop_pwm();
    printf("BATT CRIT: %ld mV -> motores desativados\r\n", (long)mv);
  } else if (!battery_halted && was_halted) {
    printf("BATT RECOVER: %ld mV\r\n", (long)mv);
  }

  if (battery_low && !was_low) {
    printf("BATT WARN: %ld mV (beacon ligado)\r\n", (long)mv);
  } else if (!battery_low && was_low) {
    printf("BATT OK: %ld mV (beacon off)\r\n", (long)mv);
  }
}


static void battery_try_request(void) {
  static uint32_t tLast = 0;
  uint32_t now = HAL_GetTick();
  if ((now - tLast) >= BATTERY_QUERY_MS) {
    tLast = now;
    request_battery();
  }
}

static void handle_ssl_command(const ssl_command_payload_t* cmd_data,
                               uint8_t robot_id,
                               uint8_t seq_num)
{
  if (cmd_data == NULL) return;

  if (robot_id != SSL_ROBOT_ID) {
    return; // ignora comandos destinados a outro robô
  }

  // Marca recebimento para timeout de segurança.
  extern uint32_t g_last_cmd_tick_ms;
  g_last_cmd_tick_ms = HAL_GetTick();

  if (cmd_data->command_subtype != SSL_CMD_SET_VELOCITIES) {
    return;
  }

  if (debug_usb) {
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_print) > 50U) { // evita spam muito rápido
      last_print = now;
      printf("[RX CMD] id=%u seq=%u vx=%d vy=%d w=%d kickF=%u\r\n",
             cmd_data->robot_id,
             (unsigned)seq_num,
             (int)cmd_data->vx,
             (int)cmd_data->vy,
             (int)cmd_data->vw,
             (unsigned)cmd_data->kick_front);
    }
  }

  if (cmd_data->movement_locked) {
    ssl_vx = 0.0f;
    ssl_vy = 0.0f;
    ssl_w  = 0.0f;
    return;
  }

  ssl_vx = (float)cmd_data->vx;
  ssl_vy = (float)cmd_data->vy;
  ssl_w  = (float)cmd_data->vw;

  /* Indicador visual de comando recebido. */
  if (!battery_low) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

  /* Telemetria de ACK simples (espelha o exemplo). */
  ssl_telemetry_payload_t tel = {0};
  tel.telemetry_subtype = SSL_TELEMETRY_SUBTYPE_STANDARD;
  tel.robot_id = SSL_ROBOT_ID;
  tel.battery_level = (battery_mV > 0) ? (uint8_t)(battery_mV / 100U) : 0;
  tel.ball_detected = 0;
  tel.command_seq_num_ack = seq_num;
  (void)Comm_Send_SSL_Telemetry(&tel);
}

static void ssl_radio_init(void) {
  comm_ready = Comm_Init_P2P(COMM_ROBOT_TYPE_SSL,
                             SSL_RF_CHANNEL,
                             SSL_MY_ADDR,
                             SSL_PEER_ADDR);
  if (!comm_ready) {
    printf("COMM init falhou (NRF24)\r\n");
    return;
  }

  Comm_Register_SSL_CommandHandler(handle_ssl_command);
}

void SSL_Init(void) {
  uint32_t t0 = HAL_GetTick();
  while (!usb_ready() && (HAL_GetTick() - t0) < 2000) HAL_Delay(10);
  printf("\r\nBOOT UART\r\n");

#if MOTOR_FLASH_RESET_ON_BOOT
  flash_reset();
  HAL_Delay(200);
#endif

  send_upload_data(0, 0, 0);
  HAL_Delay(10);
  set_motor_type(MOTOR_TYPE);
  HAL_Delay(100);
  set_pluse_phase(MOTOR_PULSE_PHASE);
  HAL_Delay(100);
  set_pluse_line(MOTOR_PULSE_LINE);
  HAL_Delay(100);
  set_wheel_diameter_x1000(MOTOR_WHEEL_DIAMETER_X1000);
  HAL_Delay(100);
  set_deadzone(MOTOR_DEADZONE);
  HAL_Delay(100);

#if MOTOR_UPLOAD_DATA == 1
  send_upload_data(1, 0, 0);
#elif MOTOR_UPLOAD_DATA == 2
  send_upload_data(0, 1, 0);
#elif MOTOR_UPLOAD_DATA == 3
  send_upload_data(0, 0, 1);
#else
  send_upload_data(0, 0, 0);
#endif
  HAL_Delay(10);


  xdrive_init_default(&kin, 90.0f); // raio do robô: 180/2 = 90mm

  //ssl_radio_init();

  /* Força uma primeira leitura de bateria logo após inicializar. */
  //request_battery();

  if (debug_usb) {
    printf("DBG: init ok, RF ch=%d my=%02X%02X%02X%02X%02X peer=%02X%02X%02X%02X%02X comm=%d\r\n",
           SSL_RF_CHANNEL,
           SSL_MY_ADDR[0], SSL_MY_ADDR[1], SSL_MY_ADDR[2], SSL_MY_ADDR[3], SSL_MY_ADDR[4],
           SSL_PEER_ADDR[0], SSL_PEER_ADDR[1], SSL_PEER_ADDR[2], SSL_PEER_ADDR[3], SSL_PEER_ADDR[4],
           (int)comm_ready);
  }
}

void SSL_Run(void) {
  uart_poll_rx_parse();

  // if (comm_ready) {
  //   Comm_ProcessReceivedPackets();
  // }

  // uint32_t now = HAL_GetTick();
  // battery_beacon_update(now);


  // // Timeout de segurança: se ficar sem comandos por >200ms, zera e para motores.
  // if (g_last_cmd_tick_ms == 0) g_last_cmd_tick_ms = now; // inicializa
  // if ((now - g_last_cmd_tick_ms) > 200U) {
  //   ssl_vx = 0.0f;
  //   ssl_vy = 0.0f;
  //   ssl_w  = 0.0f;
  //   stop_pwm();
  //   control_speed(0, 0, 0, 0);
  //   g_last_cmd_tick_ms = now; // evita spam de stop
  // }

  // /* Garante leitura periódica da bateria e corta se necessário. */
  // battery_try_request();
  // /* Para depuração, não corte o loop se a leitura de bateria falhar. */

  // if (debug_usb) {
  //   static uint32_t last_dbg = 0;
  //   if ((now - last_dbg) > 1000U) {
  //     last_dbg = now;
  //     printf("DBG: vx=%.1f vy=%.1f w=%.1f batt=%ld comm=%d\r\n",
  //            ssl_vx, ssl_vy, ssl_w, (long)battery_mV, (int)comm_ready);
  //   }
  // }

  // static uint32_t tCmd = 0;
  // if (now - tCmd >= 10) {
  //   tCmd = now;

  //   float m[4];
  //   if (battery_halted) {
  //     stop_pwm();
  //     control_speed(0,0,0,0);
  //     return;
  //   }
  //   xdrive_speeds(&kin, ssl_vx, ssl_vy, ssl_w, m);

  //   int16_t cmd[4];
  //   scale_to_i16_4(m, cmd, 2000, 30);

  //   control_speed(cmd[0], cmd[1], cmd[2], cmd[3]);
  // }
  static uint32_t tCmd = 0;
  uint32_t now = HAL_GetTick();
  if ((now - tCmd) >= 10U) {
    tCmd = now;

    if (ssl_read_flash_req) {
      ssl_read_flash_req = 0;
      read_flash();
    }

    static int pid_inited = 0;
    static int32_t last_p = 0;
    static int32_t last_i = 0;
    static int32_t last_d = 0;
    if (!pid_inited) {
      last_p = ssl_pid_p_x1000;
      last_i = ssl_pid_i_x1000;
      last_d = ssl_pid_d_x1000;
      pid_inited = 1;
    }
    if (ssl_pid_p_x1000 != last_p || ssl_pid_i_x1000 != last_i || ssl_pid_d_x1000 != last_d) {
      set_pid_x1000(ssl_pid_p_x1000, ssl_pid_i_x1000, ssl_pid_d_x1000);
      last_p = ssl_pid_p_x1000;
      last_i = ssl_pid_i_x1000;
      last_d = ssl_pid_d_x1000;
    }

    float vx = ssl_vx;
    float vy = ssl_vy;
    float w  = ssl_w;

    if (fabsf(vx) < 0.001f && fabsf(vy) < 0.001f && fabsf(w) < 0.001f) {
      ssl_m_dbg[0] = 0.0f;
      ssl_m_dbg[1] = 0.0f;
      ssl_m_dbg[2] = 0.0f;
      ssl_m_dbg[3] = 0.0f;
      stop_pwm();
      control_speed(0, 0, 0, 0);
      return;
    }

    xdrive_speeds(&kin, vx, vy, w, (float*)ssl_m_dbg);
    int16_t cmd[4];
    scale_to_i16_4(ssl_m_dbg, cmd, 2000, 30);
    control_speed(cmd[0], cmd[1], cmd[2], cmd[3]);
  }
}
