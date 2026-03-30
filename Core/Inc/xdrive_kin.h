#pragma once
#include <stdint.h>

typedef struct {
  float theta_deg;   // ângulo da roda (graus)
  uint8_t motor_idx; // 1..4
  int8_t invert;     // +1 ou -1
} wheel_t;

typedef struct {
  float R_mm;        // raio do robô (mm)
  int8_t tangent_ccw;// 1 = tangente CCW (padrão)
  int8_t rot_sign;   // +1 normal, -1 inverte só rotação
  wheel_t w[4];
} xdrive_t;

void xdrive_init_default(xdrive_t *k, float robot_radius_mm);
void xdrive_speeds(const xdrive_t *k, float vx, float vy, float w_rad_s, float out_m[4]);
