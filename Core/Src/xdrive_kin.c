#include "xdrive_kin.h"
#include <math.h>

static float wheel_speed(const xdrive_t *k, float vx, float vy, float w, float theta_deg)
{
  const float th = theta_deg * (float)M_PI / 180.0f;

  const float x = k->R_mm * cosf(th);
  const float y = k->R_mm * sinf(th);

  float ux, uy; // direção de tração (tangente)
  if (k->tangent_ccw) { ux = -sinf(th); uy =  cosf(th); }
  else               { ux =  sinf(th); uy = -cosf(th); }

  const float v_trans = ux * vx + uy * vy;

  float v_rot = ux * (-w * y) + uy * (w * x);
  v_rot *= (float)k->rot_sign;

  return v_trans + v_rot;
}

void xdrive_init_default(xdrive_t *k, float robot_radius_mm)
{
  k->R_mm = robot_radius_mm;
  k->tangent_ccw = 1;
  k->rot_sign = 1;

  // igual teu Python
  // FL 45° -> motor 2 (+)
  k->w[0] = (wheel_t){ .theta_deg = 45.0f,  .motor_idx = 2, .invert = +1 };
  // FR 315° -> motor 1 (+)
  k->w[1] = (wheel_t){ .theta_deg = 315.0f, .motor_idx = 1, .invert = +1 };
  // RR 225° -> motor 3 (-)
  k->w[2] = (wheel_t){ .theta_deg = 225.0f, .motor_idx = 3, .invert = -1 };
  // RL 135° -> motor 4 (-)
  k->w[3] = (wheel_t){ .theta_deg = 135.0f, .motor_idx = 4, .invert = -1 };
}

void xdrive_speeds(const xdrive_t *k, float vx, float vy, float w_rad_s, float out_m[4])
{
  // out_m index 0..3 corresponde motor 1..4
  out_m[0] = out_m[1] = out_m[2] = out_m[3] = 0.0f;

  for (int i = 0; i < 4; i++) {
    const wheel_t *wh = &k->w[i];
    float vi = wheel_speed(k, vx, vy, w_rad_s, wh->theta_deg);
    vi *= (float)wh->invert;
    if (wh->motor_idx >= 1 && wh->motor_idx <= 4) {
      out_m[wh->motor_idx - 1] = vi;
    }
  }
}
