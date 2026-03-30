#ifndef SSL_APP_H
#define SSL_APP_H

#include "stm32f4xx_hal.h"

/* Interface para o código de aplicação SSL (separado do código gerado da STM32Cube). */
void SSL_Init(void);
void SSL_Run(void);

/* Velocidades expostas para outros módulos ajustarem conforme necessário. */
extern volatile float ssl_vx;
extern volatile float ssl_vy;
extern volatile float ssl_w;

#endif /* SSL_APP_H */
