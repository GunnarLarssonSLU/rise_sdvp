# ROV_MCU (Upwis MP101_323): stiftkarta och firmware

Kortet (95 × 160 mm) ersätter sladdhärvan: Raspberry Pi CM5, STM32F415VGT, två
u-blox ZED-F9P, BMI270 + IIS2MDC, USB-hubb, 2 × CAN, 4 DI/O. Stiftkartan är
avläst ur `MP101_323_sch.pdf` (2026-09-30) och **inte provad på hårdvara än**.

Bygg och flasha på kortets CM5: `./flash_styrkort_rovmcu.sh` (i rise_sdvp-roten).
Firmware: `make rovmcu` = F9-kortets layout (`IS_F9_BOARD`) + `IS_ROVMCU`,
maskinprofil som RobAnt (VESC via CAN).

## STM32F415VGT

| Funktion | Ben | Signal i schemat | I firmwaren |
|---|---|---|---|
| USB mot CM5 (via hubb) | PA11/PA12 | MCU_USB_DM/DP | som F9-kortet |
| SWD | PA13/PA14 | FMU_SWDIO/SWCLK | flashning |
| GPS 1 (U10) seriell | PC6 TX / PC7 RX | MCU_TX_F9P_RX / MCU_RX_F9P_TX | USART6, ublox.c |
| GPS 1 reset / timepulse | PC9 / PC8 | FMU_CH7_F9P_RESET_N / FMU_CH6_TIMEPULSE | ublox.c (PC9) |
| GPS 2 (U7) seriell | PA9 / PA10 | USART1_TX/RX | används inte än (byglar väljer CM5/STM) |
| GPS 2 reset / timepulse | PA15 / PA8 | FMU_CH77_F9P_RESET_N / FMU_CH66_TIMEPULSE | PA15 hög i `rovmcu_board_init()` |
| CAN1 | PB8 RX / PB9 TX | CAN1_RX/TX | comm_can.c (som F9-kortet) |
| CAN2 | PB12 RX / PB13 TX | CAN2_RX/TX | används inte än |
| CAN1/CAN2 tystläge (TCAN1057A S) | PB15 / PB14 | SILENT_CAN / SILENT_CAN2 | **låga** i `rovmcu_board_init()` (hög = bara lyssna) |
| IMU BMI270 + kompass IIS2MDC | PB10 SCL / PB11 SDA | I2C2_*_GPS2_MAG_LED | mjukvaru-I2C, `imu/bmi270_wrapper.c` |
| IMU-avbrott / kompassavbrott | PB5 / PE6 | FMU_CH5_IMU_INT2 / IIS2MDC_IRQ | används inte |
| I2C1 (extern) | PB6 / PB7 | I2C1_SCL/SDA | används inte |
| DI1 (J18/J27) | PB0 | SERV_0 | TIM3, servo_simple/pwm_esc |
| DI2 (J23/J32) | PB1 | SERV_1 | pwm_esc |
| DI3 (J24/J37) | PA2 | SERV_2 | pwm_esc (F9-varianten) |
| DI4 (J26/J42) | PA3 | SERV_3 | pwm_esc (F9-varianten) |
| Riktning DI1-4 (74AXP1T45, hög = STM32 → kontakt) | PE4 / PB3 / PB4 / PD7 | DIR_U34 / U38 / U39 / U40 | **höga** (ut) i `rovmcu_board_init()` |
| Lysdioder | PC10 / PC11 | LED0 / LED1 | LED_RED / LED_GREEN (som F9-kortet) |
| Spänningsmätning | PC0 / PC1 | VOLT_METER / PWR_5VDIV | adconv.c |
| Analoga in | PC2, PC3, PA4, PA5, PA6 | ADC1, ADC2, ADC11-13 | PC2/PC3 i adconv.c |
| Hjulpulser (även till F9P stift 22) | PD1 / PD0 | WHEEL_1 / WHEEL_2 | används inte än |
| Telemetri/radio | PC12/PD2 (UART5), PD3-6 (USART2), PA0/PA1 (UART4) | TEL2 / TEL3 / UART4 | används inte |
| Debug-seriell | PD8 / PD9 | USART3_TX/RX_DEBUG | används inte |
| Summer | PD14 | FMU_BUZZER | används inte |
| Extern avbrottsingång | PE15 | EXT_INT | används inte (ID-switchar av på F9-kortet) |
| Reset / BOOT0 | NRST / BOOT0 | FMU_RST / FMU_BOOT_0 | styrs av CM5 |

## CM5 (40-stiftsbenen, BCM-numrering)

| CM5 | Signal | Till |
|---|---|---|
| GPIO11 | FMU_SWCLK (R52) | STM32 PA14 |
| GPIO8 | FMU_SWDIO (R39 0R; R37 till GPIO25 ej monterad) | STM32 PA13 |
| GPIO17 | FMU_RST (R49) | STM32 NRST |
| GPIO16 | FMU_BOOT_0 (R24) | STM32 BOOT0 |
| GPIO6 | FMU_NRST (R103) | se sida 2 |

Rekommenderat i `/boot/firmware/config.txt`: `gpio=17=op,dh` och `gpio=16=op,dl`
(Pi:ns standardneddragning på GPIO17 mot STM32:ans svaga uppdragning på NRST kan
annars ge en odefinierad resetnivå).

## Kvar

- Prova allt på riktigt kort: USB, CAN (VESC/ADDIO), GPS 1, BMI270, DI1-4, lysdioder.
- BMI270:ns axlar och kortets riktning på maskinen (`BOARD_YAW_ROT`).
- Kompass IIS2MDC (drivrutin saknas; kompassen används inte i dag).
- Två antenner: RXD2/TXD2 på F9P:erna är inte inkopplade, så RTCM från GPS 1 till
  GPS 2 går via CM5 (`str2str` USB → USB); kursen (RELPOSNED) ska sedan in i firmwaren.
- CAN2, GPS 2 via USART1, hjulpulser på PD0/PD1.
