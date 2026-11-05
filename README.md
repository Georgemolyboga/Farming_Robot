# Precise Motor Control & Real-Time Monitoring System

**Project Repository & Technical Specification**

* **Institution:** HAW Hamburg — BSc Information Engineering (Elective Project 1 - CJ1)
* **Working Title:** Precise Motor Control and Real-Time Monitoring System
* **Registration Title:** Dual Motor Control and Real Time Monitoring
* **Team Members:** Luciano Carricart, Georgii Molyboga, Stephen Onyewuenyi
* **Supervisor:** Prof. Dr. Benno Radt
* **Date:** 19 October 2026

---

## Abstract & Scope

This project delivers an embedded dual motor control and real-time telemetry system for a differential-drive electric rover. Powered by an ARM Cortex-M microcontroller (STM32 Nucleo), the system independently regulates two DC gearmotors with quadrature encoder feedback, executes closed-loop speed control, displays real-time angular position and RPM metrics on an SPI graphical screen, and supports remote wireless operation over Bluetooth Low Energy (BLE).

---

## System Architecture & Hardware Setup

### Block Diagram

```text
+-----------------------------------------------------------------------------------+
|                                   SYSTEM ARCHITECTURE                             |
+-----------------------------------------------------------------------------------+
|                                                                                   |
|  +-----------------------+                    +--------------------------------+  |
|  |  Remote Control App   | <--- BLE / UART -> |  BLE Module (e.g. RN4870)      |  |
|  +-----------------------+                    +--------------------------------+  |
|                                                                |                  |
|                                                          USART | (RX/TX Interrupt)|
|                                                                v                  |
|  +-----------------------+                    +--------------------------------+  |
|  | Hardware E-Stop Switch| --- Direct Disable |   STM32 Nucleo Microcontroller |  |
|  +-----------------------+                    |   - Bare-Metal C State Machine |  |
|                                               |   - 50 Hz Control Loop (20 ms) |  |
|  +-----------------------+                    |   - Dual Timer Encoder Counters|  |
|  | Physical Fallback CTR | ---> GPIO / ADC -- |   - Independent Watchdog (IWDG)|  |
|  +-----------------------+                    +--------------------------------+  |
|                                                    |              |               |
|                                           PWM / DIR|              | SPI           |
|                                                    v              v               |
|  +-----------------------+                    +----------+  +------------------+  |
|  |  Dual H-Bridge Driver |                    |  Motors  |  | Local Graphics   |  |
|  |  (Power Stage)        | ==================>|  & A/B   |  | SPI Display      |  |
|  +-----------------------+                    | Encoders |  +------------------+  |
|                                               +----------+                        |
+-----------------------------------------------------------------------------------+
```
*(The microcontroller pinout and peripheral mapping C Code Header has been moved to Section 7.1)*

---

## Firmware Architecture & Algorithms

### Kinematics & Telemetry Equations

Given decoded encoder resolution $N$ (counts per revolution):

$$
\text{Relative Angle}\ (^\circ) = 360 \times \frac{\text{count} - \text{count}_{\text{zero}}}{N}
$$

$$
\text{Speed}\ (\text{RPM}) = 60 \times \frac{\Delta\text{count}}{N \times \Delta t}
$$
### Finite State Machine Diagram

*(The associated C Code structures for telemetry and system state have been moved to Section 7.2)*

```text
       +--------------+
       |     INIT     | (Driver output held low / Hardware checks)
       +--------------+
              |
              v
       +--------------+  Fresh RUN Cmd  +--------------+
       |     IDLE     | --------------> |     RUN      |
       +--------------+                 +--------------+
              ^                                |
              | Rest Verified                  | Stop / Lease Timeout / Fault
              | (< 3 RPM for 300 ms)           v
       +--------------+                 +--------------+
       |   STOPPING   | <-------------- |    FAULT     |
       +--------------+                 +--------------+
```

---

## Use Case Specifications

### Use Case Diagram (PlantUML Format)

![Use Case Diagram](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/Georgemolyboga/Farming_Robot/main/docs/uc_diagram1.puml)


#### Specification Summary Matrix

| UC ID | Name | Primary Actor | Trigger | Core Metric / Target | Primary Fallback |
|-----|---|---|---|---|---|
| **UC-1** | Remote Drive Control | Phone BLE App | BLE Packet Received | Response time $< 1.0\text{ s}$ | Discard malformed packet |
| **UC-2** | Closed-Loop Speed Control | 50 Hz Interrupt | 20 ms Timer Expiry | Execution period $20\text{ ms} \pm 2\text{ ms}$ | Anti-windup clamping |
| **UC-3** | Encoder Feedback & Angle | Encoders | Shaft Rotation | Count accuracy $\pm 2\%$ | Hardware 16-bit auto-wrap |
| **UC-4** | Local Telemetry Display | SPI Display | Main Loop ($\ge 10\text{ Hz}$) | Telemetry age $\le 200\text{ ms}$ | Non-blocking SPI transfer |
| **UC-5** | Hardware Emergency Stop | Physical Switch | Button Press | Disable time $\le 100\text{ ms}$ | Direct hardware disable |
| **UC-6** | Command Lease Timeout | System Lease Timer | Idle elapsed $> 2.0\text{ s}$ | Disable outputs by $2.1\text{ s}$ | Manual re-initiation required |
| **UC-7** | Physical Fallback Mode | Operator | Control Adjustment | Response time $\le 100\text{ ms}$ | Default target to 0 RPM |

---

## Acceptance Test Protocols (M1–M10)

| Requirement ID | Module / Feature | Acceptance Criteria & Test Procedure | Verification Target / Metric | Responsible Role |
|---|---|---|---|---|
| **M1** | Independent Actuation | Command 30, 60, 90 RPM in both directions for each motor independently, then Stop. | Selected wheel moves correctly; uncommanded wheel remains disabled ($0\text{ RPM}$). | Motor Control |
| **M2** | Count & Direction | Turn each wheel 10 revolutions forward and 10 back manually (drive disabled). | Signed count change equals $10N \pm 2\%$; no wrap discontinuities over 60s. | Motor Feedback |
| **M3** | Relative Angle | Stop at 0°, 90°, 180°, and 270° in 3 passes per wheel (including reverse approach). | Circular error against physical reference $\le 10^\circ$ across 24 points. | Motor Feedback |
| **M4** | Speed Regulation | Test 30, 60, and 90 RPM targets under no-load condition for 10 seconds. | Mean measured RPM within $\le \max(5\text{ RPM}, 0.10 \times \text{target})$. | Motor Control |
| **M5** | Coordinated Movement | Issue paired forward, reverse, left-arc, and right-arc drive targets 3 times. | Target values update simultaneously within one 20 ms loop cycle. | Motor Control |
| **M6** | Graphical Telemetry | Monitor local SPI screen during full speed execution under test M4. | Display updates at $\ge 10\text{ Hz}$ with telemetry age $\le 200\text{ ms}$. | Motor Interface |
| **M7** | BLE Control & Lease | Transmit 20 valid and 10 malformed commands; simulate 5 disconnections at 3 m. | Valid commands ack $< 1.0\text{ s}$; outputs disable within $2.1\text{ s}$ of disconnect. | Motor Interface |
| **M8** | Timing & Real-Time | Run 5-minute stress test (15,000 updates) with display active and 5 BLE commands/s. | Control period: $18\text{--}22\text{ ms}$; task execution $\le 2\text{ ms}$; zero deadline misses. | Motor Control |
| **M9** | Watchdog & Startup | Inject 2 main loop hangs; evaluate hardware state during startup/reset sequences. | IWDG forces reset within $1.0\text{ s}$; driver held disabled during reboot. | Motor Control |
| **M10** | Emergency Stop & Assembly | Trigger physical E-Stop 5 times during movement; conduct 10-min vibration run. | Motor driver deactivates within $\le 100\text{ ms}$; physical mounts remain secure. | Motor Feedback |

---

## Work Breakdown & 12-Week Plan

```text
Week  1  2  3  4  5  6  7  8  9 10 11 12
-----------------------------------------
M1   [* ] .  .  .  .  .  .  .  .  .  .  .   Scope & Targets Approved
M2    .  . [*]  .  .  .  .  .  .  .  .  .   Single Channel Bench Test Complete
BLE   .  .  . [*]  .  .  .  .  .  .  .  .   BLE Protocol & Phone Command Verified
M3    .  .  .  .  . [*]  .  .  .  .  .  .   Dual Closed Loop & SPI Display Integrated
M4    .  .  .  .  .  .  . [*]  .  .  .  .   Full Rover System Integration Complete
M5    .  .  .  .  .  .  .  .  . [*]  .  .   Acceptance Tests M1-M10 Passed
M6    .  .  .  .  .  .  .  .  .  .  . [*]   Final Documentation & Presentation
```

---

## C Code Headers & Structures

### Microcontroller Pinout & Peripheral Mapping (C Code Header)

```c
/**
 * @file hardware_config.h
 * @brief Peripheral allocation & pin mappings for STM32F411RE Nucleo
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "stm32f4xx_hal.h"

/* --- Quadrature Encoders --- */
#define TIM_ENCODER_LEFT           TIM2
#define PIN_ENC_L_A                GPIO_PIN_0    /* PA0 - TIM2_CH1 */
#define PIN_ENC_L_B                GPIO_PIN_1    /* PA1 - TIM2_CH2 */

#define TIM_ENCODER_RIGHT          TIM3
#define PIN_ENC_R_A                GPIO_PIN_6    /* PA6 - TIM3_CH1 */
#define PIN_ENC_R_B                GPIO_PIN_7    /* PA7 - TIM3_CH2 */

/* --- Motor Driver (PWM & Direction) --- */
#define TIM_PWM_MOTORS             TIM4
#define PIN_PWM_LEFT               GPIO_PIN_8    /* PB8 - TIM4_CH3 */
#define PIN_PWM_RIGHT              GPIO_PIN_9    /* PB9 - TIM4_CH4 */

#define PORT_MOTOR_DIR             GPIOB
#define PIN_DIR_LEFT               GPIO_PIN_4    /* PB4 - Output */
#define PIN_DIR_RIGHT              GPIO_PIN_5    /* PB5 - Output */

/* --- Hardware Safety / E-Stop --- */
#define PORT_ESTOP                 GPIOC
#define PIN_HARDWARE_ESTOP         GPIO_PIN_13   /* PC13 - EXTI Interrupt */
#define PIN_DRIVER_DISABLE         GPIO_PIN_10   /* PB10 - Direct Hardware Disable Output */

/* --- Communication & Display --- */
#define USART_BLE                  USART2        /* PA2 (TX), PA3 (RX) */
#define SPI_DISPLAY                SPI1          /* PA5 (SCK), PA7 (MOSI) */
#define PIN_DISPLAY_CS             GPIO_PIN_6    /* PB6 - Chip Select */
#define PIN_DISPLAY_DC             GPIO_PIN_7    /* PC7 - Data/Command */

/* --- Control Constants --- */
#define CONTROL_LOOP_FREQ_HZ       50
#define CONTROL_LOOP_PERIOD_MS     20
#define COMMAND_LEASE_TIMEOUT_MS   2000
#define WATCHDOG_TIMEOUT_MS        500

#endif /* HARDWARE_CONFIG_H */
```

### Finite State Machine (C Code Structure)

```c
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_RUN,
    STATE_STOPPING,
    STATE_FAULT
} SystemState_t;

typedef struct {
    float target_rpm_l;
    float target_rpm_r;
    float measured_rpm_l;
    float measured_rpm_r;
    int32_t encoder_count_l;
    int32_t encoder_count_r;
    uint32_t last_command_timestamp;
    SystemState_t state;
} RoverTelemetry_t;
```