# Independent Quad-Motor Control and Deterministic Position Tracking

**A Linear and Angular Velocity-Based Robotic Control Subsystem**

* **Institution:** HAW Hamburg — BSc Information Engineering (Elective Project 1 - CJ1)
* **Main Title:** Independent Quad-Motor Control and Deterministic Position Tracking
* **Subtitle:** A Linear and Angular Velocity-Based Robotic Control Subsystem
* **Team Members:** Luciano Carricart, Georgii Molyboga, Stephen Onyewuenyi
* **Supervisor:** Prof. Dr. Benno Radt
* **Date:** 19 October 2026

---

## Abstract & Scope

This project delivers an embedded quad-motor control and deterministic position-tracking subsystem for a differential-drive electric rover. Powered by an ARM Cortex-M33 microcontroller (STM32 NUCLEO-C562RE), the subsystem converts commanded linear and angular velocities into wheel speeds, independently regulates four DC motors using quadrature encoder feedback, displays real-time motor position and velocity metrics on an SPI graphical screen, and supports remote wireless operation over Bluetooth Low Energy (BLE).

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
|  +-----------------------+                    |   - Quad Encoder Counters      |  |
|  | Physical Fallback CTR | ---> GPIO / ADC -- |   - Independent Watchdog (IWDG)|  |
|  +-----------------------+                    +--------------------------------+  |
|                                                    |              |               |
|                                           PWM / DIR|              | SPI           |
|                                                    v              v               |
|  +-----------------------+                    +----------+  +------------------+  |
|  |  Four Motor Channels  |                    |  Four    |  | Local Graphics   |  |
|  |  (H-Bridge Power)     | ==================>| Motors & |  | SPI Display      |  |
|  +-----------------------+                    | Encoders |  |                  |  |
|                                               +----------+  +------------------+  |
+-----------------------------------------------------------------------------------+
```
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

```text
       +---------------------------------+
       | INIT                            |
       |---------------------------------|
       | do: driver output held low      |
       | do: hardware checks             |
       +---------------------------------+
               |
               v
       +--------------+  RUN command [fresh]   +--------------+
       | IDLE         | ---------------------> | RUN          |
       +--------------+                        +--------------+
              ^                                       |
              | rest verified                         | stop | lease timeout | fault
              | [all motors < 3 RPM for 300 ms]       v
       +--------------+                        +--------------+
       | STOPPING     | <--------------------- | FAULT        |
       +--------------+                        +--------------+

Legend - Standard UML FSM notation
  event [guard] / action     Transition label format
  event 1 | event 2          Alternative triggering events
  unlabeled arrow            Completion transition
```

---

## Use Case Specifications

### Use Case Diagram (PlantUML Format)

![Use Case Diagram](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/Georgemolyboga/Farming_Robot/main/docs/uc_diagram1.puml)


#### Specification Summary Matrix

| UC ID | Name | Primary Actor | Trigger | Core Metric / Target | Primary Fallback |
|-----|---|---|---|---|---|
| **UC-1** | Remote Drive Control | Phone BLE App | BLE Packet Received | Response time $< 1.0\text{ s}$ | Discard malformed packet |
| **UC-2** | Quad-Motor Velocity Control | 50 Hz Interrupt | 20 ms Timer Expiry | Four independent updates every $20\text{ ms} \pm 2\text{ ms}$ | Anti-windup clamping |
| **UC-3** | Encoder Feedback & Position Tracking | Encoders | Shaft Rotation | Per-motor count accuracy $\pm 2\%$ | Counter wrap handling |
| **UC-4** | Local Telemetry Display | SPI Display | Main Loop ($\ge 10\text{ Hz}$) | Telemetry age $\le 200\text{ ms}$ | Non-blocking SPI transfer |
| **UC-5** | Hardware Emergency Stop | Physical Switch | Button Press | Disable time $\le 100\text{ ms}$ | Direct hardware disable |
| **UC-6** | Command Lease Timeout | System Lease Timer | Idle elapsed $> 2.0\text{ s}$ | Disable outputs by $2.1\text{ s}$ | Manual re-initiation required |
| **UC-7** | Physical Fallback Mode | Operator | Control Adjustment | Response time $\le 100\text{ ms}$ | Set all targets to 0 RPM |

---

## Acceptance Test Protocols (M1–M10)

| Requirement ID | Module / Feature | Acceptance Criteria & Test Procedure | Verification Target / Metric | Responsible Role |
|---|---|---|---|---|
| **M1** | Independent Actuation | Command 30, 60, and 90 RPM in both directions for each of the four motors independently, then stop. | The selected motor moves correctly; the other three remain disabled ($0\text{ RPM}$). | Motor Control |
| **M2** | Count & Direction | Turn each motor 10 revolutions forward and 10 back manually with the drive disabled. | Each signed count change equals $10N_i \pm 2\%$; no wrap discontinuities occur over 60 s. | Motor Feedback |
| **M3** | Relative Position | Stop each motor at 0°, 90°, 180°, and 270° in three passes, including reverse approach. | Circular error against the physical reference is $\le 10^\circ$ across 48 points. | Motor Feedback |
| **M4** | Velocity Regulation | Test 30, 60, and 90 RPM targets on all four motors under no-load conditions for 10 seconds. | Each mean measured RPM is within $\le \max(5\text{ RPM}, 0.10 \times \text{target})$. | Motor Control |
| **M5** | Coordinated Movement | Issue forward, reverse, and turning commands as linear and angular velocity pairs three times. | All four motor targets update within the same 20 ms control-loop cycle. | Motor Control |
| **M6** | Graphical Telemetry | Monitor local SPI screen during full speed execution under test M4. | Display updates at $\ge 10\text{ Hz}$ with telemetry age $\le 200\text{ ms}$. | Motor Interface |
| **M7** | BLE Control & Lease | Transmit 20 valid and 10 malformed commands; simulate 5 disconnections at 3 m. | Valid commands ack $< 1.0\text{ s}$; outputs disable within $2.1\text{ s}$ of disconnect. | Motor Interface |
| **M8** | Timing & Real-Time | Run 5-minute stress test (15,000 updates) with display active and 5 BLE commands/s. | Control period: $18\text{--}22\text{ ms}$; task execution $\le 2\text{ ms}$; zero deadline misses. | Motor Control |
| **M9** | Watchdog & Startup | Inject 2 main loop hangs; evaluate hardware state during startup/reset sequences. | IWDG forces reset within $1.0\text{ s}$; driver held disabled during reboot. | Motor Control |
| **M10** | Emergency Stop & Assembly | Trigger the physical E-Stop five times during movement; conduct a 10-minute vibration run. | All four motor outputs deactivate within $\le 100\text{ ms}$; physical mounts remain secure. | Motor Feedback |

---

## Work Breakdown & 12-Week Plan

```text
Week  1  2  3  4  5  6  7  8  9 10 11 12
-----------------------------------------
M1   [*] .  .  .  .  .  .  .  .  .  .  .   Scope & Targets Approved
M2    .  . [*] .  .  .  .  .  .  .  .  .   Single Channel Bench Test Complete
BLE   .  .  . [*] .  .  .  .  .  .  .  .   BLE Protocol & Phone Command Verified
M3    .  .  .  .  . [*] .  .  .  .  .  .   Four Closed Loops & SPI Display Integrated
M4    .  .  .  .  .  .  . [*] .  .  .  .   Full Rover System Integration Complete
M5    .  .  .  .  .  .  .  .  . [*] .  .   Acceptance Tests M1-M10 Passed
M6    .  .  .  .  .  .  .  .  .  .  . [*]  Final Documentation & Presentation
```

---

## C Code Headers & Structures

### Microcontroller Pinout & Peripheral Mapping (C Code Header)

```c
/**
 * @file hardware_config.h
 * @brief Peripheral allocation & pin mappings for STM32C562RE Nucleo
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "stm32_hal.h"

/* --- Independent Motor Channels --- */
#define MOTOR_COUNT                4

/* --- Quadrature Encoders --- */
/* Front-left motor */
#define TIM_ENCODER_FRONT_LEFT     TIM1
#define PORT_ENCODER_FRONT_LEFT    GPIOA
#define PIN_ENCODER_FRONT_LEFT_A   GPIO_PIN_8    /* PA8 - TIM1_CH1 */
#define PIN_ENCODER_FRONT_LEFT_B   GPIO_PIN_9    /* PA9 - TIM1_CH2 */

/* Front-right motor */
#define TIM_ENCODER_FRONT_RIGHT    TIM2
#define PORT_ENCODER_FRONT_RIGHT   GPIOA
#define PIN_ENCODER_FRONT_RIGHT_A  GPIO_PIN_0    /* PA0 - TIM2_CH1 */
#define PIN_ENCODER_FRONT_RIGHT_B  GPIO_PIN_1    /* PA1 - TIM2_CH2 */

/* Rear-left motor */
#define TIM_ENCODER_REAR_LEFT      TIM5
#define PORT_ENCODER_REAR_LEFT     GPIOB
#define PIN_ENCODER_REAR_LEFT_A    GPIO_PIN_4    /* PB4 - TIM5_CH1 */
#define PIN_ENCODER_REAR_LEFT_B    GPIO_PIN_5    /* PB5 - TIM5_CH2 */

/* Rear-right motor */
#define TIM_ENCODER_REAR_RIGHT     TIM8
#define PORT_ENCODER_REAR_RIGHT    GPIOC
#define PIN_ENCODER_REAR_RIGHT_A   GPIO_PIN_6    /* PC6 - TIM8_CH1 */
#define PIN_ENCODER_REAR_RIGHT_B   GPIO_PIN_7    /* PC7 - TIM8_CH2 */

/* --- Motor Driver (PWM & Direction) --- */
/* Front-left motor */
#define TIM_PWM_FRONT_LEFT         TIM12
#define PORT_PWM_FRONT_LEFT        GPIOB
#define PIN_PWM_FRONT_LEFT         GPIO_PIN_14   /* PB14 - TIM12_CH1 */
#define PORT_DIR_FRONT_LEFT        GPIOA
#define PIN_DIR_FRONT_LEFT         GPIO_PIN_4    /* PA4 - Output */

/* Front-right motor */
#define TIM_PWM_FRONT_RIGHT        TIM12
#define PORT_PWM_FRONT_RIGHT       GPIOB
#define PIN_PWM_FRONT_RIGHT        GPIO_PIN_15   /* PB15 - TIM12_CH2 */
#define PORT_DIR_FRONT_RIGHT       GPIOB
#define PIN_DIR_FRONT_RIGHT        GPIO_PIN_0    /* PB0 - Output */

/* Rear-left motor */
#define TIM_PWM_REAR_LEFT          TIM16
#define PORT_PWM_REAR_LEFT         GPIOB
#define PIN_PWM_REAR_LEFT          GPIO_PIN_7    /* PB7 - TIM16_CH1 */
#define PORT_DIR_REAR_LEFT         GPIOB
#define PIN_DIR_REAR_LEFT          GPIO_PIN_1    /* PB1 - Output */

/* Rear-right motor */
#define TIM_PWM_REAR_RIGHT         TIM17
#define PORT_PWM_REAR_RIGHT        GPIOC
#define PIN_PWM_REAR_RIGHT         GPIO_PIN_2    /* PC2 - TIM17_CH1 */
#define PORT_DIR_REAR_RIGHT        GPIOB
#define PIN_DIR_REAR_RIGHT         GPIO_PIN_2    /* PB2 - Output */

/* --- Hardware Safety / E-Stop --- */
#define PORT_ESTOP                 GPIOC
#define PIN_HARDWARE_ESTOP         GPIO_PIN_13   /* PC13 - EXTI / B1, active HIGH */
#define PORT_DRIVER_DISABLE        GPIOC
#define PIN_DRIVER_DISABLE         GPIO_PIN_1    /* PC1 - Direct Disable Output */

/* --- BLE Communication --- */
#define USART_BLE                  USART3
#define PORT_BLE                   GPIOC
#define PIN_BLE_TX                 GPIO_PIN_10   /* PC10 - USART3_TX */
#define PIN_BLE_RX                 GPIO_PIN_11   /* PC11 - USART3_RX */

/* --- SPI Display --- */
#define SPI_DISPLAY                SPI1
#define PORT_DISPLAY_SPI           GPIOA
#define PIN_DISPLAY_SCK            GPIO_PIN_5    /* PA5 - SPI1_SCK */
#define PIN_DISPLAY_MOSI           GPIO_PIN_7    /* PA7 - SPI1_MOSI */
#define PORT_DISPLAY_CS            GPIOC
#define PIN_DISPLAY_CS             GPIO_PIN_9    /* PC9 - GPIO Chip Select */
#define PORT_DISPLAY_DC            GPIOB
#define PIN_DISPLAY_DC             GPIO_PIN_6    /* PB6 - Data/Command */

/* --- Control Constants --- */
#define CONTROL_LOOP_FREQ_HZ       50
#define CONTROL_LOOP_PERIOD_MS     20
#define COMMAND_LEASE_TIMEOUT_MS   2000
#define WATCHDOG_TIMEOUT_MS        500

/* Confirm final alternate-function selections in STM32CubeMX before wiring. */

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
    float target_rpm;
    float measured_rpm;
    int32_t encoder_count;
} MotorTelemetry_t;

typedef struct {
    MotorTelemetry_t front_left;
    MotorTelemetry_t front_right;
    MotorTelemetry_t rear_left;
    MotorTelemetry_t rear_right;
    float linear_velocity;
    float angular_velocity;
    uint32_t last_command_timestamp;
    SystemState_t state;
} RoverTelemetry_t;
```
