# Precise Motor Control & Real-Time Monitoring System

**Project Repository & Technical Specification**

* **Institution:** HAW Hamburg — BSc Information Engineering (Elective Project 1 - CJ1)
* **Working Title:** Precise Motor Control and Real-Time Monitoring System
* **Registration Title:** Dual Motor Control and Real Time Monitoring
* **Team Members:** Luciano Carricart, Georgii Molyboga, Stephen Onyewuenyi
* **Supervisor:** Prof. Dr. Benno Radt
* **Date:** 19 October 2026

---

## 1. Abstract & Scope

This project delivers an embedded dual motor control and real-time telemetry system for a differential-drive electric rover. Powered by an ARM Cortex-M microcontroller (STM32 Nucleo), the system independently regulates two DC gearmotors with quadrature encoder feedback, executes closed-loop speed control, displays real-time angular position and RPM metrics on an SPI graphical screen, and supports remote wireless operation over Bluetooth Low Energy (BLE).

---

## 2. System Architecture & Hardware Setup

### 2.1 Block Diagram

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

### 2.2 Microcontroller Pinout & Peripheral Mapping (C Code Header)

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

---

## 3. Firmware Architecture & Algorithms

### 3.1 Kinematics & Telemetry Equations

Given decoded encoder resolution $N$ (counts per revolution):

$$\text{Relative Angle } (^\circ) = 360 \times \frac{\text{count} - \text{zero\_count}}{N}$$

$$\text{Speed } (\text{RPM}) = 60 \times \frac{\Delta\text{count}}{N \times \Delta t}$$

### 3.2 Finite State Machine (C Code Structure)

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

## 4. Use Case Specifications

### 4.1 Use Case Diagram (Mermaid)

```mermaid
flowchart TD
    actor Operator as "Rover Operator"
    actor PhoneApp as "BLE Mobile App"
    actor EStop as "Physical E-Stop Switch"

    subgraph STM32 Motor Control System
        UC1("UC-1: Remote Drive Control")
        UC2("UC-2: Closed-Loop Speed Control")
        UC3("UC-3: Encoder Feedback & Position Tracking")
        UC4("UC-4: Local Telemetry Display")
        UC5("UC-5: Hardware Emergency Stop")
        UC6("UC-6: Command Lease Timeout Protection")
        UC7("UC-7: Physical Fallback Control Mode")
    end

    PhoneApp --> UC1
    Operator --> PhoneApp
    Operator --> EStop
    Operator --> UC7

    UC1 ..> UC2 : <<include>>
    UC2 ..> UC3 : <<include>>
    UC3 --> UC4
    EStop --> UC5
    UC6 ..> UC2 : <<forces stop>>
```

### 4.2 Detailed Use Case Specifications

#### Code Schema Definition (YAML Format)

```yaml
use_cases:
  - id: "UC-1"
    name: "Remote Drive Control"
    actor: "Phone BLE Application"
    preconditions: "System in IDLE or RUN state; BLE connection established."
    trigger: "Direction/speed command string received via BLE UART."
    main_flow:
      - "1. UART interrupt receives command packet and places payload into circular buffer."
      - "2. Main loop parses packet format, checksum, and target values."
      - "3. MCU updates Left/Right target RPMs and resets command lease timer."
      - "4. State transitions to RUN."
    exception_flow:
      - "If packet is corrupt or out-of-range, discard packet, maintain current target, and log error."

  - id: "UC-2"
    name: "Closed-Loop Speed Control"
    actor: "Internal 50 Hz Timer Interrupt"
    preconditions: "System in RUN state; motor driver enabled."
    trigger: "20 ms hardware timer interrupt fires."
    main_flow:
      - "1. Sample hardware encoder register counts for Left and Right wheels."
      - "2. Calculate delta count over elapsed 20 ms window to derive measured RPM."
      - "3. Compute speed error: e(t) = target_RPM - measured_RPM."
      - "4. Execute PI algorithm with anti-windup clamping."
      - "5. Output updated PWM duty cycles and direction pin signals."
    exception_flow:
      - "If control output exceeds saturation bounds, anti-windup accumulator clamp activates."

  - id: "UC-3"
    name: "Encoder Feedback & Position Tracking"
    actor: "Quadrature Encoders"
    preconditions: "Hardware timers configured in Encoder Mode (TIM2/TIM3)."
    trigger: "Rotation of motor shafts."
    main_flow:
      - "1. Hardware encoder interfaces decode A/B phase shifts without CPU load."
      - "2. Timer count registers increment or decrement dynamically."
      - "3. Calculate relative angle modulo 360 degrees and track cumulative revolutions."
    exception_flow:
      - "16-bit hardware counter register overflow/underflow managed automatically."

  - id: "UC-4"
    name: "Local Telemetry Display"
    actor: "SPI Display Controller"
    preconditions: "SPI bus initialized; system active."
    trigger: "Main loop execution tick (>= 10 Hz refresh rate)."
    main_flow:
      - "1. Read coherent snapshot of telemetry data (RPM, angle, state, data age)."
      - "2. Render dual wheel circular gauges with direction pointers."
      - "3. Display numerical speed values and communication status indicator."
    exception_flow:
      - "If display transmission stalls, execution continues without blocking 50 Hz control interrupt."

  - id: "UC-5"
    name: "Hardware Emergency Stop"
    actor: "Physical E-Stop Switch / Operator"
    preconditions: "Rover system powered."
    trigger: "Operator toggles physical E-Stop switch."
    main_flow:
      - "1. Hardware line directly disables H-Bridge driver outputs in < 100 ms."
      - "2. EXTI line signals MCU interrupt; system state jumps to FAULT."
      - "3. PWM generators forced to 0% duty cycle."
    exception_flow:
      - "Releasing E-stop switch alone will not resume movement; requires full hardware/software reset."

  - id: "UC-6"
    name: "Command Lease Timeout Protection"
    actor: "Internal Watchdog / Lease Timer"
    preconditions: "System running in RUN state."
    trigger: "Time since last valid BLE command exceeds 2.0 seconds."
    main_flow:
      - "1. MCU clears target RPM setpoints to 0."
      - "2. State transitions to STOPPING, waiting for speed to drop < 3 RPM."
      - "3. Transitions to IDLE/FAULT; driver outputs disabled."
    exception_flow:
      - "Re-establishing BLE connection alone does not resume motion; fresh RUN command required."

  - id: "UC-7"
    name: "Physical Fallback Control Mode"
    actor: "Rover Operator"
    preconditions: "BLE connection offline; Fallback mode active (Supervisor approved)."
    trigger: "Operator adjusts onboard speed/direction controls."
    main_flow:
      - "1. MCU samples onboard potentiometers (ADC) / directional switches."
      - "2. Sets target RPM values within 100 ms of input adjustment."
      - "3. Standard closed-loop speed regulation applies."
    exception_flow:
      - "If analog sensor reading is out of range, set target speed to zero."
```

#### Specification Summary Matrix

| UC ID | Name | Primary Actor | Trigger | Core Metric / Target | Primary Fallback |
|---|---|---|---|---|---|
| **UC-1** | Remote Drive Control | Phone BLE App | BLE Packet Received | Response time $< 1.0\text{ s}$ | Discard malformed packet |
| **UC-2** | Closed-Loop Speed Control | 50 Hz Interrupt | 20 ms Timer Expiry | Execution period $20\text{ ms} \pm 2\text{ ms}$ | Anti-windup clamping |
| **UC-3** | Encoder Feedback & Angle | Encoders | Shaft Rotation | Count accuracy $\pm 2\%$ | Hardware 16-bit auto-wrap |
| **UC-4** | Local Telemetry Display | SPI Display | Main Loop ($\ge 10\text{ Hz}$) | Telemetry age $\le 200\text{ ms}$ | Non-blocking SPI transfer |
| **UC-5** | Hardware Emergency Stop | Physical Switch | Button Press | Disable time $\le 100\text{ ms}$ | Direct hardware disable |
| **UC-6** | Command Lease Timeout | System Lease Timer | Idle elapsed $> 2.0\text{ s}$ | Disable outputs by $2.1\text{ s}$ | Manual re-initiation required |
| **UC-7** | Physical Fallback Mode | Operator | Control Adjustment | Response time $\le 100\text{ ms}$ | Default target to 0 RPM |

---

## 5. Acceptance Test Protocols (M1–M10)

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

## 6. Work Breakdown & 12-Week Plan

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