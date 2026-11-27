# Overview

Selecting the appropriate DC geared motors and H-bridge motor drivers for a four-wheel rover requires matching mechanical load requirements, electrical ratings, and microcontroller interface constraints. This guide outlines selection criteria aligned with the project's functional and non-functional requirements.

## 1. Selecting the Geared DC Motor & Encoder

### 1.1 No-Load Speed (The 70% Rule)

* **Requirement (`FR-1`):** Maximum target wheel speed is assumed to be **90 RPM**.
* To ensure the PI controller retains sufficient headroom to regulate speed without saturating the PWM duty cycle, the 90 RPM target should be roughly **70% or less** of the motor's no-load RPM at your actual battery pack voltage. (Meaning an actual RPM of >~130–160 RPM is ideal.)

### 1.2 Encoder Resolution ($N$ Counts per Wheel Spin)

* **Requirements (`FR-2`, `FR-3`, `NFR-2`, `NFR-7`):**
  * Control loop runs at **50 Hz** ($20\text{ ms}$ period).
  * Angular accuracy within $\pm 10^\circ$; speed accuracy within $\pm 5\text{ RPM}$.
  * Rest state verified when wheel speed is $< 3\text{ RPM}$ **for** $300\text{ ms}$.
* **Approximation:** Using quadrature decoding ($\times 4$, counting both edges on channels A and B), an encoder resolution around **360 counts/rev** gives $1^\circ$ angular steps, but only ~8 RPM speed granularity per $20\text{ ms}$ loop—too coarse for the $\pm 5\text{ RPM}$ limit. (Aim for **$N \ge 1,200$ counts/rev** - ideally **$1,300\text{--}3,840$ counts/rev**)

### 1.3 Gearbox Type & Backlash

* **Requirements (`FR-9`, `M2`, `M3`):** Wheels must be back-drivable by hand when the driver is disabled, and angular error must stay within $10^\circ$.
* **Target Specification:** Use **spur or planetary gearboxes** (never worm gears, which are non-back-drivable). Metal gears are strongly preferred over plastic gears because output shaft backlash adds directly to the $10^\circ$ position error budget (a motor-shaft encoder cannot observe gearbox play).

### 1.4 Logic Voltage Compatibility (3.3 V & 5 V)

* **Requirement (`FR-10`):** Safe interfacing with all 8 encoder lines on the STM32 microcontroller.
* **Target Specification:**
  * **3.3 V Encoders:** Power directly from the 3.3 V rail; signals connect directly to any STM32 GPIO without level shifting.
  * **5 V Encoders:** Power from a regulated **5 V** rail (never 6 V+) and verify in the STM32C562RE datasheet that all 8 assigned timer pins (`PA0`, `PA1`, `PA8`, `PA9`, `PB4`, `PB5`, `PC6`, `PC7`) are **5 V-tolerant (`FT`)**, or add external level shifters.

## 2. Selecting the Motor Controller (H-Bridge Driver)

Because all four wheels require independent bidirectional velocity control, the system requires **two dual-channel H-bridge modules** (or one quad-channel board). Evaluate drivers against five criteria:

### 2.1 Continuous vs. Peak (Stall) Current Rating

* **Requirements (`NFR-6`, `NFR-8`):** Drivers must sustain continuous running current across all four wheels and survive stall/reversal transients without thermal shutdown or over-current trips.
* **Selection Criteria:**
  * **Continuous rating per channel** must exceed the loaded running current during skid-steer turns (typically **$\sim 0.5\text{--}1.5\text{ A}$** depending on motor choice).
  * **Peak rating per channel** should exceed the motor's stall current.

### 2.2 Hardware Disable / Enable Pin (E-Stop)

* **Requirements (`FR-5`, `NFR-4`, `M9`, `M10`):** A physical Emergency Stop (E-Stop) button must cut motor power in hardware even if software crashes, and floating MCU pins during reset/watchdog reboot must keep motors off.
* **Target Specification:** The driver **must** feature a dedicated hardware disable/standby pin (`STBY`, `nSLEEP`, or `D1/D2`) wired with an **external pull-down resistor to GND** (default-off) and routed through a **normally-closed (NC)** latching E-Stop switch.

### 2.3 Control Interface

* **Requirement (`FR-8`):** Current pin definitions allocate **1 PWM + 1 DIR** pin per motor plus 1 shared disable pin.
* **Remark:** Most compact dual H-bridge modules (**TB6612FNG**, **DRV8833**, **MC33926**) use an **IN1 / IN2** direction interface (2 direction pins per motor, or PWM applied directly to one IN pin). Adopting these modules requires either allocating 4 additional GPIO pins in `hardware_config.h` or adding external hardware inverters.

### 2.4 Battery Supply Voltage Window

* **Requirement (`NFR-8`):** Battery voltage under full four-motor load must remain within the operating limits of both the motors and drivers.
* **Selection Criteria:** Check the driver's minimum under-voltage lockout (UVLO) threshold. 

## 3. Suggested Hardware Pairings

| Configuration | Motors (4×) | Drivers (2× Dual) | Key Advantages | Key Trade-offs / Actions Required |
| ----- | ----- | ----- | ----- | ----- |
| **Option A: High Precision & Margin**| **6V JGB37-520**| **Pololu Dual MC33926** | Metal spur gearbox (low backlash for $\le 10^\circ$ accuracy); 3.3 V native encoder logic; $N \approx 1,320$; $3\text{ A}$ cont. / $5\text{ A}$ peak handles $\sim 1.7\text{ A}$ stall easily. | Requires $6\text{ mm}$ shaft hubs for wheels; heavier; ensure battery pack stays $> 5.0\text{ V}$ under load; update `hardware_config.h` for `IN1/IN2`. |
| **Option B: Budget & Mechanical Ease** | **DFRobot FIT0450** | **TB6612FNG** | Directly fits standard $65\text{ mm}$ TT wheels without hubs; very high encoder resolution ($N \approx 3,840$ in $\times 4$ mode); low cost. | Plastic gearbox backlash reduces angle margin; $\sim 2.8\text{ A}$ stall exceeds TB6612FNG continuous rating; power encoder at $5\text{ V}$ and confirm `FT` pin tolerance on STM32C562RE. |