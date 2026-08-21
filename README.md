# azrael_base_driver

Low-level motor/encoder driver for a 4-wheel omni/mecanum mobile base. It runs standalone on a Raspberry Pi, closes a per-wheel velocity PID loop against Phidget encoders, drives motor PWM/direction over GPIO, and exchanges velocity commands / telemetry with a remote host over UDP.

This is a plain CMake / C++17 project with **no ROS or ROS2 dependency** — it can live inside a ROS workspace `src/` directory for convenience, but it builds and runs independently of `catkin`/`ament`.

## What it does

- Reads 4 quadrature encoders (Phidget22) and low-pass filters (2nd-order Butterworth) each wheel's angular velocity.
- Receives a body-frame velocity command `[vx, vy, w]` over UDP, applies an acceleration bound, converts it to 4 wheel target velocities via omni-wheel inverse kinematics, and tracks each with an independent PID controller.
- Drives each wheel's motor via software PWM (`wiringPi`) for duty cycle and a GPIO pin for direction.
- Sends wheel velocities and commanded PWM duty cycles back over UDP every ~20 ms.
- Runs the control loop on a `SCHED_FIFO` real-time thread with locked memory (`mlockall`) for deterministic timing.

## Hardware / OS assumptions

- Raspberry Pi (compiled for Cortex-A72, e.g. Pi 4/CM4) — see `-mcpu=cortex-a72 -mtune=cortex-a72` in `CMakeLists.txt`.
- GPIO access via `wiringPi` (must be run with sufficient privileges for GPIO + `SCHED_FIFO` + `mlockall`, typically as root).
- 4 Phidget encoders (channels 0-3) connected via a Phidget22 interface.

## Dependencies

- [`wiringPi`](http://wiringpi.com/) — GPIO/software PWM
- [`phidget22`](https://www.phidgets.com/docs/Phidget22_API) — encoder driver
- [`iir1`](https://github.com/berndporr/iir1) — Butterworth low-pass filter (`find_package(iir)`)
- Boost (`system`, `asio`) — UDP sockets
- POSIX threads (`pthread`), `rt`

## Building

```bash
mkdir build && cd build
cmake ..
make
```

Produces the `azrael_mobile_driver` executable.

## Running

Must run with privileges sufficient for GPIO access, real-time scheduling, and locked memory (typically root):

```bash
sudo ./azrael_mobile_driver
```

Stop with `Ctrl+C` — the `SIGINT` handler zeroes all PWM outputs before exiting.

## Configuration

Hardware pin mapping and motion-profile constants live in `include/pin_config.h`:

| Constant | Meaning |
|---|---|
| `PWM_pin_1..4` / `PWM_dir_1..4` | Software-PWM and direction GPIO pins per wheel |
| `MAX_PWM_RANGE` | Software PWM resolution (0-100) |
| `CONTROL_LOOP_DT` | Nominal control loop period, seconds (500 Hz) |
| `MAX_LIN_ACCEL` | Max linear acceleration applied to `vx`/`vy` commands, m/s² |
| `MAX_ANG_ACCEL` | Max angular acceleration applied to the `w` command, rad/s² |

Robot geometry and network endpoints live in `include/driver.h`:

| Constant | Meaning |
|---|---|
| `radius` | Wheel radius, m |
| `lxy` | Kinematic wheelbase term used in the omni inverse/forward kinematics |
| `IPADDRESS_LOCAL` / `IPADDRESS_REMOTE` | UDP bind / peer addresses |
| `UDP_PORT` | UDP port (both directions) |

Per-wheel PID gains are set where each `PID` is constructed in `azrael_mobile_driver` (`include/driver.h`): `PID(kp, ki, kd, Ts, &vel_measured, &lock)`. A feed-forward term `ffw` (in `include/pid.h`) is added on top of the PID output.

## UDP protocol

Two independent UDP streams over the same socket pair (`IPADDRESS_LOCAL:UDP_PORT` ⇄ `IPADDRESS_REMOTE:UDP_PORT`), each message a fixed-size array of IEEE-754 doubles, native byte order:

**Command (received, 3 doubles):**

```
[vx, vy, w]
```

Body-frame linear velocity (m/s) and angular velocity (rad/s). Commands are **not** applied directly — each control cycle the driver ramps its internal target toward the latest received command by at most `MAX_LIN_ACCEL/MAX_ANG_ACCEL * CONTROL_LOOP_DT`, bounding acceleration regardless of how abruptly the command changes.

**Telemetry (sent, 8 doubles):**

```
[vel1, vel2, vel3, vel4, pwm1, pwm2, pwm3, pwm4]
```

- `vel1..vel4` — measured wheel angular velocity, rad/s (raw encoder-derived value, unfiltered).
- `pwm1..pwm4` — actual commanded PWM duty cycle per wheel, as a fraction in `[0, 1]` (direction is not encoded here; it's driven separately on the `PWM_dir_X` pins).

## Known limitations

- `odometry()` computes `vx/vy/w` and integrated pose from the encoders but its thread is not started in `main()` — it's present but inactive.
- No watchdog on the UDP command stream: if packets stop arriving, the last received command keeps being tracked (the acceleration bound will ramp it down only if a zero command is explicitly sent).
- UDP peer addresses and port are compile-time constants, not runtime-configurable.
