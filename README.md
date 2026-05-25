# Ahrs

Ahrs is an Elixir library that provides common automatic heading reference
system algorithms for processing inertial measurement unit (IMU) data. It is
designed to be modular and robust, providing strongly-typed sensor containers
and internal math utilities that handle unit conversions and 3D orientation
tracking using quaternions.

The library currently provides three primary filters:

*   **Madgwick:** A highly efficient gradient descent algorithm that is computationally inexpensive and well-suited for high-speed updates.
*   **Mahony:** A robust Proportional-Integral (PI) controller filter. It is stateful (tracks integral error) and is often preferred for its stability on low-power hardware.
*   **Complementary Filter:** A lightweight filter that combines high-pass integrated gyroscope data with a low-pass accelerometer tilt calculation. It is simple to tune and very computationally cheap.

## Usage

To use the library, you start by initializing an AHRS instance using the top-level `Ahrs` module. This provides a unified API regardless of the underlying algorithm you choose.

```elixir
alias Ahrs.Accelerometer.Sample, as: Accel
alias Ahrs.Gyroscope.Sample, as: Gyro

# Initialize the filter (Ahrs.new_madgwick(), Ahrs.new_mahony(), or Ahrs.new_complementary())
ahrs = Ahrs.new_madgwick()

# Create sensor samples from your hardware readings
measurements = %{
  accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
  gyro: %Gyro{x: 0.01, y: 0.0, z: 0.0, units: :rad_s}
}

# Update the filter state.
# The library automatically calculates the time delta (dt) between calls.
ahrs = Ahrs.update(ahrs, measurements)

# You can convert the internal state into human-readable Euler angles.
# Supports both :radians (default) and :degrees.
{roll, pitch, yaw} = Ahrs.euler_angles(ahrs, units: :degrees)
```

## Timing and Automatic Delta Tracking

One of the challenges in AHRS systems is accurately measuring the time elapsed between sensor updates. This library simplifies this by automatically querying the system monotonic clock during the `Ahrs.update/3` call. The internal state stores the timestamp of the last update and uses it to calculate the delta time in seconds for the next step.

If you are processing historical data or want to manage timing yourself, you can override this behavior by passing an explicit `:dt` option in seconds.

```elixir
ahrs = Ahrs.update(ahrs, measurements, dt: 0.01)
```

## Configuration and Tuning

Each algorithm accepts specific options to tune its behavior:

*   **Madgwick:** Accepts `:beta` (default `0.1`), which controls the feedback gain.
*   **Mahony:** Accepts `:kp` (proportional gain, default `2.0`) and `:ki` (integral gain, default `0.0`). It also supports `:e_int_limit` (default `100.0`) to prevent integral windup.
*   **Complementary Filter:** Accepts `:alpha` (default `0.98`), which determines the weighting between the gyroscope (alpha) and the accelerometer (1 - alpha).

All filters also support an `:accel_threshold` (default `0.1` G) to ignore noisy accelerometer corrections during near-free-fall conditions.


## Integration Tips

When integrating this library with live hardware, it is important to ensure that
your sensors are properly calibrated. While the filters can compensate for some
noise, they assume that your accelerometer readings are centered around 1G and
that your gyroscope has its static offset (bias) removed.

The `beta` parameter in the Madgwick algorithm controls the balance between the
gyroscope and the accelerometer. A higher value will cause the filter to respond
more quickly to changes detected by the accelerometer, but it may also introduce
jitter if the sensor is noisy. A default value of 0.1 is usually a good starting
point for most IMU hardware.

The library is designed to be process-agnostic. You can maintain the filter
state in a long-running `GenServer` or pass it between different parts of your
pipeline. Because the timing is stored within the state struct itself, the
calculations remain accurate even if you move the state between processes.
