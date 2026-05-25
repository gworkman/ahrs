# Ahrs

Ahrs is an Elixir library that provides common automatic heading reference
system algorithms for processing inertial measurement unit (IMU) data. It is
designed to be modular and robust, providing strongly-typed sensor containers
and internal math utilities that handle unit conversions and 3D orientation
tracking using quaternions.

The library currently focuses on the Madgwick filter, which is a computationally
efficient gradient descent algorithm used to determine the orientation of a
device in 3D space by combining accelerometer and gyroscope readings.

## Usage

To use the library, you start by initializing the state for your chosen
algorithm. Each algorithm provides a struct that tracks the current orientation
and the timing of the last update.

```elixir
alias Ahrs.Madgwick
alias Ahrs.Accelerometer.Sample, as: Accel
alias Ahrs.Gyroscope.Sample, as: Gyro

# Initialize the filter state
state = %Madgwick{}

# Create sensor samples from your hardware readings
measurements = %{
  accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
  gyro: %Gyro{x: 0.01, y: 0.0, z: 0.0, units: :rad_s}
}

# Update the filter state.
# The library automatically calculates the time delta (dt) between calls.
state = Madgwick.update(state, measurements)

# You can convert the internal quaternion state into human-readable Euler angles.
{roll, pitch, yaw} = Ahrs.Math.quaternion_to_euler(state.q)
```

## Timing and Automatic Delta Tracking

One of the challenges in AHRS systems is accurately measuring the time elapsed
between sensor updates. This library simplifies this by automatically querying
the system monotonic clock during the `update/3` call. The state struct stores
the timestamp of the last update and uses it to calculate the delta time in
seconds for the next step.

If you are processing historical data or want to manage timing yourself, you can
override this behavior by passing an explicit `:dt` option in seconds.

```elixir
state = Madgwick.update(state, measurements, dt: 0.01)
```

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
