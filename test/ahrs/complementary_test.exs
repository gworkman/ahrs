defmodule Ahrs.ComplementaryTest do
  use ExUnit.Case, async: true
  alias Ahrs.Complementary
  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Quaternion, as: Q
  alias Ahrs.Math

  describe "update/3" do
    test "initial run sets last_update_at" do
      state = %Complementary{}
      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      new_state = Complementary.update(state, measurements)
      assert is_integer(new_state.last_update_at)
    end

    test "converges toward accelerometer tilt" do
      initial_state = %Complementary{q: %Q{w: 1.0, x: 0.0, y: 0.0, z: 0.0}, last_update_at: 0}
      
      measurements = %{
        accel: %Accel{x: -1.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      final_state =
        Enum.reduce(1..500, initial_state, fn _i, state ->
          Complementary.update(state, measurements, dt: 0.01, alpha: 0.95)
        end)

      {_roll, pitch, _yaw} = Math.quaternion_to_euler(final_state.q)
      assert_in_delta pitch, :math.pi() / 2.0, 0.1
    end

    test "preserves yaw during accelerometer correction (Fixes Yaw Leakage)" do
      # Initial state with 90 degree Yaw (z-axis rotation)
      initial_q = Math.euler_to_quaternion(0.0, 0.0, :math.pi() / 2.0)
      initial_state = %Complementary{q: initial_q, last_update_at: 0}

      # Measurement: Device is flat, zero rotation
      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      # Run multiple update steps
      final_state =
        Enum.reduce(1..10, initial_state, fn _i, state ->
          Complementary.update(state, measurements, dt: 0.1)
        end)

      {_roll, _pitch, yaw} = Math.quaternion_to_euler(final_state.q)

      # Yaw should still be 90 degrees
      assert_in_delta yaw, :math.pi() / 2.0, 1.0e-6
    end

    test "calculates alpha from time_constant" do
      initial_state = %Complementary{q: %Q{w: 1.0, x: 0.0, y: 0.0, z: 0.0}, last_update_at: 0}
      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      # If tc = 1.0 and dt = 1.0, alpha = 1.0 / (1.0 + 1.0) = 0.5
      # We verify this by feeding a 90 deg tilt and seeing it move halfway in one step
      measurements_tilt = %{measurements | accel: %Accel{x: -1.0, y: 0.0, z: 0.0, units: :g}}
      
      final_state = Complementary.update(initial_state, measurements_tilt, dt: 1.0, time_constant: 1.0)
      {_roll, pitch, _yaw} = Math.quaternion_to_euler(final_state.q)

      # 45 degrees is halfway to 90
      assert_in_delta pitch, :math.pi() / 4.0, 0.01
    end

    test "respects accel_threshold" do
      initial_state = %Complementary{q: %Q{w: 1.0, x: 0.0, y: 0.0, z: 0.0}, last_update_at: 0}
      
      # Acceleration below 0.1G threshold
      measurements = %{
        accel: %Accel{x: 0.05, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      final_state = Complementary.update(initial_state, measurements, dt: 0.1)
      assert final_state.q == initial_state.q
    end
  end
end
