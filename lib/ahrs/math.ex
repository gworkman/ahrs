defmodule Ahrs.Math do
  @moduledoc """
  Stateless mathematical utilities for unit conversions, Euler angles, and tilt calculations.
  """

  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Quaternion, as: Q

  @gravity 9.80665

  @doc """
  Converts a sensor sample to the specified units.
  """
  @spec convert(Accel.t() | Gyro.t(), atom()) :: Accel.t() | Gyro.t()
  def convert(%Accel{units: units} = sample, units), do: sample

  def convert(%Accel{x: x, y: y, z: z, units: :g}, :m_s2) do
    %Accel{x: x * @gravity, y: y * @gravity, z: z * @gravity, units: :m_s2}
  end

  def convert(%Accel{x: x, y: y, z: z, units: :m_s2}, :g) do
    %Accel{x: x / @gravity, y: y / @gravity, z: z / @gravity, units: :g}
  end

  def convert(%Gyro{units: units} = sample, units), do: sample

  def convert(%Gyro{x: x, y: y, z: z, units: :deg_s}, :rad_s) do
    factor = :math.pi() / 180.0
    %Gyro{x: x * factor, y: y * factor, z: z * factor, units: :rad_s}
  end

  def convert(%Gyro{x: x, y: y, z: z, units: :rad_s}, :deg_s) do
    factor = 180.0 / :math.pi()
    %Gyro{x: x * factor, y: y * factor, z: z * factor, units: :deg_s}
  end

  @doc """
  Converts a quaternion orientation into Euler angles (radians).
  Returns a tuple: {roll, pitch, yaw}
  Uses the standard Z-Y-X Tait-Bryan convention.
  """
  @spec quaternion_to_euler(Q.t()) :: {roll :: float(), pitch :: float(), yaw :: float()}
  def quaternion_to_euler(%Q{w: w, x: x, y: y, z: z}) do
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)

    if abs(sinp) >= 0.99999 do
      # Gimbal lock (Pitch = +/- 90 degrees)
      pitch = if sinp < 0, do: -:math.pi() / 2, else: :math.pi() / 2
      # In gimbal lock, yaw and roll are combined. We set yaw to 0.
      yaw = 0.0
      roll = normalize_angle(2.0 * :math.atan2(x, w))
      {roll, pitch, yaw}
    else
      # Roll (x-axis rotation)
      sinr_cosp = 2.0 * (w * x + y * z)
      cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
      roll = :math.atan2(sinr_cosp, cosr_cosp)

      # Clamp sinp to handle potential float precision issues
      pitch = :math.asin(max(-1.0, min(1.0, sinp)))

      # Yaw (z-axis rotation)
      siny_cosp = 2.0 * (w * z + x * y)
      cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
      yaw = :math.atan2(siny_cosp, cosy_cosp)

      {roll, pitch, yaw}
    end
  end

  @doc """
  Calculates pitch and roll (radians) directly from an accelerometer sample using trigonometry.
  Note: This calculation is susceptible to linear acceleration and cannot determine yaw.
  Returns a tuple: {roll, pitch}
  """
  @spec accel_to_tilt(Accel.t()) :: {roll :: float(), pitch :: float()}
  def accel_to_tilt(%Accel{x: ax, y: ay, z: az}) do
    case :math.sqrt(ax * ax + ay * ay + az * az) do
      norm when norm == 0.0 ->
        # Free-fall or zero reading: Orientation is unknown, return zeroed angles
        {0.0, 0.0}

      _norm ->
        roll = :math.atan2(ay, az)
        pitch = :math.atan2(-ax, :math.sqrt(ay * ay + az * az))
        {roll, pitch}
    end
  end

  # Normalizes an angle to the range (-PI, PI]
  defp normalize_angle(angle) do
    two_pi = 2.0 * :math.pi()
    a = :math.fmod(angle + :math.pi(), two_pi)
    if a <= 0, do: a + :math.pi(), else: a - :math.pi()
  end
end
