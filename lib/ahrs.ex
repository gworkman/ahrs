defmodule Ahrs do
  @moduledoc """
  Unified API for Attitude and Heading Reference System (AHRS) algorithms.

  This module provides an algorithm-agnostic wrapper around various AHRS filters
  like Madgwick and Mahony. By using this module, applications can switch between
  underlying filter implementations without changing their sensor integration logic.
  """

  alias Ahrs.Quaternion, as: Q

  defstruct [:algorithm, :state]

  @type t :: %__MODULE__{
          algorithm: module(),
          state: struct()
        }

  @doc """
  Initializes a new Madgwick filter instance.

  ## Options
    * `:initial_q` - An explicit starting `Ahrs.Quaternion`.
    * `:initial_accel` - An `Ahrs.Accelerometer.Sample`. The library will use this to
      calculate an immediate starting tilt, eliminating initial convergence delay.
  """
  def new_madgwick(opts \\ []) do
    q = extract_initial_q(opts)
    %__MODULE__{algorithm: Ahrs.Madgwick, state: %Ahrs.Madgwick{q: q}}
  end

  @doc """
  Initializes a new Mahony filter instance.

  ## Options
    * `:initial_q` - An explicit starting `Ahrs.Quaternion`.
    * `:initial_accel` - An `Ahrs.Accelerometer.Sample`. The library will use this to
      calculate an immediate starting tilt, eliminating initial convergence delay.
  """
  def new_mahony(opts \\ []) do
    q = extract_initial_q(opts)
    %__MODULE__{algorithm: Ahrs.Mahony, state: %Ahrs.Mahony{q: q}}
  end

  @doc """
  Initializes a new Complementary filter instance.

  ## Options
    * `:initial_q` - An explicit starting `Ahrs.Quaternion`.
    * `:initial_accel` - An `Ahrs.Accelerometer.Sample`. The library will use this to
      calculate an immediate starting tilt, eliminating initial convergence delay.
  """
  def new_complementary(opts \\ []) do
    q = extract_initial_q(opts)
    %__MODULE__{algorithm: Ahrs.Complementary, state: %Ahrs.Complementary{q: q}}
  end

  defp extract_initial_q(opts) do
    cond do
      q = opts[:initial_q] ->
        q

      accel = opts[:initial_accel] ->
        {roll, pitch} = Ahrs.Math.accel_to_tilt(accel)
        Ahrs.Math.euler_to_quaternion(roll, pitch, 0.0)

      true ->
        %Q{}
    end
  end

  @doc """
  Updates the filter state based on new sensor measurements.

  ## Options
    * `:dt` - Explicit delta time in seconds. If omitted, the library automatically
      calculates the delta using system monotonic time.
    * `:beta` - Filter gain (Madgwick only, default 0.1).
    * `:kp` - Proportional gain (Mahony only, default 2.0).
    * `:ki` - Integral gain (Mahony only, default 0.0).
    * `:alpha` - Fixed gyroscope weight (Complementary only, default 0.98).
    * `:time_constant` - Time constant (tau) in seconds (Complementary only). If provided,
      overrides `:alpha` with a frequency-independent calculation.
    * `:accel_threshold` - Acceptable deviation from 1G for accelerometer readings (default 0.1).
    * `:e_int_limit` - Integral error clamping limit (Mahony only, default 100.0).
  """
  def update(%__MODULE__{algorithm: algo, state: state} = ahrs, measurements, opts \\ []) do
    new_state = algo.update(state, measurements, opts)
    %__MODULE__{ahrs | state: new_state}
  end

  @doc """
  Returns the current orientation quaternion from the filter state.
  """
  def quaternion(%__MODULE__{state: %{q: q}}), do: q

  @doc """
  Converts the current filter state into Euler angles.

  ## Options
    * `:units` - can be `:radians` (default) or `:degrees`.

  Returns a `{roll, pitch, yaw}` tuple.
  """
  def euler_angles(%__MODULE__{state: %{q: q}}, opts \\ []) do
    Ahrs.Math.quaternion_to_euler(q, opts)
  end
end
