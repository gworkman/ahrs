Mix.install([
  {:ahrs, path: Path.expand("..", __DIR__)},
  {:phoenix_playground, "~> 0.1"}
])

defmodule Simulator.Live do
  use Phoenix.LiveView
  alias Ahrs.Quaternion, as: Q
  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Math

  @tick_ms 20
  @dt 0.02

  def mount(_params, _session, socket) do
    if connected?(socket) do
      :timer.send_interval(@tick_ms, self(), :tick)
    end

    socket =
      socket
      |> assign(:true_q, %Q{})
      |> assign(:ahrs, Ahrs.new_madgwick())
      |> assign(:angular_velocity, {0.0, 0.0, 0.0})
      |> assign(:rotation_speed, 1.5)
      |> assign(:gyro_noise, 0.02)
      |> assign(:accel_noise, 0.05)
      |> assign(:filter_type, "madgwick")
      |> assign(:active_keys, MapSet.new())
      |> assign(:true_euler, {0.0, 0.0, 0.0})
      |> assign(:filter_euler, {0.0, 0.0, 0.0})

    {:ok, socket}
  end

  def handle_info(:tick, socket) do
    {gx, gy, gz} = calculate_velocity(socket.assigns.active_keys, socket.assigns.rotation_speed)

    # 1. Integrate True orientation (perfect world)
    {dw, dx, dy, dz} = Q.gyro_derivative(socket.assigns.true_q, gx, gy, gz)

    true_q =
      %Q{
        w: socket.assigns.true_q.w + dw * @dt,
        x: socket.assigns.true_q.x + dx * @dt,
        y: socket.assigns.true_q.y + dy * @dt,
        z: socket.assigns.true_q.z + dz * @dt
      }
      |> Q.normalize()

    # 2. Generate Noisy Sensors
    gyro_noise = socket.assigns.gyro_noise
    accel_noise = socket.assigns.accel_noise

    gyro_reading = %Gyro{
      x: gx + random_noise(gyro_noise),
      y: gy + random_noise(gyro_noise),
      z: gz + random_noise(gyro_noise),
      units: :rad_s
    }

    {ax, ay, az} = Math.rotate_vector({0, 0, 1.0}, Q.conjugate(true_q))
    accel_reading = %Accel{
      x: ax + random_noise(accel_noise),
      y: ay + random_noise(accel_noise),
      z: az + random_noise(accel_noise),
      units: :g
    }

    # 3. Update Filter
    ahrs = Ahrs.update(socket.assigns.ahrs, %{accel: accel_reading, gyro: gyro_reading}, dt: @dt)

    # 4. Broadcast orientations to JS
    filter_q = Ahrs.quaternion(ahrs)
    
    true_euler = Math.quaternion_to_euler(true_q, units: :degrees)
    filter_euler = Ahrs.euler_angles(ahrs, units: :degrees)

    socket =
      socket
      |> assign(:true_q, true_q)
      |> assign(:ahrs, ahrs)
      |> assign(:true_euler, true_euler)
      |> assign(:filter_euler, filter_euler)
      |> push_event("update_truth", Map.from_struct(true_q))
      |> push_event("update_filter", Map.from_struct(filter_q))

    {:noreply, socket}
  end

  def handle_event("visualizer_ready", _params, socket) do
    {:noreply, push_event(socket, "load_obj", %{data: File.read!("examples/utah_teapot.obj")})}
  end

  def handle_event("keydown", %{"key" => key}, socket) do
    {:noreply, assign(socket, :active_keys, MapSet.put(socket.assigns.active_keys, key))}
  end

  def handle_event("keyup", %{"key" => key}, socket) do
    {:noreply, assign(socket, :active_keys, MapSet.delete(socket.assigns.active_keys, key))}
  end

  def handle_event("change_filter", %{"filter" => type}, socket) do
    ahrs = case type do
      "madgwick" -> Ahrs.new_madgwick()
      "mahony" -> Ahrs.new_mahony()
      "complementary" -> Ahrs.new_complementary()
    end
    {:noreply, assign(socket, ahrs: ahrs, filter_type: type)}
  end

  def handle_event("change_gyro_noise", %{"noise" => val}, socket) do
    {:noreply, assign(socket, gyro_noise: parse_float(val))}
  end

  def handle_event("change_accel_noise", %{"noise" => val}, socket) do
    {:noreply, assign(socket, accel_noise: parse_float(val))}
  end

  def handle_event("change_speed", %{"speed" => val}, socket) do
    {:noreply, assign(socket, rotation_speed: parse_float(val))}
  end

  defp parse_float(val) do
    case Float.parse(val) do
      {f, _} -> f
      :error -> 0.0
    end
  end

  defp random_noise(level), do: (:rand.uniform() - 0.5) * level

  defp calculate_velocity(keys, speed) do
    gx = (if MapSet.member?(keys, "a"), do: -speed, else: 0.0) + (if MapSet.member?(keys, "d"), do: speed, else: 0.0)
    gy = (if MapSet.member?(keys, "w"), do: speed, else: 0.0) + (if MapSet.member?(keys, "s"), do: -speed, else: 0.0)
    gz = (if MapSet.member?(keys, "q"), do: -speed, else: 0.0) + (if MapSet.member?(keys, "e"), do: speed, else: 0.0)
    {gx, gy, gz}
  end

  def render(assigns) do
    ~H"""
    <script src="https://cdn.tailwindcss.com"></script>
    <div class="flex flex-row w-full h-screen font-sans bg-black overflow-hidden">
      <!-- Sidebar Controls -->
      <div class="w-80 h-screen space-y-6 bg-gray-800 p-8 shadow-2xl border-r border-white/10 text-white overflow-y-auto no-scrollbar shrink-0">
        <h1 class="text-xl font-black text-blue-500 tracking-tight border-b border-white/10 pb-4 uppercase">AHRS Simulator</h1>
        
        <div class="space-y-2">
          <label class="block text-[10px] font-bold text-gray-400 uppercase tracking-widest">Algorithm</label>
          <form phx-change="change_filter">
            <select name="filter" class="w-full bg-gray-700 border-none rounded-lg p-3 text-sm font-medium focus:ring-4 focus:ring-blue-500/50 appearance-none cursor-pointer text-white">
              <option value="madgwick" selected={@filter_type == "madgwick"}>Madgwick (GD)</option>
              <option value="mahony" selected={@filter_type == "mahony"}>Mahony (PI)</option>
              <option value="complementary" selected={@filter_type == "complementary"}>Complementary</option>
            </select>
          </form>
        </div>

        <div class="space-y-2">
          <label class="block text-[10px] font-bold text-gray-400 uppercase tracking-widest flex justify-between">
            Rotation Speed <span><%= @rotation_speed %> rad/s</span>
          </label>
          <form phx-change="change_speed">
            <input type="range" name="speed" min="0.1" max="5.0" step="0.1" value={@rotation_speed} class="w-full h-3 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-blue-500" />
          </form>
        </div>

        <div class="space-y-4 pt-4 border-t border-white/5">
          <h2 class="text-[10px] font-black text-gray-400 uppercase tracking-widest">Sensor Noise</h2>
          
          <div class="space-y-2">
            <label class="block text-[9px] font-bold text-gray-500 uppercase flex justify-between">
              Gyro (rad/s) <span><%= @gyro_noise %></span>
            </label>
            <form phx-change="change_gyro_noise">
              <input type="range" name="noise" min="0" max="0.5" step="0.005" value={@gyro_noise} class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-blue-500" />
            </form>
          </div>

          <div class="space-y-2">
            <label class="block text-[9px] font-bold text-gray-500 uppercase flex justify-between">
              Accel (G) <span><%= @accel_noise %></span>
            </label>
            <form phx-change="change_accel_noise">
              <input type="range" name="noise" min="0" max="0.5" step="0.005" value={@accel_noise} class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-blue-500" />
            </form>
          </div>
        </div>

        <div class="bg-blue-900/20 p-4 rounded-xl border border-blue-800/30 text-xs mt-auto">
          <p class="font-black text-blue-300 uppercase tracking-widest mb-3">Controls</p>
          <div class="space-y-2 text-gray-300">
            <div class="flex justify-between"><span>Pitch:</span> <b class="text-blue-400 font-mono">W/S</b></div>
            <div class="flex justify-between"><span>Roll:</span> <b class="text-blue-400 font-mono">A/D</b></div>
            <div class="flex justify-between"><span>Yaw:</span> <b class="text-blue-400 font-mono">Q/E</b></div>
          </div>
        </div>
      </div>

      <!-- Visualization Area -->
      <div class="flex-1 flex flex-row relative">
        <div class="flex-1 relative border-r border-white/10 group">
          <div id="visualizer-truth" phx-hook="ThreeJsView" data-type="truth" class="w-full h-full" phx-update="ignore"></div>
          <div class="absolute top-6 left-1/2 -translate-x-1/2 px-6 py-2 bg-white/5 backdrop-blur-xl rounded-full border border-white/10 pointer-events-none shadow-2xl">
            <span class="text-[11px] font-black text-white uppercase tracking-[0.3em]">Ground Truth</span>
          </div>
          <div class="absolute bottom-10 left-1/2 -translate-x-1/2 bg-black/60 backdrop-blur-xl px-8 py-4 rounded-3xl border border-white/10 shadow-2xl pointer-events-none">
            <.euler_display label="Perfect" values={@true_euler} />
          </div>
        </div>

        <div class="flex-1 relative group">
          <div id="visualizer-filter" phx-hook="ThreeJsView" data-type="filter" class="w-full h-full" phx-update="ignore"></div>
          <div class="absolute top-6 left-1/2 -translate-x-1/2 px-6 py-2 bg-blue-500/10 backdrop-blur-xl rounded-full border border-blue-500/20 pointer-events-none shadow-2xl">
            <span class="text-[11px] font-black text-blue-400 uppercase tracking-[0.3em]">Sensor Fusion</span>
          </div>
          <div class="absolute bottom-10 left-1/2 -translate-x-1/2 bg-blue-950/40 backdrop-blur-xl px-8 py-4 rounded-3xl border border-blue-500/20 shadow-2xl pointer-events-none">
            <.euler_display label={@filter_type} values={@filter_euler} color="text-blue-400" />
          </div>
        </div>
      </div>
    </div>

    <style>
      .no-scrollbar::-webkit-scrollbar { display: none; }
      .no-scrollbar { -ms-overflow-style: none; scrollbar-width: none; }
    </style>

    <script type="importmap">
    {
      "imports": {
        "three": "https://cdnjs.cloudflare.com/ajax/libs/three.js/0.160.0/three.module.min.js",
        "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
      }
    }
    </script>
    <script type="module">
      import * as THREE from 'three';
      import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

      window.hooks = window.hooks || {};
      window.hooks.ThreeJsView = {
        mounted() {
          const type = this.el.dataset.type;
          const container = this.el;
          const scene = new THREE.Scene();
          const camera = new THREE.PerspectiveCamera(40, container.clientWidth / container.clientHeight, 0.1, 1000);
          const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
          
          renderer.setSize(container.clientWidth, container.clientHeight);
          renderer.setPixelRatio(window.devicePixelRatio);
          container.appendChild(renderer.domElement);

          const axes = new THREE.AxesHelper(5);
          scene.add(axes);
          scene.add(new THREE.AmbientLight(0xffffff, 1.0));
          const dirLight = new THREE.DirectionalLight(0xffffff, 2);
          dirLight.position.set(10, 10, 10);
          scene.add(dirLight);

          let model = null;
          const loader = new OBJLoader();
          this.pushEvent("visualizer_ready", {});

          this.handleEvent("load_obj", (payload) => {
            if (model) scene.remove(model);
            model = loader.parse(payload.data);
            model.rotation.x = -Math.PI / 2;
            model.traverse((child) => {
              if (child.isMesh) {
                const color = (type === "truth") ? 0xffffff : 0x60a5fa;
                child.material = new THREE.MeshPhongMaterial({ color: color, shininess: 80, specular: 0x222222 });
              }
            });
            const box = new THREE.Box3().setFromObject(model);
            const size = box.getSize(new THREE.Vector3());
            const maxDim = Math.max(size.x, size.y, size.z);
            const scale = 5 / maxDim;
            model.scale.set(scale, scale, scale);
            scene.add(model);
          });

          camera.position.set(12, 12, 12);
          camera.lookAt(0, 0, 0);

          if (type === "truth") {
            window.addEventListener("keydown", (e) => { if(!e.repeat) this.pushEvent("keydown", {key: e.key.toLowerCase()}); });
            window.addEventListener("keyup", (e) => { this.pushEvent("keyup", {key: e.key.toLowerCase()}); });
          }

          const eventName = (type === "truth") ? "update_truth" : "update_filter";
          this.handleEvent(eventName, (q) => {
            if (model) model.quaternion.set(q.x, q.y, q.z, q.w);
          });

          const animate = () => { requestAnimationFrame(animate); renderer.render(scene, camera); };
          window.addEventListener('resize', () => {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
          });
          animate();
        }
      };
    </script>
    """
  end

  def euler_display(assigns) do
    assigns = assign_new(assigns, :color, fn -> "text-white" end)
    ~H"""
    <div class="flex flex-row gap-6 items-center">
      <div class="flex flex-col items-center">
        <span class="text-[8px] text-gray-500 uppercase font-black tracking-widest mb-0.5">Roll</span>
        <span class={"font-mono text-sm tracking-tight #{@color}"}><%= Float.round(elem(@values, 0), 1) %>°</span>
      </div>
      <div class="flex flex-col items-center">
        <span class="text-[8px] text-gray-500 uppercase font-black tracking-widest mb-0.5">Pitch</span>
        <span class={"font-mono text-sm tracking-tight #{@color}"}><%= Float.round(elem(@values, 1), 1) %>°</span>
      </div>
      <div class="flex flex-col items-center">
        <span class="text-[8px] text-gray-500 uppercase font-black tracking-widest mb-0.5">Yaw</span>
        <span class={"font-mono text-sm tracking-tight #{@color}"}><%= Float.round(elem(@values, 2), 1) %>°</span>
      </div>
    </div>
    """
  end
end

PhoenixPlayground.start(live: Simulator.Live)
