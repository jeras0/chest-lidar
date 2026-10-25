# Chest Lidar: setup guide

```
Hokuyo UST-10LX --Ethernet cable--> Jetson Orin Nano --Wi-Fi--> laptop browser
                                          |
                                     GPIO pin 7 --> transistor --> buzzer / vibration motor
```

One Python program (`obstacle_alert.py`) runs on the Orin. It reads the lidar, beeps the buzzer when something is in front of the wearer, and serves a live map at `http://<orin-ip>:8000`. The laptop needs nothing installed, just a browser. It uses only the Python standard library plus `Jetson.GPIO` (preinstalled on JetPack).

Files:

| File | What it does |
|---|---|
| `lidar_scip.py` | Tiny driver for the UST-10LX (SCIP 2.0 over TCP), plus a fake lidar for testing |
| `obstacle_alert.py` | Main program: alert logic, buzzer, web server |
| `viewer.html` | Live top-down view served to the laptop |
| `chest-lidar.service` | Makes it start automatically when the Orin boots |
| `vision.py` | Optional camera add-on: YOLO object names + lidar distance, spoken through a Bluetooth speaker (step 6) |

## 0. Try it right now on a laptop (no hardware)

```bash
python3 obstacle_alert.py --sim
```

Open http://localhost:8000. You'll see a fake corridor with a wall moving toward you and the status going CLEAR, CAUTION, CLOSE, STOP. This lets your team work on the UI while someone else wires hardware.

## 1. Connect the lidar to the Orin (wired Ethernet)

The UST-10LX talks over Ethernet only (its fixed IP is `192.168.0.10`). Plug its Ethernet cable straight into the Orin's RJ45 port, power the lidar (12 V or 24 V DC), then give the Orin's wired port a static address on the same subnet.

```bash
ip link                       # find the wired interface: usually enP8p1s0 (JetPack 6) or eth0
IFACE=enP8p1s0                # <- use the name you found

sudo nmcli con add type ethernet ifname $IFACE con-name lidar \
  ipv4.method manual ipv4.addresses 192.168.0.15/24 ipv4.never-default yes ipv6.method ignore
sudo nmcli con up lidar

ping -c 3 192.168.0.10        # should reply
```

`ipv4.never-default yes` stops the lidar link from stealing the Orin's internet route.

Quick check that data is flowing (should print about 1081 numbers in millimetres; index 540 is straight ahead):

```bash
python3 -c "from lidar_scip import Lidar; r = Lidar().scan(); print(len(r), 'readings, straight ahead =', r[540], 'mm')"
```

## 2. Wireless link from the Orin to your laptop

**Recommended: make the Orin its own Wi-Fi hotspot.** Campus and hackathon Wi-Fi usually blocks device-to-device traffic, so don't rely on it. With a hotspot the Orin always has the same address and it works anywhere you walk.

```bash
sudo nmcli dev wifi hotspot ifname wlP1p1s0 ssid ChestLidar password "sbuhacks2026"
sudo nmcli con modify Hotspot connection.autoconnect yes   # come back after reboot
```

Connect your laptop to the `ChestLidar` Wi-Fi network. The Orin is then at **`10.42.0.1`**, so the viewer is **http://10.42.0.1:8000**.

(Your Orin's Wi-Fi is `wlP1p1s0`; on other Jetsons run `nmcli dev` to see the name. If the Orin has no Wi-Fi card, a USB Wi-Fi dongle works.)

**Backup option: phone hotspot.** Put both the Orin and laptop on one phone hotspot, run `hostname -I` on the Orin, and open `http://<that-ip>:8000`. This keeps internet on the laptop (the Orin hotspot doesn't), but the Orin's IP can change each time.

To copy this folder onto the Orin from your laptop:

```bash
scp -r lidar-stack jetson@10.42.0.1:~/      # replace "jetson" with your Orin username
```

## 3. Wire the buzzer (or vibration motor)

Orin GPIO pins are 3.3 V and can only supply a few mA, so **never drive a buzzer or motor directly from the pin**. Use one NPN transistor as a switch:

```
Orin pin 2 (5V) ─────────────── buzzer (+)
                                buzzer (−) ─────── collector
Orin pin 7 (GPIO) ── 1 kΩ ──────────────────────── base      2N2222 / 2N3904 / PN2222
Orin pin 6 (GND) ───────────────────────────────── emitter
```

- Use an **active** buzzer (it has its own oscillator and beeps when given DC). A passive buzzer will only click.
- For a **coin vibration motor** (nicer for a wearable, the person feels it instead of hearing it), use the same circuit and add a diode (1N4148 or 1N4001) across the motor, stripe toward 5 V.
- Pin numbers are the physical numbers on the 40-pin header (pin 1 is next to the small triangle / "1" mark).

Test the buzzer alone:

```bash
sudo python3 -c "
import Jetson.GPIO as G, time
G.setmode(G.BOARD); G.setup(7, G.OUT)
for _ in range(5): G.output(7, 1); time.sleep(0.2); G.output(7, 0); time.sleep(0.2)
G.cleanup()"
```

If nothing happens, try another pin (for example 15) and pass `--pin 15` to the program. If no pin works, run `sudo /opt/nvidia/jetson-io/jetson-io.py` to check the header's pin configuration.

To use GPIO without `sudo`: `sudo groupadd -f -r gpio && sudo usermod -aG gpio $USER`, then reboot.

## 4. Run it

```bash
cd ~/lidar-stack
sudo python3 obstacle_alert.py
```

Open http://10.42.0.1:8000 on the laptop. Click **Laptop sound** if you want the laptop to beep too (good for judges who can't hear the buzzer).

Alert behaviour (default, change with `--zones stop,close,caution` in metres):

| Nearest obstacle in front | Buzzer |
|---|---|
| more than 2.0 m | silent |
| 1.5 to 2.0 m | slow beeps |
| 0.75 to 1.5 m | fast beeps |
| under 0.75 m | solid tone |

Only obstacles inside a 90° wedge straight ahead count (45° each side), so walking past a side wall doesn't beep. `--fov` sets the total wedge width in degrees at startup, and the **FOV** dropdown in the browser view changes it live (30 to 270°). Example: `sudo python3 obstacle_alert.py --fov 70 --zones 0.5,1.0,1.5`. Other options: `--lidar-ip`, `--pin`, `--port`.

## 5. Start automatically on boot (do this before the demo)

So the wearer doesn't need a keyboard: edit the paths in `chest-lidar.service` to match your username, then:

```bash
sudo cp chest-lidar.service /etc/systemd/system/
sudo systemctl enable --now chest-lidar
journalctl -u chest-lidar -f      # watch its log
```

Now powering on the harness is enough: Orin boots, hotspot comes up, lidar connects, buzzer is live. To keep custom settings, add them to the end of the `ExecStart=` line, e.g. `... obstacle_alert.py --fov 70 --zones 0.5,1.0,1.5`, then run `sudo systemctl daemon-reload && sudo systemctl restart chest-lidar`. The program also reconnects by itself if the lidar cable is bumped.

## 6. Camera: name what's ahead and say how far (vision.py)

`vision.py` runs YOLO on the Logitech webcam, looks up each object's distance in the lidar scan at the same angle, and speaks things like "person ahead, 2 meters" or "chair on the left, 1.5 meters" through the Nest Mini. The browser view shows the labelled camera feed in the corner and puts yellow labels on the lidar map. It's a separate program from `obstacle_alert.py`, so if the camera or YOLO has a problem, the buzzer keeps working.

```
webcam --USB--> vision.py (YOLO) --reads scan from :8000--> obstacle_alert.py
                    |--> espeak-ng --Bluetooth--> Nest Mini
                    '--> :8001 camera + labels --> viewer.html on the laptop
```

**Try it on a laptop first:** run `python3 obstacle_alert.py --sim` and, in a second terminal, `python3 vision.py --sim` (needs `pip install opencv-python numpy`). Open http://localhost:8000 and click **Laptop voice** to hear the announcements. To use the laptop's own webcam with real YOLO, also `pip install ultralytics` and run `python3 vision.py` without `--sim`.

### 6a. Mount the camera

Put it directly under (or above) the lidar, centred and pointing straight ahead, so the camera's centre line matches the lidar's 0°. Tilt it down a little (about 10°) so it sees the floor a few metres ahead.

### 6b. Install YOLO on the Orin (JetPack 6, Python 3.10)

Check the version first with `cat /etc/nv_tegra_release` (R36 means JetPack 6). These are the Ultralytics Jetson guide's commands; for other JetPack versions see https://docs.ultralytics.com/guides/nvidia-jetson/.

```bash
sudo dpkg --configure -a          # only if apt says "dpkg was interrupted"; let it finish
sudo apt update && sudo apt install -y python3-pip espeak-ng
python3 -m pip install -U pip
python3 -m pip install ultralytics
python3 -m pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.10.0-cp310-cp310-linux_aarch64.whl
python3 -m pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.25.0-cp310-cp310-linux_aarch64.whl

# this torch build also needs NVIDIA cuDSS
wget https://developer.download.nvidia.com/compute/cudss/0.7.1/local_installers/cudss-local-tegra-repo-ubuntu2204-0.7.1_0.7.1-1_arm64.deb
sudo dpkg -i cudss-local-tegra-repo-ubuntu2204-0.7.1_0.7.1-1_arm64.deb
sudo cp /var/cudss-local-tegra-repo-ubuntu2204-0.7.1/cudss-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update && sudo apt-get -y install cudss

python3 -c "import ultralytics, torch; print(torch.__version__, torch.cuda.is_available())"   # must end in True

# needed for the TensorRT export (onnxruntime-gpu is not on PyPI for Jetson)
python3 -m pip install "onnx>=1.12.0,<2.0.0" "onnxslim>=0.1.82"
python3 -m pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl
python3 -c "import tensorrt; print(tensorrt.__version__)"   # if missing: sudo apt install -y python3-libnvinfer python3-libnvinfer-dev
```

Then build the fast TensorRT version of the model (takes about 5 to 10 minutes, once). Run it inside `~/lidar-stack` so `vision.py` finds `yolo11n.engine` automatically:

```bash
cd ~/lidar-stack
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='engine', half=True, imgsz=640)"
```

Expect about 25 to 30 fps with the engine. If the export fails, `vision.py` still runs with `yolo11n.pt` on the GPU at around 10 to 15 fps, which is fine for a demo. Running `sudo jetson_clocks` also helps (it locks the clocks at the current power mode's maximum, so check the battery budget with Jay).

### 6c. Pair the Nest Mini over Bluetooth

1. In the **Google Home** app, tap the Nest Mini, then the gear icon, then **Audio**, then **Paired Bluetooth devices**, then **Enable pairing mode**.
2. On the Orin desktop, open **Settings > Bluetooth**, pick the Nest Mini, then in **Settings > Sound** choose it as the output. (Terminal alternative: `bluetoothctl`, then `scan on`, `pair <MAC>`, `trust <MAC>`, `connect <MAC>`.)
3. Test it: `espeak-ng "hello from the chest lidar"`.

If it pairs but plays no sound, run `sudo apt install pulseaudio-module-bluetooth` and reboot. Note that the Nest Mini needs wall power, so it sits on the demo table (Bluetooth reaches about 10 m). For a real wearer, the same setup works with Bluetooth earbuds. If Bluetooth gives you trouble at the demo, click **Laptop voice** in the browser view instead and the laptop speaks the same phrases.

### 6d. Run it

```bash
cd ~/lidar-stack
sudo python3 obstacle_alert.py        # terminal 1 (or the chest-lidar service)
python3 vision.py                     # terminal 2, WITHOUT sudo, so the Bluetooth audio works
```

The laptop view at http://10.42.0.1:8000 now shows the camera in the bottom right. The raw camera stream is also at http://10.42.0.1:8001/camera.

**Line up camera and lidar:** walk slowly from side to side about 2 m in front of the wearer and watch the map. Your yellow "person" dot should sit on your blue lidar points. If the dot swings further out to the sides than you do, lower `--hfov` (try 45); if it doesn't swing far enough, raise it (try 55). If it's always shifted to one side, use `--cam-offset` (positive moves it left, in degrees).

What it says and when:

- Only useful things (people, vehicles, bikes, dogs, chairs, tables, benches, and so on: the `ANNOUNCE` list at the top of `vision.py`).
- The nearest object first, one phrase at a time, and only within 4 m (`--speak-range`).
- The same kind of object isn't repeated for 6 s (`--cooldown`) unless it got 1 m closer.
- An object must be seen in 3 frames in a row, so one-frame misdetections stay quiet.
- "Ahead" means within 12° of straight ahead. Objects the lidar can't measure (below or above its slice) are only mentioned if they fill a big part of the picture.

Other options: `--camera 1` (if the webcam isn't `/dev/video0`), `--conf` (detection threshold, default 0.45), `--model`.

## Things judges may ask about

- **The lidar sees a single flat slice at chest height.** Mount it level. If it tilts down it will see the floor and beep constantly; if it tilts up it misses things. It won't catch low obstacles (curbs, chairs below chest height) or overhangs. Say this upfront, it shows you understand the limits.
- **Noise rejection.** The alert uses the 5th closest reading in the wedge, so one stray point doesn't beep.
- **Latency.** Lidar runs at 40 scans per second; buzzer reacts within about 25 ms.

## Troubleshooting

| Problem | Fix |
|---|---|
| `ping 192.168.0.10` fails | Check lidar power (green LED), check `nmcli con show --active` lists `lidar`, try `sudo nmcli con up lidar` |
| "waiting for lidar" forever | Same as above; also make sure no other program (like a ROS node) is connected to the lidar |
| Page won't load on laptop | Laptop on the `ChestLidar` network? Program running? Try `curl http://10.42.0.1:8000` from the Orin itself |
| "buzzer disabled" in the log | Run with `sudo`, or set up the gpio group (step 3) |
| Beeps constantly | Lidar tilted toward the floor, or harness strap in view. Narrow `--fov` (e.g. `--fov 60`) or re-mount |
| Camera box missing in the view | Is `vision.py` running? Try http://10.42.0.1:8001/camera directly. `could not open camera`: check `ls /dev/video*` and use `--camera N` |
| Labels speak but no sound from the Nest | Run `vision.py` without `sudo`; check `espeak-ng hello` works; re-pick the Nest Mini in Settings > Sound |

## Alternative: ROS 2 + RViz

If someone on the team already knows ROS 2 (Humble on JetPack 6): `sudo apt install ros-humble-urg-node`, then `ros2 run urg_node urg_node_driver --ros-args -p ip_address:=192.168.0.10` publishes `/scan`, and RViz on a laptop with the same `ROS_DOMAIN_ID` can display it. It's more setup (ROS on the laptop too) and ROS 2 discovery over Wi-Fi is often flaky, so the browser approach above is the safer bet for a hackathon.
