# DWE OS 2 Migration Evidence

Phase 5 (DWE OS Camera Stack) recorded evidence. This file is git-tracked
because evidence that lives
only in a planning directory is evidence the team cannot read. `05-02` and
`05-03` append their own sections below rather than editing these.

## OS and ROS Distribution on the Pi

The three read-only diagnostic commands were run directly on the Pi
(`pi@10.0.0.3`) before anything in this phase touched the system:

```
$ . /etc/os-release && echo "$VERSION_ID $UBUNTU_CODENAME"
24.04 noble
$ ls /opt/ros/
jazzy
$ cat /etc/apt/sources.list.d/ros2.list
deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main
```

All three signals agree: the Pi is **Ubuntu 24.04 (noble) running ROS 2
Jazzy**, with a matching noble ROS 2 apt source. This is the plan's
**second** candidate state (24.04 running Jazzy), not the fragile third one
(Humble binaries forced onto noble system libraries) — there is no
jammy/Humble anywhere on this host. `rpi_ws` builds against Jazzy, and
`PROJECT.md`'s "ROS 2 Humble across workstation and Pi" constraint line is
wrong for the Pi (flagged below; a distribution correction is a separate
piece of work and is not made here).

The workstation (`10.0.0.69`) runs ROS 2 Humble at `/opt/ros/humble`. The
workstation and the Pi are therefore on two different ROS 2 distributions.
This is irrelevant to this phase — the camera path is RTP/UDP end to end
and never touches ROS — but it is a cross-distro risk worth carrying into
whichever later phase plans the sensor/ROS integration between the two
machines.

**Checkpoint decision:** `proceed` — the install runs as planned. The
risk of the vendor installer disturbing the ROS apt tree is bounded by the
pre-install snapshot below and
Task 2's `colcon build` gate, and Track B (this phase) is independent of
the other tracks, so a problem here blocks nothing else.

**Process deviation, recorded plainly (per coordinator instruction, not
smoothed over):** the plan's `checkpoint:decision gate="blocking-human"`
for this install was passed to execution as already resolved by the
orchestrator rather than confirmed interactively through the standard
checkpoint channel at the start of the work. A direct consequence: the
required ordering — take the legacy glass-to-glass latency measurement
*first*, while it is still cheap, then install — was not honored. The
executor started the legacy stream and waited, but installation (and
later, the DWE OS-side stream configuration) proceeded before either
stopwatch photograph was actually taken. Both latency numbers are
therefore deferred; see Glass-to-Glass Latency below. This is a process
fact worth the team seeing, not a technical outcome to excuse.

Recorded: 2026-09-09

## Pre-Install Snapshot

Captured by `install_dweos.sh`'s `snapshot_system_state` before
`install_dweos` runs, into `SETUP_RPI/dweos_evidence/`:

- `os-release-before.txt`
- `ros-distros-before.txt`
- `ros2-apt-source-before.txt`
- `dpkg-ros-before.txt`
- `systemd-analyze-blame-before.txt`
- `systemd-analyze-critical-chain-before.txt`
- `networkmanager-before.txt`

Independently confirmed ahead of the scripted run: `network-manager` is
**not installed** on this Pi (`dpkg -l network-manager` reports `un`,
uninstalled) and `NetworkManager.service` is `not-found`. DWE OS's
network-management feature is therefore inert on
this host — `apply_service_override` will not add `--no-wifi`, and this is
recorded as a checked fact, not an assumption.

Recorded: 2026-09-09

## USB Port to Camera Map

Three exploreHD cameras are present, confirmed via `v4l2-ctl
--list-devices` and `udevadm`:

| Physical port label | Camera | bus_info | H264 node | UDP port |
|---|---|---|---|---|
| bottom (per operator) | main_cam (this task) | usb-xhci-hcd.0-2 | /dev/video2 (confirmed) | 5001 |
| (pending, assigned by hand) | cam_1 | usb-xhci-hcd.1-1 | not yet checked | 5002 |
| (pending, assigned by hand) | cam_2 | usb-xhci-hcd.1-2 | not yet checked | 5003 |

`bus_info` values are read directly from `v4l2-ctl -d <dev> --info` on the
Pi, matching DWE OS's own `bus_info`-keyed device model (Pitfall 4). Each
exploreHD exposes 4 `/dev/videoN` nodes; only one per camera actually
advertises the `H264` pixel format — confirmed via `v4l2-ctl
--list-formats` for the `usb-xhci-hcd.0-2` group only (node 0 is MJPG/YUYV,
node 2 is H264, first stream_cam.sh attempt on node 0 failed with
"not-negotiated" until corrected to node 2). The other two cameras' H264
nodes were not checked in this task — DWE OS's own device enumeration finds
the right node internally, so this only matters if `stream_cam.sh` is run
directly against them.

Per operator decision, the physical port-to-camera labelling (which of the
three physical USB positions is `main_cam`/`cam_1`/`cam_2`) is being
assigned by hand after this phase rather than during this bench session.
The camera used for this task's tracer proof (`usb-xhci-hcd.0-2`) is
physically the **bottom**-mounted camera per the operator, not necessarily
the vehicle's front-facing "main" camera — it plays the `main_cam` role
(port 5001) for this proof only, per the plan's "one exploreHD" scope. Final
physical assignment is out of scope for this task.

## Stream Configuration Does Not Survive a Power Cycle

Checked on 2026-09-10, on a Pi brought up from cold with the vehicle
battery reconnected, before anything was started by hand.

`dwe_os_2` came back active on its own and enumerated all three exploreHD
units correctly. Their stream configuration did not come back with them.
All three report:

```
"stream": { "encode_type": "H264", "stream_type": "UDP",
            "endpoints": [], "enabled": false }
```

The `main_cam` endpoint configured in the previous session, H264 to
10.0.0.69:5001, is gone. So is `is_managed`, which reads `false` on all
three.

**Operational consequence:** `./install_dweos.sh configure` has to be run
after every boot of the vehicle. Since the battery is disconnected
between sessions, that means every session. This is the strongest
argument yet for giving the Pi a systemd unit that runs the configure
step at startup, which today it does not have for anything.

Two things did survive, and are worth separating from the above:

- The service itself is enabled and starts unattended.
- The two vendor patches held. `string3` reads `"exploreHD USB Camera"`
  on all three devices rather than null, which is why the vendor web UI
  renders. The vendor installer was not re-run, so this only proves the
  patches persist across a reboot, not across a reinstall.

### H.264 device nodes, read back after the power cycle

Each exploreHD exposes four `/dev/videoN` nodes and only one of them
advertises H264. Confirmed for all three units this time, where the
previous session had only checked the first:

| bus_info | H264 node | all nodes |
|---|---|---|
| usb-xhci-hcd.0-2 | /dev/video2 | video0-3 |
| usb-xhci-hcd.1-1 | /dev/video6 | video4-7 |
| usb-xhci-hcd.1-2 | /dev/video10 | video8-11 |

These are the device paths `stream_cam.sh` needs for the rollback. The
first node of each group is MJPG and YUYV only, and a rollback aimed at
it fails to negotiate.

Recorded: 2026-09-10

## Receive-Side Parity

No receive-side edit was required, and that absence is the point.

`CamStreamer` in `gui_ws/src/gui_pkg/gui_pkg/gui_node.py` already builds
`udpsrc ... ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264`,
and DWE OS emits `rtph264pay config-interval=10 pt=96 ! multiudpsink`.
The payload types match exactly, so the two sides already agree on the
wire format.

The three port literals at `VideoImageProvider.__init__` were checked
against the installer's settings and already match:

| Role | install_dweos.sh setting | gui_node.py literal |
|---|---|---|
| main_cam | `MAIN_CAM_PORT="5001"` | `CamStreamer(port=5001)` |
| cam_1 | `CAM_1_PORT="5002"` | `CamStreamer(port=5002)` |
| cam_2 | `CAM_2_PORT="5003"` | `CamStreamer(port=5003)` |

`gui_node.py` is therefore unmodified in this phase, and the frozen
`CamStreamer` region stays byte-identical to `main`.

One expected behavioural difference, recorded so it is not mistaken for a
fault: DWE OS repeats the H.264 parameter sets every ten seconds
(`config-interval=10`) where `stream_cam.sh` repeated them every second,
so a video pane can stay blank for up to about ten seconds after a GUI
restart before the decoder has what it needs to show a picture.

Recorded: 2026-09-10

## Port Identification Procedure

`install_dweos.sh identify` was added for the physical port labelling that is
still outstanding. It reads the DWE OS device list, then for each role in
turn asks the operator to unplug that camera and reports which `bus_info`
disappeared, printing the three settings lines to paste back into the
script. It configures nothing and is safe to run repeatedly.

The `bus_info` values in the map below are already filled into the
installer from this session's readings; `identify` exists to bind them to
physical USB sockets, which is the part no software can determine.

Recorded: 2026-09-10

## Glass-to-Glass Latency

**The latency comparison is NOT done.** Both measurements are
**deferred to 05-03**, by explicit user decision made at the bench: 05-03's
rollback drill already has to stop DWE OS and start the legacy pipeline,
so both numbers get captured in that one session instead of two separate
ones. Do not read the table below as complete.

| Pipeline | Latency | Capture |
|---|---|---|
| Legacy | deferred to 05-03 | — |
| DWE OS | deferred to 05-03 | — |

Method, still to be applied: an on-screen millisecond stopwatch
on the workstation screen, an exploreHD pointed at that screen, one
photograph capturing both the real clock and the GUI's older copy of it.
Their difference is the number. Resolution is roughly one frame at 30 fps
(~33 ms). Both measurements use identical 1280x720 @ 30 fps hardware-H264
settings — no re-encode on either side. Note: the legacy
pipeline was live and available on this camera for several minutes before
DWE OS took it over, and the intended ordering (legacy measurement before
install) was not followed in this task — see the process deviation
recorded at the top of this document.

## DWE OS Install

`install.sh` reported `Successfully installed DWE_OS 2 (v0.7.3)`. Two real
findings surfaced during the bench run, beyond what `05-RESEARCH.md`
anticipated:

**1. Transient bench-network loss mid-install (environmental, not a script
bug).** Partway through `install_requirements.sh` (after `apt-get install`
had already upgraded `python3`/`libpython3.12*`/GStreamer packages), the
Pi's default route to the workstation NAT (`10.0.0.69`) disappeared and
`resolvectl` showed no DNS server configured on `eth0` — `ttyd`'s apt and
GitHub-binary fallback downloads both failed
(`Temporary failure resolving 'ports.ubuntu.com'`, `Could not resolve host:
github.com`). Because DWE OS's own `install.sh` runs
`sh install_requirements.sh && sh create_venv.sh && <stop-if-active>`
followed by `cp .../service/* /etc/...; systemctl enable; systemctl start`
as **unconditional** trailing statements (not part of the `&&` chain), the
partial `install_requirements.sh` failure silently skipped `create_venv.sh`
while still enabling and starting the service — leaving `/opt/DWE_OS_2`
without a `.venv`, and `dwe_os_2.service` crash-looping
(`.venv/bin/python3: No such file or directory`, exit 127). Fixed by
restoring the Pi's default route (`sudo ip route add default via
10.0.0.69 dev eth0`) and a temporary DNS resolver (`sudo resolvectl dns
eth0 8.8.8.8 1.1.1.1`), both bench-only and non-persistent, then re-running
`install_dweos`/`apply_service_override` (skipping `snapshot_system_state`
so the pre-install baseline captured before the network dropped was
not overwritten). This is a real fragility in the vendor's `install.sh`
(no `set -e` across the whole chain, later steps unconditional) worth
carrying forward, not something this phase's script can safely patch
around without editing vendor code.

**2. `--no-wifi` does not gate `NetworkWrapper.initialize()` in v0.7.3
(falsifies research Assumption A2).** With `network-manager` genuinely
absent on this Pi (confirmed in the Pre-Install Snapshot above),
`server.py`'s `serve()` calls `await self.network_wrapper.initialize()`
**unconditionally** — reading the installed source
(`/opt/DWE_OS_2/backend_py/src/server.py`, `serve()`) shows no
`if self.feature_support.wifi:` guard around this call at all, and
`routes/network.py` never references `feature_support` either. The result:
`AsyncNetworkManager.initialize()` raises
`sdbus.dbus_exceptions.DbusServiceUnknownError: The name
org.freedesktop.NetworkManager was not provided by any .service files`,
and the whole FastAPI app fails to start (`exit code=3/NOTIMPLEMENTED`),
regardless of `--no-wifi` being passed. Assumption A2 ("absent
NetworkManager, the D-Bus calls fail gracefully") is **falsified on
hardware** — the failure is not graceful, it prevents the server from
starting at all. This also means `--no-wifi`, as shipped in v0.7.3, is
**not a functional security boundary** on this codebase — it does not gate
`routes/network.py` either. **Resolution:** `apply_service_override` now
patches `server.py` to add the missing
`if self.feature_support.wifi:` guard directly (mirroring the existing
`ttyd` pattern in the same function) instead of installing NetworkManager,
which would have both added a new always-on system service to the Pi and
left the unauthenticated `/api/network/*` routes fully live regardless of
the flag — a materially larger surface than intended.
Verified after the patch: `dwe_os_2` stays `active (running)`,
`GET /api/devices` succeeds, and `Uvicorn running on http://0.0.0.0:80`
appears in the journal.

**3. `DeviceModel.string3` rejects `None` (a second, independent v0.7.3
defect).** `GET /api/devices` and `GET /api/devices/map` both returned
HTTP 500 (`Internal Server Error`) on every request, even after fix #2.
The journal traceback showed a pydantic `ValidationError` on
`DeviceModel.string3` — typed `str = ""` in
`models/cameras.py`, but the underlying device object's actual `string3`
attribute is `None` for (at least) one of the three connected exploreHD
units, an unpopulated UVC string descriptor pydantic's `from_attributes`
mode does not fall back to the field default for. **Resolution:**
`apply_service_override` relaxes the field to `str | None = ""`, matching
the model's own existing style for other optional fields (`name`,
`manufacturer`, `device_info` are already `| None`). Verified: `GET
/api/devices` returns all three exploreHD units with `bus_info` values
(`usb-xhci-hcd.0-2`, `usb-xhci-hcd.1-1`, `usb-xhci-hcd.1-2`) matching the
independent `v4l2-ctl`/`udevadm` readings in the USB Port to Camera Map
section above.

Both patches are idempotent (guarded by a `grep` check) and re-applied by
`apply_service_override` on every run, since `install.sh` recreates
`/opt/DWE_OS_2` from a fresh release tarball each time.

**4. DWE OS's own web UI "Cameras" page crashed — root-caused and fixed.**
Navigating to the Cameras page in DWE OS's frontend (`http://10.0.0.3/`)
threw `Unexpected Application Error! can't access property "length", n is
null` (minified `index-BuKe0Scg.js`) and listed no cameras at all.

The cause is defect #3's own fix. Widening `DeviceModel.string3` to
`str | None` stopped `GET /api/devices` returning HTTP 500, but the API
then emitted `"string3": null` for the one exploreHD whose UVC string
descriptor is unpopulated — and the vendor's own frontend reads `.length`
on that value with no null guard, so the page died before rendering any
camera. A null-walk over the full `/api/devices` payload confirmed
`string3` was the only genuinely null field anywhere in the three
devices' JSON.

**Resolution:** the annotation goes back to `string3: str = ""` and a
pydantic `@field_validator(..., mode="before")` coerces `None` to the
field's own default, so the value is a `str` all the way to the client.
Both the API and the vendor UI then work. Verified: `GET /api/devices`
returns `'exploreHD USB Camera'` for `string3` on all three units, and the
Cameras page renders.

This is worth carrying upstream as a single report: the backend is willing
to produce a value its own frontend cannot consume.

Recorded: 2026-09-09

## Stream Configuration — main_cam

`configure_stream "usb-xhci-hcd.0-2" "5001"` (the `main_cam` role for this
task; the operator identifies this specific camera as the vehicle's
**bottom**-mounted unit — see the USB Port to Camera Map above) via
`./install_dweos.sh configure`, after `MAIN_CAM_BUS_INFO` was filled from
the `list_devices`/`GET /api/devices` output.

Request: `POST http://127.0.0.1:80/api/devices/configure_stream`, body
`stream_type=UDP`, `stream_format` 1280x720 @ 30fps, `encode_type=H264`,
`enabled=true`, `endpoints=[{host:10.0.0.69, port:5001}]`.

Response: `{"success":true}`.

**Upgrade evaluation (user-requested, bench-verified before deciding to
keep the patches):** before patching vendor code, the option of pinning a
newer DWE OS 2 release that might already fix defects #2/#3 instead was
checked directly against GitHub:

- The newest published GitHub *release* for `DeepwaterExploration/DWE_OS_2`
  is still **`v0.7.3`** (assets: `release.tar.gz`, `openapi.json`). A
  `v0.7.4` *tag* exists but publishes no release asset —
  `releases/tags/v0.7.4` 404s, confirming `05-RESEARCH.md` Pitfall 1's
  finding independently and a second time. There is nothing newer to pin.
- Both defects are **still present on the upstream `main` branch today**:
  `backend_py/src/server.py` line 190's
  `await self.network_wrapper.initialize()` is still unconditional, and
  `backend_py/src/models/cameras.py` line 208's `string3: str = ""` is
  still not `Optional`. Even tracking `main` instead of a tag (which the pinning decision
  explicitly forbids) would not have resolved either issue.

**Decision: keep both patches, stay pinned at `v0.7.3`.** This is a
**documented, user-approved deviation from the plan's original scope** —
the plan specified installing the pinned release as-shipped, not modifying
it. It is recorded here as such, not as silent scope creep: the
alternative (installing `network-manager` as a new system service, with
its unauthenticated network-config API fully live) was assessed as
strictly worse on every axis (footprint, reversibility, security surface)
in the Defect #2 writeup above, and `GET /api/devices` being permanently
broken (Defect #3) blocks this task's own `list_devices` deliverable
outright. Both defects are worth reporting upstream to
DeepwaterExploration; that report is not part of this phase's scope.

Confirmed in the `dwe_os_2` journal immediately after: `Starting streams:
/dev/video2`, `Single stream detected: Using GStreamerProcessEngine`, and
the exact spawned pipeline —

```
v4l2src device=/dev/video2 ! video/x-h264,width=1280,height=720,framerate=30/1 ! \
  h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! \
  multiudpsink sync=true clients=10.0.0.69:5001
```

— matching `05-RESEARCH.md`'s Pattern 2 prediction exactly, including
DWE OS's own automatic selection of `/dev/video2` (the H264-capable node)
out of the four nodes the `usb-xhci-hcd.0-2` device exposes, with no
manual node selection needed from this script.

**Independent receive-side proof (orchestrator-verified from the
workstation, `10.0.0.69`):** binding UDP port 5001 directly and capturing
for under a second recorded **200 RTP packets / 253,776 bytes**. This is
stronger evidence than a GUI eyeball check — it proves real H.264/RTP
payload is actually arriving at the workstation on the exact port
`CamStreamer`'s `main_cam` binds, independent of `gui_node.py`'s own
decode/render path. Read together with `GET /api/devices` (confirmed by
the orchestrator to report `main_cam`/`usb-xhci-hcd.0-2` configured H264 to
`{host: 10.0.0.69, port: 5001}`, with the other two cameras' endpoints
correctly still empty — they come later), one camera is proven end to end:
DWE OS emits the stream
`CamStreamer` expects, unicast to the right destination, with zero edits
to `gui_node.py`.

**Frozen-boundary and rollback confirmation:** `git diff
732960d..HEAD -- SETUP_RPI/stream_cam.sh
gui_ws/src/gui_pkg/gui_pkg/gui_node.py` is empty — both files are
byte-identical to the branch's base commit. `SETUP_RPI/stream_cam.sh`
(the legacy rollback) and `gui_node.py`'s receive-side pipeline
(deliberately frozen) are confirmed untouched by every commit on
`feat/dweos-camera-stack`, not merely assumed so.

Recorded: 2026-09-09

## ROS 2 Integrity After Install

This is the ROS-integrity check: the install ran `apt-get install` for
eleven GStreamer/GLib development packages and, in the process, upgraded
`python3`, `libpython3.12*` and several GStreamer packages. Whether that
disturbed the ROS 2 tree is a question with a checkable answer.

dpkg -l 'ros-*' diff: none. Both `dpkg-ros-before.txt` and
`dpkg-ros-after.txt` are byte-identical (34,081 bytes each, `diff`
returns no output).

colcon build: Summary: 2 packages finished [16.9s], 0 errors, 1 package
(`nereo_sensors_pkg`) had pre-existing stderr warnings unrelated to this
phase.

**`dpkg -l 'ros-*'` before vs. after: byte-identical.** `diff
dweos_evidence/dpkg-ros-before.txt dweos_evidence/dpkg-ros-after.txt`
returns no output; both files are 34,081 bytes. Not one ROS package was
added, removed, upgraded or reconfigured by the DWE OS install.

**`colcon build` on `rpi_ws`: clean, from scratch.** `build/`, `install/`
and `log/` were deleted first so this is a genuine full rebuild, not an
incremental no-op:

```
$ cd ~/nereo_ros2_code/rpi_ws && rm -rf build install log
$ source /opt/ros/jazzy/setup.bash && colcon build
Finished <<< sonar_pkg [2.2s]
Finished <<< nereo_sensors_pkg [16.7s]
Summary: 2 packages finished [16.9s]
  1 package had stderr output: nereo_sensors_pkg
```

Zero errors. `nereo_sensors_pkg`'s stderr is warnings only, all
pre-existing and unrelated to this phase: unused-function warnings from
the bundled `WT61P.h` IMU driver's forward declarations and from
`qos_profiles.hpp`'s `getReliableQoS()`. Nothing about them is new.

**Distribution actually used: Jazzy**, sourced from
`/opt/ros/jazzy/setup.bash` — confirming the distribution recorded at the top of
this file by building against it rather than merely reading a directory
listing. `PROJECT.md`'s "ROS 2 Humble across workstation and Pi"
constraint line is **flagged as wrong for the Pi** and deliberately left
unedited here; correcting it is separate work.

The ROS tree is provably intact.

Recorded: 2026-09-09

## Boot Time — Before and After

Both `systemd-analyze` captures are in `dweos_evidence/`
(`systemd-analyze-blame-{before,after}.txt`,
`systemd-analyze-critical-chain-{before,after}.txt`).

**The "after" capture does not yet mean anything, and must not be read as
a result.** `systemd-analyze` reports the *last completed boot*, and the
Pi has not been rebooted since DWE OS was installed — so the two
critical-chain files are identical by construction, both describing the
pre-install boot (`graphical.target @56.919s`, total
`6.467s kernel + 57.869s userspace = 1min 4.337s`).

The boot-time comparison therefore remains **open**, and is properly
answered in `05-02`, which requires a reboot anyway for its device-node
exclusivity evidence. The first post-reboot `systemd-analyze`
capture taken there is the real "after" number; this section's `-after`
files are a pre-install baseline duplicate and are kept only so the gap is
visible rather than silently filled with a wrong number.

Recorded: 2026-09-09

## Bench-Session-Only Changes To Undo

None of the following is persistent or intended to survive this phase.
All of it exists because the bench needed it, and each item must be
removed when Phase 5 closes:

| Where | What | Undo |
|---|---|---|
| Pi | Passwordless sudo for `pi` | `sudo rm /etc/sudoers.d/99-bench-nereo` |
| Pi | Default route via the workstation | `sudo ip route del default via 10.0.0.69` (non-persistent; a reboot clears it) |
| Pi | Temporary DNS resolvers on `eth0` | `sudo resolvectl revert eth0` (non-persistent) |
| Workstation | IP forwarding | `sudo sysctl -w net.ipv4.ip_forward=0` |
| Workstation | NAT masquerade + forward rules on `wlo1` | the three `iptables` rules re-run with `-D` in place of `-A` (non-persistent; a reboot clears them) |

The Pi's internet access existed **only** to let the vendor `install.sh`
fetch its release tarball. Nothing in DWE OS's runtime or boot path
reaches the network, so the isolated competition tether remains a
supported operating condition — that is the whole point and it is
unaffected by how the install was fed.

Recorded: 2026-09-09
