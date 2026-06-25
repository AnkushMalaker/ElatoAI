# BLE peripheral mode (OMI-compatible)

The Elato firmware can run in one of two transports, chosen **at boot**:

- **WiFi** (default) — direct Wyoming-over-WebSocket to the Chronicle backend, full duplex
  (mic up, speaker down). This is the original `CHRONICLE_MODE` behaviour.
- **BLE** — the device becomes an **OMI-compatible BLE peripheral**. It streams mic audio as
  Opus and emits button events over the OMI GATT contract, so Chronicle's existing OMI BLE
  clients (the React Native phone app and the macOS `local-wearable-client`) connect to it
  **with no changes**. The client relays the audio to the backend.

Only one transport is active per boot — we never run WiFi and BLE at the same time (shared
radio, no PSRAM). Switching is a **reboot-swap**: the chosen mode is stored in NVS and the
device restarts into the other stack.

## Switching transports

- **On-device gesture (touch mode):** **double-tap-hold** — tap the touch pad once, then on
  the second touch *press and hold* (~0.7 s). The LED flashes (blue = BLE, green = WiFi) and
  the device reboots into the other transport. (Plain long-hold still = sleep; two quick taps
  still = double-press.)
- **Build-time:** add `-D BLE_ONLY` to `build_flags` to force BLE and (optionally) drop the
  WiFi path from the image.

The current mode is shown at boot by a brief LED flash and on serial: `[BOOT] transport = …`.

## What BLE mode supports

| Feature | BLE mode |
|---|---|
| Mic → client → backend | ✅ Opus, 16 kHz mono, 60 ms frames |
| Button / touch events | ✅ single / double press |
| Speaker / TTS downlink | ❌ (OMI clients don't push audio back over BLE — use WiFi) |
| OTA / WiFi provisioning | ❌ (WiFi-based — boot into WiFi mode to update, then swap) |

## GATT contract (matches `friend-lite-sdk/uuids.py`)

| Service | Characteristic | Props | Payload |
|---|---|---|---|
| `19B10000-…1214` | Audio `19B10001` | NOTIFY | `[3-byte header][Opus packet]` |
| `19B10000-…1214` | Codec `19B10002` | READ | `0x14` (20 = Opus) |
| `23BA7924-…2E92` | Button `23BA7925` | NOTIFY | 8 bytes: two LE uint32, first = state |

Audio is decoded client-side by `friend-lite-sdk/decoder.py` (`Decoder(16000, 1)`,
`decode(data, 960)` after stripping the 3-byte header).

## Build notes

- Adds `h2zero/NimBLE-Arduino@^2.2.3` (NimBLE 2.x API).
- `partition.csv` app slots were grown 2M → 3M because the single image now links **both**
  the WiFi stack and NimBLE. **A device on the old partition table needs a one-time full
  erase + flash** (not an OTA) to adopt the new layout.
- The swap gesture is implemented for `TOUCH_MODE` (the active input mode). In button mode
  there is no swap gesture (use `-D BLE_ONLY` or add one to the button callbacks).

## Code map

- `Transport.{h,cpp}` — mode enum, NVS load/save, `requestTransportSwap()` (save + reboot).
- `BLEManager.{h,cpp}` — NimBLE GATT server, audio/button notify, advertising.
- `Audio.cpp` — `micTask` Opus-encodes in BLE mode; `sendButtonEvent` notifies in BLE mode.
- `main.cpp` — `setup()` brings up only the chosen transport; `touchTask` double-tap-hold.
