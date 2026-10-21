# DVT Legend 510MR (s/n 510221142) bring-up from Ubuntu: findings

Every claim below is backed by the captured output in this folder (step*.txt).
Status: camera found; TCP+UDP ports characterised; the protocol on each port is still undocumented.

## Step 1: interface and link (CONFIRMED)
- The laptop's only wired port is `enp2s0` (r8169, MAC bc:ec:a0:1e:2e:fe).
- Carrier up, 100 Mb/s full duplex (`ethtool enp2s0`, step1b.txt).
- Before probing it had no IPv4 address. NetworkManager was stuck in "connecting (getting IP configuration)", meaning DHCP with no answer.
- The operator reports the camera is cabled to this port. The link LED state is not yet reported by the operator.
- No passwordless sudo (`sudo -n true` -> "a password is required"). arp-scan and nmap are not installed. tcpdump exists but cannot capture without root.

## Step 2: discovery (no root)
- Laptop side only (no write to the camera): I added a removable NetworkManager profile `dvt-probe`.
  It sets 169.254.1.10/16 and an on-link route 192.168.0.0/24 (no 192.168.0.x address taken).
  To undo: `nmcli connection down dvt-probe; nmcli connection delete dvt-probe`.
- The first attempt (step2b/2c) is **INVALID**. GNOME Shell (pid 2963) disconnected the profile 3 s after activation ("user-requested", journal in step2d.txt), so those probes went out with no address.
- The valid attempt is step2e.txt; the address was verified present before and after:
  - ICMP + kernel ARP to 192.168.0.2..254, **192.168.0.1 excluded**: 0 replies, 253/253 ARP entries FAILED.
  - Broadcast ping 169.254.255.255 from 169.254.1.10: 0 replies.
  - Broadcast ping 255.255.255.255: 0 replies.
  - IPv6 all-nodes ping ff02::1: only the laptop itself answered (step1b.txt).
  - RX counter on enp2s0: **241461 packets before and after** (about 2 min including the 30 s idle watch in step2c.txt), while TX rose from 110273 to 111064.
    => Over that window **the camera transmitted nothing at all**: no ARP replies, no broadcasts, no announcements.

## What this does and does not prove
- CONFIRMED: the camera has no IP in 192.168.0.2-254 that answers ARP.
- CONFIRMED: it does not answer broadcast ICMP, and it did not transmit spontaneously over about 2 minutes.
- UNKNOWN: its actual IP. ARP only answers for an exact IP, so an address in any other subnet (10.x, 172.16.x, 192.168.1.x, a specific 169.254.x.x) is invisible to this method.
- UNKNOWN: whether it is booted. Needs the status LED.

## Needed to continue
1. Root for packet capture and raw layer-2 probing (tcpdump, arp-scan, nmap -sS).
2. A power cycle of the camera while capturing. Embedded stacks normally ARP/announce at boot, which would reveal its MAC and IP even on an unknown subnet.
3. The status LED state and any IP/MAC label on the camera.

## Step 2 (continued): camera found (CONFIRMED)
- Boot capture `boot.pcap` (operator ran tcpdump as root and power-cycled the camera), see step2f.txt:
  - `00:90:68:00:f6:12 > ff:ff:ff:ff:ff:ff ARP who-has 169.254.246.18 tell 169.254.246.18` (gratuitous)
  - 14 s later: `ARP who-has 192.168.10.242 tell 192.168.10.242` (gratuitous)
- OUI 00:90:68 = **DVT CORP.** (nmap-mac-prefixes, arp-scan ieee-oui.txt, ieee-data oui.txt).
- Laptop given 192.168.10.10/24 on the `dvt-probe` profile. `ping 192.168.10.242`: 3/3 replies, 0.37 ms.
  The neighbour table shows `192.168.10.242 lladdr 00:90:68:00:f6:12 REACHABLE`.
  169.254.246.18 does not answer after boot (step2g.txt).
- **Camera = 192.168.10.242, MAC 00:90:68:00:f6:12.**

## Step 3: exposed services (TCP, CONFIRMED)
`nmap -Pn -sT -p- --reason -T4` (a connect scan; -sS needs root): 7 open ports, 65528 closed (conn-refused) (step3_tcp.txt).

| Port | nmap name | Passive banner (connect, send nothing, 5 s) | Response to nmap -sV probes |
|---|---|---|---|
| 502 | mbap (Modbus TCP) | none | none |
| 3246 | dvt-system | `System Terminal Driver\r\n?` | the same banner; `\r\n` gives another `?` prompt |
| 3247 | dvt-data | `1\r\n` | replies `1\r\n` per probe line |
| 4883 | (wmlserver) | none | none |
| 5000 | (upnp) | `System Terminal Driver\r\n?` | the same as 3246 |
| 5001 | (commplex-link) | `1\r\n` | the same as 3247 |
| 7500 | (silhouette) | 64 bytes: `04 00` + 62 zero bytes | sends repeating 64-byte records `04 00 NN 00 ...`, NN counting 00..08 |

- Terminal (3246): `help\r\n` -> `\r\n!` (not a known command; step3_terminal_help.txt). No other commands sent.
- **Closed:** 21 (FTP), 23 (telnet), 80 (HTTP), so there is no FTP image server and no web page.
- **UDP** (`sudo nmap -Pn -sU --top-ports 100`, run by the operator, step3_udp.txt): 99 closed (port-unreach); **5000/udp open|filtered** (no reply to nmap's probe, so open or filtered cannot be told apart). nmap also resolved the MAC as DVT.

## Step 4: verdicts so far
- Retrieve an image from Linux: **UNKNOWN**. No FTP/HTTP. The candidates are 3247/5001 ("data" channels) or 7500 (binary records), but the protocol is not observed.
- Trigger an inspection from Linux: **UNKNOWN**. Terminal ports 3246/5000 accept text commands; the command set is not documented here and was not guessed.
- Read Pass/Fail from Linux: **UNKNOWN**. The candidates are Modbus 502 (registers, readable with standard tools) and 3247/5001; not read yet.
- Configuration access from Linux: **UNKNOWN**. No web/telnet/FTP.

## Phase 2, Task A: Modbus TCP, READ-ONLY (CONFIRMED)
Client: pymodbus 3.16.1 `read_*` calls only (modbus_read.py; no write calls exist in the file). Unit/device id 1. Addresses 0..255 for FC1, FC2, FC3, FC4. Raw: modbus_unit1.json, modbus_base_1..3.json.
- **All four function codes answer for the whole 0..255 range, no exception.**
- FC1 and FC2 return identical bits; FC3 and FC4 return identical registers (the camera mirrors them).
- Non-zero at idle (baseline, 3 reads 5 s apart):
  - bits 32, 52, 58, 73 = 1 constantly; **bit 55 = 1 in one of the three reads**
  - register 2 = 256 (0x0100), register 3 = 4100 (0x1004) normally, **36868 (0x9004) in the same read where bit 55 was set**, register 4 = 2
- HYPOTHESIS (unverified): register 3 and bit 55 are a live status that flips by itself, such as a heartbeat or a busy/ready flag. Not yet tied to a scene change.
- Not yet done: the diff with a part in front of the lens / lens blocked (needs the operator).

## Phase 2, Task C: port 7500, passive 60 s (CONFIRMED)
Capture port7500_baseline.bin, nothing sent: 3840 bytes = **60 records of 64 bytes, one per second**. Bytes 0-1 = `04 00` constant; byte 2 = counter 0,1,2..59 in order; byte 3 and bytes 4..63 = 0 in all 60 records. In an empty or static scene this is a pure heartbeat/frame counter. UNKNOWN whether it reacts to the scene (needs the operator).

## Phase 2, Task B: system terminal, documented command only (CONFIRMED)
Operator-authorised, source = operator's spec (IAI integration note, not independently verified). Raw: step_taskB_yi.txt, step_taskB_malformed.txt, step_taskB_timeseries.txt.
| Sent | Reply |
|---|---|
| `#YI\r\n` | `\r\n% 1 \r\n!` |
| `*\r\n` | `\r\n)06\r\n!` |
| `#\r\n` | `\r\n% 6 \r\n!` |
| `help\r\n` (earlier) | `\r\n!` |
- `#YI` is answered with `% 1` then the `!` prompt, so the syntax and the channel work. HYPOTHESIS: `% N` carries a code (1 for the trigger, 6 for the bare `#`); the meaning is not read from a source.
- Modbus around a single `#YI` (timeseries): **no change**. The inspection is either not running, produces no Modbus-visible result, or the result is elsewhere. UNKNOWN which.

## Phase 2, Task C with a part under the lens
port7500_part.bin: 57 records, only byte 2 (counter) differs; all other bytes 0 => **the stream does not react to the scene** (CONFIRMED).

## Phase 2, Task D: ports 3247 and 5001, 120 s (CONFIRMED)
step_taskD.txt: both send `1\r\n` **once per second, continuously** (357 and 360 bytes in 120 s), with nothing sent to them. A `#YI` on 3246 at t=40 s produced no change on either. => heartbeat/keepalive, not a result channel.

## Verdicts (updated)
- Read Pass/Fail from Linux: **UNKNOWN**. Modbus 0..255 is readable but shows no result signal; 7500, 3247, 5001 are heartbeats.
- Trigger an inspection from Linux: **CONFIRMED as accepted** (`#YI` -> `% 1`, `!`), but **no observable effect was found**, so it is unverified that an inspection actually ran. Needs the camera's status LED or a known output.
- Get an image: **UNKNOWN / no route found**. No FTP, HTTP; no port carries image data; port 4883 and UDP 5000 not examined.
- Configuration access: **UNKNOWN**. Terminal accepts commands; none undocumented were tried.

## Step 0 result: vendor documentation (see protocol.md for file:line of every item)
FrameWork 2.8.0 installer unpacked without running it (Wise overlay = raw deflate streams, carve_wise.py). Contains the User Guide, the Script Reference and 6 help files.

### Verdicts after reading the documentation
- **Read Pass/Fail from Linux: CONFIRMED (documented and observed).** Connect TCP to 192.168.10.242:7500 (VDX output stream) and read 64-byte telegrams, one per inspection: byte0 bit0 Pass, bit1 Warn, bit2 Fail, bit3 Wrong Code; byte1 error code; byte2 cycle count; byte3 user bits (SR text/0098_017eeb30.txt:5396). `python3 dvt_result.py` does this, receive-only. Live output: `cycle 0..3  result FAIL  error_code 0`.
- **The camera runs ~1 inspection per second and every result is FAIL** (byte0 = 0x04), also with a part under the lens. UNKNOWN which product is loaded or why it fails (needs FrameWork or the operator).
- **Trigger an inspection from Linux:** two routes. (a) terminal `#YI\r\n` on 3246: accepted (`% 1`, `!`) but **not in the vendor documents** (UNVERIFIED). (b) VDX input telegram byte0 bit0 Trigger on the rising edge, documented (SR:5404-5412), requires sending to port 7500; **not tried** (it writes control bits; needs approval). With internal trigger the camera already inspects once per second, so an explicit trigger may be unnecessary.
- **Image from Linux: UNKNOWN / no documented route.** No FTP (also absent from the documents), no HTTP; image transfer exists only inside FrameWork (UG:2295), protocol not documented.
- **Configuration access from Linux: UNKNOWN.** Terminal accepts commands and `#Rs` writes registers (chm 0041, :149), but no command list is in these files; nothing undocumented was sent.

## CORRECTION (read from the vendor Ethernet command reference, chm_out/0045_0114ec77/Sample_Terminal_Command_Session.htm)
Terminal replies end with `% <code>` and then `?` (success) or `!` (error). Codes: 0 ok, 1 general error, 2 access denied, 3 out of memory, 4 flash error, 5 network error, 6 invalid parameter, 7 locked.
- `#YI` -> `% 1`, `!` is therefore a **general ERROR, not an accepted trigger**. The earlier statement "#YI accepted / confirms syntax" was wrong. `#YI` is not a documented command.
- `#` alone -> `% 6` (invalid parameter) and `*` -> `)06` (same, hex format) are plain errors too.
- So "trigger an inspection from Linux" is **UNKNOWN** again. The camera already inspects ~1/s on its own (port 7500 cycle counter).

## IMAGE ACCESS: CONFIRMED (read-only, documented)
Source: vendor reference `Image_Management_Commands.htm`, `Image_Data_Format_and_Sequence.htm`, `Binary_Packet_Format.htm` (in chm_out/0045_0114ec77/).
Terminal on 192.168.10.242:3246, decimal mode (`#CMD args<CR>`), replies are ASCII lines `# <packet type> <image id> ...`.
Commands used, all read-only, output captured in step_queries.txt, is_decimal_raw.bin, is_2091_raw.bin:
| Command | Result seen |
|---|---|
| `#I` | `FrameWork Version 2.7.4 (Build 14012005)` |
| `#IC` | `1 1 2 10` / `1 640 480 1000 1000 DVT520` = one DVT520 sensor, 640x480 |
| `#PQ` | `-1 1 1`; products 1 FMS50, 2 train, 3 murat |
| `#PQ 1` | product 1 acquisition: exposure 8300, antiblooming 0, illumination 1 (on), gain 128, window 0 0 32767 32767 |
| `#Ii` | inspection statistics (2037 total at first read, no missed) |
| `#IQ` | list of the 8 newest saved images, ids 2084..2091 (rotating) |
| `#IS <id> 0 0 639 479 0 <rowskip> <colskip> 0 0` | **full image as type 16 (header), 20 (name), 18 (data) packets, end `% 0`, `?`**. Full frame = 307200 pixels in 21.7 s; rowskip=colskip=1 = 5.5 s |
| `#Is` (no arguments) | returns only a 1-pixel window, not useful |
- Not working: binary mode `0xAA ...` got no reply (syntax unconfirmed); hex `*IS` with skip returned `)01` error; `#IS -2 ...` (newest) error 1; `#IQ <id>` error 1.
- Image transfer speed is limited by the camera's terminal (~70 KB/s), not by the network.
- **Image content: saturated.** Image 2091: 99.97% of pixels = 255 (min 242), so a flooded sensor, not a broken transfer (dvt_2091.png). Product 1 runs exposure 8300 with illumination ON and gain 128.
  Whether the lens is facing a bright surface / lit ring light or the exposure is too high is UNKNOWN. Changing exposure is a WRITE (`#PP`, needs ALLOW_PRODUCT_PARAMS): **not done, needs the operator's approval.**

## Verdicts (final for this session)
- Read Pass/Fail from Linux: **CONFIRMED**, port 7500 (see protocol.md); currently FAIL on every inspection.
- Get an image from Linux: **CONFIRMED**, `#IS` on port 3246; live viewer `dvt_view.py`; the current frames are saturated white.
- Trigger an inspection from Linux: **UNKNOWN** (`#YI` is an error). The documented alternative is the VDX input telegram on 7500 (not tried).
- Configuration access: the command set is documented (PP/PN/PI/Pp/PA/PD...); nothing written.

## Exposure fix (operator-authorised, RAM only; log in exposure_changes.log)
`python3 set_exposure.py <us>` sends the documented `#PP 1 E <us>` and reads the product back with `#PQ 1`.
| Exposure | `#PP` reply | Frames (214x160) |
|---|---|---|
| 8300 (original) | n/a | 100% saturated, min 242 |
| 1000 | `% 0 ?` | min 0, mean 15, max 43 (too dark) |
| 3000 | `% 0 ?` | min 38, mean 83, max 128, 0% saturated (good) |
Image content at 3000: uniform grey with a mild vertical gradient and no objects, i.e. the lens sees a plain surface (or is out of focus/capped). The transfer path, decoding and exposure control are therefore all confirmed working.
STATE LEFT ON THE CAMERA: product 1 exposure = 3000 us (RAM only, not saved to flash; a power cycle restores the flash value). Original = 8300: `python3 set_exposure.py 8300`.

## ROOT CAUSE of "no picture" (operator-confirmed)
**No lens is fitted; the sensor is bare.** That explains every observation:
- Smooth illumination gradient with no scene detail; frame-to-frame difference = sensor noise only.
- Sharp dark specks in the full-res frame = dust on the sensor cover glass (the only thing "in focus" without a lens).
- Strobe on doubles the brightness (83 vs 40), so the strobe works and light reaches the sensor.
- Decoding verified: row-major is correct (column-major gives obvious banding; dec_rowmajor.png vs dec_colmajor.png).
=> The Linux image path (`#IS` on port 3246) is **CONFIRMED working end to end**. It needs a C/CS-mount lens to show a scene.
Camera state left: product 1 exposure 3000 us, strobe ON (RAM only). Original exposure was 8300 us.
