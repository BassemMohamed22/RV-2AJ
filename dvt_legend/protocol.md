# DVT FrameWork 2.8.0 protocol facts (read from the vendor installer)

Installer: `InstallFWK280.zip` -> `InstallFWK280.EXE` (Wise installer; 7-Zip cannot open it).
Method: the 33 MB overlay is back-to-back raw-deflate streams; `carve_wise.py` inflated them without running anything. Carved files are in `carved/`, text in `text/`.
Source documents (page numbers are the PDF's own):
- **UG** = `carved/0099_01891794.pdf` = "FrameWork Installation and User Guide for DVT Vision Sensors", 9th ed., June 2007 -> `text/0099_01891794.txt`
- **SR** = `carved/0098_017eeb30.pdf` = "DVT Script Reference Manual", 5th ed., May 2007 -> `text/0098_017eeb30.txt`
- Help files `carved/0041..0046_*.chm` (extracted into `chm_out/`).

## Verified (quoted from a source)
| Item | Source (file:line) | What it says |
|---|---|---|
| System terminal port | UG text/0099_01891794.txt:2266 | "system terminal (port 3246 for new systems and 5000 for old systems)... can receive a number of commands in this port that give the user access to it without using FrameWork" |
| System Driver port | UG :4764 | "System Driver: ... port 3246 (port 5000 for older systems)" |
| DataLink output | UG :4660 | "to read the data sent out from DataLink, ... TCP protocol to port 3247 (port 5001 for older systems)". Strings are user-configured; "in some cases, no string will be sent out" |
| Modbus | UG :4680 | SmartImage Sensor can be a Modbus slave "on port 502"; the master chooses read or write |
| VDX output stream | SR text/0098_017eeb30.txt:5385 | "telegrams ... over TCP/IP on port 7500 in a cyclic fashion ... 64 bytes long"; "When the SmartImage Sensor is running inspections, the VDX server will send an output telegram **after every inspection**"; only one client at a time; the driver is enabled by default |
| Output telegram | SR :5396-5402 | **Byte 0: bit0 Pass, bit1 Warn, bit2 Fail, bit3 Wrong Code (bits 4-7 reserved); Byte 1: Error Code (1 product select failed, 2 error saving product, 3 error triggering); Byte 2: Cycle Count = inspections counter; Byte 3: User bits 1-8**; bytes 4..63 multi-purpose data (script functions) |
| Input telegram (client -> camera) | SR :5404-5412 | Byte 0: bit0 Trigger, bit1 Prod. Select, bit2 Relearn, bit3 Save Prod. Flash, bit4 Reset, bit5 Trigger Mode (0 internal / 1 external), bit6 Run Mode (0 not running / 1 running). Bytes 2-3 product ID. Trigger acts on the **rising edge** |
| Terminal command `#Rs` | chm_out/0041_00cd5f04/General/Menus/Comm/Image_Sensor_Administration_-_User_Host_Privileges.htm:149 | "ability to write to Registers using the #Rs system terminal command" (a WRITE; not used) |
| Image transfer | UG :2295 | "enable/disable image compression for image transfers" (a FrameWork option; the protocol is not described) |

## Not found in the vendor documents
- **`#YI`**: zero hits in the UG, the SR and all six .chm files. The terminal answered it (`% 1`, `!`) but this installer does not document it. Status: **UNVERIFIED** (source = your IAI integration note only).
- The full list of `#`/`*` system terminal commands: not in these files (UG :2269 defers to "the FrameWork help files"; the carved help files do not contain the list).
- DataLink string format: user-defined in FrameWork (UG :4650 "Figure 73: String setup for DataLink"); no fixed format except the XML option.
- FTP: no FTP port or procedure appears in the UG/SR text (`grep -i ftp` found nothing). Matches the closed port 21.
- An image-over-Ethernet protocol: not documented here.

## How the live camera matches the documents (from our captures)
- Port 7500 record: `04 00 NN 00 ...` with NN = 0,1,2,... once per second, rest zero (port7500_baseline.bin, port7500_part.bin).
  By the documented layout: byte 0 = 0x04 = **Fail** bit, byte 1 = 0 no error, byte 2 = **cycle count** counting 1 inspection per second.
  => the camera is **running an inspection each second and the result is FAIL, with or without a part** under the lens (UNVERIFIED which product it runs).
- Ports 3247/5001: `1\r\n` once a second = a DataLink string configured as "1" per inspection (hypothesis; fits "sent after each inspection").
- Modbus registers/bits that cycle about once a second: also in step with the inspections (hypothesis, address meanings not found in the docs).
