# NadirStation GOES GVAR Decoder

Decoder for the GVAR downlink of the GOES-N/O/P imagers (GOES-13/14/15). It takes either a **baseband IQ recording**
(`.cs8`, `.cu8`, `.cs16`, `.cf32`, `.wav`) or a **live RTL-SDR**, demodulates the signal, finds the GVAR blocks, assembles
the scan lines and writes the five imager channels as PNG files, plus a false-colour picture and the block-0 telemetry.

It only receives. It has no transmit function and makes no network connections.

Developer: AGcov (BI2QYN) · Repository: https://github.com/AGcovStudio/NadirStation_GOES_GVAR_Decoder

This is the GVAR receiver of the NadirStation ground station, split out as a stand-alone program.

## What it does

- BPSK demodulator for the 2.111 Msym/s differential NRZ-S signal at 1685.7 MHz (root-raised-cosine 0.5, carrier and symbol
  tracking), from 2.75 MS/s upward (2.88 MS/s tested).
- Block synchronisation and header recovery. GVAR has no forward error correction, so with a weak signal some block headers
  fail their CRC; headers are predicted from the block sequence so scan lines are not thrown away.
- Soft pixel recovery: pixels hit by a symbol error are re-estimated from the soft decisions of the demodulator.
- Image assembly for the visible channel (0.65 µm) and the four infrared channels (3.9, 6.5, 10.7, 13.3 µm) with the
  imager's line geometry, a false-colour picture, and block-0 documentation / telemetry written to a `Telemetry_*` folder.
- Live view while receiving: constellation, spectrum, link statistics and a growing preview of the image.
- Input from a file **or** from an RTL-SDR (loaded at run time from `rtlsdr.dll`, which is not included).

GVAR is not encrypted, so no key of any kind is needed.

## RTL-SDR notes

The signal is about 3.2 MHz wide, so the RTL-SDR has to run at 2.88–3.2 MS/s. The RTL2832U is only specified up to 2.4 MS/s
(stable up to about 2.56 MS/s on most dongles); above that you are over-clocking it, and many dongles or USB controllers
will not keep up. A poor port or cable drops samples, which shows up as missing lines in the picture. The program counts
the samples that arrive against the sample clock and tells you in the log when USB has lost some. If your dongle cannot do
2.88 MS/s, use a wider-band SDR (Airspy, HackRF, PlutoSDR, SDRplay …) and decode its baseband recording instead. Use a port directly on
the motherboard. At 1685.7 MHz you need an L-band antenna (a dish with a feed, and a low-noise amplifier close to it); a
plain whip will not do. The Bias-T option is off by default; switch it on only if your LNA needs power from the dongle.

Put a 64-bit `rtlsdr.dll` (librtlsdr) next to the program. The environment variable `NADIRSTATION_RTLSDR_DLL` can point to
another location.

## Build (Windows, MinGW)

You need Qt 6 (MinGW 64-bit), CMake and the MinGW toolchain from the Qt installer. No other libraries.

1. Edit the three paths at the top of `scripts/build_release.bat` if Qt lives somewhere else.
2. Run `scripts\build_release.bat`. The program ends up in `dist\`; if Inno Setup 6 is installed, the installer is made in `installer\output\`.

Or by hand:

```
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=C:/Qt/6.11.2/mingw_64
cmake --build build --parallel 4
```

A command-line check of the receive chain without the window is built as well:

```
gvar_cli <recording> [sample rate in Hz] [output folder]
```

## Using it

**Recording:** choose "baseband recording", drop the file into the window or browse for it, press *Start*.
`.cs8` / `.cu8` / `.cs16` / `.cf32` files have no sample-rate field. The program looks for `rate <Hz>` in a `.txt` next to the
file, then for `3200000SPS` / `3.2Msps` in the file name, then uses the value you type. WAV files carry it in the header.
`.cu8` is what `rtl_sdr` writes (unsigned 8-bit).

**RTL-SDR:** choose "RTL-SDR live", set the frequency (1685.7 MHz), sample rate, gain and device number, press *Start*.
Watch the spectrum: the GVAR signal is a flat hump about 3.2 MHz wide. When block sync is found the status turns green and the
picture starts to grow. A full disc takes about 26 minutes; the program writes each finished observation to the output
folder and keeps going.

Output goes to `<output folder>\GOES GVAR\<observation time>\` (channel PNGs, false colour, JSON) and
`<output folder>\GOES GVAR\Telemetry_*\`. A small log is kept in `Documents\NadirStation_GOES_GVAR\runtime.log`.

## Licence

The source code is released under the **GNU General Public License v3.0 or later**, see `LICENSE`.
Copyright (c) 2026 AGcov (BI2QYN).

Third-party components keep their own licences: Qt (LGPL v3, dynamically linked) and the MinGW-w64 runtime.
librtlsdr is loaded at run time and is not part of this project. See `installer/THIRD_PARTY_NOTICES.txt` and `installer/LICENSES/`.

This is an amateur project. It is not affiliated with NOAA, NASA or the satellite operator, and it is not a weather service.
Do not use it where a wrong picture could cause harm. Read `installer/EULA.txt`.

---

## 简介（中文）

GOES-N/O/P（GOES-13/14/15）GVAR 成像仪数据的解码器：输入可以是**基带录音**（`.cs8` `.cu8` `.cs16` `.cf32` `.wav`）或
**RTL-SDR 实时接收**；输出五个通道的 PNG、假彩色图和块 0 遥测。只接收、不发射、不联网。GVAR 不加密，不需要任何密钥。
RTL-SDR 需要工作在 2.88–3.2 MS/s，接近 USB 的极限；请直连主板 USB 口。1685.7 MHz 需要 L 波段天线（抛物面加低噪放）。
`rtlsdr.dll` 需要自己放到程序目录。构建方法同上（Qt 6 + MinGW），运行 `scripts\build_release.bat` 即可。
