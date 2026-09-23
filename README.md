# MT6895 VCP VA-API driver

A userspace [libva](https://github.com/intel/libva) backend for the MediaTek MT6895 video codec on mainline Linux. It exposes the stateful V4L2 decoder (`/dev/video1`) and encoder (`/dev/video0`) through VA-API. The implementation has been developed and tested on the Xiaomi xaga family (Redmi Note 11T Pro / POCO X4 GT / Redmi K50i).

The driver requires working MT6895 VCP firmware, kernel codec drivers, and a DRM render node. It does not provide firmware or kernel drivers. Hardware support and device-node numbering may differ on other systems.

## Features

| Operation | VA-API profiles |
| --- | --- |
| Decode (VLD) | H.264 Constrained Baseline, Main, High; HEVC Main, Main 10; VP9 Profile 0; MPEG-2 Simple, Main |
| Encode (EncSlice) | H.264 Constrained Baseline, Main, High; HEVC Main |
| VideoProc | RGB/NV12 conversion and scaling to NV12; linear DMA-BUF import |

The VideoProc conversion and scaling path runs on the CPU. The backend accepts only one active hardware codec session at a time. A decoder resolution change during a session requires the client to create a new context. Encoding support is limited to the modes accepted by the firmware and driver; advertising CQP/ICQ for quality-only clients currently maps their quality setting to a CBR bitrate.

## Build and install

Build natively on the target ARM64 device with a C compiler, Make, libva headers, and the corresponding V4L2 kernel headers:

```sh
make clean
make -j2
```

Install the resulting library where libva looks for DRM drivers (on the tested device, `/usr/lib/dri/`):

```sh
sudo install -m 755 mtk_vcp_drv_video.so /usr/lib/dri/mtk_vcp_drv_video.so
LIBVA_DRIVER_NAME=mtk_vcp vainfo --display drm --device /dev/dri/renderD128
```

The default V4L2 paths are `/dev/video1` for decoding and `/dev/video0` for encoding. Set `MTK_VCP_VA_DEC_NODE` and `MTK_VCP_VA_ENC_NODE` if your system uses different nodes. Rebuild from clean sources before deploying an updated library; the Makefile tracks changes to the shared header.

For a short FFmpeg encode smoke test:

```sh
LIBVA_DRIVER_NAME=mtk_vcp ffmpeg -hide_banner \
  -vaapi_device /dev/dri/renderD128 \
  -f lavfi -i 'testsrc2=size=1280x720:rate=30' \
  -vf 'format=nv12,hwupload' -frames:v 30 \
  -c:v h264_vaapi -y /tmp/mtk-vcp-test.h264
```

## Test programs

`make test` builds the driver and the C test programs. It compiles them; it does not run them. The tests also need libva-drm and FFmpeg development packages. The optional `test_va_dec_gpu` target additionally needs EGL, GLES, and GBM development files. These programs require a device with the supported codec stack and may occupy the single hardware session, so run them when other video clients are idle.

## Configuration

Set `MTK_VCP_VA_DEBUG=1` to log driver diagnostics. Set `MTK_VCP_VA_PROF=1` for periodic stage timing; `MTK_VCP_VA_PROF_FILE=/path/to/log` redirects that output to a file. `MTK_VCP_VA_RGB_INPUT=abgr32` selects the packed-RGB input mode when the client supplies that layout. `MTK_VCP_VA_ZEROCOPY=1` enables direct DMA-BUF handoff only for clients that retain ownership of each input buffer until encoding has finished; the default uses a copy to protect against producers reusing buffers early.

## License

MIT. See [LICENSE](LICENSE).
