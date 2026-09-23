# Native build for the Arch ARM phone (libva headers + gcc on device).
# Usage on device: make            -> mtk_vcp_drv_video.so
#                  make test       -> + test_va_dec harness
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -Werror -fPIC -D_GNU_SOURCE
LDFLAGS_SO = -shared -Wl,-soname,mtk_vcp_drv_video.so
LDLIBS  = -lva
FFMPEG_CFLAGS = $(shell pkg-config --cflags libavcodec libavutil 2>/dev/null)
FFMPEG_LIBS = $(shell pkg-config --libs libavcodec libavutil 2>/dev/null || echo -lavcodec -lavutil)

SRCS = va_mtkvcp_init.c va_mtkvcp_v4l2.c va_mtkvcp_dec.c va_mtkvcp_enc.c \
       va_mtkvcp_img.c va_mtkvcp_ps.c va_mtkvcp_vpp.c
OBJS = $(SRCS:.c=.o)

all: mtk_vcp_drv_video.so

mtk_vcp_drv_video.so: $(OBJS)
	$(CC) $(LDFLAGS_SO) -o $@ $(OBJS) -lpthread -lm

test: all test_va_dec test_va_prime test_va_enc test_va_replica test_va_vpp \
      test_va_rgb_chain test_va_sunshine

test_va_dec: test_va_dec.c
	$(CC) $(CFLAGS) -o $@ $< -lva -lva-drm

test_va_prime: test_va_prime.c
	$(CC) $(CFLAGS) -o $@ $< -lva -lva-drm

test_va_enc: test_va_enc.c
	$(CC) $(CFLAGS) -o $@ $< -lva -lva-drm

test_va_replica: test_va_replica.c
	$(CC) $(CFLAGS) $(FFMPEG_CFLAGS) -o $@ $< $(FFMPEG_LIBS)

test_va_vpp: test_va_vpp.c
	$(CC) $(CFLAGS) -o $@ $< -lva -lva-drm

test_va_rgb_chain: test_va_rgb_chain.c
	$(CC) $(CFLAGS) -o $@ $< -lva -lva-drm

test_va_sunshine: test_va_sunshine.c
	$(CC) $(CFLAGS) -o $@ $< -lva -lva-drm

clean:
	rm -f $(OBJS) mtk_vcp_drv_video.so test_va_dec test_va_prime test_va_enc test_va_replica test_va_vpp test_va_dec_gpu test_va_rgb_chain test_va_sunshine

# Header changes must rebuild everything (struct layouts are shared).
$(OBJS): va_mtkvcp.h

.PHONY: all test clean

# GPU readback regression; persistent imports exercise reused CAPTURE buffers.
test_va_dec_gpu: test_va_dec.c test_dmabuf_gpu.h
	$(CC) $(CFLAGS) -DTEST_DMABUF_GPU -o $@ $< -lva -lva-drm -lEGL -lGLESv2 -lgbm
