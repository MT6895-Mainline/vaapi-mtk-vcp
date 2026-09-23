/* SPDX-License-Identifier: MIT */
/* Replicates FreeRDP VAAPI-encode init exactly:
 * device create -> h264_vaapi -> hw_frames pool 20 NV12 640x480 ->
 * open2 -> get_buffer + transfer + send/receive 3 frames.
 * Usage: test_va_replica [/dev/dri/renderD128]
 */
#include <stdio.h>
#include <string.h>
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_vaapi.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/dri/renderD128";
    AVBufferRef *hwctx = 0, *frames = 0;
    const AVCodec *enc;
    AVCodecContext *ctx;
    int r, i;
    r = av_hwdevice_ctx_create(&hwctx, AV_HWDEVICE_TYPE_VAAPI, dev, 0, 0);
    printf("device create: %d\n", r);
    if (r < 0)
        return 1;
    enc = avcodec_find_encoder_by_name("h264_vaapi");
    printf("encoder: %s\n", enc ? enc->name : "(none)");
    if (!enc)
        return 1;
    ctx = avcodec_alloc_context3(enc);
    ctx->width = 640;
    ctx->height = 480;
    ctx->delay = 0;
    ctx->framerate = (AVRational){ 30, 1 };
    ctx->time_base = (AVRational){ 1, 30 };
    ctx->pix_fmt = AV_PIX_FMT_VAAPI;
    av_opt_set(ctx, "tune", "zerolatency", 0);
    frames = av_hwframe_ctx_alloc(hwctx);
    {
        AVHWFramesContext *fc = (AVHWFramesContext *)frames->data;
        fc->format = AV_PIX_FMT_VAAPI;
        fc->sw_format = AV_PIX_FMT_NV12;
        fc->width = 640;
        fc->height = 480;
        fc->initial_pool_size = 20;
    }
    r = av_hwframe_ctx_init(frames);
    printf("frames init: %d\n", r);
    if (r < 0)
        return 1;
    ctx->hw_frames_ctx = av_buffer_ref(frames);
    r = avcodec_open2(ctx, enc, 0);
    printf("open2: %d\n", r);
    if (r < 0)
        return 1;
    for (i = 0; i < 3; i++) {
        AVFrame *hw = av_frame_alloc(), *sw = av_frame_alloc();
        AVPacket *pkt = av_packet_alloc();
        sw->format = AV_PIX_FMT_NV12;
        sw->width = 640;
        sw->height = 480;
        av_frame_get_buffer(sw, 0);
        memset(sw->data[0], 0x80, 640 * 480);
        memset(sw->data[1], 0x80, 640 * 240);
        r = av_hwframe_get_buffer(ctx->hw_frames_ctx, hw, 0);
        printf("frame %d get_buffer: %d\n", i, r);
        if (!r)
            r = av_hwframe_transfer_data(hw, sw, 0);
        printf("frame %d transfer: %d\n", i, r);
        if (!r)
            r = avcodec_send_frame(ctx, hw);
        printf("frame %d send: %d\n", i, r);
        if (!r)
            r = avcodec_receive_packet(ctx, pkt);
        printf("frame %d receive: %d size=%d\n", i, r,
               r == 0 ? pkt->size : -1);
        av_frame_free(&hw);
        av_frame_free(&sw);
        av_packet_free(&pkt);
        if (r < 0 && r != AVERROR(EAGAIN))
            return 1;
    }
    printf("REPLICA-OK\n");
    return 0;
}
