/* SPDX-License-Identifier: MIT */
/* Parameter-set synthesis for stateful decode.
 *
 * VAAPI VLD clients built on ffmpeg deliver parsed parameter sets inside
 * the picture parameters and raw, unframed slice NALs in the slice data
 * buffers. A stateful V4L2 firmware interface needs complete Annex-B
 * access units, so this unit re-encodes SPS/PPS (+VPS for HEVC) from the
 * VA picture parameters and prefixes them to every submitted picture.
 * Re-emitting identical sets every picture is a spec-level no-op and
 * avoids all change-tracking hazards.
 *
 * Deliberate v1 limits (all fail closed with a log line, never silently
 * wrong pixels):
 * - H264 pic_order_cnt_type 1 (rare; fields absent from VA pic params).
 * - Custom scaling lists (client must not send non-flat IQ matrices).
 * - H264 crops are not reconstructed (visible size is VA-invisible).
 * A minimal VUI IS emitted: the firmware will not emit a picture until
 * its DPB/reorder window is full, and it sizes that window from
 * max_num_reorder_frames / max_dec_frame_buffering. Both are bounded by
 * MaxDpbFrames for the level (the value a decoder infers when
 * bitstream_restriction is absent), because advertising more than the
 * level allows is out of spec and stalls a lockstep client: the firmware
 * waits for the window to fill while the client waits for the first
 * picture. Visible-size handling stays client-side, exactly as with
 * probe-fed streams.
 */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <va/va.h>
#include <va/va_dec_hevc.h>
#include <stdio.h>
#include <stdlib.h>

#include "va_mtkvcp.h"

/* ---- bit writer ---- */
struct mtkvcp_bw {
    uint8_t *buf;
    size_t cap;
    size_t len;
    uint32_t cur;
    int nbits;
};

static int mtkvcp_bw_init(struct mtkvcp_bw *bw, size_t cap)
{
    bw->buf = malloc(cap ? cap : 1);
    if (!bw->buf)
        return -1;
    bw->cap = cap;
    bw->len = 0;
    bw->cur = 0;
    bw->nbits = 0;
    return 0;
}

static int mtkvcp_bw_bits(struct mtkvcp_bw *bw, uint32_t v, int n)
{
    while (n-- > 0) {
        bw->cur = (bw->cur << 1) | ((v >> n) & 1u);
        if (++bw->nbits == 8) {
            if (bw->len >= bw->cap)
                return -1;
            bw->buf[bw->len++] = (uint8_t)bw->cur;
            bw->cur = 0;
            bw->nbits = 0;
        }
    }
    return 0;
}

static int mtkvcp_bw_ue(struct mtkvcp_bw *bw, uint32_t v)
{
    uint32_t c = v + 1;
    int lz = 0;
    while (c >> (lz + 1))
        lz++;
    while (lz-- > 0)
        if (mtkvcp_bw_bits(bw, 0, 1) < 0)
            return -1;
    return mtkvcp_bw_bits(bw, c, 32 - __builtin_clz(c));
}

static int mtkvcp_bw_se(struct mtkvcp_bw *bw, int32_t v)
{
    uint32_t c = v > 0 ? (uint32_t)(2 * v - 1) : (uint32_t)(-2 * v);
    return mtkvcp_bw_ue(bw, c);
}

/* rbsp_trailing_bits + emulation prevention into out (cap). */
static int mtkvcp_bw_finish(struct mtkvcp_bw *bw, uint8_t *out, size_t cap,
                            size_t *outlen)
{
    size_t i, o = 0;
    int zeros = 0;
    if (mtkvcp_bw_bits(bw, 1, 1) < 0)
        return -1;
    while (bw->nbits > 0)
        if (mtkvcp_bw_bits(bw, 0, 1) < 0)
            return -1;
    for (i = 0; i < bw->len; i++) {
        uint8_t b = bw->buf[i];
        if (zeros >= 2 && b <= 3) {
            if (o >= cap)
                return -1;
            out[o++] = 3;
            zeros = 0;
        }
        if (o >= cap)
            return -1;
        out[o++] = b;
        zeros = b ? 0 : zeros + 1;
    }
    *outlen = o;
    return 0;
}

/* ---- Exp-Golomb reader (for slice pps_id) ---- */
struct mtkvcp_br {
    const uint8_t *buf;
    size_t len;
    size_t pos; /* bits */
};

static uint32_t mtkvcp_br_bits(struct mtkvcp_br *br, int n)
{
    uint32_t v = 0;
    while (n-- > 0) {
        size_t byte = br->pos >> 3;
        int bit = 7 - (br->pos & 7);
        v <<= 1;
        if (byte < br->len)
            v |= (uint32_t)((br->buf[byte] >> bit) & 1u);
        br->pos++;
    }
    return v;
}

static uint32_t mtkvcp_br_ue(struct mtkvcp_br *br)
{
    int lz = 0;
    while (mtkvcp_br_bits(br, 1) == 0 && lz < 32)
        lz++;
    return lz ? ((1u << lz) - 1u + mtkvcp_br_bits(br, lz)) : 0;
}

/* H264 slice pps_id: skip 1-byte NAL header, read 3 UEs. */
static int mtkvcp_h264_slice_pps_id(const uint8_t *nal, size_t len)
{
    struct mtkvcp_br br;
    if (len < 2)
        return -1;
    br.buf = nal + 1;
    br.len = len - 1;
    br.pos = 0;
    mtkvcp_br_ue(&br); /* first_mb_in_slice */
    mtkvcp_br_ue(&br); /* slice_type */
    return (int)mtkvcp_br_ue(&br);
}

/* ---- H264 SPS/PPS ---- */
static int mtkvcp_h264_profile_idc(int va_profile)
{
    switch (va_profile) {
    case VAProfileH264ConstrainedBaseline:
        return 66;
    case VAProfileH264Main:
        return 77;
    default:
        return 100;
    }
}

static int mtkvcp_h264_level_idc(int w, int h)
{
    size_t area = (size_t)w * (size_t)h;
    if (area <= 640u * 480u)
        return 30;
    if (area <= 1280u * 720u)
        return 31;
    if (area <= 1920u * 1088u)
        return 40;
    if (area <= 2560u * 1600u)
        return 42;
    return 51;
}

/* MaxDpbFrames for the level mtkvcp_h264_level_idc selects, i.e. the DPB
 * size a decoder infers when bitstream_restriction_flag is zero. The
 * firmware will not emit a picture until its DPB window is full, so this
 * value also bounds how far the client may lag before output starts. */
static unsigned int mtkvcp_h264_max_dpb_frames(int w, int h)
{
    static const struct {
        int level;
        unsigned int max_dpb_mbs;
    } table[] = {
        { 30, 8100 },    /* level 3.0 */
        { 31, 18000 },   /* level 3.1 */
        { 40, 32768 },   /* level 4.0 */
        { 42, 34816 },   /* level 4.2 */
        { 51, 184320 },  /* level 5.1 */
    };
    unsigned int max_dpb_mbs = table[0].max_dpb_mbs;
    unsigned int mbs, frames, i;
    int level = mtkvcp_h264_level_idc(w, h);

    for (i = 0; i < sizeof(table) / sizeof(table[0]); i++)
        if (table[i].level == level) {
            max_dpb_mbs = table[i].max_dpb_mbs;
            break;
        }
    /* PicWidthInMbs * FrameHeightInMbs, rounded up like the spec. */
    mbs = (unsigned int)((w + 15) / 16) * (unsigned int)((h + 15) / 16);
    if (!mbs)
        return 1;
    frames = max_dpb_mbs / mbs;
    if (frames > 16)
        frames = 16;
    if (frames < 1)
        frames = 1;
    return frames;
}

/* Serialize SPS RBSP. Returns bytes in out or <0. */
static int mtkvcp_h264_write_sps(const VAPictureParameterBufferH264 *pic,
                                 int va_profile, int w, int h,
                                 uint8_t *out, size_t cap, size_t *outlen)
{
    struct mtkvcp_bw bw;
    int r = -1;
    int profile_idc = mtkvcp_h264_profile_idc(va_profile);
    if (pic->seq_fields.bits.pic_order_cnt_type == 1)
        return -2; /* fields absent from VA pic params */
    if (mtkvcp_bw_init(&bw, 256) < 0)
        return -1;
    mtkvcp_log("sps in: prof=%d w=%d h=%d fnum=%u poct=%u poclsb=%u "
               "nref=%u gaps=%u mbs=%ux%u fmo=%u mbaff=%u d8=%u",
               va_profile, w, h,
               pic->seq_fields.bits.log2_max_frame_num_minus4,
               pic->seq_fields.bits.pic_order_cnt_type,
               pic->seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4,
               pic->num_ref_frames,
               pic->seq_fields.bits.gaps_in_frame_num_value_allowed_flag,
               pic->picture_width_in_mbs_minus1,
               pic->picture_height_in_mbs_minus1,
               pic->seq_fields.bits.frame_mbs_only_flag,
               pic->seq_fields.bits.mb_adaptive_frame_field_flag,
               pic->seq_fields.bits.direct_8x8_inference_flag);
    do {
        if (mtkvcp_bw_bits(&bw, (uint32_t)profile_idc, 8) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 8) < 0) /* constraint flags */
            break;
        if (mtkvcp_bw_bits(&bw,
                           (uint32_t)mtkvcp_h264_level_idc(w, h), 8) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* seq_parameter_set_id */
            break;
        if (profile_idc >= 100) {
            /* High-profile extra fields. Custom scaling lists are
             * refused upstream; only flat is emitted here. */
            uint32_t cfi = pic->seq_fields.bits.chroma_format_idc;
            if (cfi != 1 && cfi != 2)
                cfi = 1;
            if (mtkvcp_bw_ue(&bw, cfi) < 0)
                break;
            if (cfi == 3 &&
                mtkvcp_bw_bits(&bw, pic->seq_fields.bits
                                     .residual_colour_transform_flag,
                               1) < 0)
                break;
            if (mtkvcp_bw_ue(&bw, pic->bit_depth_luma_minus8) < 0)
                break;
            if (mtkvcp_bw_ue(&bw, pic->bit_depth_chroma_minus8) < 0)
                break;
            if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* qpprime bypass */
                break;
            if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* seq_scaling_matrix */
                break;
        }
        if (mtkvcp_bw_ue(&bw,
                         pic->seq_fields.bits.log2_max_frame_num_minus4) < 0)
            break;
        if (mtkvcp_bw_ue(&bw,
                         pic->seq_fields.bits.pic_order_cnt_type) < 0)
            break;
        if (pic->seq_fields.bits.pic_order_cnt_type == 0 &&
            mtkvcp_bw_ue(&bw, pic->seq_fields.bits
                             .log2_max_pic_order_cnt_lsb_minus4) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->num_ref_frames) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->seq_fields.bits
                                 .gaps_in_frame_num_value_allowed_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->picture_width_in_mbs_minus1) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->picture_height_in_mbs_minus1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->seq_fields.bits.frame_mbs_only_flag,
                           1) < 0)
            break;
        if (!pic->seq_fields.bits.frame_mbs_only_flag &&
            mtkvcp_bw_bits(&bw, pic->seq_fields.bits
                                 .mb_adaptive_frame_field_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->seq_fields.bits.direct_8x8_inference_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* frame_cropping_flag */
            break;
        /* Minimal VUI: the firmware sizes its DPB/reorder window from
         * max_num_reorder_frames/max_dec_frame_buffering, and it will
         * not emit a picture until its window is full. Advertising more
         * than the stream needs therefore deadlocks a lockstep client
         * (ffmpeg sends one packet and blocks in SyncSurface until the
         * firmware answers, which it never does because the window never
         * fills). Advertise the spec's inference instead: the stream's
         * own bitstream_restriction is not visible here, so use
         * MaxDpbFrames for the level, which is what a decoder infers
         * when the VUI is absent. Crops stay unreconstructed (visible
         * size is VA-invisible). */
        if (mtkvcp_bw_bits(&bw, 1, 1) < 0) /* vui_present */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* aspect_ratio_info */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* overscan_info */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* video_signal_type */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* chroma_loc_info */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* timing_info */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* nal_hrd */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* vcl_hrd */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* pic_struct */
            break;
        if (mtkvcp_bw_bits(&bw, 1, 1) < 0) /* bitstream_restriction */
            break;
        if (mtkvcp_bw_bits(&bw, 1, 1) < 0) /* mv_over_pic_boundaries */
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* max_bytes_per_pic_denom */
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* max_bits_per_mb_denom */
            break;
        if (mtkvcp_bw_ue(&bw, 12) < 0) /* log2_max_mv_length_h */
            break;
        if (mtkvcp_bw_ue(&bw, 12) < 0) /* log2_max_mv_length_v */
            break;
        {
            /* Both windows are the stream's reference count, capped at
             * MaxDpbFrames for the level (the value a decoder infers when
             * bitstream_restriction is absent, so it is the largest
             * window that is ever in spec).
             *
             * The firmware will not emit a picture until its window
             * fills, so an over-large window stalls a lockstep client:
             * the firmware waits for more input while the client blocks
             * in SyncSurface waiting for the first picture. Measured at
             * 1080p (level 4.0, MaxDpbFrames 4), one clip per reference
             * count, real ffmpeg client: a 2-reference stream produced no
             * frames at a window of 4 and every frame at 2; a 4-reference
             * stream produced no frames at 6 and every frame at 4. The
             * window therefore tracks the stream's own reference count,
             * never a fixed slack above it. */
            unsigned int maxdpb = mtkvcp_h264_max_dpb_frames(w, h);
            unsigned int nref = pic->num_ref_frames;
            unsigned int buffering = nref < maxdpb ? nref : maxdpb;
            unsigned int reorder = nref < maxdpb ? nref : maxdpb;

            if (!buffering)
                buffering = 1;
            if (!reorder)
                reorder = 1;
            if (mtkvcp_bw_ue(&bw, reorder) < 0) /* max_num_reorder */
                break;
            if (mtkvcp_bw_ue(&bw, buffering) < 0) /* max_dec_buffering */
                break;
        }
        r = 0;
    } while (0);
    if (!r)
        r = mtkvcp_bw_finish(&bw, out, cap, outlen);
    free(bw.buf);
    return r;
}

/* Serialize PPS RBSP. */
static int mtkvcp_h264_write_pps(const VAPictureParameterBufferH264 *pic,
                                 uint32_t pps_id, uint32_t ref_l0,
                                 uint32_t ref_l1, uint8_t *out, size_t cap,
                                 size_t *outlen)
{
    struct mtkvcp_bw bw;
    int r = -1;
    if (mtkvcp_bw_init(&bw, 128) < 0)
        return -1;
    do {
        if (mtkvcp_bw_ue(&bw, pps_id) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* seq_parameter_set_id */
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.entropy_coding_mode_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.pic_order_present_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* num_slice_groups_minus1 */
            break;
        if (mtkvcp_bw_ue(&bw, ref_l0) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, ref_l1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.weighted_pred_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.weighted_bipred_idc, 2) < 0)
            break;
        if (mtkvcp_bw_se(&bw, pic->pic_init_qp_minus26) < 0)
            break;
        if (mtkvcp_bw_se(&bw, pic->pic_init_qs_minus26) < 0)
            break;
        if (mtkvcp_bw_se(&bw, pic->chroma_qp_index_offset) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .deblocking_filter_control_present_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .constrained_intra_pred_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .redundant_pic_cnt_present_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.transform_8x8_mode_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* pic_scaling_matrix_present */
            break;
        if (mtkvcp_bw_se(&bw, pic->second_chroma_qp_index_offset) < 0)
            break;
        r = 0;
    } while (0);
    if (!r)
        r = mtkvcp_bw_finish(&bw, out, cap, outlen);
    free(bw.buf);
    return r;
}

/*
 * Build SPS + one PPS per referenced id, each framed with a 4-byte
 * start code, into out. pps_ids/nids come from the picture's slices.
 * Returns total bytes or <0 (fail closed).
 */
int mtkvcp_h264_build_ps(const VAPictureParameterBufferH264 *pic,
                         int va_profile, int w, int h,
                         const int *pps_ids, int nids,
                         uint32_t ref_l0, uint32_t ref_l1,
                         uint8_t *out, size_t cap)
{
    uint8_t rbsp[256];
    size_t rlen = 0, o = 0;
    int i, r;
    if (nids <= 0 || nids > 8)
        return -1;
    r = mtkvcp_h264_write_sps(pic, va_profile, w, h, rbsp, sizeof(rbsp),
                              &rlen);
    if (r == -2) {
        mtkvcp_log("H264 pic_order_cnt_type 1 unsupported (v1)");
        return -1;
    }
    if (r < 0)
        return -1;
    if (o + 4 + rlen > cap)
        return -1;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 1; /* NAL header for SPS (forbidden 0, ref 3, type 7) */
    out[o++] = 0x67;
    memcpy(out + o, rbsp, rlen);
    o += rlen;
    for (i = 0; i < nids; i++) {
        if (o + 4 + 1 + 64 > cap)
            return -1;
        r = mtkvcp_h264_write_pps(pic, (uint32_t)pps_ids[i], ref_l0,
                                  ref_l1, rbsp, sizeof(rbsp), &rlen);
        if (r < 0)
            return -1;
        out[o++] = 0;
        out[o++] = 0;
        out[o++] = 0;
        out[o++] = 1;
        out[o++] = 0x68; /* PPS */
        memcpy(out + o, rbsp, rlen);
        o += rlen;
    }
    return (int)o;
}

/* Wraps the slice NAL for pps_id parsing (1-byte H264 header). */
int mtkvcp_h264_nal_pps_id(const uint8_t *nal, size_t len)
{
    return mtkvcp_h264_slice_pps_id(nal, len);
}

/* ---- HEVC VPS/SPS/PPS ---- */
static int mtkvcp_hevc_level_idc(int w, int h)
{
    size_t area = (size_t)w * (size_t)h;
    if (area <= 640u * 480u)
        return 90;
    if (area <= 1280u * 720u)
        return 120;
    if (area <= 1920u * 1088u)
        return 123;
    if (area <= 3840u * 2160u)
        return 150;
    return 153;
}

/* profile_tier_level with a single sub-layer. */
static int mtkvcp_hevc_write_ptl(struct mtkvcp_bw *bw, int profile_idc,
                                 int level_idc)
{
    uint32_t compat = profile_idc >= 1 && profile_idc <= 32 ?
                      1u << (32 - profile_idc) : 0;
    if (mtkvcp_bw_bits(bw, 0, 2) < 0) /* profile_space */
        return -1;
    if (mtkvcp_bw_bits(bw, 0, 1) < 0) /* tier */
        return -1;
    if (mtkvcp_bw_bits(bw, (uint32_t)profile_idc, 5) < 0)
        return -1;
    if (mtkvcp_bw_bits(bw, compat, 32) < 0)
        return -1;
    if (mtkvcp_bw_bits(bw, 1, 1) < 0) /* progressive */
        return -1;
    if (mtkvcp_bw_bits(bw, 0, 1) < 0) /* interlaced */
        return -1;
    if (mtkvcp_bw_bits(bw, 1, 1) < 0) /* non-packed */
        return -1;
    if (mtkvcp_bw_bits(bw, 1, 1) < 0) /* frame-only */
        return -1;
    {
        int i;
        for (i = 0; i < 43; i++)
            if (mtkvcp_bw_bits(bw, 0, 1) < 0)
                return -1;
    }
    if (mtkvcp_bw_bits(bw, 0, 1) < 0) /* inbld */
        return -1;
    if (mtkvcp_bw_bits(bw, (uint32_t)level_idc, 8) < 0)
        return -1;
    return 0;
}

static int mtkvcp_hevc_write_vps(int profile_idc, int level_idc,
                                 uint8_t *out, size_t cap, size_t *outlen)
{
    struct mtkvcp_bw bw;
    int r = -1;
    (void)level_idc;
    if (mtkvcp_bw_init(&bw, 64) < 0)
        return -1;
    do {
        if (mtkvcp_bw_bits(&bw, 0, 4) < 0) /* vps_id */
            break;
        if (mtkvcp_bw_bits(&bw, 3, 2) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 6) < 0) /* max_layers-1 */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 3) < 0) /* max_sub_layers-1 */
            break;
        if (mtkvcp_bw_bits(&bw, 1, 1) < 0) /* temporal nesting */
            break;
        if (mtkvcp_bw_bits(&bw, 0xffff, 16) < 0)
            break;
        if (mtkvcp_hevc_write_ptl(&bw, profile_idc, level_idc) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* ordering_info_present */
            break;
        if (mtkvcp_bw_ue(&bw, 4) < 0) /* buffering nominal */
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 6) < 0) /* max_layer_id */
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* num_layer_sets-1 */
            break;
        if (mtkvcp_bw_bits(&bw, 1, 1) < 0) /* layer_id_included */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* timing_info */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* extension */
            break;
        r = 0;
    } while (0);
    if (!r)
        r = mtkvcp_bw_finish(&bw, out, cap, outlen);
    free(bw.buf);
    return r;
}

static int mtkvcp_hevc_write_sps(const VAPictureParameterBufferHEVC *pic,
                                 int profile_idc, int w, int h,
                                 uint8_t *out, size_t cap, size_t *outlen)
{
    struct mtkvcp_bw bw;
    int r = -1;
    if (mtkvcp_bw_init(&bw, 256) < 0)
        return -1;
    do {
        if (mtkvcp_bw_bits(&bw, 0, 4) < 0) /* vps_id */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 3) < 0) /* max_sub_layers-1 */
            break;
        if (mtkvcp_bw_bits(&bw, 1, 1) < 0) /* temporal nesting */
            break;
        if (mtkvcp_hevc_write_ptl(&bw, profile_idc,
                                   mtkvcp_hevc_level_idc(w, h)) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* sps_id */
            break;
        if (mtkvcp_bw_ue(&bw, pic->pic_fields.bits.chroma_format_idc) < 0)
            break;
        if (pic->pic_fields.bits.chroma_format_idc == 3 &&
            mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .separate_colour_plane_flag, 1) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->pic_width_in_luma_samples) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->pic_height_in_luma_samples) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* conformance_window */
            break;
        if (mtkvcp_bw_ue(&bw, pic->bit_depth_luma_minus8) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->bit_depth_chroma_minus8) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->log2_max_pic_order_cnt_lsb_minus4) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* ordering_info_present */
            break;
        if (mtkvcp_bw_ue(&bw, pic->sps_max_dec_pic_buffering_minus1) < 0)
            break;
        {
            /* Reorder depth is not carried by VA pic params; bound it
             * by the signalled buffering depth (nominal, documented). */
            uint32_t reorder = pic->sps_max_dec_pic_buffering_minus1;
            if (pic->pic_fields.bits.NoPicReorderingFlag)
                reorder = 0;
            else if (reorder > 4)
                reorder = 4;
            if (mtkvcp_bw_ue(&bw, reorder) < 0)
                break;
        }
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* max_latency */
            break;
        if (mtkvcp_bw_ue(&bw, pic->log2_min_luma_coding_block_size_minus3) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->log2_diff_max_min_luma_coding_block_size) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->log2_min_transform_block_size_minus2) < 0)
            break;
        if (mtkvcp_bw_ue(&bw,
                         pic->log2_diff_max_min_transform_block_size) < 0)
            break;
        if (mtkvcp_bw_ue(&bw,
                         pic->max_transform_hierarchy_depth_inter) < 0)
            break;
        if (mtkvcp_bw_ue(&bw,
                         pic->max_transform_hierarchy_depth_intra) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* scaling_list_enabled */
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.amp_enabled_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .sample_adaptive_offset_enabled_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits.pcm_enabled_flag,
                           1) < 0)
            break;
        if (pic->pic_fields.bits.pcm_enabled_flag) {
            mtkvcp_log("HEVC PCM enabled unsupported (v1)");
            break;
        }
        /* Empty RPS (v1): slices referencing SPS sets fail closed. */
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* num_short_term_ref_pic_sets */
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* long_term_present */
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .sps_temporal_mvp_enabled_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .strong_intra_smoothing_enabled_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* extension_present */
            break;
        (void)w;
        (void)h;
        r = 0;
    } while (0);
    if (!r)
        r = mtkvcp_bw_finish(&bw, out, cap, outlen);
    free(bw.buf);
    return r;
}

static int mtkvcp_hevc_write_pps(const VAPictureParameterBufferHEVC *pic,
                                 uint32_t pps_id, uint8_t *out, size_t cap,
                                 size_t *outlen)
{
    struct mtkvcp_bw bw;
    int r = -1, ctrl;
    if (mtkvcp_bw_init(&bw, 256) < 0)
        return -1;
    ctrl = pic->slice_parsing_fields.bits
               .deblocking_filter_override_enabled_flag ||
           pic->slice_parsing_fields.bits.pps_disable_deblocking_filter_flag;
    do {
        if (mtkvcp_bw_ue(&bw, pps_id) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, 0) < 0) /* sps_id */
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .dependent_slice_segments_enabled_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .output_flag_present_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->num_extra_slice_header_bits, 3) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .sign_data_hiding_enabled_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .cabac_init_present_flag, 1) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->num_ref_idx_l0_default_active_minus1) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->num_ref_idx_l1_default_active_minus1) < 0)
            break;
        if (mtkvcp_bw_se(&bw, pic->init_qp_minus26) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .constrained_intra_pred_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .transform_skip_enabled_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .cu_qp_delta_enabled_flag, 1) < 0)
            break;
        if (pic->pic_fields.bits.cu_qp_delta_enabled_flag &&
            mtkvcp_bw_ue(&bw, pic->diff_cu_qp_delta_depth) < 0)
            break;
        if (mtkvcp_bw_se(&bw, pic->pps_cb_qp_offset) < 0)
            break;
        if (mtkvcp_bw_se(&bw, pic->pps_cr_qp_offset) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .pps_slice_chroma_qp_offsets_present_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.weighted_pred_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.weighted_bipred_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .transquant_bypass_enabled_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw,
                           pic->pic_fields.bits.tiles_enabled_flag, 1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .entropy_coding_sync_enabled_flag,
                           1) < 0)
            break;
        if (pic->pic_fields.bits.tiles_enabled_flag) {
            /* Uniform spacing assumed (v1, documented). */
            if (mtkvcp_bw_bits(&bw, 1, 1) < 0)
                break;
            if (mtkvcp_bw_ue(&bw, pic->num_tile_columns_minus1) < 0)
                break;
            if (mtkvcp_bw_ue(&bw, pic->num_tile_rows_minus1) < 0)
                break;
            if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                     .loop_filter_across_tiles_enabled_flag,
                               1) < 0)
                break;
        }
        if (mtkvcp_bw_bits(&bw, pic->pic_fields.bits
                                 .pps_loop_filter_across_slices_enabled_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, ctrl, 1) < 0)
            break;
        if (ctrl) {
            if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                     .deblocking_filter_override_enabled_flag,
                               1) < 0)
                break;
            if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                     .pps_disable_deblocking_filter_flag,
                               1) < 0)
                break;
            if (!pic->slice_parsing_fields.bits
                     .pps_disable_deblocking_filter_flag) {
                if (mtkvcp_bw_se(&bw, pic->pps_beta_offset_div2) < 0)
                    break;
                if (mtkvcp_bw_se(&bw, pic->pps_tc_offset_div2) < 0)
                    break;
            }
        }
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* scaling_list_data */
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .lists_modification_present_flag, 1) < 0)
            break;
        if (mtkvcp_bw_ue(&bw, pic->log2_parallel_merge_level_minus2) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, pic->slice_parsing_fields.bits
                                 .slice_segment_header_extension_present_flag,
                           1) < 0)
            break;
        if (mtkvcp_bw_bits(&bw, 0, 1) < 0) /* extension_present */
            break;
        r = 0;
    } while (0);
    if (!r)
        r = mtkvcp_bw_finish(&bw, out, cap, outlen);
    free(bw.buf);
    return r;
}

/* HEVC slice pps_id: 2-byte NAL header, then u1/u1?/ue. */
static int mtkvcp_hevc_slice_pps_id(const uint8_t *nal, size_t len,
                                    int dependent_enabled)
{
    struct mtkvcp_br br;
    int type;
    if (len < 3)
        return -1;
    type = (nal[0] >> 1) & 0x3f;
    br.buf = nal + 2;
    br.len = len - 2;
    br.pos = 0;
    mtkvcp_br_bits(&br, 1); /* first_slice_segment_in_pic_flag */
    if (type >= 16 && type <= 23)
        mtkvcp_br_bits(&br, 1); /* no_output_of_prior_pics_flag */
    (void)dependent_enabled;
    return (int)mtkvcp_br_ue(&br);
}

/* Build VPS + SPS + one PPS per id, start-code framed. */
int mtkvcp_hevc_build_ps(const VAPictureParameterBufferHEVC *pic,
                         int profile_idc, int w, int h,
                         const int *pps_ids, int nids,
                         uint8_t *out, size_t cap)
{
    uint8_t rbsp[512];
    size_t rlen = 0, o = 0;
    int i, r;
    int level_idc = mtkvcp_hevc_level_idc(w, h);
    uint8_t hdr[2];
    if (nids <= 0 || nids > 8)
        return -1;
    /* VPS: nal_unit_type 32. */
    r = mtkvcp_hevc_write_vps(profile_idc, level_idc, rbsp, sizeof(rbsp),
                              &rlen);
    if (r < 0)
        return -1;
    hdr[0] = (uint8_t)(32 << 1);
    hdr[1] = 1;
    if (o + 4 + 2 + rlen > cap)
        return -1;
    out[o++] = 0; out[o++] = 0; out[o++] = 0; out[o++] = 1;
    out[o++] = hdr[0]; out[o++] = hdr[1];
    memcpy(out + o, rbsp, rlen);
    o += rlen;
    /* SPS: type 33. */
    r = mtkvcp_hevc_write_sps(pic, profile_idc, w, h, rbsp, sizeof(rbsp),
                              &rlen);
    if (r < 0)
        return -1;
    hdr[0] = (uint8_t)(33 << 1);
    if (o + 4 + 2 + rlen > cap)
        return -1;
    out[o++] = 0; out[o++] = 0; out[o++] = 0; out[o++] = 1;
    out[o++] = hdr[0]; out[o++] = hdr[1];
    memcpy(out + o, rbsp, rlen);
    o += rlen;
    /* PPS: type 34, one per referenced id. */
    for (i = 0; i < nids; i++) {
        r = mtkvcp_hevc_write_pps(pic, (uint32_t)pps_ids[i], rbsp,
                                  sizeof(rbsp), &rlen);
        if (r < 0)
            return -1;
        hdr[0] = (uint8_t)(34 << 1);
        if (o + 4 + 2 + rlen > cap)
            return -1;
        out[o++] = 0; out[o++] = 0; out[o++] = 0; out[o++] = 1;
        out[o++] = hdr[0]; out[o++] = hdr[1];
        memcpy(out + o, rbsp, rlen);
        o += rlen;
    }
    return (int)o;
}

int mtkvcp_hevc_nal_pps_id(const uint8_t *nal, size_t len)
{
    return mtkvcp_hevc_slice_pps_id(nal, len, 0);
}

/* ---- MPEG-2 sequence/picture headers ----
 *
 * ffmpeg strips sequence/GOP/picture headers and sends bare slices
 * (with start codes). Rebuild per picture: sequence header, sequence
 * extension, picture header, picture coding extension. Every-picture
 * repeats are legal and avoid change tracking.
 *
 * v1 heuristics (documented, fail-observable not silent):
 * - aspect 1:1, 25fps, max nominal bitrate/vbv (timing/level only).
 * - temporal_reference is a running counter (display association only;
 *   the firmware decodes in bitstream order).
 * - full_pel vectors assumed absent (ancient content only).
 * - Matrices: client lists when they differ from the spec defaults,
 *   else the defaults (explicit-default equals absent).
 */
const uint8_t mtkvcp_mpeg2_intra_default[64] = {
    8, 16, 19, 22, 26, 27, 29, 34,
    16, 16, 22, 24, 27, 29, 34, 37,
    19, 22, 26, 27, 29, 34, 34, 38,
    22, 22, 26, 27, 29, 34, 37, 40,
    22, 26, 27, 29, 32, 35, 40, 48,
    26, 27, 29, 32, 35, 40, 48, 58,
    26, 27, 29, 34, 38, 46, 56, 69,
    27, 29, 35, 38, 46, 56, 69, 83
};

static void mtkvcp_mpeg2_put32(uint8_t **pp, uint32_t v, int nbits,
                               uint32_t *cur, int *nb, uint8_t *out,
                               size_t cap, size_t *o)
{
    (void)pp;
    while (nbits-- > 0) {
        *cur = (*cur << 1) | ((v >> nbits) & 1u);
        if (++(*nb) == 8) {
            if (*o < cap)
                out[(*o)++] = (uint8_t)*cur;
            *cur = 0;
            *nb = 0;
        }
    }
}

int mtkvcp_mpeg2_build_ps(const VAPictureParameterBufferMPEG2 *pic,
                          int va_profile,
                          const uint8_t *iq_intra, const uint8_t *iq_nonintra,
                          int temporal_ref, uint8_t *out, size_t cap)
{
    size_t o = 0;
    uint32_t cur = 0;
    int nb = 0, i;
    int use_intra = iq_intra != NULL;
    int use_nonintra = iq_nonintra != NULL;
    int profile = va_profile == VAProfileMPEG2Simple ? 5 : 4;
#define P32(v, n) mtkvcp_mpeg2_put32(NULL, (v), (n), &cur, &nb, out, cap, &o)
#define FLUSH()                                     \
    do {                                            \
        while (nb > 0) {                            \
            cur <<= 1;                              \
            if (++nb == 8) {                        \
                if (o < cap)                        \
                    out[o++] = (uint8_t)cur;        \
                cur = 0;                            \
                nb = 0;                             \
            }                                       \
        }                                           \
    } while (0)
    /* sequence_header_code */
    if (o + 4 > cap)
        return -1;
    out[o++] = 0; out[o++] = 0; out[o++] = 1; out[o++] = 0xb3;
    P32(pic->horizontal_size, 12);
    P32(pic->vertical_size, 12);
    P32(1, 4);          /* aspect_ratio_information: square */
    P32(3, 4);          /* frame_rate_code: 25 */
    P32(0x3ffff, 18);   /* bit_rate_value: nominal max */
    P32(1, 1);          /* marker_bit */
    P32(0x3ff, 10);     /* vbv_buffer_size_value: nominal max */
    P32(0, 1);          /* constrained_parameters_flag */
    P32(use_intra, 1);
    if (use_intra)
        for (i = 0; i < 64; i++)
            P32(iq_intra[i], 8);
    P32(use_nonintra, 1);
    if (use_nonintra)
        for (i = 0; i < 64; i++)
            P32(iq_nonintra[i], 8);
    FLUSH();
    /* sequence_extension */
    if (o + 4 > cap)
        return -1;
    {
        size_t ext_start;
        size_t dbg;
        char msg[256];
        int ml = 0;
    out[o++] = 0; out[o++] = 0; out[o++] = 1; out[o++] = 0xb5;
    ext_start = o;
    P32(1, 4);          /* extension_start_code_identifier */
    P32(0, 1);          /* escape bit */
    P32((uint32_t)profile, 3);
    P32(8, 4);          /* level: Main */
    P32(pic->picture_coding_extension.bits.progressive_frame, 1);
    P32(1, 2);          /* chroma_format: 4:2:0 */
    P32(0, 2);          /* horizontal_size_extension */
    P32(0, 2);          /* vertical_size_extension */
    P32(0, 12);         /* bit_rate_extension */
    P32(1, 1);          /* marker_bit */
    P32(0, 8);          /* vbv_buffer_size_extension */
    P32(0, 1);          /* low_delay */
    P32(0, 2);          /* frame_rate_extension_n */
    P32(0, 5);          /* frame_rate_extension_d */
    FLUSH();
        ml += snprintf(msg + ml, sizeof(msg) - ml, "mp2 ext bytes:");
        for (dbg = ext_start; dbg < o && dbg < ext_start + 8; dbg++)
            ml += snprintf(msg + ml, sizeof(msg) - ml, " %02x", out[dbg]);
        mtkvcp_log("%s", msg);
    }
    /* picture_header */
    if (o + 4 > cap)
        return -1;
    out[o++] = 0; out[o++] = 0; out[o++] = 1; out[o++] = 0x00;
    P32((uint32_t)(temporal_ref & 0x3ff), 10);
    P32((uint32_t)pic->picture_coding_type, 3);
    P32(0xffff, 16);    /* vbv_delay: VBR marker */
    /* ONE 3-bit f_code per direction here (the h/v split lives only
     * in the extension); emitting two would parse as extra_information
     * and desynchronize the header. */
    if (pic->picture_coding_type == 2 || pic->picture_coding_type == 3) {
        P32(0, 1);      /* full_pel_forward_vector (v1: absent) */
        P32((uint32_t)((pic->f_code >> 8) & 0x7), 3);
    }
    if (pic->picture_coding_type == 3) {
        P32(0, 1);
        P32((uint32_t)(pic->f_code & 0x7), 3);
    }
    FLUSH();
    /* picture_coding_extension */
    if (o + 4 > cap)
        return -1;
    out[o++] = 0; out[o++] = 0; out[o++] = 1; out[o++] = 0xb5;
    P32(8, 4);          /* extension_start_code_identifier */
    {
        /* Extension order is fh, fv, bh, bv (4 bits each). */
        int fc = pic->f_code;
        P32((uint32_t)((fc >> 8) & 0xf), 4);
        P32((uint32_t)((fc >> 12) & 0xf), 4);
        P32((uint32_t)(fc & 0xf), 4);
        P32((uint32_t)((fc >> 4) & 0xf), 4);
    }
    P32(pic->picture_coding_extension.bits.intra_dc_precision, 2);
    P32(pic->picture_coding_extension.bits.picture_structure, 2);
    P32(pic->picture_coding_extension.bits.top_field_first, 1);
    P32(pic->picture_coding_extension.bits.frame_pred_frame_dct, 1);
    P32(pic->picture_coding_extension.bits.concealment_motion_vectors, 1);
    P32(pic->picture_coding_extension.bits.q_scale_type, 1);
    P32(pic->picture_coding_extension.bits.intra_vlc_format, 1);
    P32(pic->picture_coding_extension.bits.alternate_scan, 1);
    P32(pic->picture_coding_extension.bits.repeat_first_field, 1);
    /* chroma_420_type tracks progressive_frame for our scope. */
    P32(pic->picture_coding_extension.bits.progressive_frame, 1);
    P32(pic->picture_coding_extension.bits.progressive_frame, 1);
    P32(0, 1);          /* composite_display_flag */
    FLUSH();
    return (int)o;
#undef P32
#undef FLUSH
}
