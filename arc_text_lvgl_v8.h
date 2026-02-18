#pragma once
#include <lvgl.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include "esp_heap_caps.h"

namespace arc_text {

/*
 * arc_text_lvgl_v8.h — curved text renderer for LVGL 8 on ESP32-S3
 *
 * Renders a string of glyphs along a circular arc and exposes the result as a
 * single lv_img widget.  Does not require LV_USE_CANVAS.
 *
 * Rendering pipeline (called on every set_text / set_config):
 *   1. The PSRAM framebuffer is cleared to fully transparent.
 *   2. Each glyph is processed in turn:
 *        a. Its raw alpha bitmap is fetched via lv_font_get_glyph_bitmap() and
 *           stamped into a small 64×64 scratch buffer at the correct
 *           baseline-relative position.
 *        b. If stroke is enabled, the glyph is stamped again at each neighbour
 *           offset (4- or 8-direction), merging coverage via max-blend to form
 *           the outline alpha map.
 *        c. Both the stroke and the front alpha maps are rotation-blitted into
 *           the PSRAM framebuffer using backward-mapped nearest-neighbour
 *           sampling and Porter-Duff "src over" compositing.
 *   3. lv_obj_invalidate() queues a single dirty rect, resulting in one DMA
 *      blit to the display on the next LVGL flush.
 *
 * Baseline alignment:
 *   Every glyph is positioned so that its typographic baseline sits exactly on
 *   the arc circle at radius r_base.  This is achieved by deriving the glyph's
 *   position in the 64×64 scratch buffer from gdsc.ofs_y (the signed distance
 *   from the baseline to the bottom of the glyph bounding box), rather than
 *   centering on the bounding box.  This ensures descenders (g, y, p, etc.)
 *   hang correctly below the arc without disturbing the placement of other
 *   glyphs.
 *
 *   Placement derivation (y increases downward):
 *     gdsc.ofs_y = distance from baseline to bottom of bounding box
 *                  (positive: bottom is above baseline; negative: descender)
 *     glyph_bottom_in_buffer = PIVOT - gdsc.ofs_y          (PIVOT = 32)
 *     oy (bitmap top row)    = PIVOT - gdsc.ofs_y - box_h
 *     ox (bitmap left col)   = PIVOT - adv_px/2 + gdsc.ofs_x
 *
 * Memory:
 *   PSRAM framebuffer : CANVAS_W x CANVAS_H x PX_BYTES  (~380 KB for 360x360 @ 16bpp)
 *   Stroke alpha map  : GLYPH_MAX x GLYPH_MAX = 4 KB (BSS)
 *   Front alpha map   : GLYPH_MAX x GLYPH_MAX = 4 KB (BSS)
 */

// ---------------------------------------------------------------------------
// Canvas geometry
// Must match the arc_text_container widget dimensions defined in the YAML.
// ---------------------------------------------------------------------------
static const lv_coord_t CANVAS_W = 360;
static const lv_coord_t CANVAS_H = 360;

// Scratch buffer size for per-glyph rendering.
// Must be >= max glyph bounding-box height + (2 * stroke_px) + a small margin.
// At 64 px, GLYPH_MAX/2 = 32 is the baseline pivot row.  Glyphs must satisfy:
//   box_h + |ofs_y| < 32  (true for roboto_28 and similar fonts at this size).
static const int GLYPH_MAX = 64;

// Pixel stride for LV_IMG_CF_TRUE_COLOR_ALPHA:
//   sizeof(lv_color_t) colour bytes followed by 1 alpha byte.
static const int PX_BYTES = (int)sizeof(lv_color_t) + 1;

// ---------------------------------------------------------------------------
// Performance statistics
// Reset periodically via reset_stats() or stats_log_and_reset().
// ---------------------------------------------------------------------------
struct Stats {
  uint32_t set_text_calls  = 0;  // total calls to set_text()
  uint32_t set_text_same   = 0;  // calls that were no-ops (text unchanged)
  uint32_t layout_calls    = 0;  // total full redraws
  uint32_t last_layout_us  = 0;  // duration of the most recent redraw
  uint64_t total_layout_us = 0;  // cumulative redraw time
  uint32_t glyphs          = 0;  // glyph count from the last render
};
static Stats stats_;

// ---------------------------------------------------------------------------
// Configuration
// Pass a populated Config to set_config() to apply changes and re-render.
// ---------------------------------------------------------------------------
struct Config {
  // Arc geometry
  int   cx              = 180;      // canvas x-centre of the arc circle (px)
  int   cy              = 180;      // canvas y-centre of the arc circle (px)
  float r_base          = 160.0f;   // radius from centre to text baseline (px)
  float mid_deg         = 90.0f;    // angle at which the text string is centred
                                    //   (0 = right, 90 = bottom, 180 = left, 270 = top)
  bool  clockwise       = false;    // glyph travel direction along the arc
  bool  outward         = false;    // true: tops of glyphs face outward (away from centre)
                                    // false: tops face inward (toward centre)
  float extra_letter_px = 1.0f;     // additional spacing added between glyphs (px)
  float max_span_deg    = 320.0f;   // maximum arc angle the string may occupy
  bool  ellipsize       = true;     // truncate with U+2026 when text exceeds max_span_deg

  // Glyph fill (drawn on top of the outline)
  lv_color_t front_color = lv_color_hex(0x000000);  // black
  uint8_t    front_opa   = LV_OPA_COVER;

  // Outline (drawn first, beneath the fill)
  // Swap front_color and stroke_color for white-outline-on-black vs black-outline-on-white.
  bool       stroke_enable    = true;
  int        stroke_px        = 2;      // outline thickness in pixels
  bool       stroke_diagonals = false;  // false: 4-neighbour outline (faster)
                                        // true:  8-neighbour outline (smoother corners)
  lv_color_t stroke_color     = lv_color_hex(0xFFFFFF);  // white
  uint8_t    stroke_opa       = 200;    // outline opacity (0-255); slightly transparent
                                        // softens the outline against busy backgrounds
};

// ---------------------------------------------------------------------------
// Module state (internal)
// ---------------------------------------------------------------------------
static lv_obj_t*    parent_     = nullptr;
static lv_obj_t*    img_obj_    = nullptr;   // the single visible LVGL widget
static uint8_t*     canvas_buf_ = nullptr;   // PSRAM-backed pixel buffer
static lv_img_dsc_t img_dsc_    = {};        // LVGL image descriptor pointing at canvas_buf_

// Per-glyph 8-bit alpha scratch maps; reused for every glyph (BSS, not heap)
static uint8_t stroke_alpha_[GLYPH_MAX * GLYPH_MAX];
static uint8_t front_alpha_ [GLYPH_MAX * GLYPH_MAX];

static std::string text_;
static Config      cfg_;

// ---------------------------------------------------------------------------
// Math utilities
// ---------------------------------------------------------------------------
static inline float deg2rad(float d) { return d * 3.14159265f / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / 3.14159265f; }

// Normalise an angle to [0, 360).
static inline float norm_deg(float d) {
  while (d <    0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

// ---------------------------------------------------------------------------
// UTF-8 decoder — BMP codepoints only (1, 2, and 3-byte sequences)
// ---------------------------------------------------------------------------
static std::vector<uint32_t> utf8_to_cps(const std::string& s) {
  std::vector<uint32_t> out;
  for (size_t i = 0; i < s.size();) {
    uint8_t c = (uint8_t)s[i];
    if (c < 0x80) {
      out.push_back(c); i += 1;
    } else if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) {
      out.push_back(((c & 0x1F) << 6) | ((uint8_t)s[i+1] & 0x3F)); i += 2;
    } else if ((c & 0xF0) == 0xE0 && i + 2 < s.size()) {
      out.push_back(((c & 0x0F) << 12) |
                    (((uint8_t)s[i+1] & 0x3F) << 6) |
                    ((uint8_t)s[i+2] & 0x3F)); i += 3;
    } else {
      i++;  // skip unrecognised or incomplete sequence
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Arc span helpers
// ---------------------------------------------------------------------------

// Advance width (pixels) for a single glyph, including any kern pair.
static inline float glyph_advance_px(const lv_font_t* f, uint32_t cp, uint32_t ncp) {
  return (float)lv_font_get_glyph_width(f, cp, ncp);
}

// Total arc angle (degrees) required to render all codepoints in cps.
static float total_span_deg(const lv_font_t* f, const std::vector<uint32_t>& cps) {
  float px = 0.0f;
  for (size_t i = 0; i < cps.size(); i++)
    px += glyph_advance_px(f, cps[i], i+1 < cps.size() ? cps[i+1] : 0) + cfg_.extra_letter_px;
  return rad2deg(px / std::max(cfg_.r_base, 1.0f));
}

// Trim cps so that its arc span fits within cfg_.max_span_deg.
// If cfg_.ellipsize is set, the last fitting glyph is replaced with U+2026.
static std::vector<uint32_t> fit_to_span(const lv_font_t* f, std::vector<uint32_t> cps) {
  if (cfg_.max_span_deg <= 0.0f || cps.empty() ||
      total_span_deg(f, cps) <= cfg_.max_span_deg) return cps;

  if (!cfg_.ellipsize) {
    while (!cps.empty() && total_span_deg(f, cps) > cfg_.max_span_deg)
      cps.pop_back();
    return cps;
  }

  cps.push_back(0x2026u);  // U+2026 HORIZONTAL ELLIPSIS
  while (cps.size() > 1 && total_span_deg(f, cps) > cfg_.max_span_deg)
    cps.erase(cps.end() - 2);  // remove the glyph just before the ellipsis
  return cps;
}

// ---------------------------------------------------------------------------
// Pixel primitives
// ---------------------------------------------------------------------------

// Extract an 8-bit alpha value from an LVGL 8 raw font bitmap.
// bi is the linear pixel index: row * box_w + col.
static inline uint8_t bitmap_alpha(const uint8_t* bmp, int bpp, int bi) {
  switch (bpp) {
    case 1: return ((bmp[bi >> 3] >> (7 - (bi & 7))) & 1) ? 255 : 0;
    case 2: { uint8_t v = (bmp[bi >> 2] >> (6 - (bi & 3) * 2)) & 3;             return (uint8_t)(v * 85); }
    case 4: { uint8_t v = (bi & 1) ? (bmp[bi>>1] & 0x0F) : (bmp[bi>>1] >> 4);   return (uint8_t)(v * 17); }
    case 8: return bmp[bi];
    default: return 0;
  }
}

/*
 * stamp_glyph_alpha — write a glyph's coverage into an 8-bit alpha map.
 *
 * abuf : destination map, GLYPH_MAX x GLYPH_MAX, 1 byte per pixel
 * ox   : x position of the glyph bitmap top-left corner within abuf
 * oy   : y position of the glyph bitmap top-left corner within abuf
 *
 * Pixels are merged using max-blend, so multiple calls with different offsets
 * accumulate as a coverage union — exactly what is needed to build the stroke
 * outline from repeated offset stamps of the same glyph.
 */
static void stamp_glyph_alpha(uint8_t* abuf,
                               const lv_font_t* font, uint32_t cp, uint32_t next_cp,
                               int ox, int oy) {
  lv_font_glyph_dsc_t gdsc;
  if (!lv_font_get_glyph_dsc(font, &gdsc, cp, next_cp)) return;
  const uint8_t* bmp = lv_font_get_glyph_bitmap(font, cp);
  if (!bmp || gdsc.bpp == 0) return;  // space or glyph with no bitmap

  const int bw = (int)gdsc.box_w;
  const int bh = (int)gdsc.box_h;

  for (int row = 0; row < bh; row++) {
    const int dy = oy + row;
    if ((unsigned)dy >= (unsigned)GLYPH_MAX) continue;
    for (int col = 0; col < bw; col++) {
      const int dx = ox + col;
      if ((unsigned)dx >= (unsigned)GLYPH_MAX) continue;
      const uint8_t a = bitmap_alpha(bmp, gdsc.bpp, row * bw + col);
      uint8_t& dst = abuf[dy * GLYPH_MAX + dx];
      if (a > dst) dst = a;
    }
  }
}

/*
 * blend_pixel — Porter-Duff "src over dst" composite for one pixel.
 *
 * p points to a LV_IMG_CF_TRUE_COLOR_ALPHA pixel:
 *   [sizeof(lv_color_t) bytes: colour value] [1 byte: alpha]
 *
 * Fast paths are taken for the fully-transparent destination and fully-opaque
 * source cases to avoid unnecessary arithmetic.
 */
static inline void blend_pixel(uint8_t* p, lv_color_t src_col, uint8_t src_a) {
  if (src_a == 0) return;

  const uint8_t dst_a = p[sizeof(lv_color_t)];

  // Destination is fully transparent: just write the source pixel.
  if (dst_a == 0) {
    memcpy(p, &src_col, sizeof(lv_color_t));
    p[sizeof(lv_color_t)] = src_a;
    return;
  }
  // Source is fully opaque: source completely replaces the destination.
  if (src_a == 255) {
    memcpy(p, &src_col, sizeof(lv_color_t));
    p[sizeof(lv_color_t)] = 255;
    return;
  }

  // General case:
  //   out_a  = src_a + dst_a * (1 - src_a/255)
  //   out_ch = (src_ch * src_a + dst_ch * dst_a * (1 - src_a/255)) / out_a
  const uint16_t ia         = 255u - src_a;
  const uint16_t da_contrib = ((uint16_t)dst_a * ia + 127u) / 255u;
  uint16_t       out_a      = (uint16_t)src_a + da_contrib;
  if (out_a > 255) out_a = 255;
  const uint16_t oa = out_a ? out_a : 1u;  // guard against division by zero

#if LV_COLOR_DEPTH == 16
  // RGB565: bits [15:11] = R (5 bits), [10:5] = G (6 bits), [4:0] = B (5 bits)
  uint16_t sc, dc;
  memcpy(&sc, &src_col, 2);
  memcpy(&dc, p,        2);
  uint8_t or_ = (uint8_t)(((uint16_t)((sc>>11)&0x1F)*src_a + (uint16_t)((dc>>11)&0x1F)*da_contrib + oa/2) / oa);
  uint8_t og  = (uint8_t)(((uint16_t)((sc>> 5)&0x3F)*src_a + (uint16_t)((dc>> 5)&0x3F)*da_contrib + oa/2) / oa);
  uint8_t ob  = (uint8_t)(((uint16_t)( sc     &0x1F)*src_a + (uint16_t)( dc     &0x1F)*da_contrib + oa/2) / oa);
  if (or_ > 31) or_ = 31;
  if (og  > 63) og  = 63;
  if (ob  > 31) ob  = 31;
  uint16_t oc = ((uint16_t)or_ << 11) | ((uint16_t)og << 5) | ob;
  memcpy(p, &oc, 2);
#else
  // 32-bit fallback via LVGL colour channel accessors
  const uint8_t sr = lv_color_red(src_col),  sg = lv_color_green(src_col), sb = lv_color_blue(src_col);
  lv_color_t dc_col; memcpy(&dc_col, p, sizeof(lv_color_t));
  const uint8_t dr = lv_color_red(dc_col),   dg = lv_color_green(dc_col),  db = lv_color_blue(dc_col);
  lv_color_t nc = lv_color_make(
    (uint8_t)(((uint16_t)sr*src_a + (uint16_t)dr*da_contrib + oa/2) / oa),
    (uint8_t)(((uint16_t)sg*src_a + (uint16_t)dg*da_contrib + oa/2) / oa),
    (uint8_t)(((uint16_t)sb*src_a + (uint16_t)db*da_contrib + oa/2) / oa)
  );
  memcpy(p, &nc, sizeof(lv_color_t));
#endif

  p[sizeof(lv_color_t)] = (uint8_t)out_a;
}

/*
 * blit_alpha_rotated — rotate and composite a glyph alpha map into the canvas.
 *
 * Uses backward-mapped nearest-neighbour sampling: for each output pixel in the
 * bounding box, the inverse rotation R(-angle) is applied to determine which
 * source pixel to sample from.  This avoids holes that forward-mapping would
 * create at non-axis-aligned angles.
 *
 * The source pivot (GLYPH_MAX/2, GLYPH_MAX/2) maps to (dest_cx, dest_cy) in
 * the canvas after rotation.  Because glyphs are stamped with their baseline
 * at row GLYPH_MAX/2, dest_cx/dest_cy is the baseline midpoint on the arc.
 */
static void blit_alpha_rotated(const uint8_t* abuf,
                                int dest_cx, int dest_cy,
                                float angle_deg,
                                lv_color_t color, uint8_t opa) {
  if (opa == 0) return;

  const float ar    = deg2rad(angle_deg);
  const float cos_a = cosf(ar);
  const float sin_a = sinf(ar);
  const float src_cx = (float)(GLYPH_MAX / 2);
  const float src_cy = (float)(GLYPH_MAX / 2);

  // The rotated glyph fits within a square whose side equals the buffer diagonal.
  // sqrt(2)/2 ~= 0.7072 gives a conservative half-extent for any rotation angle.
  const int half = (int)ceilf((float)GLYPH_MAX * 0.7072f);
  const int x0 = std::max(0,          dest_cx - half);
  const int x1 = std::min(CANVAS_W-1, dest_cx + half);
  const int y0 = std::max(0,          dest_cy - half);
  const int y1 = std::min(CANVAS_H-1, dest_cy + half);

  for (int py = y0; py <= y1; py++) {
    const float dy = (float)(py - dest_cy);
    for (int px = x0; px <= x1; px++) {
      const float dx = (float)(px - dest_cx);

      // Inverse rotation R(-a):
      //   forward:  dst_offset = R(a) * src_offset
      //   inverse:  src_offset = R(-a) * dst_offset
      //   R(-a) = [[ cos_a,  sin_a ],
      //             [-sin_a,  cos_a ]]
      const float sx =  dx * cos_a + dy * sin_a + src_cx;
      const float sy = -dx * sin_a + dy * cos_a + src_cy;

      const int isx = (int)(sx + 0.5f);
      const int isy = (int)(sy + 0.5f);
      // Unsigned comparison handles both < 0 and >= GLYPH_MAX in one branch.
      if ((unsigned)isx >= (unsigned)GLYPH_MAX ||
          (unsigned)isy >= (unsigned)GLYPH_MAX) continue;

      const uint8_t a = abuf[isy * GLYPH_MAX + isx];
      if (a == 0) continue;

      // Scale glyph alpha by the layer opacity, then composite into the canvas.
      const uint8_t fa = (opa == 255) ? a : (uint8_t)((uint16_t)a * opa / 255u);
      blend_pixel(canvas_buf_ + (py * CANVAS_W + px) * PX_BYTES, color, fa);
    }
  }
}

// ---------------------------------------------------------------------------
// Core render pass — called internally after any state change
// ---------------------------------------------------------------------------
static void layout_now() {
  if (!canvas_buf_ || !img_obj_) return;
  const uint32_t t0 = micros();

  const lv_font_t* font = lv_obj_get_style_text_font(parent_, LV_PART_MAIN);
  if (!font) font = lv_font_default();

  // Clear the framebuffer to fully transparent.
  memset(canvas_buf_, 0, (size_t)CANVAS_W * CANVAS_H * PX_BYTES);

  auto cps = utf8_to_cps(text_);
  cps = fit_to_span(font, cps);

  if (!cps.empty()) {
    const int  dir_sign      = cfg_.clockwise ? +1 : -1;
    // When outward != clockwise, the natural glyph orientation is inverted,
    // so we reverse the codepoint sequence and flip the rotation by 180 degrees
    // to keep glyphs upright and reading left-to-right.
    const bool baseline_flip = (cfg_.outward != cfg_.clockwise);
    if (baseline_flip) std::reverse(cps.begin(), cps.end());

    const float total_deg = total_span_deg(font, cps);
    float theta_deg = cfg_.mid_deg - dir_sign * (total_deg * 0.5f);

    const int  s          = std::max(cfg_.stroke_px, 0);
    const bool use_stroke = cfg_.stroke_enable && s > 0;
    const bool use_diag   = use_stroke && cfg_.stroke_diagonals;

    // Unit-direction vectors for 4-neighbour and diagonal stroke offsets.
    // Scaled by stroke_px (s) when computing the actual stamp positions.
    static const int offs4[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
    static const int offsD[4][2] = {{-1,-1},{1,-1},{-1,1},{1,1}};

    // Row in the 64x64 scratch buffer that represents the text baseline.
    // Every glyph is positioned relative to this row so that all baselines
    // are co-linear, and descenders hang uniformly below it.
    static const int PIVOT = GLYPH_MAX / 2;  // = 32

    for (size_t i = 0; i < cps.size(); i++) {
      const uint32_t cp   = cps[i];
      const uint32_t next = (i+1 < cps.size()) ? cps[i+1] : 0;

      lv_font_glyph_dsc_t gdsc;
      const bool has_glyph = lv_font_get_glyph_dsc(font, &gdsc, cp, next);
      const int  adv_px    = (int)lv_font_get_glyph_width(font, cp, next);

      // Compute baseline-relative placement within the 64x64 scratch buffer.
      // See the file-level comment for the derivation of these formulae.
      int ox = 0, oy = 0;
      if (has_glyph) {
        ox = PIVOT - adv_px / 2 + (int)gdsc.ofs_x;
        oy = PIVOT - (int)gdsc.ofs_y - (int)gdsc.box_h;
      }

      // Angular extent of this glyph on the arc (advance + letter spacing).
      const float adv_deg = rad2deg((float)adv_px / std::max(cfg_.r_base, 1.0f))
                            + rad2deg(cfg_.extra_letter_px / std::max(cfg_.r_base, 1.0f));
      // Arc angle at the horizontal centre of this glyph.
      const float theta_c = theta_deg + dir_sign * adv_deg * 0.5f;

      // Canvas coordinates of the glyph baseline midpoint.
      const float t_rad  = deg2rad(theta_c);
      const int   dest_x = (int)lroundf(cfg_.cx + cfg_.r_base * cosf(t_rad));
      const int   dest_y = (int)lroundf(cfg_.cy + cfg_.r_base * sinf(t_rad));

      // Rotation to apply: tangent direction (+90 for CW arc so glyphs stand up),
      // with an additional 180 degrees when the glyph sequence is reversed.
      const float rot = norm_deg(theta_c
                                 + (cfg_.clockwise  ? 90.0f  : -90.0f)
                                 + (baseline_flip   ? 180.0f :   0.0f));

      // Stroke pass: stamp the glyph at each neighbour offset, accumulate into
      // stroke_alpha_ via max-blend, then blit with the stroke colour.
      if (use_stroke && has_glyph) {
        memset(stroke_alpha_, 0, sizeof(stroke_alpha_));
        for (int k = 0; k < 4; k++)
          stamp_glyph_alpha(stroke_alpha_, font, cp, next,
                            ox + s * offs4[k][0], oy + s * offs4[k][1]);
        if (use_diag) {
          for (int k = 0; k < 4; k++)
            stamp_glyph_alpha(stroke_alpha_, font, cp, next,
                              ox + s * offsD[k][0], oy + s * offsD[k][1]);
        }
        blit_alpha_rotated(stroke_alpha_, dest_x, dest_y, rot,
                           cfg_.stroke_color, cfg_.stroke_opa);
      }

      // Front pass: stamp the glyph at its correct position and blit on top.
      memset(front_alpha_, 0, sizeof(front_alpha_));
      if (has_glyph)
        stamp_glyph_alpha(front_alpha_, font, cp, next, ox, oy);
      blit_alpha_rotated(front_alpha_, dest_x, dest_y, rot,
                         cfg_.front_color, cfg_.front_opa);

      theta_deg += dir_sign * adv_deg;
    }

    stats_.glyphs = (uint32_t)cps.size();
  } else {
    stats_.glyphs = 0;
  }

  // Mark the widget dirty so LVGL issues a single blit on the next flush cycle.
  // lv_img_set_src() is intentionally NOT called here — the img_dsc_ pointer is
  // set once in init() and never changes.  Calling set_src() on every update
  // forces LVGL to re-layout the image widget, which corrupts the display along
  // the horizontal seam between the two render passes produced by buffer_size: 50%.
  lv_obj_invalidate(img_obj_);

  const uint32_t dt     = micros() - t0;
  stats_.last_layout_us  = dt;
  stats_.total_layout_us += dt;
  stats_.layout_calls++;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

/** Log current stats via ESP_LOGI (tag: "arc_text"). */
static inline void log_stats() {
  ESP_LOGI("arc_text",
    "layout: calls=%u last=%uus total=%lluus | set_text: calls=%u same=%u | glyphs=%u",
    (unsigned)stats_.layout_calls,
    (unsigned)stats_.last_layout_us,
    (unsigned long long)stats_.total_layout_us,
    (unsigned)stats_.set_text_calls,
    (unsigned)stats_.set_text_same,
    (unsigned)stats_.glyphs);
}

/** Zero all counters. */
static inline void reset_stats() { stats_ = Stats{}; }

/** Log current stats then zero all counters. */
static inline void stats_log_and_reset() { log_stats(); reset_stats(); }

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * init() — initialise the renderer.
 *
 * Must be called once after the parent LVGL widget has been created and laid
 * out (e.g. in the lvgl.on_boot lambda).  Creates a single lv_img child inside
 * parent and allocates the PSRAM framebuffer.
 *
 * Requires psram: to be declared in the ESPHome YAML.
 * Buffer size: CANVAS_W x CANVAS_H x PX_BYTES (~380 KB for 360x360 @ 16bpp).
 */
static void init(lv_obj_t* parent) {
  parent_ = parent;

  const size_t buf_bytes = (size_t)CANVAS_W * CANVAS_H * PX_BYTES;
  canvas_buf_ = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!canvas_buf_) {
    ESP_LOGE("arc_text",
      "PSRAM allocation failed (%u bytes). "
      "Ensure psram: is declared in the ESPHome YAML.", (unsigned)buf_bytes);
    return;
  }
  memset(canvas_buf_, 0, buf_bytes);

  // Populate the image descriptor once; the data pointer never changes.
  img_dsc_.header.always_zero = 0;
  img_dsc_.header.w           = CANVAS_W;
  img_dsc_.header.h           = CANVAS_H;
  img_dsc_.header.cf          = LV_IMG_CF_TRUE_COLOR_ALPHA;
  img_dsc_.data_size          = (uint32_t)buf_bytes;
  img_dsc_.data               = canvas_buf_;

  // Create the single visible LVGL widget.
  img_obj_ = lv_img_create(parent_);
  lv_img_set_src(img_obj_, &img_dsc_);
  lv_obj_set_pos(img_obj_, 0, 0);
  lv_obj_set_style_bg_opa(img_obj_,       LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_pad_all(img_obj_,      0,             LV_PART_MAIN);
  lv_obj_set_style_border_width(img_obj_, 0,             LV_PART_MAIN);
  lv_obj_clear_flag(img_obj_, LV_OBJ_FLAG_CLICKABLE);

  layout_now();
}

/**
 * set_text() — update the displayed string.
 *
 * No-ops immediately if the string is unchanged, avoiding an unnecessary
 * framebuffer clear and redraw.
 */
static void set_text(const char* s) {
  stats_.set_text_calls++;
  const std::string incoming = s ? std::string(s) : std::string();
  if (incoming == text_) { stats_.set_text_same++; return; }
  text_ = incoming;
  layout_now();
}

/**
 * set_config() — replace the entire configuration and re-render.
 *
 * To change individual fields, read cfg_, modify the desired members, then
 * pass the modified struct back:
 *   arc_text::cfg_.stroke_px = 3;
 *   arc_text::set_config(arc_text::cfg_);
 */
static void set_config(const Config& c) {
  cfg_ = c;
  layout_now();
}

/**
 * set_stroke_visible() — show or hide the text outline without replacing the
 * full config.  Re-renders only if the enabled state actually changes.
 */
static void set_stroke_visible(bool visible) {
  if (cfg_.stroke_enable == visible) return;
  cfg_.stroke_enable = visible;
  layout_now();
}

} // namespace arc_text