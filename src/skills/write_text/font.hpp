// Single-line stroke font for pencil writing: Latin (A-Z, a-z), digits, basic punctuation, and Hangul syllables
// composed from jamo stroke templates (initial / medial / final placed in a block by vowel shape).
//
// Glyph units: Latin cap height = 1, baseline y = 0, y up. A Hangul block spans [0,1] x [0,1].
// layout_text() scales by size_m (cap height / block height in metres) and returns strokes in the PAPER frame:
// origin at the paper centre, +x = writing direction, +y = text up, metres. Each stroke is one pen-down polyline.
#pragma once
#include <string>
#include <vector>

namespace wt {

struct P2 {
  double x = 0, y = 0;
};
using Stroke = std::vector<P2>;

struct GlyphInfo {
  char32_t cp = 0;
  int first_stroke = 0, n_strokes = 0;  // into TextLayout::strokes
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // box in the paper frame (m)
};

struct TextLayout {
  std::vector<Stroke> strokes;   // paper frame, metres, in writing order
  std::vector<GlyphInfo> glyphs;
  double w = 0, h = 0;           // block size (m)
  double minx = 0, miny = 0, maxx = 0, maxy = 0;  // ink bbox in the paper frame
  std::vector<std::string> unsupported;  // characters with no glyph (skipped)
  double ink_length = 0;         // total pen-down length (m)
  double travel_length = 0;      // pen-up length between strokes (m)
};

struct LayoutOptions {
  double size_m = 0.025;        // cap height / Hangul block height
  double cx = 0, cy = 0;        // block centre in the paper frame
  double line_spacing = 1.6;    // x size
  double letter_gap = 0.22;     // x size, Latin
  double hangul_advance = 1.12; // x size
  double space = 0.5;           // x size
  double arc_step_deg = 15;     // arc sampling
};

std::u32string utf8_decode(const std::string& s);
std::string utf8_encode(char32_t c);
bool has_glyph(char32_t c);
TextLayout layout_text(const std::string& utf8, const LayoutOptions& o);

}  // namespace wt
