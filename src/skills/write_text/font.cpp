#include "font.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>

namespace wt {
namespace {

constexpr double kPiF = 3.14159265358979323846;

// ---- glyph DSL -------------------------------------------------------------------------------------------------
// strokes separated by '|'; inside a stroke, tokens separated by spaces:
//   x,y                       a point
//   A(cx,cy,rx,ry,a0,a1)      elliptic arc from angle a0 to a1 (degrees, ccw positive), appended to the stroke
std::vector<Stroke> parse_glyph(const std::string& src, double arc_step_deg) {
  std::vector<Stroke> out;
  std::stringstream ss(src);
  std::string part;
  while (std::getline(ss, part, '|')) {
    Stroke s;
    std::stringstream ts(part);
    std::string tok;
    while (ts >> tok) {
      if (tok[0] == 'A') {
        double c[6];
        if (std::sscanf(tok.c_str(), "A(%lf,%lf,%lf,%lf,%lf,%lf)", &c[0], &c[1], &c[2], &c[3], &c[4], &c[5]) != 6) continue;
        const double span = c[5] - c[4];
        const int n = std::max(2, int(std::ceil(std::fabs(span) / arc_step_deg)));
        for (int i = 0; i <= n; ++i) {
          const double a = (c[4] + span * i / n) * kPiF / 180.0;
          P2 p{c[0] + c[2] * std::cos(a), c[1] + c[3] * std::sin(a)};
          if (!s.empty() && std::hypot(p.x - s.back().x, p.y - s.back().y) < 1e-6) continue;
          s.push_back(p);
        }
      } else {
        P2 p;
        if (std::sscanf(tok.c_str(), "%lf,%lf", &p.x, &p.y) != 2) continue;
        if (!s.empty() && std::hypot(p.x - s.back().x, p.y - s.back().y) < 1e-6) continue;
        s.push_back(p);
      }
    }
    if (!s.empty()) out.push_back(std::move(s));
  }
  return out;
}

// Latin capitals, digits, lowercase, punctuation (cap height 1, x-height 0.6, descender -0.35)
const std::map<char32_t, const char*>& latin() {
  static const std::map<char32_t, const char*> g = {
      {U'A', "0,0 0.35,1 0.7,0 | 0.13,0.38 0.57,0.38"},
      {U'B', "0,0 0,1 0.38,1 A(0.38,0.75,0.24,0.25,90,-90) 0,0.5 | 0.38,0.5 A(0.4,0.25,0.26,0.25,90,-90) 0,0"},
      {U'C', "A(0.4,0.5,0.4,0.5,45,315)"},
      {U'D', "0,0 0,1 0.28,1 A(0.28,0.5,0.4,0.5,90,-90) 0,0"},
      {U'E', "0.6,1 0,1 0,0 0.6,0 | 0,0.5 0.45,0.5"},
      {U'F', "0.6,1 0,1 0,0 | 0,0.5 0.45,0.5"},
      {U'G', "A(0.4,0.5,0.4,0.5,50,355) 0.79,0.45 0.45,0.45"},
      {U'H', "0,0 0,1 | 0.65,0 0.65,1 | 0,0.5 0.65,0.5"},
      {U'I', "0.15,0 0.15,1 | 0,1 0.3,1 | 0,0 0.3,0"},
      {U'J', "0.5,1 0.5,0.25 A(0.25,0.25,0.25,0.25,0,-180)"},
      {U'K', "0,0 0,1 | 0.6,1 0,0.4 | 0.2,0.58 0.6,0"},
      {U'L', "0,1 0,0 0.55,0"},
      {U'M', "0,0 0,1 0.4,0.3 0.8,1 0.8,0"},
      {U'N', "0,0 0,1 0.65,0 0.65,1"},
      {U'O', "A(0.4,0.5,0.4,0.5,90,450)"},
      {U'P', "0,0 0,1 0.38,1 A(0.38,0.75,0.25,0.25,90,-90) 0,0.5"},
      {U'Q', "A(0.4,0.5,0.4,0.5,90,450) | 0.45,0.25 0.8,-0.05"},
      {U'R', "0,0 0,1 0.38,1 A(0.38,0.75,0.25,0.25,90,-90) 0,0.5 | 0.33,0.5 0.65,0"},
      {U'S', "A(0.33,0.75,0.3,0.25,25,270) A(0.33,0.25,0.31,0.25,90,-150)"},
      {U'T', "0,1 0.7,1 | 0.35,1 0.35,0"},
      {U'U', "0,1 0,0.3 A(0.32,0.3,0.32,0.3,180,360) 0.64,1"},
      {U'V', "0,1 0.35,0 0.7,1"},
      {U'W', "0,1 0.2,0 0.45,0.7 0.7,0 0.9,1"},
      {U'X', "0,1 0.65,0 | 0,0 0.65,1"},
      {U'Y', "0,1 0.33,0.5 0.66,1 | 0.33,0.5 0.33,0"},
      {U'Z', "0,1 0.65,1 0,0 0.65,0"},
      {U'0', "A(0.3,0.5,0.3,0.5,90,450)"},
      {U'1', "0.1,0.8 0.3,1 0.3,0 | 0.1,0 0.5,0"},
      {U'2', "A(0.3,0.7,0.3,0.3,155,-35) 0,0 0.6,0"},
      {U'3', "A(0.3,0.76,0.27,0.24,150,-90) A(0.3,0.27,0.3,0.27,90,-150)"},
      {U'4', "0.45,0 0.45,1 0,0.3 0.62,0.3"},
      {U'5', "0.55,1 0.08,1 0.04,0.55 A(0.3,0.32,0.3,0.32,140,-150)"},
      {U'6', "0.5,1 A(0.3,0.3,0.3,0.3,150,510)"},
      {U'7', "0,1 0.6,1 0.2,0"},
      {U'8', "A(0.3,0.75,0.25,0.25,-90,270) A(0.3,0.27,0.3,0.27,90,450)"},
      {U'9', "A(0.3,0.7,0.3,0.3,-20,340) 0.1,0"},
      {U'a', "A(0.25,0.3,0.25,0.3,0,360) | 0.5,0.6 0.5,0"},
      {U'b', "0,1 0,0 | A(0.25,0.3,0.25,0.3,180,540)"},
      {U'c', "A(0.27,0.3,0.27,0.3,40,320)"},
      {U'd', "0.5,1 0.5,0 | A(0.25,0.3,0.25,0.3,0,360)"},
      {U'e', "0,0.3 0.5,0.3 A(0.25,0.3,0.25,0.3,0,320)"},
      {U'f', "0.42,0.92 A(0.3,0.85,0.12,0.15,30,180) 0.18,0 | 0,0.6 0.38,0.6"},
      {U'g', "A(0.25,0.3,0.25,0.3,0,360) | 0.5,0.6 0.5,-0.15 A(0.25,-0.15,0.25,0.2,0,-180)"},
      {U'h', "0,1 0,0 | 0,0.35 A(0.25,0.35,0.25,0.25,180,0) 0.5,0"},
      {U'i', "0.05,0.6 0.05,0 | 0.05,0.86 0.05,0.8"},
      {U'j', "0.2,0.6 0.2,-0.15 A(0.05,-0.15,0.15,0.15,0,-150) | 0.2,0.86 0.2,0.8"},
      {U'k', "0,1 0,0 | 0.45,0.6 0,0.25 | 0.15,0.35 0.45,0"},
      {U'l', "0.05,1 0.05,0"},
      {U'm', "0,0.6 0,0 | 0,0.4 A(0.2,0.4,0.2,0.2,180,0) 0.4,0 | 0.4,0.4 A(0.6,0.4,0.2,0.2,180,0) 0.8,0"},
      {U'n', "0,0.6 0,0 | 0,0.35 A(0.25,0.35,0.25,0.25,180,0) 0.5,0"},
      {U'o', "A(0.27,0.3,0.27,0.3,90,450)"},
      {U'p', "0,0.6 0,-0.35 | A(0.25,0.3,0.25,0.3,180,540)"},
      {U'q', "0.5,0.6 0.5,-0.35 | A(0.25,0.3,0.25,0.3,0,360)"},
      {U'r', "0,0.6 0,0 | 0,0.35 A(0.25,0.35,0.25,0.25,180,60)"},
      {U's', "A(0.22,0.45,0.2,0.15,20,270) A(0.22,0.15,0.22,0.15,90,-160)"},
      {U't', "0.15,0.9 0.15,0.1 A(0.3,0.1,0.15,0.1,180,300) | 0,0.6 0.35,0.6"},
      {U'u', "0,0.6 0,0.25 A(0.25,0.25,0.25,0.25,180,360) | 0.5,0.6 0.5,0"},
      {U'v', "0,0.6 0.25,0 0.5,0.6"},
      {U'w', "0,0.6 0.18,0 0.35,0.45 0.52,0 0.7,0.6"},
      {U'x', "0,0.6 0.5,0 | 0,0 0.5,0.6"},
      {U'y', "0,0.6 0.25,0 | 0.5,0.6 0.15,-0.35"},
      {U'z', "0,0.6 0.5,0.6 0,0 0.5,0"},
      {U'.', "0.05,0.06 0.05,0"},
      {U',', "0.1,0.08 0,-0.14"},
      {U'!', "0.05,1 0.05,0.3 | 0.05,0.06 0.05,0"},
      {U'?', "A(0.3,0.72,0.28,0.28,160,-60) 0.3,0.3 | 0.3,0.06 0.3,0"},
      {U'-', "0,0.45 0.4,0.45"},
      {U'+', "0,0.45 0.5,0.45 | 0.25,0.2 0.25,0.7"},
      {U'=', "0,0.6 0.5,0.6 | 0,0.3 0.5,0.3"},
      {U':', "0.05,0.66 0.05,0.6 | 0.05,0.06 0.05,0"},
      {U'\'', "0.05,1 0.05,0.75"},
      {U'/', "0,0 0.5,1"},
      {U'(', "A(0.45,0.45,0.4,0.62,120,240)"},
      {U')', "A(-0.15,0.45,0.4,0.62,60,-60)"},
      {U'#', "0.2,0 0.3,1 | 0.45,0 0.55,1 | 0,0.65 0.7,0.65 | 0,0.35 0.7,0.35"},
      {U'&', "0.7,0 A(0.3,0.75,0.17,0.22,-40,250) 0.65,0.35 | 0.2,0.55 A(0.3,0.25,0.28,0.25,120,330)"},
  };
  return g;
}

// ---- Hangul jamo templates in a unit box ------------------------------------------------------------------------
const char* kCons[] = {
    /*ㄱ*/ "0,1 1,1 1,0", /*ㄴ*/ "0,1 0,0 1,0", /*ㄷ*/ "0,1 1,1 | 0,1 0,0 1,0", /*ㄹ*/ "0,1 1,1 1,0.5 0,0.5 0,0 1,0",
    /*ㅁ*/ "0,1 0,0 | 0,1 1,1 1,0 | 0,0 1,0", /*ㅂ*/ "0,1 0,0 | 1,1 1,0 | 0,0.5 1,0.5 | 0,0 1,0",
    /*ㅅ*/ "0.5,1 0,0 | 0.5,0.55 1,0", /*ㅇ*/ "A(0.5,0.5,0.5,0.5,90,450)",
    /*ㅈ*/ "0,1 1,1 | 0.6,1 0,0 | 0.45,0.5 1,0", /*ㅊ*/ "0.5,1 0.5,0.86 | 0,0.75 1,0.75 | 0.6,0.75 0,0 | 0.45,0.38 1,0",
    /*ㅋ*/ "0,1 1,1 1,0 | 0,0.5 1,0.5", /*ㅌ*/ "0,1 1,1 | 0,0.5 1,0.5 | 0,1 0,0 1,0",
    /*ㅍ*/ "0,1 1,1 | 0.3,1 0.3,0 | 0.7,1 0.7,0 | 0,0 1,0",
    /*ㅎ*/ "0.5,1 0.5,0.86 | 0,0.75 1,0.75 | A(0.5,0.3,0.34,0.3,90,450)"};
enum Cons { kG, kN, kD, kR, kM, kB, kS, kNg, kJ, kCh, kK, kT, kP, kH, kNone = -1 };
struct Comp { int a, b; };  // single (b = -1) or a pair drawn side by side
// initial consonants (19)
const Comp kInit[19] = {{kG, -1}, {kG, kG}, {kN, -1}, {kD, -1}, {kD, kD}, {kR, -1}, {kM, -1}, {kB, -1}, {kB, kB}, {kS, -1},
                        {kS, kS}, {kNg, -1}, {kJ, -1}, {kJ, kJ}, {kCh, -1}, {kK, -1}, {kT, -1}, {kP, -1}, {kH, -1}};
// finals (28, index 0 = none)
const Comp kFinal[28] = {{kNone, -1}, {kG, -1}, {kG, kG}, {kG, kS}, {kN, -1}, {kN, kJ}, {kN, kH}, {kD, -1}, {kR, -1}, {kR, kG},
                         {kR, kM}, {kR, kB}, {kR, kS}, {kR, kT}, {kR, kP}, {kR, kH}, {kM, -1}, {kB, -1}, {kB, kS}, {kS, -1},
                         {kS, kS}, {kNg, -1}, {kJ, -1}, {kCh, -1}, {kK, -1}, {kT, -1}, {kP, -1}, {kH, -1}};
// medial vowels (21): horizontal part (ㅗㅛㅜㅠㅡ or none) + vertical part (ㅏㅐㅑㅒㅓㅔㅕㅖㅣ or none)
enum VV { vA, vAe, vYa, vYae, vEo, vE, vYeo, vYe, vI, vNone = -1 };
enum HV { hO, hYo, hU, hYu, hEu, hNone = -1 };
const char* kVert[] = {
    /*ㅏ*/ "0.35,1 0.35,0 | 0.35,0.5 0.85,0.5", /*ㅐ*/ "0.25,1 0.25,0 | 0.25,0.5 0.55,0.5 | 0.75,1 0.75,0",
    /*ㅑ*/ "0.35,1 0.35,0 | 0.35,0.62 0.85,0.62 | 0.35,0.38 0.85,0.38",
    /*ㅒ*/ "0.25,1 0.25,0 | 0.25,0.62 0.55,0.62 | 0.25,0.38 0.55,0.38 | 0.75,1 0.75,0",
    /*ㅓ*/ "0.15,0.5 0.65,0.5 | 0.65,1 0.65,0", /*ㅔ*/ "0.05,0.5 0.4,0.5 | 0.4,1 0.4,0 | 0.8,1 0.8,0",
    /*ㅕ*/ "0.15,0.62 0.65,0.62 | 0.15,0.38 0.65,0.38 | 0.65,1 0.65,0",
    /*ㅖ*/ "0.05,0.62 0.4,0.62 | 0.05,0.38 0.4,0.38 | 0.4,1 0.4,0 | 0.8,1 0.8,0", /*ㅣ*/ "0.5,1 0.5,0"};
const char* kHorz[] = {/*ㅗ*/ "0.5,0.9 0.5,0.4 | 0,0.4 1,0.4", /*ㅛ*/ "0.33,0.9 0.33,0.4 | 0.67,0.9 0.67,0.4 | 0,0.4 1,0.4",
                       /*ㅜ*/ "0,0.6 1,0.6 | 0.5,0.6 0.5,0.05", /*ㅠ*/ "0,0.6 1,0.6 | 0.33,0.6 0.33,0.05 | 0.67,0.6 0.67,0.05",
                       /*ㅡ*/ "0,0.5 1,0.5"};
struct Med { int h, v; };
const Med kMed[21] = {{hNone, vA},  {hNone, vAe}, {hNone, vYa}, {hNone, vYae}, {hNone, vEo}, {hNone, vE}, {hNone, vYeo},
                      {hNone, vYe}, {hO, hNone},  {hO, vA},     {hO, vAe},     {hO, vI},      {hYo, hNone}, {hU, hNone},
                      {hU, vEo},    {hU, vE},     {hU, vI},     {hYu, hNone},  {hEu, hNone},  {hEu, vI},    {hNone, vI}};

struct Box { double x0, y0, x1, y1; };

void place(const std::vector<Stroke>& unit, const Box& b, double margin, std::vector<Stroke>& out) {
  const double mx = (b.x1 - b.x0) * margin, my = (b.y1 - b.y0) * margin;
  const double x0 = b.x0 + mx, y0 = b.y0 + my, w = (b.x1 - b.x0) - 2 * mx, h = (b.y1 - b.y0) - 2 * my;
  for (const auto& s : unit) {
    Stroke t;
    for (const auto& p : s) t.push_back({x0 + p.x * w, y0 + p.y * h});
    out.push_back(std::move(t));
  }
}

void place_one(int id, Box b, double margin, double step, std::vector<Stroke>& out) {
  if (id == kNg || id == kH) {  // round jamo: keep the circle from turning into a flat ellipse
    const double w = b.x1 - b.x0, h = b.y1 - b.y0;
    if (w > 1.3 * h) { const double c = 0.5 * (b.x0 + b.x1); b.x0 = c - 0.65 * h; b.x1 = c + 0.65 * h; }
  }
  place(parse_glyph(kCons[id], step), b, margin, out);
}

void place_cons(const Comp& c, const Box& b, double step, std::vector<Stroke>& out) {
  if (c.a < 0) return;
  if (c.b < 0) {
    place_one(c.a, b, 0.1, step, out);
    return;
  }
  const double xm = 0.5 * (b.x0 + b.x1);
  place_one(c.a, {b.x0, b.y0, xm, b.y1}, 0.12, step, out);
  place_one(c.b, {xm, b.y0, b.x1, b.y1}, 0.12, step, out);
}

std::vector<Stroke> hangul_syllable(char32_t c, double step) {
  const int code = int(c - 0xAC00);
  const int L = code / 588, V = (code % 588) / 28, T = code % 28;
  const Med med = kMed[V];
  const bool fin = T != 0;
  std::vector<Stroke> out;
  if (med.h < 0) {  // vertical vowel: initial left, vowel right
    const Box bi = fin ? Box{0.0, 0.42, 0.56, 1.0} : Box{0.0, 0.08, 0.56, 0.95};
    const Box bv = fin ? Box{0.5, 0.36, 1.0, 1.0} : Box{0.5, 0.0, 1.0, 1.0};
    place_cons(kInit[L], bi, step, out);
    place(parse_glyph(kVert[med.v], step), bv, 0.05, out);
  } else if (med.v < 0) {  // horizontal vowel: initial top, vowel below
    const Box bi = fin ? Box{0.2, 0.62, 0.8, 1.0} : Box{0.15, 0.45, 0.85, 1.0};
    const Box bh = fin ? Box{0.0, 0.32, 1.0, 0.64} : Box{0.0, 0.0, 1.0, 0.48};
    place_cons(kInit[L], bi, step, out);
    place(parse_glyph(kHorz[med.h], step), bh, 0.06, out);
  } else {  // compound: initial top-left, horizontal part below it, vertical part on the right
    const Box bi = fin ? Box{0.0, 0.62, 0.6, 1.0} : Box{0.0, 0.5, 0.6, 0.98};
    const Box bh = fin ? Box{0.0, 0.34, 0.72, 0.64} : Box{0.0, 0.04, 0.72, 0.52};
    const Box bv = fin ? Box{0.56, 0.3, 1.0, 1.0} : Box{0.56, 0.0, 1.0, 1.0};
    place_cons(kInit[L], bi, step, out);
    place(parse_glyph(kHorz[med.h], step), bh, 0.06, out);
    place(parse_glyph(kVert[med.v], step), bv, 0.05, out);
  }
  if (fin) place_cons(kFinal[T], Box{0.15, 0.0, 0.85, 0.3}, step, out);
  return out;
}

// compatibility jamo (U+3131..U+3163) drawn alone in the block
std::vector<Stroke> hangul_jamo(char32_t c, double step) {
  static const char32_t cons[] = {U'ㄱ', U'ㄴ', U'ㄷ', U'ㄹ', U'ㅁ', U'ㅂ', U'ㅅ', U'ㅇ', U'ㅈ', U'ㅊ', U'ㅋ', U'ㅌ', U'ㅍ', U'ㅎ'};
  static const char32_t dbl[] = {U'ㄲ', U'ㄸ', U'ㅃ', U'ㅆ', U'ㅉ'};
  static const int dbl_of[] = {kG, kD, kB, kS, kJ};
  static const char32_t vert[] = {U'ㅏ', U'ㅐ', U'ㅑ', U'ㅒ', U'ㅓ', U'ㅔ', U'ㅕ', U'ㅖ', U'ㅣ'};
  static const char32_t horz[] = {U'ㅗ', U'ㅛ', U'ㅜ', U'ㅠ', U'ㅡ'};
  std::vector<Stroke> out;
  for (int i = 0; i < 14; ++i)
    if (c == cons[i]) { place(parse_glyph(kCons[i], step), {0.1, 0.1, 0.9, 0.9}, 0.05, out); return out; }
  for (int i = 0; i < 5; ++i)
    if (c == dbl[i]) { place_cons({dbl_of[i], dbl_of[i]}, {0.0, 0.1, 1.0, 0.9}, step, out); return out; }
  for (int i = 0; i < 9; ++i)
    if (c == vert[i]) { place(parse_glyph(kVert[i], step), {0.15, 0.0, 0.85, 1.0}, 0.05, out); return out; }
  for (int i = 0; i < 5; ++i)
    if (c == horz[i]) { place(parse_glyph(kHorz[i], step), {0.0, 0.15, 1.0, 0.85}, 0.05, out); return out; }
  return out;
}

bool is_syllable(char32_t c) { return c >= 0xAC00 && c <= 0xD7A3; }
bool is_jamo(char32_t c) { return c >= 0x3131 && c <= 0x3163; }

}  // namespace

std::u32string utf8_decode(const std::string& s) {
  std::u32string out;
  for (size_t i = 0; i < s.size();) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    char32_t cp;
    int n;
    if (c < 0x80) { cp = c; n = 1; }
    else if ((c >> 5) == 6) { cp = c & 0x1F; n = 2; }
    else if ((c >> 4) == 14) { cp = c & 0x0F; n = 3; }
    else if ((c >> 3) == 30) { cp = c & 0x07; n = 4; }
    else { ++i; continue; }
    if (i + size_t(n) > s.size()) break;
    for (int k = 1; k < n; ++k) cp = (cp << 6) | (static_cast<unsigned char>(s[i + size_t(k)]) & 0x3F);
    out.push_back(cp);
    i += size_t(n);
  }
  return out;
}

std::string utf8_encode(char32_t c) {
  std::string o;
  if (c < 0x80) o += char(c);
  else if (c < 0x800) { o += char(0xC0 | (c >> 6)); o += char(0x80 | (c & 0x3F)); }
  else if (c < 0x10000) { o += char(0xE0 | (c >> 12)); o += char(0x80 | ((c >> 6) & 0x3F)); o += char(0x80 | (c & 0x3F)); }
  else { o += char(0xF0 | (c >> 18)); o += char(0x80 | ((c >> 12) & 0x3F)); o += char(0x80 | ((c >> 6) & 0x3F)); o += char(0x80 | (c & 0x3F)); }
  return o;
}

bool has_glyph(char32_t c) { return c == U' ' || c == U'\n' || is_syllable(c) || is_jamo(c) || latin().count(c); }

TextLayout layout_text(const std::string& utf8, const LayoutOptions& o) {
  TextLayout L;
  const std::u32string text = utf8_decode(utf8);
  const double s = o.size_m;
  // pass 1: glyphs in text units (x advancing, lines stacked downwards)
  struct G { char32_t cp; std::vector<Stroke> st; double x, line; double x0, x1; };
  std::vector<G> gs;
  std::vector<double> line_w(1, 0.0);
  double x = 0;
  int line = 0;
  for (char32_t c : text) {
    if (c == U'\n') { line_w[size_t(line)] = x; ++line; line_w.push_back(0); x = 0; continue; }
    if (c == U' ') { x += o.space; continue; }
    std::vector<Stroke> st;
    double adv;
    if (is_syllable(c) || is_jamo(c)) {
      st = is_syllable(c) ? hangul_syllable(c, o.arc_step_deg) : hangul_jamo(c, o.arc_step_deg);
      adv = o.hangul_advance;
    } else if (latin().count(c)) {
      st = parse_glyph(latin().at(c), o.arc_step_deg);
      double mx = 0;
      for (auto& k : st) for (auto& p : k) mx = std::max(mx, p.x);
      adv = mx + o.letter_gap;
    } else {
      L.unsupported.push_back(utf8_encode(c));
      continue;
    }
    gs.push_back({c, std::move(st), x, double(line), x, x + adv});
    x += adv;
  }
  line_w[size_t(line)] = x;
  const int nlines = line + 1;
  double maxw = 0;
  for (double w : line_w) maxw = std::max(maxw, w);
  // strip the trailing gap of each line from its width
  L.w = maxw * s;
  L.h = (1.0 + (nlines - 1) * o.line_spacing) * s;
  // pass 2: place in the paper frame, block centred at (cx, cy); each line left-aligned in the block
  const double left = o.cx - 0.5 * L.w;
  const double top = o.cy + 0.5 * L.h;  // top of the first line's cap height
  L.minx = L.miny = 1e9;
  L.maxx = L.maxy = -1e9;
  for (auto& g : gs) {
    const double base_y = top - s - g.line * o.line_spacing * s;
    GlyphInfo gi;
    gi.cp = g.cp;
    gi.first_stroke = int(L.strokes.size());
    gi.x0 = left + g.x0 * s;
    gi.x1 = left + g.x1 * s;
    gi.y0 = base_y;
    gi.y1 = base_y + s;
    for (auto& st : g.st) {
      Stroke t;
      for (auto& p : st) {
        P2 q{left + (g.x + p.x) * s, base_y + p.y * s};
        L.minx = std::min(L.minx, q.x); L.maxx = std::max(L.maxx, q.x);
        L.miny = std::min(L.miny, q.y); L.maxy = std::max(L.maxy, q.y);
        t.push_back(q);
      }
      // a dot: give it a minimum length so the pen actually marks
      if (t.size() == 2 && std::hypot(t[1].x - t[0].x, t[1].y - t[0].y) < 0.0008) t[1].y = t[0].y - 0.0008;
      L.strokes.push_back(std::move(t));
    }
    gi.n_strokes = int(L.strokes.size()) - gi.first_stroke;
    L.glyphs.push_back(gi);
  }
  if (!L.strokes.empty()) {  // centre the ink (not the advance boxes) on (cx, cy)
    const double dx = o.cx - 0.5 * (L.minx + L.maxx), dy = o.cy - 0.5 * (L.miny + L.maxy);
    for (auto& st : L.strokes) for (auto& p : st) { p.x += dx; p.y += dy; }
    for (auto& g : L.glyphs) { g.x0 += dx; g.x1 += dx; g.y0 += dy; g.y1 += dy; }
    L.minx += dx; L.maxx += dx; L.miny += dy; L.maxy += dy;
    L.w = L.maxx - L.minx;
    L.h = L.maxy - L.miny;
  }
  for (size_t i = 0; i < L.strokes.size(); ++i) {
    const auto& st = L.strokes[i];
    for (size_t k = 1; k < st.size(); ++k) L.ink_length += std::hypot(st[k].x - st[k - 1].x, st[k].y - st[k - 1].y);
    if (i > 0) {
      const auto& a = L.strokes[i - 1].back();
      L.travel_length += std::hypot(st[0].x - a.x, st[0].y - a.y);
    }
  }
  if (L.strokes.empty()) L.minx = L.miny = L.maxx = L.maxy = 0;
  return L;
}

}  // namespace wt
