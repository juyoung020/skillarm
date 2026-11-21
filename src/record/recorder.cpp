#include "record/recorder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <random>
#include <sstream>

#pragma warning(push, 0)
#include "stb/stb_image_write.h"
#pragma warning(pop)

namespace skillarm {
namespace fs = std::filesystem;
using nlohmann::json;

namespace {
std::string utc_stamp() {
  std::time_t t = std::time(nullptr);
  std::tm g{};
  gmtime_s(&g, &t);
  char b[32];
  std::strftime(b, sizeof b, "%Y%m%dT%H%M%SZ", &g);
  return b;
}
// ASPIRE artifacts.py slug(): lower case, runs of other characters -> '-', trimmed, at most 96 characters
std::string slug(const std::string& s, const char* keep = "") {
  std::string o;
  bool dash = false;
  for (char c : s) {
    const char l = char(std::tolower(static_cast<unsigned char>(c)));
    const bool ok = (l >= 'a' && l <= 'z') || (l >= '0' && l <= '9') || (l && std::strchr(keep, l));
    if (ok) { o += l; dash = false; }
    else if (!dash) { o += '-'; dash = true; }
  }
  const std::string strip = std::string("-") + keep;
  const auto b = o.find_first_not_of(strip), e = o.find_last_not_of(strip);
  o = b == std::string::npos ? "unnamed" : o.substr(b, e - b + 1);
  return o.substr(0, 96);
}
// ASPIRE _component(): the same but keeps '_' and '.' (camera names in file names)
std::string component(const std::string& s) { return slug(s, "_."); }

// float32 C-order .npy (NumPy format 1.0), as np.save writes it
void write_npy(const fs::path& p, const std::vector<float>& v, int h, int w) {
  std::string hdr = "{'descr': '<f4', 'fortran_order': False, 'shape': (" + std::to_string(h) + ", " + std::to_string(w) + "), }";
  while ((10 + hdr.size() + 1) % 64) hdr += ' ';
  hdr += '\n';
  std::ofstream f(p, std::ios::binary);
  f.write("\x93NUMPY\x01\x00", 8);
  const uint16_t n = uint16_t(hdr.size());
  f.write(reinterpret_cast<const char*>(&n), 2);
  f.write(hdr.data(), std::streamsize(hdr.size()));
  f.write(reinterpret_cast<const char*>(v.data()), std::streamsize(v.size() * sizeof(float)));
}
std::string read_file(const fs::path& p) {
  std::ifstream f(p, std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// --- SHA-256 (FIPS 180-4) ---
struct Sha256 {
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  static constexpr uint32_t k[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98,
      0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
      0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8,
      0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
      0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
      0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
      0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
      0xc67178f2};
  static uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
  void block(const uint8_t* p) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) w[i] = uint32_t(p[4 * i]) << 24 | uint32_t(p[4 * i + 1]) << 16 | uint32_t(p[4 * i + 2]) << 8 | p[4 * i + 3];
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t t1 = hh + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
      const uint32_t t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }
};
}  // namespace

std::string sha256_hex(const std::string& data) {
  Sha256 s;
  const uint64_t bits = uint64_t(data.size()) * 8;
  size_t i = 0;
  for (; i + 64 <= data.size(); i += 64) s.block(reinterpret_cast<const uint8_t*>(data.data()) + i);
  uint8_t tail[128] = {};
  const size_t rem = data.size() - i;
  std::memcpy(tail, data.data() + i, rem);
  tail[rem] = 0x80;
  const size_t len = rem + 1 + 8 <= 64 ? 64 : 128;
  for (int b = 0; b < 8; ++b) tail[len - 1 - b] = uint8_t(bits >> (8 * b));
  s.block(tail);
  if (len == 128) s.block(tail + 64);
  char out[65];
  for (int b = 0; b < 8; ++b) std::snprintf(out + 8 * b, 9, "%08x", s.h[b]);
  return out;
}

Recorder::Recorder(const fs::path& out_root, const std::string& benchmark, const std::string& task_id, int seed,
                   const fs::path& program_source)
    : t0_(std::chrono::steady_clock::now()) {
  stbi_write_png_compression_level = 1;  // ASPIRE PNG_COMPRESS_LEVEL: ~4x faster than the default for ~10 % more bytes
  std::random_device rd;
  char hex[9];
  std::snprintf(hex, sizeof hex, "%08x", rd());
  char sd[8];
  std::snprintf(sd, sizeof sd, "%04d", seed);
  const std::string run_id = "run-" + utc_stamp() + "-" + hex;
  dir_ = out_root / sd / run_id;
  for (const char* d : {"source", "episode", "trace/calls", "media/keyframes", "logs"}) fs::create_directories(dir_ / d);
  if (!program_source.empty() && fs::exists(program_source)) fs::copy_file(program_source, dir_ / "source/program.cpp");
  events_.open(dir_ / "trace/events.jsonl");
  steps_.open(dir_ / "episode/steps.jsonl");
  out_.open(dir_ / "logs/program.stdout");
  err_.open(dir_ / "logs/program.stderr");
  run_ = {{"run_id", run_id}, {"benchmark", benchmark}, {"task_id", task_id}, {"seed", seed},
          {"started_utc", utc_stamp()}, {"status", "in_progress"}, {"harness", "skillarm"}};
  write_json("run.json", run_);
}

void Recorder::write_json(const fs::path& rel, const json& j) {
  const auto p = dir_ / rel;
  fs::create_directories(p.parent_path());
  const auto tmp = p.string() + ".tmp";
  { std::ofstream f(tmp); f << j.dump(2) << "\n"; }
  fs::rename(tmp, p);  // atomic replace
}

int Recorder::span_start(const std::string& name, const std::string& category, int parent) {
  const int id = next_span_++;
  const int call = category == "api" ? next_call_++ : 0;
  open_.push_back({name, category, parent, std::chrono::steady_clock::now(), call});
  events_ << json{{"event", "span_start"}, {"span_id", id}, {"parent_id", parent}, {"name", name}, {"category", category},
                  {"t_s", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count()}}.dump()
          << "\n";
  events_.flush();
  return id;
}

void Recorder::span_end(int span, bool ok, const json& input, const json& output, const json& error) {
  const auto& o = open_[size_t(span - 1)];
  const double dur = std::chrono::duration<double>(std::chrono::steady_clock::now() - o.t).count();
  json ev = {{"event", "span_end"}, {"span_id", span}, {"parent_id", o.parent}, {"name", o.name},
             {"category", o.category}, {"ok", ok}, {"duration_s", dur}};
  if (!error.is_null()) ev["error"] = error;
  events_ << ev.dump() << "\n";
  events_.flush();
  if (o.call > 0) {
    char nn[16];
    std::snprintf(nn, sizeof nn, "%06d", o.call);
    const fs::path base = fs::path("trace/calls") / ("call-" + std::string(nn) + "-" + slug(o.name));
    write_json(base / "input.json", input);
    write_json(base / "output.json", output);
    if (!error.is_null()) write_json(base / "error.json", error);
  }
}

void Recorder::step(const json& rec) { steps_ << rec.dump() << "\n"; steps_.flush(); }
void Recorder::reset_info(const json& j) {
  write_json("episode/reset.json", j);
  if (j.contains("episode_max_steps")) {  // run identity for validation / evo-eval (docs/03 §0.4)
    run_["capture"]["episode_max_steps"] = j["episode_max_steps"];
    write_json("run.json", run_);
  }
}
void Recorder::stdout_line(const std::string& s) { out_ << s << "\n"; out_.flush(); }
void Recorder::stderr_line(const std::string& s) { err_ << s << "\n"; err_.flush(); }
void Recorder::time_bucket(const std::string& name, double seconds) {
  buckets_[name] = buckets_.value(name, 0.0) + seconds;
}
void Recorder::result(const json& r) { write_json("source/program-result.json", r); }

void Recorder::keyframe(const std::string& label, int step_index, const std::vector<CameraObservation>& cameras) {
  if (cameras.empty()) return;
  const auto t0 = std::chrono::steady_clock::now();
  char nn[16];
  std::snprintf(nn, sizeof nn, "%06d", ++keyframe_index_);
  const fs::path rel = fs::path("media/keyframes") / (std::string(nn) + "-" + slug(label));
  fs::create_directories(dir_ / rel);
  json cams = json::object();
  for (const auto& c : cameras) {
    if (c.rgb.width <= 0) continue;
    const std::string n = component(c.name);
    stbi_write_png((dir_ / rel / (n + ".png")).string().c_str(), c.rgb.width, c.rgb.height, 3, c.rgb.rgb.data(),
                   c.rgb.width * 3);
    json rec = {{"rgb", n + ".png"}, {"frame", c.name}};
    if (!depth_saved_ && c.depth_m.size() == size_t(c.rgb.width) * c.rgb.height) {
      write_npy(dir_ / rel / (n + "-depth.npy"), c.depth_m, c.rgb.height, c.rgb.width);
      rec["depth"] = n + "-depth.npy";
      depth_saved_ = true;
    }
    cams[c.name] = rec;
  }
  write_json(rel / "metadata.json", {{"label", label}, {"step_index", step_index}, {"cameras", cams}});
  time_bucket("keyframe", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}

void Recorder::overlay(const SegmentationSet& r, const CameraObservation& cam) {
  if (!r.ok || r.items.empty()) return;
  const auto& first = r.items.front();
  const int w = cam.rgb.width, h = cam.rgb.height;
  if (first.width != w || first.height != h) return;
  const auto t0 = std::chrono::steady_clock::now();
  static const uint8_t colors[5][3] = {{255, 64, 64}, {64, 255, 64}, {64, 128, 255}, {255, 192, 64}, {224, 64, 255}};
  std::vector<uint8_t> img = cam.rgb.rgb;
  json records = json::array();
  for (size_t k = 0; k < r.items.size() && k < 5; ++k) {
    const auto& s = r.items[k];
    if (s.width != w || s.height != h || s.camera_name != first.camera_name) continue;
    const uint8_t* col = colors[k];
    int pixels = 0;
    for (size_t i = 0; i < s.mask.size(); ++i)
      if (s.mask[i]) {
        ++pixels;
        for (int c = 0; c < 3; ++c) img[3 * i + size_t(c)] = uint8_t(std::lround(0.68 * img[3 * i + size_t(c)] + 0.32 * col[c]));
      }
    const int x1 = std::clamp(s.box_xyxy[0], 0, w - 1), y1 = std::clamp(s.box_xyxy[1], 0, h - 1);
    const int x2 = std::clamp(s.box_xyxy[2], 0, w - 1), y2 = std::clamp(s.box_xyxy[3], 0, h - 1);
    auto put = [&](int x, int y) {
      if (x < 0 || y < 0 || x >= w || y >= h) return;
      for (int c = 0; c < 3; ++c) img[3 * (size_t(y) * w + x) + size_t(c)] = col[c];
    };
    for (int t = 0; t < 3; ++t) {  // 3 px outline, as ASPIRE draws it
      for (int x = x1; x <= x2; ++x) { put(x, y1 + t); put(x, y2 - t); }
      for (int y = y1; y <= y2; ++y) { put(x1 + t, y); put(x2 - t, y); }
    }
    records.push_back({{"index", k}, {"label", s.label}, {"score", s.score}, {"box_xyxy", {x1, y1, x2, y2}},
                       {"mask_pixels", pixels}});
  }
  char nn[16];
  std::snprintf(nn, sizeof nn, "%06d", ++overlay_index_);
  const std::string stem = "sam3-" + std::string(nn) + "-" + component(first.camera_name);
  fs::create_directories(dir_ / "media/overlays");
  stbi_write_png((dir_ / "media/overlays" / (stem + ".png")).string().c_str(), w, h, 3, img.data(), w * 3);
  write_json(fs::path("media/overlays") / (stem + ".json"),
             {{"camera_name", first.camera_name}, {"camera_frame", first.camera_name}, {"image_shape", {h, w}},
              {"candidates", records}});
  time_bucket("overlay", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}

void Recorder::finalize(json outcome) {
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
  outcome["wall_duration_s"] = wall;
  double attributed = 0;
  for (auto& [k, v] : buckets_.items()) attributed += v.get<double>();
  outcome["timing"] = {{"buckets", buckets_}, {"unattributed_s", std::max(0.0, wall - attributed)}};
  write_json("outcome.json", outcome);
  run_["status"] = "finalized";
  run_["finished_utc"] = utc_stamp();
  write_json("run.json", run_);
  events_.close(); steps_.close(); out_.close(); err_.close();
  json files = json::array();
  for (const auto& e : fs::recursive_directory_iterator(dir_)) {
    if (!e.is_regular_file()) continue;
    const auto rel = fs::relative(e.path(), dir_).generic_string();
    if (rel == "manifest.json") continue;
    files.push_back({{"path", rel}, {"size", e.file_size()}, {"sha256", sha256_hex(read_file(e.path()))}});
  }
  write_json("manifest.json", {{"files", files}});
}

}  // namespace skillarm
