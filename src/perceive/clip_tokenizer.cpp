#include "perceive/clip_tokenizer.hpp"

#include <algorithm>
#include <climits>
#include <fstream>
#include <stdexcept>

#include "nlohmann/json.hpp"

namespace skillarm {
namespace {

std::string utf8(uint32_t cp) {
  std::string s;
  if (cp < 0x80) s += char(cp);
  else if (cp < 0x800) { s += char(0xC0 | (cp >> 6)); s += char(0x80 | (cp & 0x3F)); }
  else { s += char(0xE0 | (cp >> 12)); s += char(0x80 | ((cp >> 6) & 0x3F)); s += char(0x80 | (cp & 0x3F)); }
  return s;
}

bool is_space(unsigned char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }
bool is_digit(unsigned char c) { return c >= '0' && c <= '9'; }
bool is_letter(unsigned char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c >= 0x80; }

}  // namespace

ClipTokenizer::ClipTokenizer(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot read " + path);
  const auto j = nlohmann::json::parse(f);
  const auto& model = j.at("model");
  for (auto& [k, v] : model.at("vocab").items()) vocab_[k] = v.get<int64_t>();
  int rank = 0;
  for (const auto& m : model.at("merges")) {
    std::string a, b;
    if (m.is_array()) { a = m[0].get<std::string>(); b = m[1].get<std::string>(); }
    else { const auto s = m.get<std::string>(); const auto sp = s.find(' '); a = s.substr(0, sp); b = s.substr(sp + 1); }
    ranks_.emplace(a + '\x01' + b, rank++);
  }
  for (const auto& t : j.value("added_tokens", nlohmann::json::array())) {
    const auto c = t.value("content", "");
    if (c == "<|startoftext|>") bos_ = t.at("id").get<int64_t>();
    if (c == "<|endoftext|>") eos_ = pad_ = t.at("id").get<int64_t>();
  }
  if (j.contains("padding") && j["padding"].is_object()) pad_ = j["padding"].value("pad_id", pad_);
  // GPT-2 byte -> unicode table: printable bytes map to themselves, the rest to 256 + n
  int n = 0;
  for (int b = 0; b < 256; ++b) {
    const bool printable = (b >= '!' && b <= '~') || (b >= 0xA1 && b <= 0xAC) || (b >= 0xAE && b <= 0xFF);
    byte_enc_[size_t(b)] = utf8(printable ? uint32_t(b) : uint32_t(256 + n++));
  }
}

std::vector<int64_t> ClipTokenizer::bpe_ids(const std::string& word) const {
  if (auto it = cache_.find(word); it != cache_.end()) return it->second;
  std::vector<std::string> sym;
  for (unsigned char c : word) sym.push_back(byte_enc_[c]);
  if (sym.empty()) return {};
  sym.back() += "</w>";
  while (sym.size() > 1) {
    int best = INT_MAX;
    size_t at = 0;
    for (size_t i = 0; i + 1 < sym.size(); ++i) {
      auto it = ranks_.find(sym[i] + '\x01' + sym[i + 1]);
      if (it != ranks_.end() && it->second < best) { best = it->second; at = i; }
    }
    if (best == INT_MAX) break;
    // merge every occurrence of the best pair, left to right
    const std::string a = sym[at], b = sym[at + 1];
    std::vector<std::string> out;
    for (size_t i = 0; i < sym.size();) {
      if (i + 1 < sym.size() && sym[i] == a && sym[i + 1] == b) { out.push_back(a + b); i += 2; }
      else out.push_back(sym[i++]);
    }
    sym.swap(out);
  }
  std::vector<int64_t> ids;
  for (const auto& s : sym) {
    auto it = vocab_.find(s);
    ids.push_back(it == vocab_.end() ? eos_ : it->second);  // unk = <|endoftext|>
  }
  cache_.emplace(word, ids);
  return ids;
}

ClipTokenizer::Encoded ClipTokenizer::encode(const std::string& text, int length) const {
  // normalise: collapse whitespace, trim, lower-case
  std::string t;
  bool sp = false;
  for (unsigned char c : text) {
    if (is_space(c)) { sp = !t.empty(); continue; }
    if (sp) { t += ' '; sp = false; }
    t += char(c >= 'A' && c <= 'Z' ? c + 32 : c);
  }
  std::vector<int64_t> ids{bos_};
  static const char* contractions[] = {"'s", "'t", "'re", "'ve", "'m", "'ll", "'d"};
  for (size_t i = 0; i < t.size();) {
    const unsigned char c = t[i];
    if (is_space(c)) { ++i; continue; }
    size_t j = i;
    bool done = false;
    if (c == '\'') {
      for (const char* k : contractions) {
        const size_t n = std::char_traits<char>::length(k);
        if (t.compare(i, n, k) == 0) { j = i + n; done = true; break; }
      }
    }
    if (!done) {
      if (is_letter(c)) while (j < t.size() && is_letter(t[j])) ++j;
      else if (is_digit(c)) j = i + 1;
      else while (j < t.size() && !is_space(t[j]) && !is_letter(t[j]) && !is_digit(t[j])) ++j;
    }
    for (auto id : bpe_ids(t.substr(i, j - i))) ids.push_back(id);
    i = j;
  }
  if (int(ids.size()) > length - 1) ids.resize(size_t(length - 1));
  ids.push_back(eos_);
  Encoded e;
  e.attention_mask.assign(size_t(length), 0);
  std::fill(e.attention_mask.begin(), e.attention_mask.begin() + ptrdiff_t(ids.size()), 1);
  e.ids = ids;
  e.ids.resize(size_t(length), pad_);
  return e;
}

}  // namespace skillarm
