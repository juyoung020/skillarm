// CLIP byte-level BPE tokenizer (the SAM3 text encoder's), read from a Hugging Face tokenizer.json.
// Matches CLIPTokenizer for the text SAM3 sees: whitespace collapsed, lower-cased, split on the CLIP pattern
// ('s 't 're 've 'm 'll 'd | letters+ | one digit | other non-space runs), bytes mapped to unicode, BPE with "</w>"
// on the last symbol, then <|startoftext|> ... <|endoftext|> padded to a fixed length with <|endoftext|>.
// Non-ASCII UTF-8 bytes count as letters (no Unicode tables); object names are ASCII in practice.
#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace skillarm {

class ClipTokenizer {
 public:
  explicit ClipTokenizer(const std::string& tokenizer_json);
  struct Encoded { std::vector<int64_t> ids, attention_mask; };
  Encoded encode(const std::string& text, int length = 32) const;
  std::vector<int64_t> bpe_ids(const std::string& word) const;  // one pre-token -> ids (testing)

 private:
  std::unordered_map<std::string, int64_t> vocab_;
  std::unordered_map<std::string, int> ranks_;  // "a\x01b" -> merge rank
  std::array<std::string, 256> byte_enc_;       // byte -> UTF-8 of its mapped code point
  int64_t bos_ = 49406, eos_ = 49407, pad_ = 49407;
  mutable std::unordered_map<std::string, std::vector<int64_t>> cache_;
};

}  // namespace skillarm
