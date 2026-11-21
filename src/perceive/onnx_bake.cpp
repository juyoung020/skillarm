#include "perceive/onnx_bake.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string_view>

namespace skillarm {
namespace {
using Bytes = std::string;

uint64_t get_varint(std::string_view s, size_t& i) {
  uint64_t v = 0;
  for (int shift = 0; i < s.size() && shift < 64; shift += 7) {
    const uint8_t b = uint8_t(s[i++]);
    v |= uint64_t(b & 0x7F) << shift;
    if (!(b & 0x80)) return v;
  }
  throw std::runtime_error("onnx: bad varint");
}
void put_varint(Bytes& out, uint64_t v) {
  while (v >= 0x80) { out.push_back(char(uint8_t(v) | 0x80)); v >>= 7; }
  out.push_back(char(v));
}

struct Field {
  uint32_t num = 0, wire = 0;
  std::string_view body;  // payload of a length-delimited field
  size_t begin = 0, end = 0;  // whole field (key included) in the parent
};

// next field of a message; false at the end
bool next(std::string_view s, size_t& i, Field& f) {
  if (i >= s.size()) return false;
  f.begin = i;
  const uint64_t key = get_varint(s, i);
  f.num = uint32_t(key >> 3);
  f.wire = uint32_t(key & 7);
  switch (f.wire) {
    case 0: get_varint(s, i); break;
    case 1: i += 8; break;
    case 5: i += 4; break;
    case 2: {
      const uint64_t n = get_varint(s, i);
      if (i + n > s.size()) throw std::runtime_error("onnx: truncated field");
      f.body = s.substr(i, size_t(n));
      i += size_t(n);
      break;
    }
    default: throw std::runtime_error("onnx: unsupported wire type");
  }
  f.end = i;
  return true;
}

std::string_view sub(std::string_view msg, uint32_t num) {  // first length-delimited field `num`
  size_t i = 0;
  Field f;
  while (next(msg, i, f))
    if (f.num == num && f.wire == 2) return f.body;
  return {};
}

void put_len(Bytes& out, uint32_t num, std::string_view body) {
  put_varint(out, (uint64_t(num) << 3) | 2);
  put_varint(out, body.size());
  out.append(body);
}

// ValueInfoProto(1 name, 2 type) -> TypeProto(1 tensor_type) -> Tensor(2 shape) -> TensorShapeProto(1 dim)*
int declared_rank(std::string_view value_info) {
  const auto shape = sub(sub(sub(value_info, 2), 1), 2);
  int rank = 0;
  size_t i = 0;
  Field f;
  while (next(shape, i, f)) rank += f.num == 1;
  return rank;
}

Bytes rewrite_graph(std::string_view g, const std::string& name, const std::vector<int64_t>& values) {
  Bytes out;
  out.reserve(g.size() + 64);
  int rank = -1;
  size_t i = 0;
  Field f;
  while (next(g, i, f)) {
    if (f.num == 11 && f.wire == 2 && sub(f.body, 1) == name) {  // GraphProto.input
      rank = declared_rank(f.body);
      continue;
    }
    out.append(g.substr(f.begin, f.end - f.begin));
  }
  if (rank < 0) throw std::runtime_error("onnx: no graph input named " + name);
  // TensorProto: 1 dims, 2 data_type (7 = INT64), 7 int64_data (packed), 8 name
  Bytes t;
  if (rank >= 1) { put_varint(t, (1u << 3) | 0); put_varint(t, values.size()); }
  put_varint(t, (2u << 3) | 0);
  put_varint(t, 7);
  Bytes packed;
  for (int64_t v : values) put_varint(packed, uint64_t(v));
  put_len(t, 7, packed);
  put_len(t, 8, name);
  put_len(out, 5, t);  // GraphProto.initializer
  return out;
}
}  // namespace

void onnx_bake_int64_input(const std::string& in_path, const std::string& out_path, const std::string& name,
                           const std::vector<int64_t>& values) {
  std::ifstream in(in_path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + in_path);
  const Bytes model((std::istreambuf_iterator<char>(in)), {});
  Bytes out;
  out.reserve(model.size() + 64);
  bool found = false;
  size_t i = 0;
  Field f;
  while (next(model, i, f)) {
    if (f.num == 7 && f.wire == 2) {  // ModelProto.graph
      put_len(out, 7, rewrite_graph(f.body, name, values));
      found = true;
    } else {
      out.append(std::string_view(model).substr(f.begin, f.end - f.begin));
    }
  }
  if (!found) throw std::runtime_error(in_path + " has no graph");
  std::ofstream o(out_path, std::ios::binary);
  o.write(out.data(), std::streamsize(out.size()));
  if (!o) throw std::runtime_error("cannot write " + out_path);
}

}  // namespace skillarm
