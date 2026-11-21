// Run recorder: writes the same run-directory layout and key names as ASPIRE (docs/01_aspire_analysis.md §4),
// so the iterative-debugging skill documents can point at the same files:
//   run.json, outcome.json, manifest.json, source/program.cpp, source/program-result.json,
//   episode/reset.json, episode/steps.jsonl, trace/events.jsonl, trace/calls/call-NNNNNN-<tool>/{input,output,error}.json,
//   media/keyframes/<NNNNNN>-before-<tool>/<camera>.png, logs/program.stdout, logs/program.stderr
#pragma once
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "nlohmann/json.hpp"

namespace skillarm {

class Recorder {
 public:
  Recorder(const std::filesystem::path& out_root, const std::string& benchmark, const std::string& task_id, int seed,
           const std::filesystem::path& program_source);
  const std::filesystem::path& dir() const { return dir_; }

  int span_start(const std::string& name, const std::string& category, int parent = -1);
  void span_end(int span, bool ok, const nlohmann::json& input, const nlohmann::json& output,
                const nlohmann::json& error = nullptr);
  void step(const nlohmann::json& rec);
  void reset_info(const nlohmann::json& j);
  void stdout_line(const std::string& s);
  void stderr_line(const std::string& s);
  void time_bucket(const std::string& name, double seconds);
  void result(const nlohmann::json& program_result);
  // Writes outcome.json, finalizes run.json, then manifest.json (relative path, size, sha256 of every file).
  void finalize(nlohmann::json outcome);

 private:
  void write_json(const std::filesystem::path& rel, const nlohmann::json& j);
  std::filesystem::path dir_;
  std::ofstream events_, steps_, out_, err_;
  nlohmann::json run_;
  nlohmann::json buckets_ = nlohmann::json::object();
  int next_span_ = 1, next_call_ = 1;
  std::chrono::steady_clock::time_point t0_;
  struct Open { std::string name, category; int parent; std::chrono::steady_clock::time_point t; int call; };
  std::vector<Open> open_;
};

std::string sha256_hex(const std::string& data);

}  // namespace skillarm
