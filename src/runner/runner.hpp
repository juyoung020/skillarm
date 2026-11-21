// program.cpp -> checked -> compiled to a DLL -> run once per seed, each seed in its own process.
// The checks mirror ASPIRE's generated-program rules (runtime.py): a small allow-list of headers, no file / process /
// thread / OS access. Like ASPIRE's watchdog this is a guard rail against mistakes, not a security sandbox.
#pragma once
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

#include "api/segmenter.hpp"

namespace skillarm {

std::string check_program_source(const std::filesystem::path& src);  // "" when acceptable, else the reason
// Compiles with MSVC (Visual Studio 2022). Returns the DLL path; throws with the compiler output on failure.
std::filesystem::path compile_program(const std::filesystem::path& src, const std::filesystem::path& build_dir);
// Perception provider for trials (e.g. SAM3 when built with the GPU). Called on a program's first segmentation
// request, so trials that never segment do not load the networks. Without a factory, segmentation reports
// PROVIDER_UNAVAILABLE.
void set_segmenter_factory(std::function<std::unique_ptr<Segmenter>()> factory);
// Runs one seed in this process and writes a run directory under out_root. Returns the process exit code.
int run_trial(const std::filesystem::path& dll, const std::filesystem::path& src, const std::string& task, int seed,
              const std::filesystem::path& out_root);

}  // namespace skillarm
