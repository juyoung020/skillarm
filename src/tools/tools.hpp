// Skill helper subcommands of the skillarm CLI (docs/03_skill_scripts_spec.md).
#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

namespace skillarm::tools {

// Runs the subcommand `cmd` if it is one of the helper tools; returns false when `cmd` is not a tool.
bool dispatch(const std::string& cmd, const std::vector<std::string>& args, int& exit_code);

int cmd_init_run(const std::vector<std::string>& a);
int cmd_progress(const std::vector<std::string>& a);
int cmd_skill_sha(const std::vector<std::string>& a);
int cmd_check_skill_report(const std::vector<std::string>& a);
int cmd_dispatch(const std::vector<std::string>& a);
int cmd_lint_skills(const std::vector<std::string>& a);
int cmd_validate(const std::vector<std::string>& a);
int cmd_watchdog(const std::vector<std::string>& a);
int cmd_trace_stats(const std::vector<std::string>& a);
int cmd_audit_seeding(const std::vector<std::string>& a);
int cmd_evo_eval(const std::vector<std::string>& a);
int cmd_evo_analyze(const std::vector<std::string>& a);
int cmd_score(const std::vector<std::string>& a);
int cmd_evo_compare(const std::vector<std::string>& a);
int cmd_evo_progress(const std::vector<std::string>& a);

// §3.1: problems with a skill_report.json ("" list = valid).
std::vector<std::string> validate_skill_report(const nlohmann::json& report, const std::filesystem::path& skills_dir,
                                               bool check_skill_files);
// §2: regenerate progress.md (used by validate after a sweep). Returns the exit code.
int write_progress(const std::filesystem::path& run_root, const std::filesystem::path& tasks, bool print_only,
                   double rerun_threshold);
// §5 in process: lint one staged proposal. Returns the exit code and fills the report text.
int lint_skills(const std::filesystem::path* proposal, std::string& report);

}  // namespace skillarm::tools
