# C++ diagnostics missing from Problems

This template carries a troubleshooting procedure verified in CP18 on 2026-10-09.
A command-line task successfully displayed diagnostics; the precise cause of the
native extension publication failure was not isolated. This is historical evidence,
not a claim that pid_maze_solver currently has the same failure.

## Minimum checks

1. Confirm the WSL distribution, enabled extension, effective settings scope, active
   C++ source, language standard and matching compilation database.
2. Run clang-tidy directly; distinguish parser errors from style warnings.
3. Check Problems text/severity filters and the extension's Debug log for its actual
   executable and arguments.
4. If CLI diagnostics exist but native Problems entries do not, run the package's
   `clang-tidy (Problems)` task. Verify file, line, severity and clickable navigation
   with a separate known-diagnostic probe, without breaking learner source.
5. Compare CLI and UI results, record the evidence, and restore temporary logging.

An empty Problems view alone is not a clean analysis result. For a workspace opened
above the package, merge the task and its uniquely named input into the actual editor
root; nested .vscode files are not automatically applied. Preserve existing tasks.
The task does not run a robot or apply automatic fixes. If it also fails, inspect its
exit code, text format, path mapping, matcher and compilation database membership.

## Macro-depth compatibility

`readability-function-size.IgnoreMacros` was verified with the installed LLVM23
checker and a paired macro/handwritten-nesting probe; system LLVM14 lacks this option.
Check the actual executable before enabling it. Keep meaningful source nesting and
length checks. Older environments should explicitly defer this option rather than
silently disable all diagnostics.

The local bundled checker required an explicit installed Clang resource-header path
for stddef.h. This is environment-specific: do not copy extension versions or user
paths into a portable template. Verify complete header parsing on the target host.
Later CP18 evidence showed both task and native diagnostics again; identical
file/line/rule entries from two providers are duplicates, not two code defects.

Synchronize tool changes with the engineering record, Obsidian summary and template
strategy. Ruff placeholder errors are a separate issue; changing PID behavior or
turning off unrelated tools is not a diagnostic-display repair.
