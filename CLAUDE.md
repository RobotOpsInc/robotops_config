# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Git Workflow

For Linear issues, do all work in a worktree created from the Linear issue branch name. Set-head the worktree to `development` before branching, and open PRs against `development`.

## AI Contribution Policy

When using generative AI assistance, review and test all changes before submitting them. Do not include confidential, proprietary, or third-party content in prompts or generated output unless RobotOps has the right to share and license it. Disclose meaningful AI assistance in the commit message or pull request description.

## Internal Robot Ops Guidance

For Robot Ops safety and consistency checks, consult `RobotOpsInc/canopy` (`vault/`) alongside the repo and related code paths before making changes. This repo is documented at `vault/projects/robotops-config/`.

Use the repo name lowercased with underscores replaced by hyphens as the vault slug
(e.g. `robot_agent` → `robot-agent`, `web_app` → `web-app`).

### Cross-check before…

* Making an architectural decision — review `vault/decisions/` and
  `vault/projects/robotops-config/decisions/`
* Touching a shared interface (protos, RMW API, config schema) — review the relevant
  project notes and current code paths to understand what downstream repos depend on
* Investigating a regression that might be a known incident — check `vault/incidents/`

### Leave a raw note when…

When something notable happens — a decision is made, a public interface changes, a
non-obvious bug is fixed, a constraint is discovered — create a file at:

`vault/_raw/robotops-config-YYYY-MM-DD-<short-slug>.md`

in the `RobotOpsInc/canopy` repo and open a PR against `main`. Keep it factual: what
changed, why, any cross-repo implications. Especially for anything architectural or a
new feature, describe in detail. You can use illustrations, links, text — the ingestion
pipeline is flexible. The ingest workflow handles everything from there.
Do not write vault pages directly.
