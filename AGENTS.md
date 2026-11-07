# AGENTS.md

## §1 Project & team

DroneHacks is a drone simulation/hacking hackathon project.

**Maintainer & merge authority:** @moterodiaz

**Team:** @jsun4231, @mg343, @Orca6722, @TheSleepyPP

Area ownership is not yet assigned — see docs/BOARD.md owners table. This file's area list grows as code lands; do not add areas here speculatively.

**Code preloaded on the drone:** `drone-snapshot.zip` is the existing code that came preloaded on the drone's Jetson. Treat it as the reference for what is already on the drone. It is not in this repo: @jsun4231 holds it at `C:\Users\zsun\Downloads\drone-snapshot.zip` (captured 2026-11-06, SHA-256 `f44b5788f9e99615837e8cce65d363153789f82bdb36be8131c77a03702e175e`). Ask @jsun4231 for a copy. It contains:

- `home/dronehacks/skybeta/`: the organizer-supplied stack, `skybeta` `VERSION` 0.3.2. Its README names the upstream as `Artash-N/skybeta`, branch `fleet-ready`. Its `README.md` and `docs/competitor_guide.md` describe the interfaces and on-drone rules.
- `home/dronehacks/team_ws/src/`: empty. Team code goes here on the drone.
- `etc/skybeta/drone.env`: per-drone identity and launch settings.

## §2 Scope & review

- State the files/dirs you intend to change before editing.
- Keep PRs focused to one area. Cross-area changes require explicit maintainer approval before work starts.
- All merges go through the board workflow in docs/BOARD.md.
- Merge only when `@moterodiaz` authorizes it at the reviewed SHA.

## §3 Non-negotiable invariants

A reviewer may cite any invariant by number (e.g. "§3.4").

1. **Contracts beat code.** If code disagrees with `contracts/`, the code is wrong. To change a contract: open a PR, add a `contracts/CHANGELOG.md` entry, and ping affected area owners — then change the code in the same PR or a follow-up that links back to it.
2. **No secrets in the repo, in commits, or in `.env`.** See `.gitignore` and `contracts/README.md`. If a secret leaks, notify @moterodiaz immediately so it can be rotated before any history cleanup.
3. **Never force-push, rewrite `main`, or destroy another contributor's uncommitted work.** Revert bad merges via a new PR.
4. **Every PR goes through docs/BOARD.md's label lifecycle before merge.** There is no out-of-band merge.
5. **Escalate to a human (`needs-human` label + assignee) rather than guessing on:** spend, production/deployment actions, security-relevant guard changes, or any cross-area contract break.
6. **Acro mode is not allowed.** No code, launch file, PX4 parameter or flight-mode mapping may command, enable or fall back to PX4 Acro mode (`ACRO`, nav_state 10). Set by @jsun4231, 2026-11-06.

Invariant 7 is reserved for a real constraint that emerges once code and areas exist. Do not invent entries here.

## §4 Tech conventions

No stack has been chosen yet. Add language/framework conventions here once the first service/module lands; keep this section a living pointer, not boilerplate.

## §5 Validation

No test suite exists yet. Once one does, document the exact command here (e.g. `make test-<area>`). Until then, PRs are validated by review against docs/BOARD.md and AGENTS.md §3.

## §6 Parallel work

Use isolated git worktrees for parallel implementers. Assign explicit non-overlapping file lists. Each implementer commits on its own branch; only the branch author pushes and opens the PR. A reviewer who did not write the diff must independently check it per docs/BOARD.md.

Don't assume every task needs a dedicated worktree or standing orchestration loop — use judgment.
