# Autonomous exploration follow-up

Current checkpoint: [HANDOFF-2026-11-16-autonomia.md](HANDOFF-2026-11-16-autonomia.md),
17 November 09:51 MYT / 01:51 UTC.

9503b92 is now qualified (107 compute + 16 Pi), sealed and deployed on both hosts;
previous is fa91885. Native mapper 0.15 sync interval and library prefix verified.
The Pi kernel regression was repaired: failed VL805 PCIe USB DMA mask 36 -> 40,
exact-kernel module built, activated, persisted and included in matching initramfs.
Astra/lidar streams recovered without reboot. The supported arm pose was returned
to travel_stow by collision-checked MoveIt at 0.1 scaling; fresh strict gate passed.
Default armed policy and all safety limits remain active.

PR #190 (head 67530c3) contains reproducible recovery tools, documentation and
139 passing local tests. Ruff passes. Hosted build-test-lint is still running;
CodeQL passed. Branch clean/pushed; PR remains draft pending final CI/review.

The first new Explore trial (generation 55) terminated safely with region stopping
margin reached during mapping-frame transition, stop confirmed, resources free,
no physical base movement. No further motion task or recorder remains active.
Diagnose and reproduce coherent map/TF startup before another bounded trial.
Do not widen region/freshness limits. Wrist JYU2C USB camera is missing (-71);
front GENERAL, Astra, scan and depth are functioning.

A private Pi journal export is still running (compute SSH 237859, Pi journalctl
74588 nice 15/gzip74587). Verify it completes before vacuuming archived journals.
Preserve selected/previous releases, .env and qualification evidence.
