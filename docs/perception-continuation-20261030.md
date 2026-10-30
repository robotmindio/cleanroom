# Perception transport and Fiber continuation, 2026-10-30

## HTTP transport

An interleaved 32-request trial used the same stored JPEG, `cup` query and
480-pixel proposal profile. Sixteen requests per transport produced identical
detections. Opening a connection per request measured median 0.970 s and p95
2.252 s; reusing one connection measured median 0.700 s and p95 1.497 s.
This longer trial supersedes the inconclusive short trial in
[the initial findings](three-object-qualification-20261030.md).

`RemoteProvider` now serializes requests on a reusable standard-library HTTP(S)
connection. A timeout, malformed HTTP exchange or oversized response closes it;
the next caller reconnects. It never automatically replays a POST whose result
is uncertain. Response size, identity, image, query and profile checks remain.
Tests cover reuse, timeout without replay, reconnect, HTTP/1.0, error status,
invalid JSON and oversized replies.

Private live reports are under
`~/.ros/lekiwi/qualification/20261030-perception-continuation/`.
The initial 120-second camera run loaded the installed client, so it is a
baseline, not evidence for the transport change. Its 101 observation events and
63/120 stale-output diagnostic samples show why the two-second gate remains
necessary. A source-verified canary is measured separately.

The source-verified 120-second canary produced 130 events versus 101 in the
baseline. Final processing p95 decreased from 2.094 s to 1.689 s; stale-output
diagnostics remained frequent (67/120 samples versus 63/120). These sequential
live runs include variable network conditions and are not an independent
accuracy comparison or continuous-readiness acceptance. A direct Tailscale ping
to the GPU host measured 254 ms; its HTTP health round trip measured 515 ms.

## Fiber binding

The existing managed rosbridge is bound to the compute host's private Tailscale
address. No additional bridge is required. Fiber's stored connector was disabled
and allowed only task services. `config/fiber_perception_integration.json` is an
additive patch: union its topic/service lists with the existing lists and preserve
the connection URL, robot identity, headers, connect frames and guarded topics.
The integration PUT replaces its configuration; never send this partial file
alone or round-trip a response containing masked credentials (`***`).

`config/fiber_perception_profile.json` is the body for
`PUT /robot/profiles/near-field-stationary`. Its model hash is bound to the
deployed perception runtime (`PYTHONNOUSERSITE=1`, OpenCV 4.6.0, NumPy 1.26.4),
not an arbitrary development interpreter. Camera and proposal revisions are
checked against tracked configuration. It is image-only, has no search views,
and remains disabled pending qualification; no metric calibration is asserted.
The durian-seed appearance concept is not claimed as a qualified native category.

Applied through the official authenticated API: connector enabled, original
connection settings retained, and disabled profile registered. Live Fiber
observation query and evidence retrieval succeeded; the retrieved 18,284-byte
JPEG matched both the observation's and evidence service's SHA-256. Starting a
stationary goal with the disabled profile returned HTTP 409, `robot goal policy
disabled or revoked`. No completed object-finding goal is claimed.

Independent scene qualification, consistently fresh output, metric calibration
and manipulation acceptance remain separate requirements.
