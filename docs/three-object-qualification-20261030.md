# Nearby object qualification findings, 2026-10-30

The live development scene contains a patterned mug, a green spiky ball and an
operator-identified durian seed. Robot viewpoints were changed using the existing
bounded test helper, with the original physical authorization reference retained.
The conservative early translation guard remains 14 cm inside the authorized
30 cm radius; collision, stow, freshness and speed gates remain active.
Private inputs and reports are under
`~/.ros/lekiwi/qualification/20261030-three-objects/`.

## Corrected software boundaries

The primary provider's ten-second retry window is shared across cameras. A failure
previously marked only its triggering camera, leaving another camera's fallback
frame without degradation provenance and potentially reporting healthy. All cameras
sharing that window now retain the failure; primary recovery clears the serving
camera. The two-camera regression fails before this change and passes afterward.

The provider serves the same pinned Grounding DINO weights under a newer runtime.
Its operating-point digest `0986e83de3b27f7fa69634a0d13270681423446b248338da60b984a9a0c0a87e`
was recomputed from its exact manifest and runtime (Torch 2.7.1+cu126,
transformers 5.10.1, Pillow 13.0.0, GTX 1070). The tracked pin now matches it;
response/query/image/profile validation and fallback remain in force. This is
runtime compatibility evidence, not independent recognition acceptance.

Learned concepts were rejected unless accompanied by a plain label, even though
the provider supports concept-only queries. That extra phrase changes Grounding
DINO proposals: the nearby seed was proposed with `seed` alone but lost beside
`cup`. Both query construction and the ROS service now admit a nonempty concept-only
query. Empty labels **and** empty concepts still restore the default at the service;
empty semantic queries remain invalid in `make_query`. Parts still require their
named whole, and phrase/count/canonical-recording bounds are preserved. Regression
coverage includes service admission, immutable query replay and the HTTP contract.

## Measured limits and trial policy

The standard open-vocabulary prompt missed the mug in some views. An explicit
single-category `cup` query found it. Closed-catalogue RT-DETR also mislabeled the
mug as a vase and the gripper as an aeroplane. Neither model scores nor repeated
captures alone establish category correctness.

Local CPU Grounding DINO exceeded the two-second freshness window, including at
240 pixels; it was not selected as a replacement. The opt-in tracked `near_field`
remote profile uses a 480-pixel shortest edge with the same score, text and collision
policies. Initial scene trials were faster than 800 pixels; some network outliers
still exceeded freshness and were rejected. The standard profile and disabled
production perception startup remain unchanged. A persistent HTTP connection trial
did not consistently improve the tail, so no transport rewrite was added.

Plain `durian seed` incorrectly localized the spiky ball. An appearance concept
learned from manually boxed seed views and negative ball/mug/floor regions rejected
that confusion and localized the seed in a newly captured close view when queried
alone. Its small same-arrangement teaching/evaluation sample is development
evidence, not independent category or room-scale qualification.

Live query checks exercised capture/store epoch, duplicate IDs, pinned-query
idempotency, byte-exact JPEG hashes and malformed/stale cursor rejection. Pausing
only the detector made the observation service report stale; resume recovered.
Restarting the perception service preserved the pinned observation and evidence.
The Fiber stationary tool returned `robot profile not configured`; admission did
not occur and no goal completion is claimed.

Independent scene coverage, the deployed Fiber tenant/connector/profile journey,
metric camera calibration and manipulation acceptance require their own evidence.
These development images must not be split across teaching and independent
qualification datasets to manufacture scene independence.
