# Remote provider recovery

The first daylight compact-receipt batch completed two of three stationary cup
goals. The failed 15-second goal received local fallback frames for its first
12 seconds after remote timeouts. Those frames reported `vase`; the admitted
remote model only returned `cup` near the deadline. Activity queue waits were
under 11 ms during the failed goal, so this failure was distinct from the earlier
171-second workflow scheduling incident.

The detector now retries a failed primary on later captures after 1, 2, 4, 8,
then at most 10 seconds. A successful primary response resets that progression
when no newer failure window is active. Failures share one locked retry state
across cameras. An older in-flight success cannot clear a newer failure or its
backoff progression. The existing request-start rate and two-request remote
concurrency bound still apply. The HTTP transport never replays an uncertain
POST; retries consume later captures.

Validation uses a controlled clock to check transient recovery, sustained-outage
backoff, its ten-second cap, recovery reset, cross-camera sharing and an
out-of-order success. The recovery regression fails against the fixed-ten-second
baseline. The affected HTTP, ROS scheduling, query and profile suites pass.
Positive robot acceptance of this candidate remains a separate check; the first
2/3 batch is preserved as a failure rather than replaced by a later retry.

## Lossless request transport

The remote adapter sends ordinary JSON until its configured peer explicitly
advertises `Accept-Encoding: gzip`. It then compresses later request bodies on
that same connection when doing so reduces their byte count. A closed connection
or a peer withdrawing support returns later captures to ordinary JSON. Neither
case replays a POST whose reply was lost. Connection count and request-start
limits are unchanged.

On a captured daylight image, the JSON request measured 25,762 bytes and gzip
19,460 bytes, with an exact byte-for-byte decode and under a millisecond of local
compression. The JPEG and the model's decoded pixels are unchanged. The peer must
bound both compressed and decoded bodies; this requires the matching provider
implementation. This transport study alone does not qualify positive robot
goals under the two-second final evidence limit.

## Live failover measurement

After sourcing `scripts/setup.bash`, run:

```bash
scripts/test-perception-latency.py --output /path/to/provider-failover.json
```

The script relays real front/wrist images unchanged from source domain 0 into
an otherwise unused test domain 222. All inference publishers and services
remain in the isolated domain; it sends no robot commands and writes no
observations. It uses the tracked remote model, CPU fallback, total request
rate and freshness limits. Install both pinned models before running it.

After warmup, it measures normal operation, process-local provider failure
and recovery for 60 seconds each, excluding 12-second transition windows.
The failure is injected at the remote adapter boundary as `ProviderUnavailable`;
it does not interrupt the server or network. The report records stage sample
counts and p50/p95/p99, fresh/late answers and per-camera output/fallback counts.
Both cameras must produce fresh outputs in each phase, with fallback-only
outputs during injected loss and primary outputs after recovery.

This establishes live inference failover and measures the observed load. It
does not establish detector accuracy, independent GPU saturation, a real
transport outage, navigation/skill latency or replacement production budgets
for #102. Run under an external bounded process supervisor for crash cleanup.
