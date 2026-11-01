# Remote capture throughput trial

The physical front camera showed the patterned cup, while local RT-DETR
classified it as a vase. Grounding DINO recognized it with an explicit `cup`
query at the unchanged 0.5 threshold. This is image-only evidence; metric
distance, grasping and general category accuracy remain unqualified.

Serial remote requests spent network time between captures. The front bridge's
2 Hz cap added up to another 0.5 seconds before inference. Increasing only the
front cap to 5 Hz and the detector timer to 4 Hz reduced measured queue p95
from 0.459 to 0.196 seconds, but all three deployed goals still failed. Lidar
remained at 4.97 Hz with 0.122-second p95 age. That trial was rolled back.

The revised implementation keeps the original two inference starts per second
and allows at most two remote requests in flight on separate reusable HTTP
connections. Local models remain exclusive. A full pipeline retains only each
camera's newest pending frame. Queries, profiles and timing samples remain
bound to individual requests; an older successful request cannot erase another
request's failure cooldown or move the published timestamp backwards.

The actual candidate executable was built in an isolated overlay using
`scripts/build-lekiwi.sh --device`; its bytes matched the reviewed source.
The main robot runtime was not replaced. On this candidate, 10 of 12 native
compound reads returned two fresh captures with verified JPEG hashes and live
owned pin receipts. No native records were available in the earlier serial
12-read sample. These are short, separate live samples, not a statistical
acceptance claim.

Three deployed goals with automatic query setup and three with a preset `cup`
query still ended unverifiable. Native throughput improvement does not resolve
the complete WAN verification deadline. All temporary profiles were disabled,
the preset query was restored, and the ordinary sensor bridge was restored.
No base commands or torque cuts were issued.

Validation: 121 affected scheduling, HTTP, query, profile and bridge tests;
71 observation-store tests; three observation-node tests; Ruff and whitespace
checks passed. Three added regressions fail against the serial baseline while
the local exclusive scheduling control passes. A separate Temporal incident
showed a 171.5-second workflow scheduling delay and a ten-second task timeout;
this change does not claim to fix cloud scheduling.
