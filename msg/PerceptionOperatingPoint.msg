# The operating point one detector frame ran under, stamped with that frame's capture header and
# published beside its source image and detections so the collector records exactly what produced them.
std_msgs/Header header
# Recognizer identity: weights, runtime and input normalization.
string model_sha256
# Canonical JSON of the proposal profile document (lekiwi_rmf/perception_profiles.py).
string profile_json
# Canonical JSON of the open-vocabulary query document; empty for a closed native catalogue.
string query_json
# SHA-256 of the evidence artifact bytes the recognizer consumed.
string input_sha256
# JSON list aligned with the detections: suppressed alternatives and concept matches per detection.
string detections_json
# JSON of the provider's run reference (run_id, inference_id, proposals); empty for local inference.
string run_json
# Non-empty when the fallback served this frame because the primary was unavailable.
string degraded_reason
