Native evidence receipts

- Implemented opt-in native_sha256_v1 representation for the existing compound service; default JPEG behavior preserved.
- Both modes read/hash actual immutable bytes before pinning. New receipts carry actual byte length and an explicit verifier tag, not JPEG data.
- Verified: 81 native store/node tests pass, including invalid formats and corrupted evidence in both modes.
- Next: implement explicit Fiber connector opt-in and strict receipt checks; preserve JPEGs for entity reidentification; test both positive and negative outcomes. Then compare real native and deployed goal timings before promotion.
- Pending: live qualification, peer interoperability, PR/CI. No robot runtime deployment performed by this branch.
