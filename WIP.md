# CAD navigation footprint follow-up

PR194/candidate2d51bfb is pushed; canonical compute/Pi qualification and hosted
CI are running. Runtime remains ffc9566/main af9e42e, ARMED/stowed and held by the
old rectangular StopZone. No physical goal is active.

Measured front-left points are external to every physical CAD part. Replace
empty rectangle corners with the certified full-stow CAD envelope, retain all
sensor measurements, limits and reserves, and extend predictive braking to3.3s.
The camera wrist is optional and must not block Explore/navigation/front search.
Readiness lifecycle queries are now bounded at4Hz in the candidate.

See the newest handoff section for staging units, exact source, evidence,
prepared one-shot Fiber trial and the required deploy/physical validation.
