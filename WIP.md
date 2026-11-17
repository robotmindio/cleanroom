# CAD navigation footprint follow-up

PR194/candidate216516d is pushed; canonical compute/Pi qualification and hosted
CI are running. Runtime remains ffc9566/main af9e42e, ARMED/stowed and held by the
old rectangular StopZone. No physical goal is active.

Measured front-left points are external to every physical CAD part. Replace
empty rectangle corners with the certified full-stow CAD envelope, retain all
sensor measurements, limits and reserves, and extend predictive braking to3.3s.
The camera wrist is optional and must not block Explore/navigation/front search.
Readiness lifecycle queries are now bounded at4Hz in the candidate.

See the newest handoff section for staging units, exact source, evidence,
prepared one-shot Fiber trial and the required deploy/physical validation.

First failed compute qualification and Pi timing failures are preserved; final
source includes corrected geometry fixtures. Final CI run44056267191; final
compute/Pi staging units have cad-final/cad-device-final in their names.
Live front observation stream_ready is true despite the absent wrist.

Final head216516d fixes only stop/reverse test fixtures after complete109/111
51ef qualification. New canonical units cad-v3/cad-device-v3 and CI44058959385
are running; use only216 helpers. Nothing deployed or moved yet.
