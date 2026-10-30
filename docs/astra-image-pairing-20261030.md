# Astra RGB/depth transport pairing

The split deployment previously throttled RGB and depth independently. A
25-second compute-side audit found only 33 of 47 color frames had a transported
depth frame within 50 ms. Identical frame names do not make those captures
simultaneous.

The shared Astra filter now retains at most one image per camera, discards the
older unmatched capture until the stamps differ by at most 50 ms, and encodes
both images before publishing either. The shared image rate stays at 2 Hz.
Original capture stamps, JPEG quality 70 and lossless millimetre depth PNG are
preserved. Invalid stamps, encodings or conversion failures discard the pending
pair. The independent compact-cloud callback and its 10 Hz collision-sensing
configuration are unchanged.

An isolated 45-second Pi canary subscribed to the real driver and published only
under `/qualification/astra`, without restarting the driver or replacing live
topics. The cloud input was remapped to an unused private topic. Compared in the
same interval:

| Measurement | Existing transport | Capture-paired canary |
| --- | ---: | ---: |
| RGB / depth captures | 84 / 85 | 78 / 78 |
| RGB with depth within 50 ms | 35 / 84 | 78 / 78 |
| Maximum nearest-depth skew | 267.5 ms | 46.6 ms |
| Median nearest-depth skew | 66.8 ms | 17.5 ms |
| RGB rate | 1.86 Hz | 1.73 Hz |
| Encoded traffic | 114.0 kB/s | 105.5 kB/s |

The temporary unit was stopped after the measurement. Private source hash and
reports are under `~/.ros/lekiwi/qualification/20261030-astra-pairing/` on compute
and the Pi. Tests exercise both arrival orders, rejection just beyond the skew
boundary, acceptance at exactly 50 ms, invalid inputs without a half-pair,
preserved pixel data/stamps and the shared rate limit.

Pairing is a transport prerequisite, not metric calibration. The Astra still
needs independent measurements of RGB/depth registration, depth scale and
camera-to-base geometry before a tracked metric calibration may be enabled.
