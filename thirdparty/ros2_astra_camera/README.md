# Astra native driver fixes

The driver is pinned in `scripts/install.sh`. Installation and
`scripts/build-lekiwi.sh` apply the same patches; split deployment rebuilds
an installed Astra driver on the USB host.

`0001` supplies Jazzy build compatibility.

`0002` rejects unavailable, nonfinite or nonpositive factory intrinsics,
reports missing calibration explicitly, and preserves finite field-of-view
fallbacks instead of copying invalid factory values into depth CameraInfo.
It applies depth principal-point offsets once to both K and P and preserves
factory color distortion when valid calibration is available. The factory
parameter service reports failure when its calibration is invalid.

Field-of-view defaults are **uncalibrated**. They prevent invalid metadata;
they do not establish RGB/depth alignment or precise visual odometry. On the
current Astra Pro, the factory query returned NaN for every calibration field.
Measured optical calibration remains required for those accuracy claims.
