# OpenCV Zoo YOLOX

`yolox.py` is from OpenCV Zoo commit
`47534e27c9851bb1128ccc0102f1145e27f23f98`,
`models/object_detection_yolox/yolox.py`, under the included Apache 2.0 license.
The wrapper is installed as `lekiwi_rmf.opencv_yolox`; no extra inference package
or NumPy upgrade is needed. The model installer downloads the exact 35,858,002
byte ONNX artifact and verifies its Git LFS SHA-256. Weights stay outside Git.

Local modification: use OpenCV's per-class `NMSBoxes` on versions lacking
`NMSBoxesBatched`, preserving class-aware suppression on Ubuntu's OpenCV.

Upstream: https://github.com/opencv/opencv_zoo/tree/47534e27c9851bb1128ccc0102f1145e27f23f98/models/object_detection_yolox
