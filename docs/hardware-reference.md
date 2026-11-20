# Hardware reference values (vendor data, not acceptance)

The numbers below are what the manufacturers and the upstream project publish for
this robot's parts. They exist to sanity-check the tracked limits and to keep a
default from being looser than the hardware can support.

They are **not acceptance evidence**. `config/safety_acceptance.yaml` stays the
only record that permits motion, and it is bound to measured stopping trials,
measured stop latency and fault-injection results on this assembled robot, at a
recorded payload, surface and software revision. A vendor rating says what a part
was designed for; it says nothing about this robot's stopping distance in this
room, the collision matrix of this assembled arm, or whether the arm actually
rests on its contacts inside `stow_tolerance`. Nothing here may set
`validated: true`, and nothing here may turn on an `allow_*` capability.

## Parts

| Part | Vendor value | Source |
| --- | --- | --- |
| LeKiwi base | Three omni (kiwi) wheels, holonomic; speed modes 0.1, 0.2 and 0.3 m/s (30, 60, 90 deg/s); base about 33 x 33 x 12 cm, 2.4 kg | [LeRobot LeKiwi](https://huggingface.co/docs/lerobot/en/lekiwi), [SIGRobotics LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) |
| SO-101 arm | 6 DOF (5 joints + gripper), reach about 500 mm, payload about 500 g | [LeRobot SO-101](https://huggingface.co/docs/lerobot/en/so101), [spec sheet](https://roboticscenter.ai/hardware/so-101/specs) |
| Feetech STS3215 | Serial-bus servo, 1/345 gearing on the follower; stall about 30 kg.cm (about 2.9 N.m) at 12 V, 15 kg.cm (about 1.5 N.m) at 6 V | [SO-101 assembly](https://github.com/huggingface/lerobot/blob/main/docs/source/so101.mdx), [servo figures](https://flomotion.app/robots/therobotstudio/so-101-robot-arm-therobotstudio) |
| LD06 lidar | 360 deg, 12 m radius, 10 Hz typical (6-13 Hz), 4500-5000 Hz ranging; tolerance +/-20 mm between 0.5 and 2 m, +/-30 mm between 2 and 12 m | [LDROBOT LD06 datasheet](https://inno-maker.com/wp-content/uploads/2020/11/LDROBOT_LD06_Datasheet.pdf), [ArduPilot LD06](https://ardupilot.org/copter/docs/common-ld06.html) |
| Orbbec Astra Pro | Depth 0.6-8.0 m (optimal 0.6-5.0), 640x480 at 30 FPS; RGB up to 1280x960; depth FOV 58.4 x 45.5 deg, RGB FOV 63.1 x 49.4 deg; accuracy +/-1-3 mm at 1 m, about +/-12.7 mm at 2-3 m | [Astra Pro datasheet](https://cdn-reichelt.de/documents/datenblatt/C300/ORBBEC_ASTRA_PRO_FINAL.pdf), [Astra wiki](http://astra-wiki.readthedocs.io/en/latest/about.html) |

## What these values confirm, and what they do not

- The accepted base speed, 0.25 m/s, sits between the vendor's medium and fast
  modes, so no measured limit is being asked to exceed the base's design speed.
- The accepted payload, 0.2 kg, is below the arm's published ~0.5 kg. That does
  not make 0.2 kg a rating: the accepted number describes the load used in the
  stopping trials, and the holder mass and attachment are still unconfirmed.
- The lidar's +/-30 mm at range is inside the 60 mm measurement uncertainty the
  stopping record already carries.
- The Astra Pro's nearest valid depth is 0.6 m, which agrees with the 0.55 m
  observed on this robot and is why the depth cloud cannot serve as a near-field
  collision source (see DEFERRED.md).
- The servo's stall figures are design limits, not a safe continuous load; the
  arm trials under DEFERRED.md issue #7 remain the only evidence for arm motion.

## Checks that only measurement can settle

Stopping distance and latency per direction at the recorded payload and surface,
fault injection for every input the profile requires, the assembled arm's
collision matrix, the calibration pose, workspace intrusion stopping, and the
arm resting inside `stow_tolerance` on its contacts. See
[DEFERRED.md](../DEFERRED.md) and [safety](safety.md#physical-acceptance).