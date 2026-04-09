mc_g1
=======

This package provides a RobotModule implementation for the G1 robot.

Dependencies
------------

This package requires:
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [g1_description](https://github.com/isri-aist/g1_description)

Optional runtime dependency for composed variants:
- [mc_revo2](https://github.com/isri-aist/mc_revo2)

How to use
------------

Put the following in `mc_rtc.yaml`.
```yaml
MainRobot: G1
Timestep: 0.002
```

Available module names:
- `G1` (alias of `G1_23dof`)
- `G1_23dof`
- `G1_29dof`
- `G1_no_hands` (uses the `29dof` model with hand links removed)
- `G1_Revo2` (uses the `29dof` no-hands base and connects `Revo2_LeftHand` and `Revo2_RightHand`)
