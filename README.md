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
- `G1_23dof_no_hands`
- `G1_29dof_no_hands`
- `G1_23dof_Revo2` (requires `Revo2_LeftHand` and `Revo2_RightHand` robot modules available)
- `G1_29dof_Revo2` (requires `Revo2_LeftHand` and `Revo2_RightHand` robot modules available)
