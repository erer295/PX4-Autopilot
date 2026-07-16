# Horizontal LADRC and suspended-load anti-swing testing

## Safety configuration

Keep the stock PX4 attitude and rate controllers for this research path:

```sh
param set MC_LADRC_EN 0
param set MC_RBF_EN 0
param set MC_RBF_LEARN_EN 0
```

The paper controller is selected with:

```sh
param set MC_PLADRC_EN 3
param set MC_HANG_AS_EN 1
param set MC_HANG_MODE 2
param set MC_HANG_LEN 0.60
```

`MC_PLADRC_EN=3` means second-order LADRC on NED X/Y and the original PX4
position/velocity PID on NED Z. `MC_HANG_MODE=1` selects the retained LegacyPD
baseline and `MC_HANG_MODE=2` selects energy damping.

For the first suspended-load hover, apply a conservative total horizontal
acceleration envelope. `MC_HANG_TOT_A` now limits the primary LADRC command
even when anti-swing is disabled. Once anti-swing is engaged, its bounded
damping command is reserved first and the primary command uses the remaining
budget:

```sh
param set MC_PLADRC_LIM_XY 0.8
param set MC_HANG_TOT_A 0.8
```

Do the height-only check with `MC_HANG_AS_EN=0`. A QGC position-mode takeoff
does not engage anti-swing while `MC_HANG_OFFB=1`; use the OFFBOARD sequence
below for energy anti-swing evaluation.

Start energy-mode sign validation conservatively:

```sh
param set MC_HANG_ZETA 0.10
param set MC_HANG_ACC_LIM 0.20
param set MC_HANG_ACC_SLW 0.50
param set MC_HANG_E_MIN 0.0
param set MC_HANG_E_FULL 0.02
param set MC_HANG_FRQ_EN 0
```

Do not increase `MC_HANG_ZETA` or the acceleration limit until a single-axis
Gazebo test confirms that the swing-energy envelope decreases. If the measured
energy grows, verify `MC_HANG_SIGN_X/Y` against the Gazebo joint convention.
The unit test proves the controller formula sign, but cannot prove a simulator
joint-axis mapping.

## Build and unit tests

From the PX4 repository root:

```sh
make px4_sitl_default
make tests TESTFILTER=PositionControl
```

For a fast local rerun after the test build has been generated:

```sh
ninja -C build/px4_sitl_test unit-PositionControl unit-SuspendedLoadAntiSwing
build/px4_sitl_test/unit-PositionControl
build/px4_sitl_test/unit-SuspendedLoadAntiSwing
```

## Gazebo SITL sequence

Start the existing suspended-load model:

```sh
make px4_sitl gz_zd680_hang
```

Use OFFBOARD flight because `MC_HANG_OFFB=1` is the safe default. The recorder
option `--position-controller ladrc` selects `MC_PLADRC_EN=3` (second-order
LADRC on X/Y plus stock PID on Z). It deliberately does not select all-axis
mode 2. Set the anti-swing and frequency parameters in the PX4 shell first,
then use `--position-controller ladrc --rate-controller pid`.

Run the following configurations with identical trajectories and disturbances:

1. `MC_PLADRC_EN=0`, `MC_HANG_AS_EN=0`: stock PID baseline.
2. `MC_PLADRC_EN=2`, `MC_HANG_AS_EN=0`: retained all-axis LADRC2 baseline.
3. `MC_PLADRC_EN=3`, `MC_HANG_AS_EN=0`: hybrid LADRC2-XY/PID-Z ablation.
4. `MC_PLADRC_EN=3`, `MC_HANG_AS_EN=1`, `MC_HANG_MODE=1`: LegacyPD baseline.
5. `MC_PLADRC_EN=3`, `MC_HANG_MODE=2`, `MC_HANG_FRQ_EN=0`: energy damping with fixed LADRC bandwidths.
6. Same as 5 with `MC_HANG_FRQ_EN=1`: complete frequency-scheduled method.

For configurations 5 and 6, first test one axis at small angle and low
acceleration. Then use step, sine (0.2--1.5 Hz), circle and figure-eight
trajectories. Repeat at least five runs for rope lengths 0.4, 0.6 and 0.8 m.
`MC_HANG_LEN` must match the SDF rope geometry for every run.

## Frequency scheduling

When `MC_HANG_FRQ_EN=1`, only X/Y bandwidths are scheduled:

```text
wn     = sqrt(g / L)
wc_eff = min(MC_PLADRC_WC_XY, MC_HANG_WC_R * wn)
wo_eff = max(MC_HANG_WO_MIN * wc_eff,
             min(MC_PLADRC_WO_XY, MC_HANG_WO_R * wn))
```

For L=0.6 m and the default base bandwidths, `MC_HANG_WC_R=0.45` and
`MC_HANG_WO_R=1.50` do not reduce either bandwidth. To verify that scheduling
is actually active, use the conservative initial values
`MC_HANG_WC_R=0.25`, `MC_HANG_WO_R=0.80`, and `MC_HANG_WO_MIN=2.5`; they give
approximately `wc_eff=1.01 rad/s` and `wo_eff=3.23 rad/s`. Scheduling is
calculated on parameter update; treat rope length as fixed during each flight.

## Logged debug arrays

`debug_array` named `hangas` (ID 681) contains:

```text
0 angle_body_x [rad]       1 angle_body_y [rad]
2 rate_body_x [rad/s]      3 rate_body_y [rad/s]
4 anti_requested_north     5 anti_requested_east [m/s^2]
6 active                   7 engaged
8 ramp_scale               9 safety_state (0 normal, 1 rearm, 2 abort)
10 wn [rad/s]              11 energy_per_mass
12 energy_gate             13 kd_eff [m/s]
14 anti_raw_north          15 anti_raw_east [m/s^2]
16 anti_applied_north      17 anti_applied_east [m/s^2]
18 mode
```

`debug_array` named `posladrc` (ID 683) contains TD states in 0--5, LADRC raw
acceleration in 6--8, disturbance compensation in 9--11, enable/TD/mode in
12--14, observer input X/Y in 15--16, controller raw X/Y in 17--18, final
command X/Y in 19--20, thrust-reconstructed applied acceleration X/Y in 21--22,
and effective `wc_xy`/`wo_xy` in 23--24.

`debug_array` named `hangcoord` (ID 684) is a shadow-only decomposition of the
horizontal control path. It does not alter the controller output:

```text
0/1   LADRC nominal north/east
2/3   raw disturbance compensation north/east
4/5   selected disturbance compensation north/east (currently equals raw)
6/7   anti-swing requested north/east
8/9   anti-swing applied north/east
10/11 final command north/east
12/13 thrust-reconstructed north/east
14    predicted nominal power
15    predicted raw-disturbance power
16    predicted selected-disturbance power
17    predicted requested anti-swing power
18    predicted applied anti-swing power
19    predicted final-command power
20    predicted thrust-reconstructed power
21    filtered swing-rate norm
22    diagnostic valid
23/24 selector mode/blend (both zero in Commit A+B)
25/26 passivity shadow norm/ACTIVE-control flag (ACTIVE remains zero)
27    total horizontal acceleration saturation flag
28/29 passivity SHADOW correction north/east
30    passivity candidate predicted power
31    filtered positive candidate power
32    predicted power after the SHADOW correction
33    passivity mode (0=OFF, 1=SHADOW)
34    passivity SHADOW gate active
35    passivity gate dwell elapsed
```

Power fields use `-L * dot(a_heading, swing_rate_heading)`. The NED
acceleration is rotated only by yaw into the heading-aligned horizontal frame,
matching the current small-tilt anti-swing model. These are command-level
predictions, not measurements of the true payload energy derivative. Invalid
or stale suspended-load measurements produce `valid=0` and NaN power fields.

The energy supervisor currently supports only `MC_HANG_PAS_MD=0` (OFF) and
`MC_HANG_PAS_MD=1` (SHADOW). SHADOW evaluates a gated, limited, slew-limited
minimum-power correction using the `base LADRC + anti-swing requested`
candidate. The correction is recorded in indices 28--35 but is never added to
the final acceleration command; index 26 therefore remains zero. ACTIVE is not
implemented. When swing management is requesting acceleration, it keeps
priority inside `MC_HANG_TOT_A`, and the base position controller uses the
remaining horizontal acceleration budget.

Check that `posladrc[15:16]` equals `[21:22]` each cycle. Compare
`hangas[4:5]` with `[16:17]` to quantify acceleration-budget clipping. During
energy-mode sign validation, verify
`-MC_HANG_LEN * dot(anti_raw_body, swing_rate_body) <= 0` after rotating the
logged NED acceleration back to body axes.

## Acceptance metrics

Report position RMSE, maximum position error, swing-angle peak/RMSE, settling
time, swing-energy integral, control-acceleration RMS and saturation ratio.
The complete method should improve at least two swing metrics relative to
hybrid LADRC2 without anti-swing, while position RMSE should preferably degrade
by less than 10%. Inspect `z3`/disturbance compensation against
`anti_applied`; persistent opposite-sign cancellation indicates an input or
coordinate inconsistency.
