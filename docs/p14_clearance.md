# P14 corner clearance repair

The rejected waiting and speed trials stopped before P14 with a laser point near
body x=0.18, y=-0.15 m. The point lay close to the footprint front-right corner,
although front/right axis-window median ranges were about 0.33 m. Sector medians
alone did not represent corner clearance.

Using both failed scans and their recorded stopped odometry poses, transforming
the observed points to the old nominal P14 predicted a minimum footprint gap of
about -6 to -9 mm (overlap). The observed stopped gaps were about 14.4 mm, close
to the unchanged 15 mm guard. These are local scan-based estimates, not a map
survey or a claim that physical collision occurred; the contact monitor was zero.

Shorten P13_P14 forward motion from 0.448 to 0.378 m. At the 135-degree heading,
this moves P14 +49.497 mm in route x and -49.497 mm in route y. The following
180-degree-heading step becomes forward 0.6134974746830583 m, left
-0.0494974746830583 m, preserving the final nominal P15 and all headings.
The final step therefore has a small simultaneous forward/right component,
consistent with the retained holonomic controller. No strict single-axis
movement rule was introduced.

The static and next-45-degree-turn scan predictions select this larger margin;
the full path must still be verified with live feedback and contact observation.
Cruise speed remains 0.12 m/s, stopped hold remains 0.4 ROS seconds, and all
hard guards remain unchanged. No rejected timing/speed feature is re-enabled.
## Local verification — 2026-10-09

| Run | Cruise parameter | Wall elapsed | Reached | Exit | Wall contact points | P14 hold logs |
| --- | --- | --- | --- | --- | --- | --- |
| p14_default | 0.12 m/s | 107.404 s | 14 | 0 | 0 | 0 |
| p14_slow | 0.11 m/s override | 112.166 s | 14 | 0 | 0 | 0 |

Both runs had two transient P08 obstacle-hold log entries. The default run's
maximum stopped target errors were 0.013118 m / 0.001021 rad; the slower run's
were 0.013336 m / 0.000769 rad. The stopped P14 scans estimated 57.7 and 76.8 mm
minimum footprint clearance, respectively. Fixed-scan predictions of the next
45-degree turn were 57.4 and 76.8 mm. These predictions are not continuous
collision guarantees; the contact stream independently recorded zero wall
points and nonzero floor contacts in both complete runs.

The tests vary motion timing, but do not exhaust spawn, sensor or cloud changes.
The final nominal P15 is preserved geometrically; its actual wall-adjusted target
can still differ through the existing side-centering behavior.

Ten GoogleTests passed, including P14 shift, endpoint preservation and unchanged
headings. Formatting, clang-tidy, Doxygen (zero warnings) and YAML checks passed.
No cloud acceptance or Task5 tag is claimed. Existing runtime guard fixtures
were verified at the preceding slice; this repair changes route data only.

[Default summary](evidence/optimization/p14_default/summary.json) ·
[Default scan](evidence/optimization/p14_default/p14_scan.json) ·
[Slower summary](evidence/optimization/p14_slow/summary.json) ·
[Slower scan](evidence/optimization/p14_slow/p14_scan.json).
Exact commands, configuration snapshots, logs and source patches are alongside
those summaries. The 0.11 m/s value is a test override, not the new default.
