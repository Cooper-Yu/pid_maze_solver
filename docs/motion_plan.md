# Task5 motion interpretation

The supplied yaw is the destination heading, held during translation. Positive yaw is counterclockwise. The GIF supports the qualitative sequence; exact angles and distances below come from the supplied pose table, not image pixels.

| Leg | Turn before movement | Translation in destination body frame | Distance (m) | Held yaw (deg) |
|---|---:|---|---:|---:|
| P01 -> P02 | +0 deg | Forward | 0.351879 | 0 |
| P02 -> P03 | -45 deg | Forward | 0.208237 | -45 |
| P03 -> P04 | -45 deg | Forward | 1.163954 | -90 |
| P04 -> P05 | +90 deg | Forward | 0.487891 | 0 |
| P05 -> P06 | +90 deg | Forward | 0.528819 | 90 |
| P06 -> P07 | +0 deg | Right strafe | 0.368007 | 90 |
| P07 -> P08 | +0 deg | Forward | 0.524001 | 90 |
| P08 -> P09 | +0 deg | Right strafe | 0.614977 | 90 |
| P09 -> P10 | +0 deg | Forward | 0.796001 | 90 |
| P10 -> P11 | +90 deg | Forward | 0.470895 | 180 |
| P11 -> P12 | +0 deg | Left strafe | 0.255954 | 180 |
| P12 -> P13 | +0 deg | Forward | 0.528000 | 180 |
| P13 -> P14 | -45 deg | Forward | 0.447573 | 135 |
| P14 -> P15 | +45 deg | Forward | 0.563960 | 180 |

A front-facing wall measurement and travel-direction clearance are different for a strafe. A front wall constrains one position component; a side wall or another observation is needed for the other component. Laser range alone does not produce an absolute odom coordinate. Keep the initial route anchor, record pose and TF-transformed scans, and treat changes as measured candidates requiring another complete run.
