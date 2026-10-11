# A single DVS/event-camera change-of-contrast event.
# ts is seconds since epoch (not builtin_interfaces/Time) so PL-EVIO's many
# ported `e.ts.toSec()`-style call sites reduce to a plain `e.ts` field read.
uint16 x
uint16 y
float64 ts
bool polarity
