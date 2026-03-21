# Right-Hand Joint Span Reference

This reference is primarily for the right hand calibration data.

## Scope
- Hand: right
- Source: right structured calibration map
- Use case: theoretical-vs-measured span alignment for grouped joints

## Joint Mapping Table

| Joint Group | Sensor | Motor | Expected Signed Span (deg) | Expected Abs Span (deg) | Motor Max Open (ticks) | Motor Max Closed (ticks) | Sensor Min Recorded (deg) | Sensor Max Recorded (deg) | Confidence (high/med/low) | Notes |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| Index abduction | 0 | 8 |  | 23 | 1758 | 1542 | 80.86 | 87.28 |  | Sensor 0 has no direct closed sample under motor 8 entry |
| Index MCP | 3 | 9 |  | 100 | 1376 | 715 | 68.82 | 90.53 |  |  |
| Index PIP | 2 | 7 |  | 100 | 2943 | 1609 | 34.54 | 135.70 |  |  |
| Index DIP | 1 | 7 |  | 90 | 2943 | 1609 | 118.56 | 229.48 |  |  |
| Thumb CMC | 6 | 12 |  | 173 | 1177 | 574 | 40.17 | 63.63 |  |  |
| Thumb MCP | 5 | 14 |  | 100 | 2365 | 1752 | 302.08 | 313.77 |  |  |
| Thumb DIP | 4 | 13 |  | 90 | 3163 | 2053 | 98.26 | 212.78 |  |  |

## Global Convention

| Global Convention | Value |
|---|---|
| Closing direction sign rule | pending |
| Same expected span for both joints sharing motor 7? | yes |

## Reminder
- This is a right-hand reference baseline.
- If left-hand data is added, keep it in a separate table/file to avoid mixing sign and range assumptions.
