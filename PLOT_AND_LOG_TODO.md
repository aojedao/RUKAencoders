# Pending Changes Tracker

This file tracks pending improvements for `replay_encoders.py` and related plotting/logging flow.

## To Do

- [x] Add an option to **replay plots from already collected logs** (no new recording run required), so plots can be iterated/fixed quickly.
  - ✅ Implemented `--replay-only` flag in replay_encoders.py
  - Usage: `python replay_encoders.py --replay-only --label <log_name>`
- [ ] Verify the **angle direction/sign** when turning (sensor vs motor angle direction consistency).
- [ ] Adjust/improve the **plot scaling in the generated images**.
- [ ] Improve log filename saving so the **log name is not repeated** in saved output filenames.

## Notes

- Requested scope for now: create this tracker only; implementation will be done later.



The code for running the test: python replay_encoders.py --serial-port /dev/ttyACM0 --hand right --plot

sudo chmod 666 /dev/ttyUSB0



remmeber the location of the calibration for the left hand vs the right hand

maybe fix the calibration script