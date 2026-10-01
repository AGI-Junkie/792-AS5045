# stage2-v7-limits1 - the build that ran on the machine

- Firmware id in the hex and in every LINK line: `stage2-v7-limits1`
- firmware.hex sha256: `a73295d82fa88dac29a80878acbc9bc4ea673d4ba9450ff47d71123234e14385`
- Built 2026-09-29 17:00 by build-release.ps1 as `deliverables/stage2-workbench-v7-unified-limits`.
  The matching host (593 KB) is only in that folder; it is not archived here.
- Hardware evidence: host logs `stage1-20260929-170210` .. `stage1-20260929-221337` (6 logs),
  including the 15-minute 540-node run reviewed in `RUN_REVIEW_20260929.md`.
- q2 convention: legacy (motor-2 elbow-relative). The host self-test treats limits1 as legacy
  firmware that must not be accepted under the absolute-q2 convention.
- SOURCE WAS NOT SAVED. The working tree moved on (global1, then absolute-q2) before the first
  commit. `SOURCE_SHA256SUMS.txt` lists the hashes of the 28 files this build came from;
  commit 8006de3 is a later, different snapshot.
