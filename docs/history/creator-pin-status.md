# UART Update - 2026-09-26

Current firmware is stage2-v6: `HostUart` UDB UART now routes RX=P12[6], TX=P12[7], 115200 8N1, 256-byte interrupt RX/TX buffers. All 25 effective physical pin assignments and electrical settings remain verified. Use `verify-creator-pins.ps1 -NamedInstances -HardwareUart`.

The two old software-only serial symbols are excluded with CY_REMOVE=true and named Legacy_DBG_RX/TX. The connected DBG_RX/TX symbols are active. The historical "all hardware connections disabled" statement below no longer applies to the two UART pins. See STAGE2_HARDWARE_UART.md and deliverables/stage2-workbench-v6-uart for current build evidence. No programming or hardware validation was performed.

The following section records the pin-only baseline before the UART migration.

# Creator Pins Status - 2026-09-26

Project: `G:\792\AS5045_enc\DRV8825.cydsn`.
Target: CY8C5888LTI-LP097 / CY8CKIT-059.

## Verified

- Saved schematic contains exactly 25 single digital Pins components, placed by function.
- No separate LED component: STEP1 shares P2[1] with the board LED.
- CSN and CLK are explicitly assigned P12[1] and P12[0].
- All 25 physical assignments and electrical settings match current `main.c`.
- All 25 functional instance names are saved; no default Pin_N names remain.
- Input and output HW connections are disabled on all 25 instances; no schematic hardware wiring is required.
- Full `cyprjmgr -rebuild` succeeded at 2026-09-26 16:53:19, exit code 0, with no build errors or warnings in `creator-pin-rebuild.log`.
- `verify-creator-pins.ps1 -NamedInstances` passed both physical/electrical and functional-name checks. An independent read-only audit confirmed the results.
- `main.c` was not edited. No programming or hardware test was performed.

## Final Pin Configuration

Creator rejects `DO` as a reserved instance name (`sdb.M0075`). Its component is named `AS5045_DO`, still assigned to `PIN_DO = P1[3]` in unchanged `main.c`.
The binary schematic was renamed and saved through Creator's installed native automation API, not edited as text. Reopening the saved document verified all names, unchanged electrical parameters, and all HW connection flags set to 0.

| Group | Instance Name | Physical Pin | Direction / Drive | Initial |
|---|---|---|---|---|
| AS5045 | CSN | P12[1] | Output / Strong | 1 |
| AS5045 | CLK | P12[0] | Output / Strong | 1 |
| AS5045 | AS5045_DO | P1[3] | Input / Digital Hi-Z | 0 |
| DRV1 | DIR1 | P2[0] | Output / Strong | 0 |
| DRV1 | STEP1 | P2[1] | Output / Strong | 0 |
| DRV1 | SLEEP1 | P2[2] | Output / Strong | 0 |
| DRV1 | RST1 | P2[3] | Output / Strong | 0 |
| DRV1 | M1_2 | P2[4] | Output / Strong | 0 |
| DRV1 | M1_1 | P2[5] | Output / Strong | 1 |
| DRV1 | M1_0 | P2[6] | Output / Strong | 1 |
| DRV1 | EN1 | P2[7] | Output / Strong | 1 |
| DRV2 | DIR2 | P12[4] | Output / Strong | 0 |
| DRV2 | STEP2 | P12[3] | Output / Strong | 1 |
| DRV2 | SLEEP2 | P12[2] | Output / Strong | 0 |
| DRV2 | RST2 | P1[2] | Output / Strong | 0 |
| DRV2 | M2_2 | P1[4] | Output / Strong | 0 |
| DRV2 | M2_1 | P1[5] | Output / Strong | 1 |
| DRV2 | M2_0 | P1[6] | Output / Strong | 1 |
| DRV2 | EN2 | P1[7] | Output / Strong | 1 |
| Limits / NPN | LIMIT1 | P0[3] | Input / Resistive Pull-Up | 1 |
| Limits / NPN | LIMIT2 | P0[4] | Input / Resistive Pull-Up | 1 |
| Limits / NPN | ARM2_NPN | P1[0] | Input / Resistive Pull-Up | 1 |
| Debug serial | DBG_TX | P12[7] | Output / Strong | 1 |
| Debug serial | DBG_RX | P12[6] | Input / Resistive Pull-Up | 1 |
| Fault | FAULT | P12[5] | Input / Resistive Pull-Up | 1 |

Keep digital input buffers enabled, interrupts disabled, and input synchronization disabled.
Power-on-reset port setting remains Don't Care; it is distinct from the initial drive state.

## Reproduce Verification

```powershell
& 'D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\cyprjmgr.exe' -wrk 'G:\792\AS5045_enc\DRV8825.cywrk' -rebuild
if ($LASTEXITCODE -ne 0) { throw 'Creator rebuild failed' }
& 'G:\792\AS5045_enc\verify-creator-pins.ps1' -NamedInstances
```

Use `-NamedInstances` for the final project. The default verifier is only for the historical Pin_N mapping before renaming.
Do not reapply `creator-pin-parameters.txt` after renaming: it addresses the old Pin_N instances.

## Changed Files and Backup

Manually changed schematic/resource files: `TopDesign\TopDesign.cysch`, `DRV8825.cydwr`.
Creator/build also updates project metadata and generated/build outputs.
Backup of the original schematic, DWR, and project file: `G:\792\AS5045_enc\creator-pin-backup-20260926-145135`.
Backup before final native renaming: `G:\792\AS5045_enc\creator-pin-backup-before-final-names-20260926`.
Verification inputs/output: `creator-pin-parameters.txt`, `verify-creator-pins.ps1`, `creator-pin-rebuild.log`.
Native rename helper and saved-document verification log: `creator-native-test-20260926\CreatorRename.cs`, `creator-native-test-20260926\rename-real.log`.
Final saved schematic screenshot: `creator-schematic-20260926.png`.

Schematic, explicit pin assignments, final naming, and complete rebuild verification are finished. Programming and physical hardware validation were not performed.
