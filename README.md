# Param Editor

Read and change flight controller settings from an EdgeTX or Ethos radio, without a laptop: **ArduPilot, INAV 9.1+ and Betaflight 4.5 / 2025.12 / 2026.6**. (Called "AP Param Editor" up to 0.1.1, when it was ArduPilot only.)

- **Radios:** EdgeTX colour (any screen size, touch) and black-and-white (128x64, 212x64); FrSky Ethos (System > Tools).
- **Flight controller:** found by itself. ArduPilot from its own telemetry; otherwise the tool asks over MSP whether it is INAV or Betaflight. The tab list shows which.
- **Links:**
  - ArduPilot: S.Port / F.Port passthrough (MAVlite), ELRS / TBS (ArduPlane Stratos v2.2.1+, MAVlite over CRSF), mLRS cbridge (MAVLink).
  - INAV and Betaflight: MSP over ELRS / TBS, S.Port / F.Port, or mLRS cbridge.
  - The telemetry source can be Auto or chosen by hand.
- **Not supported: ELRS in MAVLink mode** (ELRS Lua "Link Mode" = MAVLink), with any firmware: parameter requests never get through. Use Link Mode = Normal (CRSF), S.Port / F.Port, or mLRS with cbridge.
- **Tabs:**
  - ArduPilot: ArduPlane FLIGHT, LAUNCH, RETURN, FAILSAFE, ARMING (check-by-check skip editor), TECS, LANDING, PID, VTOL / Q PID (with Q_ENABLE 1), and a CUSTOM tab from `custom.txt` for any parameter on any vehicle.
  - INAV 9.1+: FLIGHT, LAUNCH, NAV THR, RTH, LANDING, MISSION, FW PID, MC PID, with INAV's own limits and option names, and a CUSTOM tab from `custom_inav.txt` for any setting by its CLI name.
  - Betaflight 4.5 / 2025.12 / 2026.6: PID, FF D (incl. feedforward smoothing, jitter, boost), TUNE (anti gravity, I-term relax cutoff, TPA, throttle boost, dynamic idle, motor output limit, vbat sag, thrust linear, angle limit), RATES (labels follow the rates type, throttle limit), FILTERS, for the active profile.
- **Safety:** one request at a time; saves verified by read-back and never repeated automatically; view-only while armed unless Air edits is on (OFF again each time the tool opens). INAV and Betaflight changes are stored (EEPROM) only when the flight controller says it is disarmed.

The INAV and Betaflight support is the same as [OmniDeck](https://github.com/erwinquilloy/OmniDeck)'s PARAMS page.

## Download

From the [releases page](https://github.com/erwinquilloy/param-editor/releases): `Param_Editor_EdgeTX_v0.2.0.zip` for EdgeTX (colour and black-and-white) or `Param_Editor_Ethos_v0.2.0.zip` for FrSky Ethos. Install steps are in [INSTALL.txt](INSTALL.txt), also inside each zip. Updating from AP Param Editor 0.1.x: delete the old `APPARAM` / `apparam` folder (and `APParam.lua` on EdgeTX); the new one is `PARAMED` / `parameditor`.

- **ArduPilot over ELRS / TBS (CRSF) needs ArduPlane Stratos v2.2.1 or later:** [releases](https://github.com/erwinquilloy/ardupilot/releases). Upstream ArduPilot: S.Port / F.Port and mLRS work as they are; parameters over CRSF wait on [ArduPilot#35160](https://github.com/ArduPilot/ardupilot/pull/35160).
- **INAV 9.0.x** answers the setting requests without the values over the radio link (fixed in 9.1): update to 9.1 or later.

## Tested on radios

| Radio | Confirmed (ArduPilot) |
| --- | --- |
| EdgeTX colour (RadioMaster TX16S, EdgeTX 2.12.5) | ELRS |
| EdgeTX black and white (RadioMaster TX12 MK2, EdgeTX 2.12.2) | ELRS |
| FrSky Ethos | ELRS |
| EdgeTX | mLRS |

S.Port and F.Port are confirmed too (on one of the radios above).

Not yet tested on a radio: INAV and Betaflight (tested in the simulator against the same fake flight controllers as OmniDeck, every link), mLRS on Ethos, the CUSTOM tabs.

## Layout

| Path | What |
| --- | --- |
| `src/common/pick.lua` | which engine: ArduPilot first, MSP when no ArduPilot is heard |
| `src/common/core.lua` | ArduPilot engine: the three links, request scheduler, edit model |
| `src/common/mspcore.lua` | INAV / Betaflight engine: MSP over CRSF, S.Port and the mLRS bridge |
| `src/common/list.lua`, `ilist.lua`, `bflist.lua` | the tabs and their rows (ArduPilot, INAV, Betaflight) |
| `src/common/ui_color.lua` | colour UI, any size, keys + touch (EdgeTX colour and Ethos) |
| `src/edgetx/ParamEd.lua`, `ui_bw.lua` | EdgeTX tool entry, black-and-white UI |
| `src/ethos/main.lua`, `icon.png` | Ethos system tool |
| `tools/build.py` | builds `dist/EdgeTX`, `dist/Ethos` and the zips |
| `tests/` | off-radio tests: ArduPilot's MAVlite, a pymavlink ArduPilot, fake INAV and Betaflight |

```
python tools/build.py
python tests/test_core.py   # ArduPilot engine, every link, Lua 5.2 and 5.4
python tests/test_msp.py    # INAV / Betaflight, every link, Lua 5.2 and 5.4
python tests/test_ui.py     # screens at radio sizes -> tests/out/*.png, both entry scripts
```

## License

GPL-3.0, see [LICENSE](LICENSE).
