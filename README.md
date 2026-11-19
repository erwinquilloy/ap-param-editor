# AP Param Editor

Read and change ArduPilot parameters from an EdgeTX or Ethos radio, without a laptop.

- **Radios:** EdgeTX colour (any screen size, touch) and black-and-white (128x64, 212x64); FrSky Ethos (System > Tools).
- **Links:** S.Port / F.Port passthrough (any ArduPilot, MAVlite), ELRS / TBS (ArduPlane Stratos v2.2.1+, MAVlite over CRSF), mLRS cbridge (any ArduPilot, MAVLink). The telemetry source can be Auto or chosen by hand.
- **Tabs:** ArduPlane FLIGHT, LAUNCH, RETURN, FAILSAFE, ARMING (check-by-check skip editor), TECS, LANDING, PID, VTOL / Q PID (with Q_ENABLE 1), plus a CUSTOM tab from `custom.txt` for any parameter on any vehicle.
- **Safety:** one request at a time, saves verified by read-back and never repeated automatically, view-only while armed unless Air edits is on (PID / trim only).

## Tested on radios

| Radio | Confirmed |
| --- | --- |
| EdgeTX colour (RadioMaster TX16S, EdgeTX 2.12.5) | ELRS |
| EdgeTX black and white (RadioMaster TX12 MK2, EdgeTX 2.12.2) | ELRS |
| FrSky Ethos | ELRS |
| EdgeTX | mLRS |

S.Port and F.Port are confirmed too (on one of the radios above).

Not yet tested on a radio: mLRS on Ethos, the CUSTOM tab.

Install and use: see [INSTALL.txt](INSTALL.txt).

## Layout

| Path | What |
| --- | --- |
| `src/common/core.lua` | engine: the three links, request scheduler, edit model (shared by both platforms) |
| `src/common/list.lua` | the tabs and their rows |
| `src/common/ui_color.lua` | colour UI, any size, keys + touch (EdgeTX colour and Ethos) |
| `src/edgetx/APParam.lua`, `ui_bw.lua` | EdgeTX tool entry, black-and-white UI |
| `src/ethos/main.lua`, `icon.png` | Ethos system tool |
| `tools/build.py` | builds `dist/EdgeTX`, `dist/Ethos` and the zips |
| `tests/` | off-radio tests against copies of ArduPilot's MAVlite and a pymavlink ArduPilot |

```
python tools/build.py
python tests/test_core.py   # engine, every link, Lua 5.2 and 5.4
python tests/test_ui.py     # screens at radio sizes -> tests/out/*.png, both entry scripts
```

## License

GPL-3.0, see [LICENSE](LICENSE).
