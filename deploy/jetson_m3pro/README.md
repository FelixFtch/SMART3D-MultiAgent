# Jetson M3Pro Recovery

This folder holds rebuild helpers for restoring the preserved M3Pro Jetson
stack from the private repository snapshot.

Main entry point:

- `restore_full_stack.sh`

Use it after cloning this private repo onto a target Jetson to restore:

- `~/mircoROS_agent/src`
- `~/yahboomcar_ws/src`
- `~/M3Pro_ws/src`
- `~/start_agent.sh`

The script restores the repository snapshot from:

- `field/jetson_m3pro_original_2026-04-12/`

and then reapplies the Mac-side mapper overlay from:

- `extras/m3pro_auto_mapper21_overlay/`

See also:

- `docs/M3PRO_JETSON_BASELINE_20260412.md`
- `docs/M3PRO_JETSON_PORTING_20260413.md`
