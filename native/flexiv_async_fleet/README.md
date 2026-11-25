# Native asynchronous fleet transport

This module owns the asynchronous CPU transport between the Isaac bridge and
Flexiv Sim Plugin `UserNode` endpoints. The physics-facing API is exposed as
the `_flexiv_async_fleet` Python extension.

Build it with the Isaac/Isaac Lab Python interpreter:

```bash
./scripts/controllers/build_async_fleet_module.sh \
  $HOME/miniforge3/envs/isaaclab/bin/python
```

The script writes the Python-version-specific module under
`build/async_fleet_pyXY/` and prints that directory on stdout. It also creates
run-private variants of the upstream static archive; the vendored source
archive is never modified.

The Flexiv SDK headers and static link dependencies are isolated under
`vendor/flexiv_sim_plugin/`. No external `flexiv_sim_plugin` source checkout is
used by this build.
