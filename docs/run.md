# Run the 16-environment experiment

Use a Linux GPU workstation. The recorded setup used Ubuntu 22.04 x86-64, Isaac Sim 5.1, Isaac Lab v2.3.2, Flexiv RDK 1.8 and Flexiv Sim Plugin 1.2. The controller runs on the host CPU and Isaac runs physics on CUDA device 0.

## Prepare the machine

1. Install Isaac Sim and Isaac Lab in a Python 3.11 Conda environment named `isaaclab`. Install PyYAML, pybind11 and spdlog in this environment. The launcher builds its native Python extension using CMake, a C++ compiler and binutils. Install `python3-yaml` for the system Python used by the manifest generator.
2. Install Flexiv Elements Studio from your licensed distribution, select external physics, and create a Rizon 4s simulator profile. Install FUSE, gocryptfs, iproute2, procps and psmisc. The launcher requires a working FUSE device and an open-file hard limit of at least 65,536. The binary preparation checks the installed build before applying a port change.
3. Create a separate Python 3.12 environment for controller clients with `flexivrdk==1.8.0` and `spdlog`. The launcher defaults to `~/miniforge3/envs/flexiv-sim-plugin/bin/python`. Pass `--runtime-python /absolute/path/to/python` in terminal 2 to select another.
4. Obtain the workcell USD and its referenced assets. The recorded run used `final_env_base_and_table.usd`. The Rizon 4s robot USD is included here. Set `scene.env_usd` in the [experiment configuration](../configs/16-environments.yaml) to the workcell’s absolute path. In the same file set `fleet.host_ip` to an active local IPv4 address, `fleet.existing_profile_root` to the installation’s `user_data_ui/simDir`, and `source_profile` and `source_internal_serial` to the source profile.

By default the launcher looks in `~/miniforge3` and `~/IsaacLab`. Set `ISAAC_PYTHON`, `CONDA_EXE` or `ISAACLAB_ROOT` for other install locations. Set `ELEM_ROOT` in terminal 2 if Elements Studio is installed outside `~/flexiv_elements_studio/FlexivElementsStudio`. Run both commands from this repository’s root.

## Launch

In terminal 1, start Isaac. The launcher creates the manifest, builds the transport and prints the matching terminal 2 command.

```bash
export FLEXIV_FLEET_MANIFEST=/tmp/flexiv-16-env-run.json
./scripts/bridges/gpu_async/run_gpu_async_fleet_from_config.sh configs/16-environments.yaml
```

In terminal 2, start the controllers once the launcher prints the command.

```bash
./scripts/controllers/run_generated_robot_fleet.sh \
  --manifest /tmp/flexiv-16-env-run.json --physics isaac
```

The launcher prepares one private controller and simulation bridge per arm. After all 32 are ready, Isaac warms up for 2,000 steps and measures 10,000 more. New CSVs go under `benchmark_results/`. Stop the controller launcher with Ctrl+C so it cleans up its processes and mounts. For a smaller first run, reduce `scene.num_envs` in the YAML before starting either terminal.
