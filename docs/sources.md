# Sources

The controller provisioning, asynchronous transport, Isaac Lab bridge and experiment configuration were extracted from the [RVL Isaac workspace at 26007b2](https://github.com/rvl-lab-utoronto/isaac_sim_ws/tree/26007b29d74630f428cb3c347b19327d93365cdd). This sample adapts launch paths for a standalone checkout. The original two-controller diagnosis and joint-state experiment are documented in the [Flexiv Sim Plugin fork at 47f37f5](https://github.com/rvl-lab-utoronto/flexiv_sim_plugin/blob/47f37f5d639f7202d46a5c444616ba1e5a1ae136/doc/dual_robot_rdk_checkpoint.md).

Flexiv provides the native controller, simulator plugin and Rizon model. NVIDIA provides Isaac Sim and Isaac Lab. SDK and dependency notices are preserved under [vendor](../vendor/flexiv_sim_plugin/README.md).

## Measurements

The connected-run CSV contains selected columns from two July 27, 2026 runs in the workspace’s local benchmark archive. The timestamps and bridge versions identify the rows from `gpu_async_scaling_summary_v2.csv`. The per-arm CSV contains all 32 rows with the matching 16-environment timestamp from `gpu_async_robot_correctness.csv`. The columns record how many times each arm’s state and applied joint efforts were sampled. These are historical measurements on the recorded bridge revisions.

The separate physics benchmark contains two Rizon 4s arms per environment and uses an RTX 5090 for the GPU series. The CSV records physics steps per second. The plot multiplies by environment count to obtain total environment-steps per second. The clip illustrates the workcells in Isaac Sim.

Regenerate both figures with Python and Matplotlib from the repository root.

```bash
python3 results/plot_connected_runs.py
python3 results/plot_physics_throughput.py
```
