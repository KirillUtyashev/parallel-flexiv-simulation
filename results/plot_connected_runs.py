"""Compare the recorded four- and sixteen-environment checkpoints."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
with (root / "connected-runs.csv").open() as source:
    rows = list(csv.DictReader(source))

fig, ax = plt.subplots(figsize=(7.2, 3.2), constrained_layout=True)
rates = [float(row["environment_steps_per_s"]) for row in rows]
labels = [
    f"{row['num_envs']} environments · {row['robot_count']} arms"
    for row in rows
]
bars = ax.barh(labels, rates, height=0.48, color=["#617581", "#19685d"])
ax.bar_label(bars, labels=[f" {rate:,.0f}" for rate in rates], padding=3)
ax.set_xlim(0, max(rates) * 1.2)
ax.invert_yaxis()
ax.set_xlabel("Environment-steps per second across all environments")
ax.set_title("Throughput with native Flexiv controllers")
ax.spines[["top", "right"]].set_visible(False)
fig.savefig(root / "connected-runs.png", dpi=170)
