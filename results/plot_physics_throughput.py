#!/usr/bin/env python3
"""Plot raw Isaac aggregate physics throughput for CPU and GPU PhysX."""

from pathlib import Path
import csv
import math

import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, FuncFormatter


BASE_DIR = Path(__file__).resolve().parent
DATA_PATH = BASE_DIR / "physics_scaling_data.csv"
OUTPUT_PATH = BASE_DIR / "physics_throughput_comparison.png"


def load_data(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    environments = [int(row["environments"]) for row in rows]

    def throughput(column: str):
        return [
            environments[index] * float(row[column])
            if row[column]
            else math.nan
            for index, row in enumerate(rows)
        ]

    return (
        environments,
        throughput("cpu_physics_hz"),
        throughput("gpu_physics_no_render_hz"),
        throughput("gpu_physics_render_hz"),
    )


def annotate_series(ax, environments, values, color, offsets):
    for x, y in zip(environments, values):
        if not math.isfinite(y):
            continue
        ax.annotate(
            f"{y:,.0f}",
            (x, y),
            xytext=(0, offsets.get(x, 9)),
            textcoords="offset points",
            ha="center",
            color=color,
            fontsize=9,
        )


def main() -> None:
    environments, cpu_throughput, gpu_no_render_throughput, gpu_render_throughput = (
        load_data(DATA_PATH)
    )

    plt.rcParams.update(
        {
            "axes.facecolor": "white",
            "figure.facecolor": "white",
            "font.size": 10,
        }
    )
    fig, ax = plt.subplots(figsize=(10.5, 6.4), constrained_layout=True)

    ax.plot(
        environments,
        gpu_no_render_throughput,
        color="#0072B2",
        marker="o",
        markersize=7,
        linewidth=2.4,
        label="GPU PhysX (render = False)",
    )
    ax.plot(
        environments,
        gpu_render_throughput,
        color="#009E73",
        marker="^",
        markersize=7,
        linewidth=2.4,
        label="GPU PhysX (render = True)",
    )
    ax.plot(
        environments,
        cpu_throughput,
        color="#D55E00",
        marker="s",
        markersize=6.5,
        linewidth=2.4,
        label="CPU PhysX (render = False)",
    )

    annotate_series(
        ax,
        environments,
        gpu_no_render_throughput,
        "#005A8D",
        {2048: 10},
    )
    annotate_series(
        ax,
        environments,
        gpu_render_throughput,
        "#007A59",
        {4: -16, 16: -16, 64: -16, 256: -16, 512: -16, 1024: -16, 2048: -16},
    )
    annotate_series(
        ax,
        environments,
        cpu_throughput,
        "#A94700",
        {4: 10, 16: 10, 64: 10, 256: -16, 512: 10},
    )

    ax.set_xscale("log", base=2)
    ax.set_yscale("log")
    ax.set_xlim(3.2, 2_600)
    ax.set_ylim(1_000, 1_000_000)
    ax.xaxis.set_major_locator(FixedLocator(environments))
    ax.xaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:,.0f}"))

    ax.set_title("Raw Isaac Physics Throughput", fontsize=16, pad=12)
    ax.set_xlabel("Number of environments", fontsize=12)
    ax.set_ylabel("Throughput (environment-steps/s)", fontsize=12)
    ax.grid(which="major", color="#D9D9D9", linewidth=0.8)
    ax.grid(which="minor", color="#EEEEEE", linewidth=0.5)
    ax.legend(loc="upper left", frameon=True, fontsize=10)

    fig.savefig(OUTPUT_PATH, dpi=220)
    print(OUTPUT_PATH)


if __name__ == "__main__":
    main()
