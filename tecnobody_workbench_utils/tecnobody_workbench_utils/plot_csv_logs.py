#!/usr/bin/env python3
"""Plot time + 3D-vector CSV logs and highlight the movement portion.

Expected CSV layout:
- first column: time
- next three columns: vector components to plot against time
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def _get_required_columns_by_index(df: pd.DataFrame) -> dict[str, str]:
	"""Use first 4 columns as time + generic 3D vector components."""
	if len(df.columns) < 4:
		raise ValueError(
			"CSV must contain at least 4 columns: "
			"[time, vector_1, vector_2, vector_3]."
		)

	return {
		"time": df.columns[0],
		"x": df.columns[1],
		"y": df.columns[2],
		"z": df.columns[3],
	}


def _to_float_series(series: pd.Series) -> pd.Series:
	"""Convert values like 1e-3 (or strings) to float."""
	cleaned = series.astype(str).str.strip().str.replace(",", ".", regex=False)
	return pd.to_numeric(cleaned, errors="coerce")


def load_and_prepare(csv_path: Path) -> tuple[pd.DataFrame, dict[str, str]]:
	"""Load CSV and ensure first 4 columns are numeric floats."""
	df = pd.read_csv(csv_path)
	columns = _get_required_columns_by_index(df)

	for key in ("time", "x", "y", "z"):
		col = columns[key]
		df[col] = _to_float_series(df[col])

	# Drop rows where we cannot parse time or all vector components.
	pos_cols = [columns["x"], columns["y"], columns["z"]]
	df = df.dropna(subset=[columns["time"]] + pos_cols, how="any").reset_index(drop=True)

	if df.empty:
		raise ValueError("No valid numeric rows found after parsing CSV columns.")

	return df, columns


def detect_movement_window(df: pd.DataFrame, columns: dict[str, str], threshold: float) -> tuple[int, int] | None:
	"""Find [start, end] indices where at least one axis is above threshold."""
	x = df[columns["x"]].to_numpy(dtype=float)
	y = df[columns["y"]].to_numpy(dtype=float)
	z = df[columns["z"]].to_numpy(dtype=float)

	moving_mask = (np.abs(x) > threshold) | (np.abs(y) > threshold) | (np.abs(z) > threshold)

	if not np.any(moving_mask):
		return None

	start_idx = int(np.argmax(moving_mask))
	end_idx = int(len(moving_mask) - 1 - np.argmax(moving_mask[::-1]))
	return start_idx, end_idx


def plot_signals(
	df: pd.DataFrame,
	columns: dict[str, str],
	threshold: float,
	movement_window: tuple[int, int] | None,
	show_full_signal: bool,
) -> None:
	"""Plot vector components over time, optionally cropped to movement window."""
	time_col = columns["time"]
	x_col = columns["x"]
	y_col = columns["y"]
	z_col = columns["z"]

	if movement_window and not show_full_signal:
		start_idx, end_idx = movement_window
		to_plot = df.iloc[start_idx : end_idx + 1]
		title_suffix = f"movement only (threshold={threshold:g})"
	else:
		to_plot = df
		title_suffix = f"full signal (threshold={threshold:g})"

	fig, ax = plt.subplots(figsize=(11, 6))
	ax.plot(to_plot[time_col], to_plot[x_col], label=x_col, linewidth=1.6)
	ax.plot(to_plot[time_col], to_plot[y_col], label=y_col, linewidth=1.6)
	ax.plot(to_plot[time_col], to_plot[z_col], label=z_col, linewidth=1.6)

	if movement_window and show_full_signal:
		start_idx, end_idx = movement_window
		start_t = df.iloc[start_idx][time_col]
		end_t = df.iloc[end_idx][time_col]
		ax.axvspan(start_t, end_t, alpha=0.15, label="Detected movement window")

	ax.axhline(y=threshold, linestyle="--", linewidth=1.0, alpha=0.5, label="+Threshold")
	ax.axhline(y=-threshold, linestyle="--", linewidth=1.0, alpha=0.5, label="-Threshold")

	ax.set_title(f"Vector signals - {title_suffix}")
	ax.set_xlabel("Time")
	ax.set_ylabel("Value")
	ax.grid(True, alpha=0.3)
	ax.legend()
	fig.tight_layout()
	plt.show()


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Load CSV logs, detect non-zero movement, and plot 3D vector signals."
	)
	parser.add_argument("csv", type=Path, help="Path to input CSV file")
	parser.add_argument(
		"--threshold",
		type=float,
		default=1e-6,
		help="Movement threshold (absolute value). Default: 1e-6",
	)
	parser.add_argument(
		"--show-full-signal",
		action="store_true",
		help="Plot full signal and highlight movement window instead of cropping.",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()

	if not args.csv.exists():
		raise FileNotFoundError(f"CSV file not found: {args.csv}")

	df, columns = load_and_prepare(args.csv)
	movement_window = detect_movement_window(df, columns, args.threshold)

	if movement_window is None:
		print(
			"No movement detected above threshold. "
			"Plotting full signal. Consider lowering --threshold."
		)
	else:
		start_idx, end_idx = movement_window
		time_col = columns["time"]
		print(
			"Detected movement window: "
			f"rows [{start_idx}, {end_idx}] | "
			f"time [{df.iloc[start_idx][time_col]}, {df.iloc[end_idx][time_col]}]"
		)

	plot_signals(df, columns, args.threshold, movement_window, args.show_full_signal)


if __name__ == "__main__":
	main()
