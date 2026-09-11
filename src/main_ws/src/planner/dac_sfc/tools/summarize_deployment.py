#!/usr/bin/env python3
"""Summarize DAC-SFC deployment CSV logs without third-party dependencies."""

import argparse
import csv
import math
import statistics
from collections import Counter
from pathlib import Path


DEFAULT_LATEST_POINTER = Path(
    "/tmp/dac_sfc_deployment/latest_csv_path.txt"
)


NUMERIC_FIELDS = (
    "astar_ms",
    "shortcut_ms",
    "obstacle_extract_ms",
    "guide_ms",
    "csgn_ms",
    "corridor_ms",
    "optimizer_setup_ms",
    "optimizer_ms",
    "total_ms",
    "corridor_count",
    "total_faces",
    "geometry_evaluations_per_call",
    "trajectory_duration_s",
    "max_velocity_mps",
    "max_acceleration_mps2",
    "optimizer_attempts",
    "final_position_weight",
    "optimizer_piece_count",
    "final_dynamic_penalty_scale",
    "terminal_velocity_alignment_angle_deg",
)


def percentile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def truthy(value):
    return value.strip().lower() in {"1", "true", "yes"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv_files",
        nargs="*",
        type=Path,
        help=("session CSV files; when omitted, use the path recorded in "
              "/tmp/dac_sfc_deployment/latest_csv_path.txt"),
    )
    args = parser.parse_args()

    csv_files = args.csv_files
    if not csv_files:
        try:
            latest_path = DEFAULT_LATEST_POINTER.read_text(
                encoding="utf-8"
            ).strip()
        except OSError as error:
            parser.error(
                f"cannot read latest session pointer "
                f"{DEFAULT_LATEST_POINTER}: {error}"
            )
        if not latest_path:
            parser.error(
                f"latest session pointer is empty: "
                f"{DEFAULT_LATEST_POINTER}"
            )
        csv_files = [Path(latest_path)]

    rows = []
    for path in csv_files:
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                # A newly created deployment CSV can contribute its own header
                # when a run slice already prepended one.  Ignore such embedded
                # headers instead of counting them as failed planning calls.
                if (row.get("run_id") == "run_id" or
                        row.get("timestamp_s") == "timestamp_s"):
                    continue
                rows.append(row)

    successful = [row for row in rows if truthy(row["pipeline_success"])]
    activated = sum(truthy(row["trajectory_activated"]) for row in rows)
    fallbacks = sum(truthy(row["ego_fallback_used"]) for row in rows)
    failures = Counter(row["failure_stage"] or "unspecified" for row in rows
                       if not truthy(row["pipeline_success"]))

    print(f"runs: {len(rows)}")
    print(f"successful: {len(successful)}")
    success_rate = 100.0 * len(successful) / len(rows) if rows else math.nan
    print(f"success_rate_percent: {success_rate:.3f}")
    print(f"activated: {activated}")
    print(f"ego_fallbacks: {fallbacks}")
    if failures:
        print("failure_stages:")
        for stage, count in failures.most_common():
            print(f"  {stage}: {count}")

    start_clearance_failures = [
        row for row in rows
        if (not truthy(row["pipeline_success"]) and
            row.get("failure_stage") == "route_start_clearance")
    ]
    if start_clearance_failures:
        print("route_start_clearance_metrics:")
        for field in (
                "requested_start_clearance_m",
                "available_start_clearance_m"):
            values = []
            for row in start_clearance_failures:
                try:
                    value = float(row.get(field, ""))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values.append(value)
            if values:
                print(
                    f"  {field}: median={statistics.median(values):.6g}, "
                    f"min={min(values):.6g}, max={max(values):.6g}"
                )

    retried = []
    for row in rows:
        try:
            attempts = int(float(row.get("optimizer_attempts", "0") or "0"))
        except (TypeError, ValueError):
            attempts = 0
        if attempts > 1:
            retried.append(row)
    if retried:
        rescued = sum(truthy(row["pipeline_success"]) for row in retried)
        exhausted = sum(
            not truthy(row["pipeline_success"]) and
            row.get("failure_stage") == "corridor_violation"
            for row in retried
        )
        print("optimizer_continuation:")
        print(f"  retried_runs: {len(retried)}")
        print(f"  rescued_runs: {rescued}")
        print(f"  exhausted_runs: {exhausted}")

    print("successful_run_metrics:")
    for field in NUMERIC_FIELDS:
        values = []
        for row in successful:
            try:
                value = float(row[field])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
        if values:
            print(f"  {field}: median={statistics.median(values):.6g}, "
                  f"p95={percentile(values, 0.95):.6g}, max={max(values):.6g}")


if __name__ == "__main__":
    main()
