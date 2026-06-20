import json
import sys
from pathlib import Path


# It will fail if its more than 20% slower
THRESHOLD = 1.20


def load_results(path):
    with open(path, "r") as f:
        data = json.load(f)

    results = {}

    for bench in data["benchmarks"]:
        results[bench["name"]] = bench["stats"]["median"]

    return results


baseline_path = Path("baseline-benchmark.json")
current_path = Path("benchmark.json")

if not baseline_path.exists():
    print("No baseline benchmark found.")
    print("Passing comparison step.")
    sys.exit(0)

baseline = load_results(baseline_path)
current = load_results(current_path)

failed = False

print("\nBenchmark Comparison\n")

for name, baseline_time in baseline.items():
    if name not in current:
        print(f"WARNING: Missing benchmark: {name}")
        continue

    current_time = current[name]

    ratio = current_time / baseline_time

    print(f"{name:<30} baseline={baseline_time:.8f}s current={current_time:.8f}s ratio={ratio:.2%}")

    if ratio > THRESHOLD:
        print(f"FAIL: {name} exceeded {(THRESHOLD - 1) * 100:.0f}% slowdown threshold")
        failed = True

if failed:
    sys.exit(1)

print("\nAll benchmarks within threshold.")
