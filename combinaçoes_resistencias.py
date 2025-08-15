from itertools import combinations
import pandas as pd
from functools import lru_cache

#Available resistances
base_resistances = [10, 20, 47, 100, 220, 330, 470, 1000, 2000, 2200, 4700, 5100, 5600]

#Series pair generation
series_resistances = set()
for r1, r2 in combinations(base_resistances, 2):
    series_resistances.add(r1 + r2)

#Base + series combination with filter r <= 5800
expanded_resistances = sorted(set(base_resistances) | series_resistances)
expanded_resistances = [r for r in expanded_resistances if r <= 5800]

#Targets and tolerance
targets = [624, 1008, 1392, 1776, 2160, 2544]
tolerance = 5


@lru_cache(maxsize=None)
def parallel_cached(group):
    return 1.0 / sum(1.0 / r for r in group)

def generate_possible_equivalents(subset, max_elements=4):
    equivalent_values = {}
    subset = sorted(set(subset))  # eliminate duplicates 
    for i in range(1, min(len(subset), max_elements) + 1):
        for group in combinations(subset, i):
            ordered_group = tuple(sorted(group))
            r_eq = parallel_cached(ordered_group)
            r_eq_round = round(r_eq, 1)
            if r_eq_round not in equivalent_values:
                equivalent_values[r_eq_round] = ordered_group
    return equivalent_values

def evaluate_subset(subset):
    equivalent_values = generate_possible_equivalents(subset)
    total_error = 0
    results = []
    for target in targets:
        candidates = [(r_eq, group) for r_eq, group in equivalent_values.items() 
                      if abs(r_eq - target)/target * 100 <= tolerance]
        if not candidates:
            return None, float('inf'), []
        r_eq, group = min(candidates, key=lambda x: abs(x[0] - target))
        error = abs(r_eq - target) / target * 100
        total_error += error
        results.append((target, group, r_eq, error))
    average_error = total_error / len(targets)
    return subset, average_error, results

def value_range(subset):
    values = [parallel_cached((r,)) for r in subset]
    return min(values), max(values)

#Search for best resistance subset
n = 4
best_subset = None
best_average_error = float('inf')
best_results = []

print(f"Searching for ideal subset of {n} resistances...")

for i, subset in enumerate(combinations(expanded_resistances, n)):
    if i % 1000 == 0:
        print(f" Tested: {i} subsets...")

    min_r, max_r = value_range(subset)
    if max_r < min(targets) * 0.98 or min_r > max(targets) * 1.02:
        continue  # Subconjunto fora da gama útil

    subconj, average_error, results = evaluate_subset(subset)
    if average_error < best_average_error:
        best_subset = subconj
        best_average_error = average_error
        best_results = results
        print(f"New best subset with average error: {average_error:.4f}%")
        if best_average_error < tolerance:
            print(f"Early stopping: average error below {tolerance}%")
            break

# Mostrar resultados
if best_subset:
    print("\nBest subset found:")
    for r in best_subset:
        print(f" - {r} Ω")
    print(f"Total average error: {best_average_error:.4f}%%")

    table = []
    for target, group, r_eq, error in best_results:
        table.append((target, " ‖ ".join(f"{int(r)}Ω" for r in group), round(r_eq, 2), round(error, 2)))

    results_df = pd.DataFrame(table, columns=["Target (Ω)", "Combination", "R_eq (Ω)", "Error (%)"])
    print(results_df.to_string(index=False))

else:
    print(f"No subset of {n} resistances covers all targets.")
