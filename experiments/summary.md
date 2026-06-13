# Exploration benchmark — greedy vs information-gain

Mean over runs (per strategy).

| strategy | runs | final_coverage | time_to_50_s | time_to_75_s | time_to_90_s | path_length_m | victims_detected | duration_s |
|---|---|---|---|---|---|---|---|---|
| greedy | 1 | 0.67 | 15.00 | nan | nan | 4.82 | 0.00 | 315.00 |
| info_gain | 1 | 0.94 | 25.00 | 245.00 | 300.00 | 15.61 | 2.00 | 315.10 |

_Hypothesis: information-gain reaches 90% coverage faster than greedy, at the cost of a longer path._
