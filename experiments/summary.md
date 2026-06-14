# Exploration benchmark — greedy vs information-gain

Mean ± std over **valid** runs per strategy (valid = reached 90% or ran ≥ 250 s).

| strategy | valid | DNF (no 90%) | mean±std final_coverage | mean±std time_to_50_s | mean±std time_to_75_s | mean±std time_to_90_s | mean±std path_length_m | mean±std victims_detected | mean±std duration_s |
|---|---|---|---|---|---|---|---|---|---|
| greedy | 3/3 | 3 | 0.72±0.04 | 11.67±8.50 | 277.50±12.50 | — | 5.52±0.50 | 0.00±0.00 | 291.63±18.79 |
| info_gain | 3/3 | 2 | 0.85±0.08 | 23.33±2.36 | 235.00±10.00 | 300.00±0.00 | 13.23±2.23 | 1.33±0.47 | 282.03±23.38 |

## Run accounting

- **greedy**: 3/3 valid, 1 hit timeout, success_rate_90=0%
- **info_gain**: 3/3 valid, 2 hit timeout, success_rate_90=33%

_Hypothesis: information-gain reaches 90% coverage faster than greedy, at the cost of a longer path._
