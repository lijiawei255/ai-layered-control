# python_bench — zero-MATLAB entry point (v1.1.0)

## [REUSABLE CORE] vs [BENCHMARK HARNESS]

| Role | Files | Purpose |
| :--- | :--- | :--- |
| **[REUSABLE CORE]** | `../c99_layered_core/layered_gatekeeper.c/.h` + `minimal_template.py` | Copy into the target project. The template is a 10 Hz + 1 kHz copy-paste skeleton; it auto-loads the C99 core via `ctypes` (or builds it once) and falls back to a minimal Python gate when no compiler exists. |
| **[BENCHMARK HARNESS]** | `run_benchmark.py` | Reproduces the 40 s paper/repo figures. **Do not copy the synthetic plant into production.** |

## Quick start

```powershell
pip install -r python_bench/requirements.txt
python python_bench/minimal_template.py      # 1 s reusable skeleton; prints [OK] ... 0 safety violations
python python_bench/run_benchmark.py         # 40 s benchmark: C core via ctypes (builds on first run)
python python_bench/run_benchmark.py --no-c  # force the pure-Python gatekeeper mirror
python python_bench/run_benchmark.py --selftest  # C core vs Python mirror, 500 random frames
```

## Minimal template: 3 slots

`minimal_template.py` is 98 lines and exposes only three replacements:

1. `[USER-PLUGIN-SLOT 1]` — `read_plant_and_reference(t)`: real sensor `(q, dq)` and reference `(q_ref, dq_ref, ddq_ref)`;
2. `[USER-PLUGIN-SLOT 2]` — `compute_nominal_residual(q, dq, u_prev)`: the target domain's minimal nominal-physics residual probe;
3. `[USER-PLUGIN-SLOT 3]` — `slow_loop_10hz_step(...)`: 4-D dimensionless macro-features, margin × proximity confidence, S3 physical-residual mode guard, coarse `ai_regime ∈ {1,2,3,4}`.

The main loop runs 1 kHz: every 100 ticks it refreshes the 10 Hz slow-loop decision; every tick it calls `Layered_Step` (or the fallback) and writes the safe command `u`.

## 40 s benchmark

The benchmark runs the industrial four-stage profile (S1 nominal, S2 +35 % drift, S3 edge latency + 20x noise + contact, S4 OOD 42 Hz sensor resonance + encoder spikes) at 1 kHz with three branches: fixed PID, a continuous ridge/RBF policy trained on nominal-unit data, and the layered 10 Hz classifier + 1 kHz gatekeeper bank.

It prints per-stage RMSE, safety-violation ticks [ms], control effort TV(u) and the OOD veto latency, and writes `python_bench/benchmark_4stages.png` (2x2, 240 dpi).

The deterministic core is the compiled `../c99_layered_core/layered_gatekeeper.c`; if no C compiler is found it falls back to a line-for-line Python mirror with identical gate decisions (`--selftest` proves gate/regime equality on 500 random frames, `max|du| <= 1e-5`). Build artefacts go to `python_bench/build/` and the output PNG is gitignored; both are regenerated.
