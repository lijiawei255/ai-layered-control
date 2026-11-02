# python_bench — zero-MATLAB entry point (v1.1.0)

Self-contained Python mirror of the four-stage layered-control benchmark. Only
`numpy` and `matplotlib` are required (plus the standard library).

```powershell
pip install -r python_bench/requirements.txt
python python_bench/run_benchmark.py             # default: C core via ctypes (builds on first run)
python python_bench/run_benchmark.py --no-c      # force the pure-Python gatekeeper mirror
python python_bench/run_benchmark.py --selftest  # C core vs Python mirror, 500 random frames
```

What it does:

* 40 s industrial four-stage benchmark (S1 nominal, S2 +35 % drift, S3 edge
  latency + 20x noise + contact, S4 OOD 42 Hz sensor resonance + encoder spikes)
  at 1 kHz with three branches: fixed PID, a continuous ridge/RBF policy trained
  on nominal-unit data, and the layered 10 Hz classifier + 1 kHz gatekeeper
  bank.
* Prints per-stage RMSE, safety-violation ticks [ms], control effort TV(u) and
  the OOD veto latency, and writes `python_bench/benchmark_4stages.png`
  (2x2, 240 dpi).
* The deterministic core is the compiled
  `../c99_layered_core/layered_gatekeeper.c`; if no C compiler is found it falls
  back to a line-for-line Python mirror with identical gate decisions
  (`--selftest` proves gate/regime equality on 500 random frames, `max|du| <= 1e-5`).
* Build artefacts go to `python_bench/build/` (gitignored). The script is
  deterministic (fixed seeds) and runs in a few seconds.
