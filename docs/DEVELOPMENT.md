# Development

Use clear names, small modules and language-appropriate comments. Keep domain behavior
independent of GUI and middleware; add behavior through the owning component and
small explicit contracts. Preserve unrelated work and required third-party notices.

## Checks

```sh
ruff check src scripts
ruff format --check src scripts
PYTHONPATH=src python -m unittest discover -s src/testing/unit -v
python scripts/dev/test_component.py
docker compose -f compose.yaml config --quiet
```

The component command uses an isolated runtime image. POSIX process tests require
Linux or a Linux container. Test/lint caches belong in `outputs/cache/`.

Run the smallest useful checks first. Integration needs the selected real services;
full flight acceptance belongs at behavioral milestones. Missing dependencies are
reported as skips or failures, never passes. Retain machine-readable results under
`outputs/reports/` and record source revisions, settings and seeds.

CPU/headless is the portable default. GPU, native display and geographic assets are
explicit profile dependencies. A readiness response does not prove a completed flight.

Alternative aircraft, swarming and general 3D exploration remain separate future work.
Do not add them during maintenance or infer real-aircraft certification from simulation.
