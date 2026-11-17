# Contributing

## Development loop

```bash
python -m pip install -e ".[dev]"
ruff check src tests
mypy
pytest --cov=autodrive_sim --cov-report=term-missing
```

CARLA is not required for core tests. Any CARLA-dependent feature should have a backend-independent unit test where possible, plus a clearly marked integration test for a live simulator.

## Commit standard

Use small, descriptive commits that represent one coherent capability or fix. Include tests and documentation with the change when they are part of the same behavior. Do not rewrite history to manufacture dates or activity.

