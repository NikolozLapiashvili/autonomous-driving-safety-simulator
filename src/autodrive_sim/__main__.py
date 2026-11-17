"""Allow ``python -m autodrive_sim`` to invoke the CLI."""

from .cli import main


raise SystemExit(main())

