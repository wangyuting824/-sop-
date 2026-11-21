__version__ = "0.1.0"

# Source checkouts expose the workflow directories at repository root.
# In installed wheels setuptools places them inside this package.
from pathlib import Path
_root = Path(__file__).resolve().parent.parent
if (_root / "pyproject.toml").is_file() and (_root / "adapters/loader.py").is_file():
    __path__.append(str(_root))
