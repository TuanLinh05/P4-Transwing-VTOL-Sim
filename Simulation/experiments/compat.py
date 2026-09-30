"""Compatibility helpers for the Ubuntu 22.04 Python runtime."""

try:
    from enum import StrEnum
except ImportError:  # Python 3.10 and older
    from enum import Enum

    class StrEnum(str, Enum):
        """Backport the Python 3.11 string-enum behavior used by this package."""

        def __str__(self) -> str:
            return str(self.value)


__all__ = ["StrEnum"]
