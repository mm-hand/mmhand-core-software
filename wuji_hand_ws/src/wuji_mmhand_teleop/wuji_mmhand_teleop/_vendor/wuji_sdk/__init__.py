"""Wuji SDK — Python package.

This package re-exports the public API from the native extension so existing
``import wuji_sdk`` code keeps working, and is also where the compiled retargeting
algorithm is bundled as ``wuji_sdk._retargeting``.
"""

from . import _wuji_sdk

# Public classes / functions registered by the native extension.
from ._wuji_sdk import *  # noqa: F401,F403

# `from ... import *` skips names starting with an underscore, but the native
# crash/atexit hooks call ``wuji_sdk._log_python_crash`` and
# ``wuji_sdk._cleanup_callback_subscriptions`` by name, and the test-suite /
# packaging read ``wuji_sdk.__version__``. Re-export them explicitly.
from ._wuji_sdk import (  # noqa: F401
    __version__,
    _log_python_crash,
    _cleanup_callback_subscriptions,
)

__doc__ = _wuji_sdk.__doc__ or __doc__
__all__ = list(
    getattr(_wuji_sdk, "__all__", None)
    or [name for name in dir(_wuji_sdk) if not name.startswith("_")]
)
