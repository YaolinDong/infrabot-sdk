# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Every public name of the Python API carries a docstring.

A new binding without a docstring fails here, so the generated API reference
never shows an undocumented entry. Docstrings live in python/docs.hpp."""
import inspect

import infrabot.infrabot_native as native

# pybind11 builds these itself; they are not part of the SDK's API.
INHERITED = {"__init__", "__eq__", "__ne__", "__hash__", "__int__", "__index__", "__repr__", "__str__",
             "__getstate__", "__setstate__", "__setattr__", "__getattr__", "name", "value", "__members__",
             "__entries", "_pybind11_conduit_v1_"}


def text_after_signature(doc):
    """pybind11 prefixes the signature; the docstring proper follows it."""
    if not doc:
        return ""
    lines = [line for line in doc.splitlines() if line.strip()]
    return "\n".join(line for line in lines
                     if "(" not in line.split(" ", 1)[0] and not line.startswith(("Overloaded function", "1. ", "2. ")))


missing = []
if not (native.__doc__ or "").strip():
    missing.append("module")
public = sorted(name for name in dir(native) if not name.startswith("_"))
for name in public:
    obj = getattr(native, name)
    if isinstance(obj, int):
        continue  # constants; documented in the protocol header
    if inspect.isclass(obj):
        if not (obj.__doc__ or "").strip() or obj.__doc__.lstrip().startswith("Members:"):
            missing.append(name)
        for attr, member in vars(obj).items():
            if attr in INHERITED or (attr.startswith("__") and attr != "__init__"):
                continue
            if isinstance(member, property):
                if not (member.__doc__ or "").strip():
                    missing.append(f"{name}.{attr}")
            elif callable(member) or isinstance(member, staticmethod):
                fn = member.__func__ if isinstance(member, staticmethod) else member
                if not text_after_signature(fn.__doc__):
                    missing.append(f"{name}.{attr}()")
    elif callable(obj):
        if not text_after_signature(obj.__doc__):
            missing.append(f"{name}()")

assert not missing, "undocumented Python API: " + ", ".join(missing)
print(f"Python API: all {len(public)} public names have docstrings.")
