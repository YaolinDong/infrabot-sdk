# API reference

The reference is built by [Doxygen](https://www.doxygen.nl) from the source, so it
always matches the code of a release. Each release has four sites:

```
build/api-doxygen/en/<version>/cpp/       the C++ API, English
build/api-doxygen/en/<version>/python/    the Python API, English
build/api-doxygen/zh/<version>/cpp/       the C++ API, Chinese
build/api-doxygen/zh/<version>/python/    the Python API, Chinese
build/api-doxygen/index.html              opens the English C++ site
```

Every page has a C++ | Python switch and an English | 简体中文 switch; both keep
the page you are on, and fall back to the site's Main Page where the other site
has no such page.

## What it reads

- **C++**: the comments of `include/infrabot/*.hpp`, with Doxygen's own Main
  Page, Classes, Files and Examples tabs, its search and its member pages.
- **Python**: `python/infrabot/infrabot_native.pyi` is the source of the API, and
  the docstrings live in `python/docs.hpp`. `pysource.py` writes that stub as a
  module Doxygen reads: the methods of `Robot` in the groups and order of
  `robot.hpp`, each enum value with its text from the C++ header, the constants
  with their values, and attribute types such as `list[float](7)` restored after
  Doxygen runs (it would split them around the name).
- **Programs**: the Examples tab lists the programs of `examples/` in full, with
  the "What it shows" text of `docs/examples.md`. The C++ sites list the `.cpp`
  programs, the Python sites the `.py` ones.
- **Chinese**: the Chinese C++ site is Doxygen's English output with each
  paragraph replaced from `zh.json` and Doxygen's own labels in Chinese. The
  Chinese Python site gets its text before Doxygen runs, so its links survive.
  The headers stay English, so the sites cannot disagree about the API.
- `mainpage.md` and `mainpage.zh.md` are the Main Page of the C++ sites;
  `mainpage.python.md` and `mainpage.python.zh.md` those of the Python sites.

## Build it locally

Install Doxygen (`sudo apt install doxygen`), then:

```bash
python3 -B docs/reference/doxygen_site.py --out build/api-doxygen
xdg-open build/api-doxygen/index.html
```

With CMake, the `api_doxygen` target does the same in the build directory:

```bash
cmake --build build --target api_doxygen     # writes build/api-doxygen/
```

## Documenting a change

- Every public C++ class, function, field and enumerator needs a comment;
  `WARN_IF_UNDOCUMENTED` is on, and CI fails on a Doxygen warning.
- Start a function comment with its timing class (`[Blocking, 2 s]`,
  `[Non-blocking]` or `[RT]`), then say what it does, what it needs and how it
  fails. Say what the call is *for*: when to use it instead of a similar call,
  what the caller must do before and after, and what happens if it fails.
- For a new Python binding, add its docstring to `python/docs.hpp` and pass it
  as the doc argument in `python/bindings.cpp`. `tests/test_python_docs.py`
  (ctest `python_docs`) fails while any public Python name lacks one.
- A new program goes in `examples/` and gets a row in the table of
  `docs/examples.md`.
- A new text in the reference needs its Chinese in `zh.json`, keyed by the
  English text. The build writes what it could not translate to
  `build/api-doxygen/missing.txt`.
- After changing the bindings, regenerate the stub against the built extension:

  ```bash
  python -m pip install pybind11-stubgen==2.5.5
  PYTHONPATH=build/python python -m pybind11_stubgen infrabot.infrabot_native -o /tmp/stubs
  { printf '# SPDX-License-Identifier: Apache-2.0\n# Copyright (c) 2026 Yaolin Dong\n'
    sed 's/[ \t]*$//' /tmp/stubs/infrabot/infrabot_native.pyi; } > python/infrabot/infrabot_native.pyi
  ```

## Publishing

`.github/workflows/api-reference.yml` builds the reference on every push and
pull request. For a release tag `vX.Y.Z` it checks that the tag matches
`project(VERSION)`. It then adds `en/X.Y.Z/` and `zh/X.Y.Z/` to the `gh-pages`
branch, keeping the earlier versions, and points the root `index.html` at the
newest release. GitHub Pages serves that branch. To enable it once in the
repository settings, choose **Pages → Deploy from a branch → `gh-pages` / root**.
