# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""How a piece of page text is compared with the translation table.

zh.json is keyed by the English text as written in the source, and a page holds
that text as HTML. Comparing the plain text of the two lets one table serve the
C++ pages, whose text is translated after Doxygen runs, and the Python ones,
whose text is translated before."""
import html
import re


def canonical(markup):
    """Plain text of an HTML fragment: tags dropped, entities decoded, space collapsed."""
    text = re.sub(r"<[^>]+>", "", markup)
    text = html.unescape(text).replace("更多...", "").replace("More...", "")
    return " ".join(text.split())
