# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Builds the API reference with Doxygen itself: C++ and Python, one site per language.

    python3 docs/reference/doxygen_site.py --out build/api-doxygen

writes, for each language (en, zh):

    build/api-doxygen/<lang>/<version>/cpp/      the C++ API, from include/infrabot
    build/api-doxygen/<lang>/<version>/python/   the Python API, from the type stub
    build/api-doxygen/<lang>/<version>/index.html, which opens cpp/
    build/api-doxygen/index.html, which opens en/<version>/cpp/

Each is Doxygen's own HTML, with its Main Page / Classes / Files / Examples tabs,
its search and its member pages. A switch in the header of every page goes to
the other API and to the same page in the other language. The Chinese sites have
Doxygen's own labels in Chinese (OUTPUT_LANGUAGE) and the prose from zh.json.

The C++ prose is translated after the fact, paragraph by paragraph: a <p> whose
plain text matches an English entry of zh.json is replaced by its Chinese text.
An inline link inside such a paragraph becomes plain text in the Chinese site.

The Python API is the type stub written as a module Doxygen reads, by
pysource.py. For the Chinese site its docstrings are taken from zh.json before
Doxygen runs, so a translated paragraph keeps its links. The headers stay
English in both sites, so the two languages cannot disagree about the API.
"""
import argparse
import html
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pysource
from text import canonical

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = Path(__file__).resolve().parent
ZH_FILE = REFERENCE / "zh.json"
APIS = ("cpp", "python")
LANGS = ("en", "zh")
MAIN_PAGES = {("cpp", "en"): "mainpage.md", ("cpp", "zh"): "mainpage.zh.md",
              ("python", "en"): "mainpage.python.md", ("python", "zh"): "mainpage.python.zh.md"}
BRIEFS = {("cpp", "en"): "C++ API reference", ("cpp", "zh"): "C++ 接口参考",
          ("python", "en"): "Python API reference", ("python", "zh"): "Python 接口参考"}

CONFIG = """\
PROJECT_NAME           = "InfraBot SDK"
PROJECT_NUMBER         = {version}
PROJECT_BRIEF          = "{brief}"
OUTPUT_DIRECTORY       = {out}
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
GENERATE_XML           = NO
GENERATE_TREEVIEW      = YES
FULL_SIDEBAR           = YES
HTML_OUTPUT            = .
TIMESTAMP              = NO
HTML_COLORSTYLE        = LIGHT
HTML_EXTRA_STYLESHEET  = {style}
OUTPUT_LANGUAGE        = {language}
INPUT                  = {input}
STRIP_FROM_PATH        = {strip}
STRIP_FROM_INC_PATH    = {strip_include}
EXAMPLE_PATH           = {examples}
EXAMPLE_RECURSIVE      = NO
USE_MDFILE_AS_MAINPAGE = {mainpage}
EXTRACT_PRIVATE        = NO
EXTRACT_STATIC         = YES
JAVADOC_AUTOBRIEF      = YES
QT_AUTOBRIEF           = NO
HAVE_DOT               = NO
QUIET                  = YES
WARN_IF_UNDOCUMENTED   = YES
WARN_AS_ERROR          = NO
INLINE_SOURCES         = NO
REFERENCED_BY_RELATION = NO
REFERENCES_RELATION    = NO
"""

# What differs between the two APIs.
API_CONFIG = {
    "cpp": """\
FILE_PATTERNS          = *.hpp *.h *.md *.dox
EXAMPLE_PATTERNS       = *.cpp *.hpp
EXTRACT_ALL            = NO
SOURCE_BROWSER         = YES
""",
    # The module is written for Doxygen (pysource.py): ## comments, no docstrings. Its
    # source is not the SDK's, so the site does not show it. Every enum value is listed,
    # including those without a text of their own.
    "python": """\
FILE_PATTERNS          = *.py *.md *.dox
EXAMPLE_PATTERNS       = *.py
PYTHON_DOCSTRING       = NO
OPTIMIZE_OUTPUT_JAVA   = YES
EXTRACT_ALL            = YES
SUBGROUPING            = NO
SOURCE_BROWSER         = NO
""",
}

STYLE = """\
/* The API and language switches, in the header of every page. */
.ib-switch { position: fixed; top: 14px; right: 16px; z-index: 101; display: flex; gap: 10px;
             font: 13px/1 system-ui, sans-serif; }
.ib-switch .seg { display: inline-flex; border: 1px solid #1a5fb4; border-radius: 5px;
                  overflow: hidden; background: #fff; }
.ib-switch .seg > * { padding: 5px 11px; color: #1a5fb4; text-decoration: none; }
.ib-switch .seg > * + * { border-left: 1px solid #1a5fb4; }
.ib-switch .seg a:hover { background: #e8f0fb; }
.ib-switch .seg .on { background: #1a5fb4; color: #fff; font-weight: 600; }
"""

# Doxygen's note on its Chinese labels, printed on every Chinese run.
CHINESE_NOTE = re.compile(r"warning: The selected output language \"chinese\" has not been updated\s+"
                          r"since release [\d.]+\.\s+As a result some sentences may appear in English\.\n*")


def version_text():
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    return re.search(r"project\(infrabot_sdk VERSION (\d+\.\d+\.\d+)", text).group(1)


SHOWS = {  # docs/examples.md, "What it shows", per program number
    line.split("|")[1].strip(): line.split("|")[3].strip()
    for line in (ROOT / "docs" / "examples.md").read_text(encoding="utf-8").splitlines()
    if line.startswith("| ") and line.split("|")[1].strip().isdigit()}

EXAMPLES = {
    "cpp": ("cpp", "*.cpp", "example.hpp"),
    "python": ("python", "*.py", "example_common.py"),
}

EXAMPLE_INTRO = {
    ("cpp", "en"): "The programs of `examples/cpp`. Each uses the helper `example.hpp`, which connects, "
                   "applies the motor profile, enables the arm and stops it at the end.",
    ("cpp", "zh"): "`examples/cpp` 中的程序。每个程序都使用辅助文件 `example.hpp`，"
                   "它负责连接、应用电机模式、使能机械臂并在结束时停止。",
    ("python", "en"): "The programs of `examples/python`. Each uses the helper `example_common.py`, which "
                      "connects, applies the motor profile, enables the arm and stops it at the end.",
    ("python", "zh"): "`examples/python` 中的程序。每个程序都使用辅助文件 `example_common.py`，"
                      "它负责连接、应用电机模式、使能机械臂并在结束时停止。",
}

HELPER_TEXT = {
    ("cpp", "en"): "The connection, enable and stop sequence every C++ program uses.",
    ("cpp", "zh"): "每个 C++ 程序使用的连接、使能和停止流程。",
    ("python", "en"): "The connection, enable and stop sequence every Python program uses.",
    ("python", "zh"): "每个 Python 程序使用的连接、使能和停止流程。",
}


def examples_page(api, lang, table):
    """The .dox file that makes Doxygen list the programs of one API and their helper."""
    folder, pattern, helper = EXAMPLES[api]
    lines = ["/** \\page examples " + ("示例" if lang == "zh" else "Examples"), "",
             EXAMPLE_INTRO[(api, lang)], ""]
    for program in sorted((ROOT / "examples" / folder).glob("[0-9][0-9]_" + pattern)):
        shown = SHOWS.get(program.name.split("_")[0], program.stem)
        if lang == "zh":
            shown = table.get(canonical(shown), shown)
        lines += [f"\\example {program.name}", f"  {shown}", ""]
    lines += [f"\\example {helper}", f"  {HELPER_TEXT[(api, lang)]}", "", "*/"]
    return "\n".join(lines) + "\n"


def translations():
    """Plain English text -> Chinese HTML, from zh.json."""
    return {canonical(k): v for k, v in json.loads(ZH_FILE.read_text(encoding="utf-8")).items()}


def translate_page(page, table, missing):
    """Replace every <p> whose plain text has a translation; count the rest."""
    def one(match):
        inner = match.group(1)
        chinese = table.get(canonical(inner))
        if chinese is None:
            text = canonical(inner)
            if len(text.split()) >= 4 and re.search(r"[A-Za-z]{3}", text) and not re.search(r"[一-鿿]", text):
                missing[text] = ""
            return match.group(0)
        return "<p>" + chinese + "</p>"
    page = re.sub(r"<p>(.*?)</p>", one, page, flags=re.S)

    def cell(match):
        inner = match.group(2)
        if "<p>" in inner:
            return match.group(0)  # already handled above
        chinese = table.get(canonical(inner))
        if chinese is None:
            return match.group(0)
        return match.group(0).replace(inner, chinese, 1)
    # The brief of a member, in its summary row.
    page = re.sub(r'(<td class="(?:mdescRight|memdoc)")[^>]*>(.*?)</td>', cell, page, flags=re.S)

    def heading(match):
        return match.group(1) + table.get(canonical(match.group(2)), match.group(2)) + match.group(3)
    # The name of a member group: a heading of its own, or a row inside a section.
    return re.sub(r'(<h2 class="groupheader">|<div class="groupHeader">)([^<]+)(</h2>|</div>)', heading, page)


def one_description(text):
    """An example page shows its description twice: Doxygen writes it as the brief and as the text.

    Both copies are the same paragraph; the second is dropped. Doxygen closes the
    brief with a period the text may not have, so the periods are not compared.
    """
    repeat = re.compile(r"(<p>(?:(?!</p>).)*</p>)\s*<p>((?:(?!</p>).)*)</p>")
    def pair(match):
        first, second = canonical(match.group(1)), canonical(match.group(2))
        if first.rstrip(".") != second.rstrip("."):
            return match.group(0)
        return match.group(1)
    while True:
        once = repeat.sub(pair, text)
        if once == text:
            return text
        text = once


def doc_key(text):
    """The zh.json key of a docstring: its text once written as HTML, `code` as <code>."""
    return canonical(re.sub(r"`([^`]+)`", r"<code>\1</code>", html.escape(text, quote=False)))


def version_header(work, version):
    """include/infrabot/version.hpp as CMake writes it from cmake/version.hpp.in."""
    major, minor, patch = version.split(".")
    header = work / "include" / "infrabot" / "version.hpp"
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text((ROOT / "cmake" / "version.hpp.in").read_text(encoding="utf-8")
                      .replace("@PROJECT_VERSION_MAJOR@", major).replace("@PROJECT_VERSION_MINOR@", minor)
                      .replace("@PROJECT_VERSION_PATCH@", patch).replace("@PROJECT_VERSION@", version),
                      encoding="utf-8")
    return header


def doxygen(doxyfile, work):
    """Runs Doxygen; its warnings pass through, except its note on the Chinese labels."""
    run = subprocess.run(["doxygen", str(doxyfile)], cwd=work, capture_output=True, text=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(CHINESE_NOTE.sub("", run.stderr))
    if run.returncode != 0:
        raise SystemExit(f"doxygen failed on {doxyfile.name}")


def build(api, lang, version, out, work, table, missing):
    """Runs Doxygen for one API in one language and returns the directory it wrote.

    For Python, also returns the placeholders of the attribute types (pysource.py).
    """
    target = out / lang / version / api
    target.mkdir(parents=True)
    (work / "examples.dox").write_text(examples_page(api, lang, table), encoding="utf-8")
    mainpage = work / MAIN_PAGES[(api, lang)]
    mainpage.write_text((REFERENCE / MAIN_PAGES[(api, lang)]).read_text(encoding="utf-8"), encoding="utf-8")
    style = work / "style.css"
    style.write_text(STYLE, encoding="utf-8")
    tokens = {}
    if api == "cpp":
        # Copied next to the generated version.hpp, so the Files tab shows one include/infrabot.
        inputs = [version_header(work, version)]
        for name in ("robot.hpp", "diagnostics.hpp", "firmware_update.hpp"):
            inputs.insert(-1, Path(shutil.copy(ROOT / "include" / "infrabot" / name, inputs[-1].parent)))
        strip, strip_include = str(work), str(work / "include")
    else:
        def translate(text):
            chinese = table.get(doc_key(text))
            if chinese is None:
                missing[doc_key(text)] = ""
            return chinese
        source, tokens = pysource.module(translate=translate if lang == "zh" else None)
        module = work / "infrabot.py"
        module.write_text(source, encoding="utf-8")
        inputs = [module]
        strip = strip_include = str(work)
    config = CONFIG.format(
        version=version, brief=BRIEFS[(api, lang)], out=target, style=style,
        language="English" if lang == "en" else "Chinese",
        input=" ".join(str(p) for p in inputs + [work / "examples.dox", mainpage]),
        strip=strip, strip_include=strip_include,
        examples=ROOT / "examples" / EXAMPLES[api][0], mainpage=mainpage) + API_CONFIG[api]
    doxyfile = work / f"Doxyfile.{api}.{lang}"
    doxyfile.write_text(config, encoding="utf-8")
    doxygen(doxyfile, work)
    return target, tokens


def python_types(text, tokens, classes):
    """Write the attribute types back over their placeholders, with links to the classes."""
    def shown(display):
        return re.sub(r"\b[A-Z]\w*\b", lambda m: (
            f'<a class="el" href="classinfrabot_1_1{m.group(0)}.html">{m.group(0)}</a>'
            if m.group(0) in classes else m.group(0)), html.escape(display, quote=False))
    for token, display in tokens.items():
        text = re.sub(rf"\b{token}\b", lambda _: shown(display), text)
    return text


def no_static_attributes(text):
    """A Python attribute is a class attribute to Doxygen, labelled "static": drop the label.

    A static method keeps it; its prototype has a parameter list.
    """
    def proto(match):
        block = match.group(0)
        if "(" in re.sub(r"<[^>]+>", "", block):
            return block
        return block.replace('<span class="mlabel">static</span>', "")
    return re.sub(r'<div class="memproto">.*?</div>', proto, text, flags=re.S)


def counterpart(directory, name):
    """The page of this name in another site: the same file, a class for a struct, or the index."""
    for candidate in (name, name.replace("struct", "class", 1), name.replace("class", "struct", 1)):
        if (directory / candidate).exists():
            return candidate
    return "index.html"


def switches(api, lang, version, out, page):
    """The C++ | Python and English | 简体中文 switches of one page."""
    def segment(items):
        return '<span class="seg">' + "".join(
            f'<span class="on">{label}</span>' if active else f'<a href="{href}">{label}</a>'
            for label, href, active in items) + "</span>"
    other_api = "python" if api == "cpp" else "cpp"
    other_lang = "zh" if lang == "en" else "en"
    api_href = f"../{other_api}/{counterpart(out / lang / version / other_api, page)}"
    lang_href = f"../../../{other_lang}/{version}/{api}/{counterpart(out / other_lang / version / api, page)}"
    apis = [("C++", api_href if api == "python" else "", api == "cpp"),
            ("Python", api_href if api == "cpp" else "", api == "python")]
    langs = [("English", lang_href if lang == "zh" else "", lang == "en"),
             ("简体中文", lang_href if lang == "en" else "", lang == "zh")]
    return f'<div class="ib-switch">{segment(apis)}{segment(langs)}</div>\n'


def redirect(path, href):
    path.write_text(f'<!doctype html>\n<meta charset="utf-8">\n<meta http-equiv="refresh" content="0; url={href}">\n'
                    f'<title>InfraBot SDK</title>\n<a href="{href}">{href}</a>\n', encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, default=ROOT / "build" / "api-doxygen")
    args = parser.parse_args()
    version = version_text()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    table = translations()
    missing = {}
    for lang in LANGS:
        site = out / lang / version
        if site.exists():
            shutil.rmtree(site)  # also what an older layout of the site left there
        for api in APIS:
            with tempfile.TemporaryDirectory() as tmp:
                target, tokens = build(api, lang, version, out, Path(tmp), table, missing)
            classes = {p.name[len("classinfrabot_1_1"):-len(".html")]
                       for p in target.glob("classinfrabot_1_1*.html") if "-members" not in p.name}
            for page in sorted(target.glob("*.html")):
                text = page.read_text(encoding="utf-8")
                if lang == "zh":
                    # The Python prose is Chinese already; its group headings are not.
                    text = translate_page(text, table, missing if api == "cpp" else {})
                if page.name.endswith("-example.html"):
                    text = one_description(text)
                if api == "python":
                    text = python_types(no_static_attributes(text), tokens, classes)
                page.write_text(text, encoding="utf-8")
        redirect(out / lang / version / "index.html", "cpp/index.html")
    # The switches link to pages of the other sites, so they go in once all four exist.
    for lang in LANGS:
        for api in APIS:
            target = out / lang / version / api
            for page in sorted(target.glob("*.html")):
                text = page.read_text(encoding="utf-8")
                if "<!-- end header part -->" in text:
                    text = text.replace("<!-- end header part -->",
                                        switches(api, lang, version, out, page.name) + "<!-- end header part -->", 1)
                    page.write_text(text, encoding="utf-8")
            print(f"{lang} {api}: {target}")
    redirect(out / "index.html", f"en/{version}/cpp/index.html")
    print(f"{len(table)} translations in {ZH_FILE.name}")
    report = out / "missing.txt"
    if missing:
        report.write_text("\n".join(sorted(missing)) + "\n", encoding="utf-8")
        print(f"{len(missing)} texts have no translation: {report}")
    elif report.exists():
        report.unlink()


if __name__ == "__main__":
    main()
