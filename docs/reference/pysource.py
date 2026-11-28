# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""The Python API as a module that Doxygen reads.

Doxygen's Python parser takes a type stub badly as it stands: pybind11 enums
list their members by name only, properties come as getter and setter pairs,
the constants carry no text and the types spell out what pybind11 accepts
(typing.SupportsFloat | typing.SupportsIndex). This module reads
python/infrabot/infrabot_native.pyi and writes the same API as infrabot.py, in
Doxygen's own comment form:

- every class, method and function keeps its docstring, as a brief (the first
  sentence) and the rest as details;
- a property becomes an attribute in the Attributes group, typed by its getter;
- an enum value takes its text from the C++ enum value of include/infrabot;
- the methods of Robot fall in the groups of the C++ class, in its order;
- a constant takes its text from CONSTANTS below, worded as docs/protocol.md;
- an attribute type with brackets is written as a placeholder name, since
  Doxygen splits "list[float]" around the attribute name; the site puts the
  type back into the HTML (module() returns the placeholders).

Given a translate function, every text is written in Chinese where zh.json has
it: the docstrings are translated whole, before Doxygen splits them.
"""
import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
STUB = ROOT / "python" / "infrabot" / "infrabot_native.pyi"
ROBOT_HPP = ROOT / "include" / "infrabot" / "robot.hpp"

# Python enum -> C++ enum of robot.hpp, where the names differ.
CPP_ENUM = {"FaultKind": "Kind", "MotionPhase": "Phase"}

# The protocol.h constants the module exports, with their meaning.
CONSTANTS = {
    "ARM_MASK": "Enabled-mask bits of J1–J7 (MotorGroup.Arm).",
    "ARM_MAX_JOINT_VELOCITY": "Joint speed cap of J1–J7, rad/s: the most a move or a stream may command.",
    "CAP_JOINT_RT": "Capability bit: RT joint targets.",
    "CAP_GROUPS": "Capability bit: the arm and the gripper are enabled separately.",
    "CAP_TELEMETRY_V3": "Capability bit: per-drive feedback and command fields in telemetry.",
    "CAP_CONTROLLED_STOP": "Capability bit: controlled stop and safety status, brake to hold.",
    "CAP_LOCAL_GRAVITY": "Capability bit: gravity compensation on the controller.",
    "CAP_ADMITTANCE": "Capability bit: joint and Cartesian admittance.",
    "CAP_PARAMETERS": "Capability bit: the parameter service of list_parameters() and set_parameter().",
    "CAP_DIAGNOSTICS": "Capability bit: the diagnostic snapshot of diagnostics().",
    "MOTOR_FEEDBACK_STALE": "Motor flag bit: the drive's feedback is stale.",
    "PARAM_GROUP_GRAVITY": "Parameter group of gravity compensation.",
    "PARAM_GROUP_IMPEDANCE": "Parameter group of the impedance modes.",
    "PARAM_GROUP_ADMITTANCE": "Parameter group of the admittance modes.",
    "REGISTER_U32": "Register value type: unsigned 32-bit integer.",
    "REGISTER_F32": "Register value type: 32-bit float.",
}

# The note of a class the SDK returns and a program never fills in.
RETURNED = "The SDK returns it; its attributes are read-only."


def py_type(text):
    """Readable Python types: the stub spells out what pybind11 accepts."""
    text = re.sub(r"typing\.Annotated\[(?:collections\.abc\.Sequence|list)\[([^\]]+)\], 'FixedSize\((\d+)\)'\]",
                  lambda m: f"list[{py_type(m.group(1).strip())}] ({m.group(2)})", text)
    text = re.sub(r"typing\.SupportsFloat \| typing\.SupportsIndex", "float", text)
    text = re.sub(r"typing\.SupportsInt \| typing\.SupportsIndex", "int", text)
    text = text.replace("collections.abc.Callable", "Callable").replace("typing.", "")
    return text


def doxygen_type(text):
    """A type as Doxygen's Python parser keeps it whole: no space before a size."""
    return py_type(text).replace("] (", "](")


def robot_groups():
    """Method name -> (group, its description), in the order of robot.hpp."""
    source = ROBOT_HPP.read_text(encoding="utf-8")
    body = source[source.index("class Robot"):]
    groups, current = {}, None  # in the order of the declarations
    pattern = re.compile(r"/\*\*\s*\n?\s*\*?\s*@name ([^\n*]+?)\s*(?:\*/|\n(.*?)\*/)", re.S)
    marks = [(m.start(), m.group(1).strip(), " ".join(
        line.strip().lstrip("*").strip() for line in (m.group(2) or "").splitlines()).strip())
        for m in pattern.finditer(body)]
    position = 0
    for line in body.splitlines(keepends=True):
        for start, name, text in marks:
            if position <= start < position + len(line):
                current = (name, text)
        if "///@}" in line:
            current = None
        match = re.match(r"\s{4}(?:static )?[\w:<>,\s&*]+?\b(\w+)\(", line)
        if match and current and not line.strip().startswith(("*", "//", "/*")):
            groups.setdefault(match.group(1), current)
        position += len(line)
    return groups


def enum_texts():
    """C++ enum name -> {value name: its ///< text}, from robot.hpp."""
    source = ROBOT_HPP.read_text(encoding="utf-8")
    found = {}
    for match in re.finditer(r"enum class (\w+)\s*:\s*\w+\s*\{(.*?)\};", source, re.S):
        values = {}
        for line in match.group(2).splitlines():
            item = re.match(r"\s*(\w+)\s*(?:=[^,]*)?,?\s*///<\s*(.+)", line)
            if item:
                values[item.group(1)] = item.group(2).strip()
        found[match.group(1)] = values
    return found


def split(doc):
    """A docstring as its first sentence and the rest, each on one line."""
    text = " ".join(doc.split())
    match = re.search(r"(?<!e\.g)(?<!i\.e)(?<!\bvs)\.\s+(?=[A-Z\[`(\"])", text)
    if not match:
        return text, ""
    return text[:match.start() + 1], text[match.end():]


def split_zh(text):
    """A Chinese text as its first sentence and the rest."""
    match = re.search(r"。(?=\S)", text)
    if not match:
        return text, ""
    return text[:match.end()], text[match.end():].strip()


def escape(text, html=False):
    """Doxygen reads @, \\, < and # in a comment as commands or markup.

    A Chinese text from zh.json is HTML already: its tags stay.
    """
    text = text.replace("\\", "\\\\").replace("@", "\\@")
    if not html:
        text = text.replace("<", "\\<").replace(">", "\\>")
    return re.sub(r"(?<![\w`&])#", r"\\#", text)


def paragraphs(doc, translate):
    """The paragraphs of a text as (one line, is Chinese HTML), translated where it can be."""
    chinese = translate(doc) if translate and doc.strip() else None
    text = doc if chinese is None else chinese
    return [(" ".join(p.split()), chinese is not None)
            for p in re.split(r"\n\s*\n", text.strip()) if p.strip()]


def comment(docs, indent, translate=None):
    """A ## block with the brief and the details of one or more texts, one paragraph a line.

    Each text is translated on its own, so a note added to a docstring needs no
    entry of its own with the docstring.
    """
    items = [p for doc in ([docs] if isinstance(docs, str) else docs)
             for p in paragraphs(doc, translate)] or [("", False)]
    first, html = items[0]
    brief, rest = split_zh(first) if html else split(first)
    pad = " " * indent
    lines = [f"{pad}## @brief {escape(brief, html)}"]
    for text, html in ([(rest, html)] if rest else []) + items[1:]:
        lines += [f"{pad}##", f"{pad}## {escape(text, html)}"]
    return lines


def attribute_type(annotation, tokens):
    """The type of an attribute; one with brackets as a placeholder that tokens maps."""
    text = doxygen_type(annotation)
    if "[" not in text:
        return text
    return tokens.setdefault(text, f"IbType{len(tokens)}")


def signature(fn, owner):
    """The def line of a function or method, with readable types."""
    args = fn.args
    names = args.args
    defaults = [None] * (len(names) - len(args.defaults)) + list(args.defaults)
    parts = []
    for arg, default in zip(names, defaults):
        if arg.arg == "self":
            parts.append("self")
            continue
        text = arg.arg
        if arg.annotation is not None:
            text += ": " + doxygen_type(ast.unparse(arg.annotation))
        if default is not None:
            value = ast.unparse(default)
            if value == "...":
                value = DEFAULTS.get((fn.name, arg.arg), "...")
            text += " = " + value
        parts.append(text)
    returns = doxygen_type(ast.unparse(fn.returns)) if fn.returns is not None else "None"
    return f"def {fn.name}({', '.join(parts)}) -> {returns}:"


# Defaults the stub writes as "...": pybind11 cannot print them; bindings.cpp names them.
DEFAULTS = {}


def binding_defaults():
    """(method, argument) -> the default that py::arg_v of bindings.cpp prints."""
    text = (ROOT / "python" / "bindings.cpp").read_text(encoding="utf-8")
    starts = list(re.finditer(r'\.def(?:_static)?\("(\w+)"', text))
    found = {}
    for index, start in enumerate(starts):
        end = starts[index + 1].start() if index + 1 < len(starts) else len(text)
        for arg in re.finditer(r'py::arg_v\("(\w+)",\s*[\w:{}]+,\s*"([^"]+)"\)', text[start.end():end]):
            found[(start.group(1), arg.group(1))] = arg.group(2)
    return found


def is_enum(node):
    return "Members:" in (ast.get_docstring(node) or "")


def enum_class(node, texts, out, translate):
    doc = (ast.get_docstring(node) or "").split("Members:")[0].strip()
    values = texts.get(CPP_ENUM.get(node.name, node.name), {})
    out += comment(doc, 0, translate) + [f"class {node.name}:", "", "    ## @name Values", "    ## @{", ""]
    members = []
    for item in node.body:
        if isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name) and \
                not item.target.id.startswith("_"):
            line = STUB_LINES[item.lineno - 1]
            number = re.search(r"value = <\w+\.\w+: (\d+)>", line)
            members.append((int(number.group(1)) if number else 0, item.target.id))
    for number, name in sorted(members):
        text = values.get(name.rstrip("_"))
        if text:
            out += comment(text, 4, translate)
        out += [f"    {name}: {node.name} = {number}"]
    out += ["", "    ## @}", "", "    ## @name Attributes", "    ## @{", ""]
    out += comment("The value as an int.", 4, translate) + ["    value: int"]
    out += comment("The name of the value.", 4, translate) + ["    name: str", "", "    ## @}", "", "", ""]


def plain_class(node, groups, out, translate, tokens):
    methods, fields, setters = [], {}, set()
    for item in node.body:
        if not isinstance(item, ast.FunctionDef):
            continue
        decorators = [ast.unparse(d) for d in item.decorator_list]
        if "property" in decorators:
            fields[item.name] = item
        elif any(d.endswith(".setter") for d in decorators):
            setters.add(item.name)
        elif item.name == "__init__" or not item.name.startswith("_"):
            methods.append((item, "staticmethod" in decorators))
    docs = [ast.get_docstring(node) or ""]
    if fields and not setters:
        docs.append(RETURNED)
    out += comment(docs, 0, translate) + [f"class {node.name}:", ""]

    def method(item, static, indent=4):
        pad = " " * indent
        doc = ast.get_docstring(item)
        if item.name == "__init__" and not doc:
            doc = "Creates one with the default values."
        lines = comment(doc, indent, translate) if doc else []
        if static:
            lines.append(pad + "@staticmethod")
        lines += [pad + signature(item, node.name), pad + "    pass", ""]
        return lines

    init = [m for m in methods if m[0].name == "__init__"]
    rest = [m for m in methods if m[0].name != "__init__"]
    for item, static in init:
        out += method(item, static)
    if groups is not None:
        # The groups of the C++ class, and the methods in each in the order of robot.hpp.
        rank = {name: index for index, name in enumerate(groups)}
        by_group = {}
        for item, static in sorted(rest, key=lambda m: rank.get(m[0].name, len(rank))):
            by_group.setdefault(groups.get(item.name, ("Other", "")), []).append((item, static))
        for (name, text), members in sorted(by_group.items(), key=lambda g: rank.get(g[1][0][0].name, len(rank))):
            out += [f"    ## @name {name}"]
            out += [f"    ## {escape(line, html)}" for line, html in paragraphs(text, translate)]
            out += ["    ## @{", ""]
            for item, static in members:
                out += method(item, static)
            out += ["    ## @}", ""]
    else:
        for item, static in rest:
            out += method(item, static)
    if fields:
        out += ["    ## @name Attributes", "    ## @{", ""]
        for name, item in fields.items():
            out += comment(ast.get_docstring(item) or "", 4, translate)
            out += [f"    {name}: {attribute_type(ast.unparse(item.returns), tokens)}"]
        out += ["", "    ## @}", ""]
    out += ["", ""]


STUB_LINES = []


def module(stub=STUB, translate=None):
    """The text of infrabot.py, and placeholder name -> the attribute type it stands for.

    translate(english) gives the Chinese HTML of a text, or None to keep the English.
    """
    global STUB_LINES
    source = stub.read_text(encoding="utf-8")
    STUB_LINES = source.splitlines()
    DEFAULTS.clear()
    DEFAULTS.update(binding_defaults())
    tree = ast.parse(source)
    groups = robot_groups()
    texts = enum_texts()
    tokens = {}
    out = ["# The Python API of the InfraBot SDK, written for Doxygen by",
           "# docs/reference/pysource.py from python/infrabot/infrabot_native.pyi.", ""]
    out += ["## @package infrabot"] + comment(MODULE_DOC, 0, translate) + ["", ""]
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            if is_enum(node):
                enum_class(node, texts, out, translate)
            else:
                plain_class(node, groups if node.name == "Robot" else None, out, translate, tokens)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            out += comment(ast.get_docstring(node) or "", 0, translate)
            out += [signature(node, None), "    pass", "", ""]
    names = [n for n in tree.body if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
             and not n.target.id.startswith("_")]
    for node in names:
        name = node.target.id
        text = CONSTANTS.get(name)
        if text is None:
            raise SystemExit(f"pysource.py: no text for the constant {name}; add it to CONSTANTS")
        out += comment(text, 0, translate)
        out += [f"{name}: {ast.unparse(node.annotation)} = {protocol_value(name, node)}"]
    return "\n".join(out) + "\n", {token: text for text, token in tokens.items()}


# The module text. The stub's docstring has the timing classes and the call order
# as lists, which the Main Page of the site shows instead.
MODULE_DOC = ("Python API of the InfraBot SDK. One Robot per arm controller. Joint values are in SI "
              "units: rad, rad/s, N·m. A call that fails raises RuntimeError with the reason, in Chinese.")


def protocol_value(name, node):
    """A constant as protocol.h writes it: masks in hex, the speed as 3.14."""
    value = ast.literal_eval(node.value)
    if isinstance(value, float):
        return f"{value:.2f}"
    if name.startswith(("CAP_", "ARM_MASK", "MOTOR_")):
        return hex(value)
    return str(value)


if __name__ == "__main__":
    print(module()[0], end="")
