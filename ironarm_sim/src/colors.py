"""ANSI colour name -> QColor mapping."""

from PySide6.QtGui import QColor

# Standard ANSI 16-colour palette (normal intensity).
_NORMAL: dict[str, str] = {
    "black": "#1a1a1a",
    "red": "#cc3333",
    "green": "#33cc33",
    "brown": "#cc8833",
    "blue": "#3366cc",
    "magenta": "#cc33cc",
    "cyan": "#33cccc",
    "white": "#cccccc",
}

# Bright variants (SGR 1 + colour).
_BRIGHT: dict[str, str] = {
    "black": "#555555",
    "red": "#ff5555",
    "green": "#55ff55",
    "brown": "#ffff55",
    "blue": "#5555ff",
    "magenta": "#ff55ff",
    "cyan": "#55ffff",
    "white": "#ffffff",
}


def to_qcolor(name: str, bold: bool = False) -> QColor:
    """Convert a pyte colour string to QColor.

    pyte stores:
    - ``"default"``          for the default foreground / background
    - ``"red"``, ``"blue"``… for the 16 standard ANSI colours
    - raw 6-digit hex **without** ``#`` for 256-colour and truecolour
      (e.g. ``"ff0000"``, ``"ff8000"``).
    """
    if name == "default":
        return QColor(204, 204, 204) if not bold else QColor(255, 255, 255)

    # Hex without leading '#'  (pyte representation of 256-colour / truecolour)
    if len(name) == 6 and all(c in "0123456789abcdef" for c in name.lower()):
        return QColor("#" + name)

    if name.startswith("#"):
        return QColor(name)

    if bold and name in _BRIGHT:
        return QColor(_BRIGHT[name])

    if name in _NORMAL:
        return QColor(_NORMAL[name])

    return QColor(204, 204, 204)
