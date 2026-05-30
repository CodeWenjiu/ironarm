# Minimal type stubs for mujoco (C extension — no source introspection possible).
# Only declares the symbols used by this project.

from typing import Any

class MjModel:
    nq: int
    njnt: int
    @staticmethod
    def from_xml_string(xml: str) -> "MjModel": ...
    def joint(self, index_or_name: int | str) -> Any: ...

class MjData:
    def __init__(self, model: MjModel) -> None: ...
    def joint(self, name: str) -> Any: ...
    def body(self, name: str) -> Any: ...

class MjvScene:
    ngeom: int
    def __init__(self, model: MjModel, maxgeom: int = 0) -> None: ...

class MjrContext:
    def __init__(self, model: MjModel, font_scale: int) -> None: ...

class MjvCamera: ...
class MjvOption: ...

def MjrRect(left: int, bottom: int, width: int, height: int) -> Any: ...
def mj_forward(model: MjModel, data: MjData) -> None: ...
def mjv_updateScene(
    model: MjModel,
    data: MjData,
    opt: MjvOption,
    pert: Any,
    cam: MjvCamera,
    catmask: int,
    scene: MjvScene,
) -> None: ...
def mjr_render(viewport: Any, scene: MjvScene, context: MjrContext) -> None: ...

class mjtFontScale:
    mjFONTSCALE_150: int

class mjtCatBit:
    mjCAT_ALL: int
