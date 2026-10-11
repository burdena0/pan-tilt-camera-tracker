#!/usr/bin/env python3
"""Static sanity checks for main/turret_config.h; no third-party modules."""

from __future__ import annotations

import ast
import operator
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "main" / "turret_config.h"


def read_defines() -> dict[str, str]:
    text = CONFIG.read_text(encoding="utf-8")
    text = re.sub(r"\\\s*\n", " ", text)
    result: dict[str, str] = {}
    for line in text.splitlines():
        match = re.match(r"\s*#define\s+([A-Z][A-Z0-9_]*)\s+(.+?)\s*(?:/\*.*)?$", line)
        if match:
            result[match.group(1)] = match.group(2).strip()
    return result


OPS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.USub: operator.neg,
    ast.UAdd: operator.pos,
}


def safe_eval(expr: str, values: dict[str, float]) -> float:
    expr = re.sub(r"(?<=\d)[fFuUlL]+\b", "", expr)
    tree = ast.parse(expr, mode="eval")

    def visit(node: ast.AST) -> float:
        if isinstance(node, ast.Expression):
            return visit(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id in values:
            return values[node.id]
        if isinstance(node, ast.BinOp) and type(node.op) in OPS:
            return OPS[type(node.op)](visit(node.left), visit(node.right))
        if isinstance(node, ast.UnaryOp) and type(node.op) in OPS:
            return OPS[type(node.op)](visit(node.operand))
        raise ValueError(f"unsupported expression: {expr!r}")

    return visit(tree)


def resolve(raw: dict[str, str]) -> dict[str, float]:
    values: dict[str, float] = {}
    pending = dict(raw)
    for _ in range(len(pending) + 1):
        progress = False
        for name, expr in list(pending.items()):
            try:
                values[name] = safe_eval(expr, values)
            except (ValueError, SyntaxError, KeyError):
                continue
            del pending[name]
            progress = True
        if not progress:
            break
    return values


def check(condition: bool, message: str, errors: list[str]) -> None:
    marker = "PASS" if condition else "FAIL"
    print(f"[{marker}] {message}")
    if not condition:
        errors.append(message)


def main() -> int:
    raw = read_defines()
    v = resolve(raw)
    errors: list[str] = []

    pin_names = [name for name in raw if name.startswith("PIN_")]
    pins = {name: int(v[name]) for name in pin_names}
    check(len(set(pins.values())) == len(pins), "all configured GPIOs are unique", errors)
    check(all(0 <= pin <= 39 for pin in pins.values()), "GPIO numbers fit classic ESP32", errors)
    check(pins["PIN_JOYSTICK_PAN"] in (32, 33, 34, 35, 36, 39) and
          pins["PIN_JOYSTICK_TILT"] in (32, 33, 34, 35, 36, 39),
          "joystick uses ADC1-capable GPIOs", errors)

    check(v["PAN_MIN_DEG"] < 0 < v["PAN_MAX_DEG"], "pan travel contains zero", errors)
    check(v["TILT_MIN_DEG"] < 0 < v["TILT_MAX_DEG"], "tilt travel contains zero", errors)
    check(v["HOME_BACKOFF_DEG"] > 0, "homing backoff is positive", errors)

    pan_rate = v["PAN_MAX_SPEED_DPS"] * v["PAN_STEPS_PER_DEGREE"]
    tilt_rate = v["TILT_MAX_SPEED_DPS"] * v["TILT_STEPS_PER_DEGREE"]
    representable = v["STEP_ISR_HZ"] / 2 - 1
    check(pan_rate < representable,
          f"pan max step rate {pan_rate:.0f}/s is below ISR limit {representable:.0f}/s", errors)
    check(tilt_rate < representable,
          f"tilt max step rate {tilt_rate:.0f}/s is below ISR limit {representable:.0f}/s", errors)

    gravity_torque = v["CAMERA_MASS_KG"] * 9.80665 * v["CAMERA_COG_OFFSET_M"]
    shaft_torque = (v["MOTOR_HOLDING_TORQUE_NM"] * v["TILT_GEAR_RATIO"] *
                    v["GEAR_EFFICIENCY"])
    factor = shaft_torque / gravity_torque
    check(factor >= v["MIN_STATIC_SAFETY_FACTOR"],
          f"estimated static tilt factor {factor:.2f} >= {v['MIN_STATIC_SAFETY_FACTOR']:.2f}",
          errors)

    print(f"\nPan resolution:  {1 / v['PAN_STEPS_PER_DEGREE']:.5f} deg/microstep")
    print(f"Tilt resolution: {1 / v['TILT_STEPS_PER_DEGREE']:.5f} deg/microstep")
    print(f"camera torque:   {gravity_torque:.3f} N m at configured offset (assumed mass)")
    print(f"Shaft estimate:  {shaft_torque:.3f} N m (static holding estimate only)")

    if errors:
        print(f"\n{len(errors)} configuration check(s) failed.")
        return 1
    print("\nConfiguration checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
