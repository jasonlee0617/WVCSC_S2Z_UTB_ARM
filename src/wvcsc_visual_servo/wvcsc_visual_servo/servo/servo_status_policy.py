from dataclasses import dataclass
from enum import Enum


class ServoStatusAction(str, Enum):
    OK = 'ok'
    DECELERATE = 'decelerate'
    HALT_RECOVERY = 'halt_recovery'


@dataclass(frozen=True)
class ServoStatusDecision:
    action: ServoStatusAction
    message: str


class ServoStatusPolicy:
    def __init__(self, decel_codes, halt_codes):
        self.decel_codes = {int(code) for code in decel_codes}
        self.halt_codes = {int(code) for code in halt_codes}

    def decide(self, code):
        code = int(code)
        if code == 0:
            return ServoStatusDecision(ServoStatusAction.OK, '')
        if code in self.decel_codes:
            return ServoStatusDecision(
                ServoStatusAction.DECELERATE,
                f'MoveIt Servo warning status {code}')
        return ServoStatusDecision(
            ServoStatusAction.HALT_RECOVERY,
            f'MoveIt Servo unsafe status {code}')
