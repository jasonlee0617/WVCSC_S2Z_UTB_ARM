from dataclasses import dataclass
from enum import Enum


class ServoStatusAction(str, Enum):
    OK = 'ok'
    DECELERATE = 'decelerate'
    RECOVERABLE_STOP = 'recoverable_stop'
    SAFETY_STOP = 'safety_stop'


@dataclass(frozen=True)
class ServoStatusDecision:
    action: ServoStatusAction
    message: str


class ServoStatusPolicy:
    _STATUS_TEXT = {
        0: 'NO_WARNING',
        1: 'DECELERATE_FOR_APPROACHING_SINGULARITY',
        2: 'HALT_FOR_SINGULARITY',
        3: 'DECELERATE_FOR_COLLISION',
        4: 'HALT_FOR_COLLISION',
        5: 'JOINT_BOUND',
        6: 'DECELERATE_FOR_LEAVING_SINGULARITY',
    }

    def __init__(self, decel_codes, recoverable_codes, halt_codes,
                 passthrough_codes=()):
        self.decel_codes = {int(code) for code in decel_codes}
        self.recoverable_codes = {int(code) for code in recoverable_codes}
        self.halt_codes = {int(code) for code in halt_codes}
        self.passthrough_codes = {int(code) for code in passthrough_codes}

    @classmethod
    def status_text(cls, code):
        code = int(code)
        return cls._STATUS_TEXT.get(code, f'UNKNOWN_STATUS_{code}')

    def decide(self, code):
        code = int(code)
        if code == 0:
            return ServoStatusDecision(ServoStatusAction.OK, '')
        if code in self.passthrough_codes:
            return ServoStatusDecision(
                ServoStatusAction.OK,
                f'MoveIt Servo internally decelerates for status {code} '
                f'({self.status_text(code)})')
        if code in self.decel_codes:
            return ServoStatusDecision(
                ServoStatusAction.DECELERATE,
                f'MoveIt Servo warning status {code} ({self.status_text(code)})')
        if code in self.recoverable_codes:
            return ServoStatusDecision(
                ServoStatusAction.RECOVERABLE_STOP,
                f'MoveIt Servo recoverable status {code} ({self.status_text(code)})')
        kind = 'unsafe' if code in self.halt_codes else 'unknown unsafe'
        return ServoStatusDecision(
            ServoStatusAction.SAFETY_STOP,
            f'MoveIt Servo {kind} status {code} ({self.status_text(code)})')
