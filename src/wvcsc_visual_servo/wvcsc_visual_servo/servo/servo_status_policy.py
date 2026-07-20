# servo_status_policy.py
"""
MoveIt Servo 状态码解析与动作决策策略模块。

职责：
1. 维护 MoveIt Servo 标准状态码的文本映射（如 2 = 奇异点停止）。
2. 根据当前节点配置，将原始状态码转化为具体的控制行动（OK、减速、安全停止等）。
3. 将状态解释与实际控制逻辑解耦，便于在 YAML 中灵活调整安全策略。
"""

from dataclasses import dataclass
from enum import Enum


class ServoStatusAction(str, Enum):
    """
    视觉伺服节点应对 MoveIt Servo 状态的统一行动枚举。
    """
    OK = 'ok'                       # 正常状态，无任何限制，继续执行视觉伺服
    DECELERATE = 'decelerate'       # 减速状态（如接近奇异点），视觉命令需降速但保持运行
    RECOVERABLE_STOP = 'recoverable_stop'   # 可恢复停止（如进入奇异点），视觉伺服终止，但允许任务管理器回退到其他观察位
    SAFETY_STOP = 'safety_stop'     # 安全硬停止（如关节越界或不可恢复的严重错误），视觉伺服失败，机械臂必须进入急停保护


@dataclass(frozen=True)
class ServoStatusDecision:
    """
    策略模块做出的最终行动决策包。
    """
    action: ServoStatusAction    # 建议采取的枚举动作
    message: str                 # 人类可读的决策原因描述


class ServoStatusPolicy:
    """
    MoveIt Servo 状态码映射策略类。

    初始化时接收四类状态码集合（减速码、可恢复停止码、硬停止码、透传码），
    并在运行时将整型状态码映射到对应的行动建议。
    """
    
    # MoveIt Servo 状态码的人类可读文本映射
    _STATUS_TEXT = {
        0: 'NO_WARNING',
        1: 'DECELERATE_FOR_APPROACHING_SINGULARITY',  # 接近奇异点，降速
        2: 'HALT_FOR_SINGULARITY',                    # 奇异点，立即停止
        3: 'DECELERATE_FOR_COLLISION',                # 接近碰撞，降速
        4: 'HALT_FOR_COLLISION',                      # 碰撞危险，立即停止
        5: 'JOINT_BOUND',                             # 关节限位越界，停止
        6: 'DECELERATE_FOR_LEAVING_SINGULARITY',      # 离开奇异点，降速
    }

    def __init__(self, decel_codes, recoverable_codes, halt_codes,
                 passthrough_codes=()):
        """
        初始化状态码分类策略。

        Args:
            decel_codes (list): 触发“视觉指令降速”的状态码集 (如 [1, 3])。
            recoverable_codes (list): 触发“可恢复停止并重试”的状态码集 (如 [2])。
            halt_codes (list): 触发“硬停止并上报失败”的状态码集 (如 [4, 5])。
            passthrough_codes (list, optional): “透传”状态码集 (如 [6])。
                这些码虽然被 Servo 视为警告，但视觉节点可以忽略，继续正常控制。
        """
        self.decel_codes = {int(code) for code in decel_codes}
        self.recoverable_codes = {int(code) for code in recoverable_codes}
        self.halt_codes = {int(code) for code in halt_codes}
        self.passthrough_codes = {int(code) for code in passthrough_codes}

    @classmethod
    def status_text(cls, code):
        """
        获取指定状态码的人类可读文本描述。
        """
        code = int(code)
        return cls._STATUS_TEXT.get(code, f'UNKNOWN_STATUS_{code}')

    def decide(self, code):
        """
        核心决策函数：输入一个来自 MoveIt Servo 的整型状态码，
        输出对应的控制行动决策。

        Args:
            code (int): MoveIt Servo 节点发布的原始状态码。

        Returns:
            ServoStatusDecision: 包含应采取的决策动作及描述。
        """
        code = int(code)
        
        # 1. 正常状态，无需特殊操作
        if code == 0:
            return ServoStatusDecision(ServoStatusAction.OK, '')
            
        # 2. 透传码：内部已在减速，外部视觉节点无需额外降速，保持 OK 状态
        if code in self.passthrough_codes:
            return ServoStatusDecision(
                ServoStatusAction.OK,
                f'MoveIt Servo internally decelerates for status {code} '
                f'({self.status_text(code)})')
                
        # 3. 减速码：视觉伺服 PID 指令需应用 `warning_speed_scale` 降速系数
        if code in self.decel_codes:
            return ServoStatusDecision(
                ServoStatusAction.DECELERATE,
                f'MoveIt Servo warning status {code} ({self.status_text(code)})')
                
        # 4. 可恢复停止码：立即停止视觉伺服，但任务管理器可以切换到新的观察位重试
        if code in self.recoverable_codes:
            return ServoStatusDecision(
                ServoStatusAction.RECOVERABLE_STOP,
                f'MoveIt Servo recoverable status {code} ({self.status_text(code)})')
                
        # 5. 硬停止码（或其他未知的危险码）：必须立即停止并判定任务失败
        kind = 'unsafe' if code in self.halt_codes else 'unknown unsafe'
        return ServoStatusDecision(
            ServoStatusAction.SAFETY_STOP,
            f'MoveIt Servo {kind} status {code} ({self.status_text(code)})')