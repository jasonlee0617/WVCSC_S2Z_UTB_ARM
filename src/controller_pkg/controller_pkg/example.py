#!/usr/bin/env python3
"""控制继电器示例"""

from set_fault import RelayController


def main() -> int:
    controller = RelayController()
    # 广域喷洒：控制继电器通道 1 断开
    controller.set_channel(1, False)  
    # 广域喷洒：控制继电器通道 1 正常
    controller.set_channel(1, True)  
    # 虫害喷洒：控制继电器通道 2 断开
    controller.set_channel(2, False)  
    # 虫害喷洒：控制继电器通道 2 正常
    controller.set_channel(2, True) 
    
if __name__ == "__main__":
    raise SystemExit(main())
