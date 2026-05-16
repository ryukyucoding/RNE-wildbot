"""
wildbot_common / robot_state.py

定義各個任務共用的狀態機（State Machine）列舉值。
讓每個 Node 都用相同的語言描述自己的狀態，方便除錯與日誌閱讀。
"""

from enum import Enum, auto


class MissionStatus(Enum):
    """任務的整體執行狀態。"""
    IDLE        = auto()  # 閒置，尚未開始
    RUNNING     = auto()  # 執行中
    SUCCESS     = auto()  # 成功完成
    FAILED      = auto()  # 失敗（可能需要重啟）
    ABORTED     = auto()  # 主動中止


class BearGrabberState(Enum):
    """組員 A - 平地夾熊任務的狀態機。"""
    IDLE                = auto()  # 等待開始
    SEARCHING_BEAR      = auto()  # 用視覺搜尋熊的位置
    NAVIGATING_TO_BEAR  = auto()  # 導航至熊附近
    ALIGNING_TO_BEAR    = auto()  # 精確對準熊
    GRABBING            = auto()  # 執行夾取動作
    RETURNING_TO_BASE   = auto()  # 帶著熊返回基地
    RELEASING           = auto()  # 在基地放下熊
    DONE                = auto()  # 完成一輪，準備下一輪


class BridgeMissionState(Enum):
    """組員 B - 上橋夾熊任務的狀態機。"""
    IDLE                    = auto()  # 等待開始
    NAVIGATING_TO_BRIDGE    = auto()  # 導航至橋樑入口
    CROSSING_BRIDGE         = auto()  # 慢速過橋（注意橋邊緣！）
    REACHING_MIDPOINT       = auto()  # 抵達橋中點（+5分！）
    SEARCHING_BEAR_ON_BRIDGE= auto()  # 在橋上找熊
    GRABBING                = auto()  # 夾取橋上的熊
    DESCENDING_BRIDGE       = auto()  # 從橋另一側下橋
    RETURNING_TO_BASE       = auto()  # 帶著熊回基地
    RELEASING               = auto()  # 在基地放下熊
    DONE                    = auto()  # 完成


class DoorOpenerState(Enum):
    """組員 C - 開門任務的狀態機。"""
    IDLE                    = auto()  # 等待開始
    NAVIGATING_TO_DOOR      = auto()  # 導航至門前
    DETECTING_HANDLE        = auto()  # 用視覺偵測門把位置
    ALIGNING_TO_HANDLE      = auto()  # 精確對準門把
    UNLOCKING               = auto()  # 解鎖門把（+5分！）
    PUSHING_DOOR            = auto()  # 將門推開至規定角度（+5分！）
    DONE                    = auto()  # 完成


# 基地（Base / Starting Position）座標
# 【TODO - 全體】：比賽當天根據實際場地標定後修改這裡
BASE_POSITION = {
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,  # 弧度
}

# 橋樑入口座標
# 【TODO - 組員 B】：根據 SLAM 地圖確認後填入
BRIDGE_ENTRY_POSITION = {
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,
}

# 門的位置
# 【TODO - 組員 C】：根據 SLAM 地圖確認後填入
DOOR_POSITION = {
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,
}
