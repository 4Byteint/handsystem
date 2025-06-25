# Socket 接收功能說明

## 概述

系統使用獨立的 `SocketReceiver` 類來處理 socket 接收功能。`SocketReceiver` 是一個 ROS2 節點，可以持續接收來自外部程序的抓取姿態數據，並與 `pubsub` 節點進行通信。

## 功能特點

1. **獨立節點**: `SocketReceiver` 作為獨立的 ROS2 節點運行
2. **持續運行**: socket 線程在後台持續運行，直到程序停止
3. **自動清理**: 程序停止時自動清理 socket 線程和連接
4. **數據處理**: 自動解析接收到的數據並處理抓取姿態邏輯
5. **ROS2 集成**: 將接收到的數據發布到 `/grasp_pose` topic
6. **直接通信**: 可以直接修改 `pubsub` 節點的狀態

## Socket 配置

- **主機**: 127.0.0.1 (localhost)
- **端口**: 5005
- **協議**: TCP

## 數據格式

接收的數據格式為：`x,y,angle`

例如：
```
10.5,35.2,45.0
```

- `x`: X 座標 (float)
- `y`: Y 座標 (float)  
- `angle`: 角度 (float)

特殊值：
- `nan,nan,nan`: 表示無效數據

## 抓取姿態邏輯

接收到的數據會根據以下條件進行處理：

1. **條件1**: y > 34.50
2. **條件2**: -90 ≤ angle ≤ 90

如果兩個條件都滿足，設置狀態為 `STATE_GRABBING`
否則設置狀態為 `STATE_RELEASING`

## 使用方法

### 1. 啟動 ROS2 節點

```bash
# 在 ROS2 工作空間中
ros2 run gripper_sub gripper_sub_main
```

程序會自動創建兩個節點：
- `pubsub` 節點：處理 gripper 控制邏輯
- `pose_node` 節點：處理 socket 接收

### 2. 發送數據

使用提供的測試客戶端：

```bash
python3 test_socket_client.py
```

或者使用自定義客戶端：

```python
import socket

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('127.0.0.1', 5005))

# 發送數據
message = "10.5,35.2,45.0\n"
client.sendall(message.encode('utf-8'))

client.close()
```

## 日誌輸出

Socket 相關的日誌會以 `[Socket]` 前綴顯示：

```
[Socket] Socket 接收線程已啟動：127.0.0.1:5005
[Socket] Socket 服務器啟動：127.0.0.1:5005
[Socket] 客戶端已連線：('127.0.0.1', 12345)
[Socket] 收到資料：x=10.50, y=35.20, angle=45.00
[Socket] 處理抓取姿態: x=10.50, y=35.20, angle=45.00
[Socket] 抓取姿態: 通過
```

## 架構說明

### 節點關係

```
SocketReceiver (pose_node)
    ↓ 發布到 /grasp_pose topic
pubsub 節點
    ↓ 訂閱 /grasp_pose topic
grasp_callback 處理邏輯
```

### 直接通信

`SocketReceiver` 也可以直接修改 `pubsub` 節點的狀態：

```python
# 在 SocketReceiver 中
with self.pubsub_instance.lock:
    self.pubsub_instance.claw.state = GripperState.STATE_GRABBING
```

## 錯誤處理

- 連接錯誤會自動重試
- 數據解析錯誤會記錄日誌但不中斷服務
- 程序關閉時會正確清理所有資源

## 注意事項

1. 確保端口 5005 沒有被其他程序佔用
2. 客戶端發送的數據必須以換行符 `\n` 結尾
3. 數據格式必須正確，否則會被忽略
4. 程序使用 daemon 線程，主程序退出時會自動結束
5. `SocketReceiver` 需要 `pubsub` 實例的引用才能直接修改狀態

## 獨立運行

`SocketReceiver` 也可以獨立運行：

```bash
python3 ros_socket_receive.py
```

獨立運行時只會發布到 `/grasp_pose` topic，不會直接修改 `pubsub` 節點的狀態。 