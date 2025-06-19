# 1. 使用官方 ROS Humble 映像
FROM ros:humble-ros-base

# 2. 安裝 can-utils（可選但常用） + pip + python-can
RUN apt update && \
    apt install -y python3-pip can-utils && \
    apt install -y iproute2 && \
    pip3 install --no-cache-dir python-can

# 3. 設定工作目錄（你掛載的 /workspace）
WORKDIR /workspace

# 4. （可選）預設進入 bash
CMD ["bash"]
