#!/bin/bash
set -euo pipefail

# VRX Humble + Gazebo Garden 一键依赖安装脚本
# 适用于 Ubuntu 22.04 + ROS 2 Humble

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ------------------- 前置检查 -------------------

if [ "$(id -u)" -eq 0 ]; then
    log_error "请不要使用 root 用户直接运行此脚本。请使用普通用户并确保 sudo 可用。"
    exit 1
fi

if ! command -v sudo &> /dev/null; then
    log_error "系统中没有找到 sudo，请先安装并配置 sudo。"
    exit 1
fi

UBUNTU_CODENAME=$(lsb_release -cs 2>/dev/null || echo "unknown")
if [ "$UBUNTU_CODENAME" != "jammy" ]; then
    log_warn "检测到系统 codename 为 '$UBUNTU_CODENAME'，本脚本主要针对 Ubuntu 22.04 (jammy)。"
    read -rp "是否继续? [y/N] " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        exit 0
    fi
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
    log_error "未检测到 ROS 2 Humble。请先安装 ROS 2 Humble 后再运行此脚本。"
    exit 1
fi

# ------------------- 安装系统依赖 -------------------

log_info "更新 apt 索引..."
sudo apt-get update

log_info "安装基础工具 (wget, gnupg, lsb-release, software-properties-common)..."
sudo apt-get install -y \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    curl \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git

# ------------------- 添加 OSRF 软件源 -------------------

OSRF_KEYRING="/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
OSRF_LIST="/etc/apt/sources.list.d/gazebo-stable.list"

if [ ! -f "$OSRF_KEYRING" ]; then
    log_info "添加 OSRF Gazebo 软件源 GPG key..."
    sudo wget -q https://packages.osrfoundation.org/gazebo.gpg -O "$OSRF_KEYRING"
else
    log_warn "OSRF GPG key 已存在，跳过。"
fi

if [ ! -f "$OSRF_LIST" ]; then
    log_info "添加 OSRF Gazebo apt 源..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$OSRF_KEYRING] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
        sudo tee "$OSRF_LIST" > /dev/null
else
    log_warn "OSRF apt 源已存在，跳过。"
fi

log_info "更新 apt 索引（包含新源）..."
sudo apt-get update

# ------------------- 安装 Gazebo Garden -------------------

log_info "安装 Gazebo Garden (gz-garden) 及其开发库..."
sudo apt-get install -y gz-garden

# ------------------- 安装 ROS-Gazebo 桥接包 -------------------

log_info "安装 ROS 2 Humble 的 Gazebo 桥接包..."
sudo apt-get install -y \
    ros-humble-ros-gzgarden \
    ros-humble-ros-gzgarden-bridge \
    ros-humble-ros-gzgarden-sim \
    ros-humble-ros-gzgarden-interfaces

log_info "安装 VRX 构建与运行依赖..."
sudo apt-get install -y \
    python3-sdformat13 \
    ros-humble-xacro

# ------------------- 初始化 rosdep -------------------

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log_info "初始化 rosdep..."
    sudo rosdep init || log_warn "rosdep init 失败（可能已初始化过，忽略）"
fi

log_info "更新 rosdep..."
rosdep update || log_warn "rosdep update 失败，请检查网络连接。"

# ------------------- 安装项目依赖 -------------------

WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
log_info "尝试为 VRX 工作空间安装 rosdep 依赖..."
cd "$WS_DIR"

# 兼容标准 ROS workspace (src/) 与非标准布局（直接在仓库根目录编译）
if [ -d "$WS_DIR/src" ]; then
    log_info "检测到标准 ROS workspace 结构，使用 --from-paths src"
    rosdep install --from-paths src --ignore-src -r -y || log_warn "rosdep install 部分失败，可能有些依赖需要手动处理。"
else
    log_warn "未检测到 src 目录，尝试从当前目录直接解析..."
    # 对非标准布局，rosdep 可能无法直接解析，仅做尝试
    rosdep install --from-paths . --ignore-src -r -y || log_warn "rosdep install 部分失败（非标准布局常见），如编译已通过则无需担心。"
fi

# ------------------- 完成 -------------------

log_info "依赖安装完成！"
echo ""
echo "=========================================="
echo "接下来请执行以下命令编译 VRX："
echo ""
echo "  cd $WS_DIR"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --merge-install"
echo ""
echo "=========================================="
