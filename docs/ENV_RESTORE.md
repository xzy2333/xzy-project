# 环境恢复指南（重装系统后使用）

本文档是“清理硬盘 → 重装系统 → 恢复原样”的路线图。仓库里的 `scripts/` 会帮你自动完成大部分工作，`docs/system_backup/` 是本次装机时抓取的环境快照。

## 0. 最重要的一条：系统版本

本项目用 **ROS1 Noetic + Gazebo 11**，ROS1 Noetic 官方只支持 **Ubuntu 20.04**。重装系统时请装 Ubuntu 20.04（或者 22.04/24.04 + Docker 跑 Noetic，仓库里的 `docker/` 是 ROS2 Humble 的容器方案）。

## 1. 基础工具

```bash
sudo apt update
sudo apt install -y git curl vim
```

## 2. ROS1 Noetic

推荐 fishros 一键安装（选 ROS1 Noetic Desktop-Full）：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

或按官方 wiki 手动安装：<http://wiki.ros.org/noetic/Installation/Ubuntu>

## 3. 项目需要的 ROS 包

快照里保存了完整的已安装包列表 `docs/system_backup/apt_packages.txt`，自动恢复脚本会筛选其中的 `ros-noetic-*` 包安装：

```bash
bash scripts/restore_env.sh
```

核心包（脚本会自动装，这里列出来方便核对）：

- turtlebot3、turtlebot3-simulations、turtlebot3-navigation、turtlebot3-slam、turtlebot3-gazebo、turtlebot3-teleop
- navigation、move-base、amcl、gmapping、map-server、dwa-local-planner
- gazebo11

## 4. Python 依赖

`m1_slam_sim` 等模块只依赖 `numpy` 和 `matplotlib`（见 `docs/system_backup/pip_packages.txt`）：

```bash
sudo apt install -y python3-numpy python3-matplotlib
```

## 5. MVS 相机 SDK（海康，/opt/MVS）

`/opt/MVS` 是海康机器视觉 SDK，不是 apt 包，需要从海康官网下载安装包重装。装好后把以下环境变量加回 `~/.bashrc`（已备份在 `docs/system_backup/bashrc_snippet.txt`）：

```bash
export PATH=$PATH:/opt/MVS/bin
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib:/opt/MVS/lib/64:/opt/MVS/lib/32:/opt/MVS/bin:$LD_LIBRARY_PATH
```

## 6. TurtleBot3 环境变量

```bash
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 7. 拉取项目仓库

```bash
cd ~
git clone git@github.com:xzy2333/xzy2333.git xzy-project
cd xzy-project
```

（如果以后仓库改名或迁移，用新的地址替换即可。）

## 8. Codex CLI（可选，用于恢复“这个助手”本身）

Codex 是通过 npm 全局安装的（`/usr/bin/codex -> /usr/lib/node_modules/@openai/codex/bin/codex.js`）：

```bash
sudo npm install -g @openai/codex
```

然后恢复配置与历史（见 `scripts/backup_codex.sh` 生成的 `~/codex_backup.tar.gz`）：

```bash
tar -xzf ~/codex_backup.tar.gz -C ~/
```

注意：备份里的 `config.toml` 含 API Key，恢复后请自行确认；重装前也可以先在备份里把 key 换成新的。

## 9. GitHub 访问

```bash
ssh-keygen -t ed25519 -C "你的邮箱"
cat ~/.ssh/id_ed25519.pub   # 粘贴到 GitHub → Settings → SSH and GPG keys
```

注意：SSH 私钥不会跟着仓库走，重装系统后需要重新生成并添加。

## 验证

```bash
rosversion -d          # noetic
echo $TURTLEBOT3_MODEL # waffle_pi
python3 -c "import numpy, matplotlib; print('ok')"
```
