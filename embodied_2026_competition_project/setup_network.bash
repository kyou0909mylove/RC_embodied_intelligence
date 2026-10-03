#!/usr/bin/env bash
# 在专用Ubuntu20.04电脑的每个ROS终端执行：source ./setup_network.bash
# 底盘是ROS master；ROS_IP是这台电脑到达底盘时使用的本机IPv4。
# 本文件只设置当前终端的ROS网络变量，不启动底盘或发送导航目标。
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf '%s\n' '请用 source ./setup_network.bash 加载到当前终端。' >&2
    exit 2
fi

NAV_TEST_PC_IPV4="$(ip -4 route get 192.168.31.200 | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}')"
if [[ ! "$NAV_TEST_PC_IPV4" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    printf '%s\n' '未取得本机IPv4。请先连接底盘网络，并查看 ip -4 route get 192.168.31.200。' >&2
    unset NAV_TEST_PC_IPV4
    return 1
fi

export ROS_MASTER_URI='http://192.168.31.200:11311'
export ROS_IP="$NAV_TEST_PC_IPV4"
# 使用IPv4时移除当前终端的hostname覆盖：两者同时设置时ROS_HOSTNAME优先。
unset ROS_HOSTNAME
printf 'ROS_MASTER_URI=%s\nROS_IP=%s\n' "$ROS_MASTER_URI" "$ROS_IP"
unset NAV_TEST_PC_IPV4
