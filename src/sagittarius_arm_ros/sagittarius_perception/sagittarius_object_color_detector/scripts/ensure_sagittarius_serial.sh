#!/usr/bin/env bash
set -euo pipefail

SERIAL_LINK="${SERIAL_LINK:-/dev/sagittarius}"
TTY_DEVICE="${TTY_DEVICE:-}"
VENDOR_ID="${VENDOR_ID:-2e88}"
PRODUCT_ID="${PRODUCT_ID:-4603}"

log() {
  printf '[sagittarius-serial] %s\n' "$*"
}

fail() {
  log "ERROR: $*"
  exit 1
}

find_tty_from_sysfs() {
  local tty_path
  tty_path="$(find /sys/class/tty -maxdepth 1 -name "ttyACM*" -print 2>/dev/null | head -n 1 || true)"
  if [[ -n "$tty_path" ]]; then
    basename "$tty_path"
    return 0
  fi

  tty_path="$(find /sys/bus/usb/devices -path "*/tty/ttyACM*" -print 2>/dev/null | head -n 1 || true)"
  if [[ -n "$tty_path" ]]; then
    basename "$tty_path"
    return 0
  fi
  return 1
}

find_usb_device() {
  local dev
  for dev in /sys/bus/usb/devices/*; do
    [[ -f "$dev/idVendor" && -f "$dev/idProduct" ]] || continue
    if [[ "$(cat "$dev/idVendor")" == "$VENDOR_ID" && "$(cat "$dev/idProduct")" == "$PRODUCT_ID" ]]; then
      echo "$dev"
      return 0
    fi
  done
  return 1
}

create_tty_node_if_needed() {
  local tty_name="$1"
  local sys_tty="/sys/class/tty/${tty_name}"
  local dev_file="/dev/${tty_name}"
  local major_minor major minor

  [[ -e "$sys_tty/dev" ]] || return 0
  [[ ! -e "$dev_file" ]] || return 0

  major_minor="$(cat "$sys_tty/dev")"
  major="${major_minor%%:*}"
  minor="${major_minor##*:}"

  log "内核已经识别 ${tty_name} (${major}:${minor})，但 ${dev_file} 不存在。"
  log "尝试用 sudo 创建设备节点。"
  if command -v sudo >/dev/null 2>&1; then
    if ! sudo mknod "$dev_file" c "$major" "$minor"; then
      log "sudo mknod 执行失败。"
      log "如果错误是 'sudo must be owned by uid 0'，请在 Windows PowerShell 执行："
      log "  wsl -d Ubuntu-20.04 -u root -- bash -lc \"chown root:root /usr/bin/sudo /bin/su && chmod 4755 /usr/bin/sudo /bin/su\""
      log "然后回到 Ubuntu 重新运行本脚本。"
      fail "无法创建 ${dev_file}"
    fi
    sudo chmod 666 "$dev_file"
  else
    fail "系统没有 sudo，请用 root 执行：mknod ${dev_file} c ${major} ${minor} && chmod 666 ${dev_file}"
  fi
}

main() {
  local usb_device tty_name dev_file

  log "检查 Sagittarius 机械臂串口设备..."

  if usb_device="$(find_usb_device)"; then
    log "找到 USB 设备: ${usb_device} (${VENDOR_ID}:${PRODUCT_ID})"
  else
    fail "没有在 WSL 中找到 HDSC Sagittarius USB 设备。请先在 Windows PowerShell 中执行 usbipd attach --wsl --busid <BUSID>。"
  fi

  if [[ -n "$TTY_DEVICE" ]]; then
    tty_name="$(basename "$TTY_DEVICE")"
  elif tty_name="$(find_tty_from_sysfs)"; then
    :
  elif [[ -e /dev/ttyACM0 ]]; then
    tty_name="ttyACM0"
  else
    log "USB 设备存在，但没有找到 ttyACM 串口。"
    log "建议在 Windows PowerShell 中 detach 后重新 attach 机械臂，或重启 WSL。"
    fail "未生成 ttyACM 串口节点"
  fi

  create_tty_node_if_needed "$tty_name"

  dev_file="/dev/${tty_name}"
  [[ -e "$dev_file" ]] || fail "${dev_file} 仍不存在，无法继续。"

  if [[ -L "$SERIAL_LINK" || -e "$SERIAL_LINK" ]]; then
    if [[ "$(readlink -f "$SERIAL_LINK" 2>/dev/null || true)" == "$dev_file" ]]; then
      log "${SERIAL_LINK} 已经指向 ${dev_file}"
    else
      log "${SERIAL_LINK} 已存在但不是目标串口，尝试更新。"
      sudo rm -f "$SERIAL_LINK"
      sudo ln -s "$dev_file" "$SERIAL_LINK"
    fi
  else
    log "创建兼容链接 ${SERIAL_LINK} -> ${dev_file}"
    sudo ln -s "$dev_file" "$SERIAL_LINK"
  fi

  log "最终设备状态："
  ls -l "$dev_file" "$SERIAL_LINK"
  log "串口准备完成。现在可以启动 demo_true 或 language_guided_grasp。"
}

main "$@"
