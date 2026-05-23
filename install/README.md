# 开机自启安装

把 `bof_26_vision` 设为登录后自动启动，崩溃自动重启，等串口设备就绪再起。

## 安装

```bash
./install/install_autostart.sh
```

会在 `~/.config/autostart/bof_vision.desktop` 写一个 `.desktop` 文件，里面 `Exec=` 指向当前仓库的 `startup.sh`。重启后生效。

## 卸载

```bash
./install/uninstall_autostart.sh
```

## 立即测试不重启

```bash
./startup.sh
tail -f logs/$(ls -t logs/*.log | head -1)
```

应该看到：

```
[xxx] launched pid=NNNN
[xxx] waiting for /dev/ttyACM0
[xxx] /dev/ttyACM0 ready
[xxx] starting .../auto_aim_debug_aimer
[Main] Recorder (video+quat) enabled.
... yolo / tracker / aimer 信号
```

## 文件清单

- `bof_vision.desktop.in` — `.desktop` 模板，安装时把 `@SCRIPT_DIR@` 替换为仓库绝对路径。
- `install_autostart.sh` — 安装脚本（仅写一个文件，不动其他系统配置）。
- `uninstall_autostart.sh` — 卸载脚本。

## 切机器要做什么

新机器：

1. clone 仓库 + build
2. `./install/install_autostart.sh`
3. 检查 `configs/<robot>.yaml` 的 `com_port` 跟实际串口一致
4. 如果默认配置不是 `standard4.yaml`，编辑 `startup.sh` 第 13 行的 `CONFIG=` 路径

仓库换路径（比如从 `~/Documents/bof_26_vision` 移到 `~/work/bof_26_vision`）：

- 重新跑一次 `./install/install_autostart.sh`，会更新 desktop 里的绝对路径

## 排错

```bash
# 看自启项是否注册
ls -la ~/.config/autostart/bof_vision.desktop

# 手动跑一次模拟自启
./startup.sh && tail logs/$(ls -t logs/*.log | head -1)

# 看自启进程是否在
pgrep -af "auto_aim_debug_aimer|startup.sh"

# 强制停掉
kill $(cat logs/bof_vision.pid 2>/dev/null) 2>/dev/null
pkill -f auto_aim_debug_aimer
rm -f logs/bof_vision.pid
```
