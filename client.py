import serial
import serial.tools.list_ports
import time
import threading
import sys
import struct
import os

# 尝试导入 colorama 以支持 Windows 颜色显示
try:
    from colorama import init, Fore, Back, Style

    init(autoreset=True)
except ImportError:
    # 如果没安装，提供无颜色的回退类
    class DummyColor:
        def __getattr__(self, name): return ""


    Fore = Back = Style = DummyColor()

try:
    import msvcrt  # 仅限 Windows 键盘输入
except ImportError:
    print("此脚本优化了 Windows 体验，Linux/Mac 下按键可能需要回车。")

# ================= 配置参数 =================
TARGET_BAUDRATE = 921600
ENCODER_COUNT = 21
CMD_CALIBRATE = b'\xCA'  # 校准触发指令
ERROR_VAL_FLAG = 0xFFFF  # 固件中定义的错误标记


# ================= 状态管理类 =================
class MachineState:
    def __init__(self):
        self.angles = [0] * ENCODER_COUNT
        self.errors = [False] * ENCODER_COUNT

        # 状态: IDLE, PENDING, SUCCESS, FAILED
        self.calib_status = "IDLE"
        self.calib_timestamp = 0

        self.last_update = 0
        self.running = True
        self.connected_port = None
        self.ser = None


state = MachineState()


# ================= 协议处理逻辑 =================
def process_data_packet(payload):
    """ 解析传感器数据包 (Type 0x01) """
    if len(payload) != ENCODER_COUNT * 2:
        return

    for i in range(ENCODER_COUNT):
        # 大端序解析两个字节
        val = (payload[i * 2] << 8) | payload[i * 2 + 1]

        if val == ERROR_VAL_FLAG:
            state.errors[i] = True
            state.angles[i] = 0
        else:
            state.errors[i] = False
            state.angles[i] = val

    state.last_update = time.time()


def process_ack_packet(payload):
    """ 解析校准反馈包 (Type 0x02) """
    if len(payload) < 1: return
    code = payload[0]

    if code == 1:
        state.calib_status = "PENDING"
    elif code == 2:
        state.calib_status = "SUCCESS"
        state.calib_timestamp = time.time()
    elif code == 3:
        state.calib_status = "FAILED"
        state.calib_timestamp = time.time()


def parse_stream(buffer):
    """ 从字节流中提取完整数据帧 """
    # 协议格式: [FE] [LEN] [TYPE] [PAYLOAD...] [FF]
    while len(buffer) >= 4:  # 最小帧长
        # 1. 寻找帧头 0xFE
        try:
            head_idx = buffer.index(b'\xFE')
        except ValueError:
            return bytearray()  # 丢弃无头数据

        # 丢弃帧头前的垃圾数据
        if head_idx > 0:
            buffer = buffer[head_idx:]

        # 2. 检查长度是否存在
        if len(buffer) < 2: break
        payload_len = buffer[1]
        total_len = payload_len + 2  # Header(1) + Len(1) + Payload(L) -> Frame excluding Tail?
        # 根据我们设计的协议: Header(1)+Len(1)+Body(N)+Tail(1)
        # 之前的 UpperCommTask.cpp 代码: buffer[1] = idx - 2 (Body + Tail)
        # 所以 Total Frame Bytes = 2 (Head+Len) + Body + Tail = 2 + buffer[1]

        frame_len = 2 + payload_len

        if len(buffer) >= frame_len:
            frame = buffer[:frame_len]
            # 3. 验证帧尾
            if frame[-1] == 0xFF:
                pkt_type = frame[2]
                payload = frame[3:-1]

                if pkt_type == 0x01:
                    process_data_packet(payload)
                elif pkt_type == 0x02:
                    process_ack_packet(payload)

            # 移除已处理帧
            buffer = buffer[frame_len:]
        else:
            break  # 数据未收全，等待下一次

    return buffer


# ================= 串口通信线程 =================
def serial_thread_func(port_name):
    try:
        # 打开串口
        ser = serial.Serial(port_name, TARGET_BAUDRATE, timeout=0.1)
        state.ser = ser
        state.connected_port = port_name

        buffer = bytearray()

        while state.running:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)
                buffer = parse_stream(buffer)
            else:
                time.sleep(0.005)

    except Exception as e:
        state.connected_port = None
        state.running = False
        print(f"\n串口错误: {e}")


# ================= 界面绘制 (UI) =================
def print_ui():
    """ 绘制无闪烁的高可读性界面 """
    # ====== 【关键修改】强制清屏 ======
    # 先用 \033[2J 清除整个屏幕，再用 \033[H 回到顶部
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()

    # --- 标题栏 ---
    print(f"{Back.BLUE}{Fore.WHITE}  XSimple S3 关节模组 - 总线监控终端  {Style.RESET_ALL}")

    # --- 状态栏 ---
    if state.connected_port:
        port_info = f"{Fore.GREEN}● 已连接: {state.connected_port}{Style.RESET_ALL}"
    else:
        port_info = f"{Fore.RED}● 未连接{Style.RESET_ALL}"

    latency = (time.time() - state.last_update) * 1000
    fps_color = Fore.GREEN if latency < 100 else Fore.YELLOW if latency < 500 else Fore.RED
    print(f"状态: {port_info}  |  延迟: {fps_color}{latency:.0f} ms{Style.RESET_ALL}")
    print("-" * 65)

    # --- 编码器矩阵 ---
    print(f"{Style.BRIGHT}编码器实时数据 (RAW | 角度):{Style.RESET_ALL}")

    rows = (ENCODER_COUNT + 2) // 3
    for r in range(rows):
        line_str = ""
        for c in range(3):
            idx = r + c * rows
            if idx < ENCODER_COUNT:
                raw = state.angles[idx]
                deg = raw * 360.0 / 16384.0
                is_err = state.errors[idx]

                # 格式化单个单元格
                id_str = f"[{idx:02d}]"
                if is_err:
                    val_str = f"{Back.RED}{Fore.WHITE} DISCONNECT {Style.RESET_ALL}"
                else:
                    # 正常数据，根据角度区分颜色方便观察变化
                    color = Fore.CYAN if idx % 2 == 0 else Fore.WHITE
                    val_str = f"{color}{raw:05d} ({deg:6.2f}°){Style.RESET_ALL}"

                line_str += f"{id_str} {val_str}   "
        print(line_str)

    print("-" * 65)

    # --- 校准控制区 ---
    print(f"{Back.MAGENTA}{Fore.WHITE}  校准操作区  {Style.RESET_ALL}")
    print(f"操作指南: 按键盘 {Fore.YELLOW}'c'{Style.RESET_ALL} 键触发机械零点校准")

    # 状态反馈逻辑
    msg = ""
    elapsed = time.time() - state.calib_timestamp

    if state.calib_status == "IDLE":
        msg = f"{Fore.WHITE}等待指令...{Style.RESET_ALL}"

    elif state.calib_status == "PENDING":
        msg = f"{Fore.YELLOW}⏳ 正在发送校准请求，请稍候...{Style.RESET_ALL}"

    elif state.calib_status == "SUCCESS":
        # 成功消息保持显示 5 秒
        if elapsed < 5.0:
            msg = f"{Back.GREEN}{Fore.WHITE} ✅  校准成功！参数已保存至 EEPROM  {Style.RESET_ALL}"
        else:
            state.calib_status = "IDLE"

    elif state.calib_status == "FAILED":
        # 失败消息保持显示 5 秒
        if elapsed < 5.0:
            msg = f"{Back.RED}{Fore.WHITE} ❌  校准失败！(超时或硬件错误)  {Style.RESET_ALL}"
        else:
            state.calib_status = "IDLE"

    # 清除行尾多余字符，防止残留
    print(f"\n当前状态: {msg}\033[K")
    print(f"\n{Fore.BLACK}{Style.DIM}(按 'q' 退出程序){Style.RESET_ALL}")


# ================= 主程序 =================
def main():
    # === 【新增】动态端口选择循环 ===
    target_port = None
    while not target_port:
        os.system('cls' if os.name == 'nt' else 'clear')
        print("🔍 正在扫描可用串口...")

        ports = serial.tools.list_ports.comports()
        valid_ports = [p.device for p in ports]

        if not valid_ports:
            print(f"{Fore.RED}❌ 未检测到任何串口设备！请检查 USB 连接。\n{Style.RESET_ALL}")
            print("按 Enter 键重试，或按 Ctrl+C 退出...")
            try:
                input()  # 等待用户按键
            except KeyboardInterrupt:
                return
            continue

        # 打印所有可用端口供用户选择
        print(f"\n{Fore.CYAN}可用串口列表:{Style.RESET_ALL}")
        for i, port in enumerate(valid_ports):
            print(f"  [{i}] {port}  (描述: {ports[i].description})")

        # 提示用户选择
        try:
            choice = input(f"\n请输入端口号编号 [{0}~{len(valid_ports) - 1}], 或直接回车选择默认第0个: ").strip()
            if choice == "":
                target_port = valid_ports[0]
            else:
                idx = int(choice)
                if 0 <= idx < len(valid_ports):
                    target_port = valid_ports[idx]
                else:
                    print(f"{Fore.YELLOW}⚠️  输入无效，请输入有效编号。{Style.RESET_ALL}")
                    time.sleep(1)
                    continue
        except ValueError:
            print(f"{Fore.YELLOW}⚠️  输入非数字，请重试。{Style.RESET_ALL}")
            time.sleep(1)
            continue
        except KeyboardInterrupt:
            return

    # === 【关键】连接成功后，启动线程 ===
    print(f"\n✅ 正在连接到 {target_port} ...")
    t = threading.Thread(target=serial_thread_func, args=(target_port,), daemon=True)
    t.start()

    # 3. 主循环
    try:
        while state.running:
            # 绘制界面
            print_ui()

            # 键盘输入检测 (Windows)
            if os.name == 'nt' and msvcrt.kbhit():
                key = msvcrt.getch().lower()
                if key == b'q':
                    state.running = False
                elif key == b'c':
                    if state.ser and state.ser.is_open:
                        state.ser.write(CMD_CALIBRATE)
                        state.calib_timestamp = time.time()  # 避免闪烁
                        # 状态变为等待，直到收到 Type 0x02 的包
                        state.calib_status = "PENDING"

                        # 刷新率控制 (15 FPS)
            time.sleep(0.06)

    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        print("\n\n程序已退出。")


if __name__ == "__main__":
    main()