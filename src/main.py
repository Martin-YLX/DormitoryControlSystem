import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import queue
import serial
import serial.tools.list_ports
import re

DEFAULT_PORT = "None"
DEFAULT_BAUD = 115200

# ===== 全部使用 HEX（二进制帧）======
USE_CMD_HEX = True

# 按钮命令 -> HEX 帧（字符串写法，程序会解析为字节）
CMD_HEX = {
    "DOOR 1":  "AA 55 01 01",
    "DOOR 0":  "AA 55 01 00",
    "LIGHT 1": "AA 55 02 01",
    "LIGHT 0": "AA 55 02 00",
    "AIR 1":   "AA 55 03 01",
    "AIR 0":   "AA 55 03 00",
    "TEP 1":   "AA 55 04 01",
    "TEP 0":   "AA 55 04 00",
}

# 接收触发（无需 ASCII）：出现以下字节序列即弹窗
UMB_SIG = bytes([0xAA, 0x55, 0x05])
KEY_SIG = bytes([0xAA, 0x55, 0x06])

# ========== HEX 工具 ==========
def bytes_to_hex(data: bytes) -> str:
    """b'\xAA\x55\x01' -> 'AA 55 01'"""
    return " ".join(f"{b:02X}" for b in data)

def parse_hex_string(s: str) -> bytes:
    """
    支持：'AA 55 01'、'0xAA 0x55 0x01'、'AA5501'、'aa 55,01' 等。
    空格、逗号分隔或连续偶数字符均可。
    """
    s = s.strip().replace(",", " ")
    if not s:
        return b""
    tokens = [t for t in s.split() if t]

    # 连续HEX串：AA5501
    if len(tokens) == 1:
        hexstr = tokens[0]
        hexstr = re.sub(r"^0x", "", hexstr, flags=re.IGNORECASE)
        if len(hexstr) % 2 != 0:
            raise ValueError("HEX 串长度应为偶数")
        return bytes(int(hexstr[i:i+2], 16) for i in range(0, len(hexstr), 2))

    # 多token：AA 55 01 或 0xAA 0x55 0x01
    out = bytearray()
    for tok in tokens:
        tok = re.sub(r"^0x", "", tok, flags=re.IGNORECASE)
        if not re.fullmatch(r"[0-9a-fA-F]{1,2}", tok):
            raise ValueError(f"非法HEX字节: {tok}")
        out.append(int(tok, 16))
    return bytes(out)

# ========== 串口客户端 ==========
class SerialClient:
    def __init__(self):
        self.ser = None
        self._rx_thread = None
        self._stop = threading.Event()

    def open(self, port, baud, on_bytes_callback):
        """on_bytes_callback: 接收到字节块时回调 callback(bytes)"""
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, args=(on_bytes_callback,), daemon=True)
        self._rx_thread.start()

    def _rx_loop(self, on_bytes_callback):
        while not self._stop.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.05)
                    continue
                n = self.ser.in_waiting
                if n:
                    data = self.ser.read(n)
                    if data:
                        on_bytes_callback(data)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    def close(self):
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_open(self):
        return bool(self.ser and self.ser.is_open)

    def send_bytes(self, data: bytes):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口未连接")
        self.ser.write(data)

# ========== GUI ==========
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STC 串口控制面板")
        self.geometry("720x520")
        self.resizable(False, False)

        self.serial = SerialClient()
        self.rx_q = queue.Queue()     # 接收字节块队列
        self.rx_buf = bytearray()     # 接收环形缓冲，用于查找标志序列

        self._build_conn_frame()
        self._build_controls()
        self._build_hex_send()
        self._build_log()

        self.after(50, self._poll_rx)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ===== 连接区 =====
    def _build_conn_frame(self):
        frm = ttk.LabelFrame(self, text="串口连接")
        frm.pack(fill="x", padx=10, pady=10)

        ttk.Label(frm, text="端口:").grid(row=0, column=0, padx=5, pady=6, sticky="e")
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.port_cb = ttk.Combobox(frm, textvariable=self.port_var, width=28)
        self.port_cb.grid(row=0, column=1, padx=5, pady=6, sticky="w")
        self._refresh_ports()

        ttk.Button(frm, text="刷新端口", command=self._refresh_ports).grid(row=0, column=2, padx=5, pady=6)

        ttk.Label(frm, text="波特率:").grid(row=1, column=0, padx=5, pady=6, sticky="e")
        self.baud_var = tk.IntVar(value=DEFAULT_BAUD)
        self.baud_cb = ttk.Combobox(frm, textvariable=self.baud_var,
                                    values=[9600, 19200, 38400, 57600, 115200, 230400], width=10)
        self.baud_cb.grid(row=1, column=1, padx=5, pady=6, sticky="w")

        self.conn_btn = ttk.Button(frm, text="连接", command=self.toggle_conn)
        self.conn_btn.grid(row=0, column=3, rowspan=2, padx=10, pady=6, sticky="ns")

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(frm, textvariable=self.status_var, foreground="#666").grid(row=0, column=4, rowspan=2, padx=5, pady=6)

        frm.grid_columnconfigure(1, weight=1)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if DEFAULT_PORT not in ports:
            ports.insert(0, DEFAULT_PORT)
        self.port_cb["values"] = ports

    def toggle_conn(self):
        if self.serial.is_open():
            self.serial.close()
            self.status_var.set("未连接")
            self.conn_btn.config(text="连接")
            self._log("[SYS] 串口已断开")
        else:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            try:
                self.serial.open(port, baud, on_bytes_callback=self._on_serial_bytes)
            except Exception as e:
                messagebox.showerror("连接失败", f"无法打开串口 {port} @ {baud}\n{e}")
                self._log(f"[ERR] 打开失败: {e}")
                return
            self.status_var.set(f"已连接 {port} @ {baud}")
            self.conn_btn.config(text="断开")
            self._log(f"[SYS] 已连接: {port} @ {baud}")

    # ===== 控制按钮区 =====
    def _build_controls(self):
        frm = ttk.LabelFrame(self, text="控制面版")
        frm.pack(fill="x", padx=10, pady=5)

        # 行1：门、灯
        row1 = ttk.Frame(frm)
        row1.pack(fill="x", padx=8, pady=6)
        ttk.Label(row1, text="门：").pack(side="left", padx=(0,6))
        ttk.Button(row1, text="开", width=7, command=lambda: self._send_cmd_hex("DOOR 1")).pack(side="left")
        ttk.Button(row1, text="关", width=7, command=lambda: self._send_cmd_hex("DOOR 0")).pack(side="left", padx=(6,18))
        ttk.Label(row1, text="灯：").pack(side="left", padx=(0,6))
        ttk.Button(row1, text="开", width=7, command=lambda: self._send_cmd_hex("LIGHT 1")).pack(side="left")
        ttk.Button(row1, text="关", width=7, command=lambda: self._send_cmd_hex("LIGHT 0")).pack(side="left", padx=(6,0))

        # 行2：空调（开/关）、温度（上/下）
        row2 = ttk.Frame(frm)
        row2.pack(fill="x", padx=8, pady=6)
        ttk.Label(row2, text="空调：").pack(side="left", padx=(0,6))
        ttk.Button(row2, text="开", width=7, command=lambda: self._send_cmd_hex("AIR 1")).pack(side="left")
        ttk.Button(row2, text="关", width=7, command=lambda: self._send_cmd_hex("AIR 0")).pack(side="left", padx=(6,18))
        ttk.Label(row2, text="温度：").pack(side="left", padx=(0,6))
        ttk.Button(row2, text="↑（+）", width=7, command=lambda: self._send_cmd_hex("TEP 1")).pack(side="left")
        ttk.Button(row2, text="↓（-）", width=7, command=lambda: self._send_cmd_hex("TEP 0")).pack(side="left", padx=(6,0))

        # 行3：倒计时（时/分/秒 -> 加标识 07：AA 55 07 h m s）
        row3 = ttk.Frame(frm)
        row3.pack(fill="x", padx=8, pady=6)
        ttk.Label(row3, text="倒计时：").pack(side="left", padx=(0,6))
        v0_255 = (self.register(self._validate_0_255), "%P")
        self.hour_var = tk.StringVar(value="0")
        self.min_var  = tk.StringVar(value="1")
        self.sec_var  = tk.StringVar(value="0")
        ttk.Entry(row3, textvariable=self.hour_var, width=6, validate="key", validatecommand=v0_255).pack(side="left")
        ttk.Label(row3, text="时").pack(side="left", padx=(2,8))
        ttk.Entry(row3, textvariable=self.min_var,  width=6, validate="key", validatecommand=v0_255).pack(side="left")
        ttk.Label(row3, text="分").pack(side="left", padx=(2,8))
        ttk.Entry(row3, textvariable=self.sec_var,  width=6, validate="key", validatecommand=v0_255).pack(side="left")
        ttk.Label(row3, text="秒").pack(side="left", padx=(2,12))
        ttk.Button(row3, text="设定", command=self._send_cnt_hms_hex).pack(side="left", padx=10)

        # 行4：提示（可按需恢复）
        # row4 = ttk.Frame(frm)
        # row4.pack(fill="x", padx=8, pady=6)

    # ===== HEX 发送行 =====
    def _build_hex_send(self):
        frm = ttk.LabelFrame(self, text="HEX 发送")
        frm.pack(fill="x", padx=10, pady=5)

        self.hex_var = tk.StringVar(value="AA 55 01 01")
        ttk.Entry(frm, textvariable=self.hex_var, width=52).pack(side="left", padx=(10,6), pady=6)
        ttk.Button(frm, text="发送", command=self._send_hex).pack(side="left", padx=6)

    # ===== 日志区 =====
    def _build_log(self):
        frm = ttk.LabelFrame(self, text="日志")
        frm.pack(fill="both", expand=True, padx=10, pady=10)
        self.log_text = tk.Text(frm, height=12, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=6, pady=6)
        self.log_text.config(state="disabled")

    def _log(self, msg: str):
        self.log_text.config(state="normal")
        ts = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ===== 校验 =====
    def _validate_0_255(self, P):
        if P == "":
            return True
        if not P.isdigit():
            return False
        v = int(P)
        return 0 <= v <= 255

    # ===== 发送：按钮命令（固定 HEX）=====
    def _send_cmd_hex(self, key: str):
        try:
            if not self.serial.is_open():
                raise RuntimeError("请先连接串口")
            if key not in CMD_HEX:
                raise RuntimeError(f"未配置的命令映射：{key}")
            data = parse_hex_string(CMD_HEX[key])
            self.serial.send_bytes(data)
            self._log(f"[TX HEX] {bytes_to_hex(data)}  ({key})")
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            self._log(f"[ERR] 发送失败: {e}")

    # ===== 发送：CNT（AA 55 07 h m s）=====
    def _send_cnt_hms_hex(self):
        try:
            if not self.serial.is_open():
                raise RuntimeError("请先连接串口")
            h = int(self.hour_var.get() or "0")
            m = int(self.min_var.get() or "0")
            s = int(self.sec_var.get() or "0")
            if not (0 <= h <= 255 and 0 <= m <= 255 and 0 <= s <= 255):
                raise ValueError("时分秒需在 0~255 之间")
            # >>> 修改点：加入 0x07 作为 CNT 标识位 <<<
            data = bytes([0xAA, 0x55, 0x07, h & 0xFF, m & 0xFF, s & 0xFF])
            self.serial.send_bytes(data)
            self._log(f"[TX HEX] {bytes_to_hex(data)}  (COUNT DOWN: h={h} m={m} s={s})")
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            self._log(f"[ERR] 发送失败: {e}")

    # ===== 发送：HEX 输入 =====
    def _send_hex(self):
        s = self.hex_var.get()
        try:
            if not self.serial.is_open():
                raise RuntimeError("请先连接串口")
            data = parse_hex_string(s)
            if not data:
                messagebox.showwarning("提示", "请输入要发送的十六进制字节")
                return
            self.serial.send_bytes(data)
            self._log(f"[TX HEX] {bytes_to_hex(data)}  (HEX手动发送)")
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            self._log(f"[ERR] 发送失败: {e}")

    # ===== 接收处理（字节流 -> HEX 日志 + 查找 UMB/KEY 序列）=====
    def _on_serial_bytes(self, data: bytes):
        self.rx_q.put(data)

    def _poll_rx(self):
        try:
            while True:
                data = self.rx_q.get_nowait()
                # 1) HEX 日志
                self._log(f"[RX HEX] {bytes_to_hex(data)}")

                # 2) 查找触发序列（AA 55 05 / AA 55 06）
                self.rx_buf.extend(data)
                # 限制缓冲大小，避免无限增长
                if len(self.rx_buf) > 1024:
                    self.rx_buf = self.rx_buf[-512:]

                # 搜索并触发（可能跨包）
                self._check_and_popup(self.rx_buf, UMB_SIG, "下位机上报：请带雨伞（UMB）")
                self._check_and_popup(self.rx_buf, KEY_SIG, "下位机上报：请带钥匙（KEY）")
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

    def _check_and_popup(self, buf: bytearray, pattern: bytes, msg: str):
        # 在 buf 中查找 pattern；若找到，弹窗并把该段前部分裁掉以避免重复弹
        idx = self._find_subsequence(buf, pattern)
        if idx != -1:
            messagebox.showinfo("出门提醒", msg)
            # 可选择性清理已匹配段之前的数据，避免重复触发
            del buf[:idx + len(pattern)]

    @staticmethod
    def _find_subsequence(buf: bytearray, pattern: bytes) -> int:
        # 朴素子串搜索（字节）
        plen = len(pattern)
        if plen == 0 or len(buf) < plen:
            return -1
        # 简单优化：查找第一个字节再比对
        first = pattern[0]
        i = 0
        L = len(buf) - plen + 1
        while i < L:
            j = buf.find(first, i, L)
            if j == -1:
                return -1
            if buf[j:j+plen] == pattern:
                return j
            i = j + 1
        return -1

    # ===== 关闭 =====
    def on_close(self):
        try:
            self.serial.close()
        except Exception:
            pass
        self.destroy()

if __name__ == "__main__":
    App().mainloop()