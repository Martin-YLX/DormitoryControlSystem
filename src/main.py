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

# 这里保留了 AA 55 开头；发送时会按“总长6字节”补0或截断
CMD_HEX = {
    "DOOR 1":   "AA 55 01 01",
    "DOOR 0":   "AA 55 01 00",
    "LIGHT 1":  "AA 55 02 01",
    "LIGHT 0":  "AA 55 02 00",
    # 防丢系统（0x04）
    "ANTI 1":   "AA 55 04 01",
    "ANTI 0":   "AA 55 04 00",
    # 护眼模式（0x03）
    "EYE 1":    "AA 55 03 01",
    "EYE 0":    "AA 55 03 00",
}

# 固定帧结构：总长度 6 字节（含 AA 55）
FRAME_HEAD = bytes([0xAA, 0x55])
FRAME_LEN  = 6  # AA 55 + 后续(4 字节)，不足补0，超出截断

# （可选）日志限流：0 表示不开启；>0 表示每收到 N 段原始数据才打印一次 [RX HEX]
LOG_EVERY_N_RX = 0

# ========== HEX 工具 ==========
def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

def parse_hex_string(s: str) -> bytes:
    """
    支持:
      - 以空格/逗号分隔: "AA 55 01 01"
      - 单个连续HEX串: "AA550101" 或 "0xAA550101"
    """
    s = s.strip().replace(",", " ")
    if not s:
        return b""
    tokens = [t for t in s.split() if t]

    if len(tokens) == 1:
        hexstr = tokens[0]
        hexstr = re.sub(r"^0x", "", hexstr, flags=re.IGNORECASE)
        if len(hexstr) % 2 != 0:
            raise ValueError("HEX 串长度应为偶数")
        return bytes(int(hexstr[i:i+2], 16) for i in range(0, len(hexstr), 2))

    out = bytearray()
    for tok in tokens:
        tok = re.sub(r"^0x", "", tok, flags=re.IGNORECASE)
        if not re.fullmatch(r"[0-9a-fA-F]{1,2}", tok):
            raise ValueError(f"非法HEX字节: {tok}")
        out.append(int(tok, 16))
    return bytes(out)

def build_frame_total6(raw: bytes) -> bytes:
    """
    构造“总长度=6字节”的帧：
      - raw 必须以 AA 55 开头
      - 若长度 < 6：在末尾补 0x00 直到 6
      - 若长度 > 6：直接截断到 6
    """
    if len(raw) < 2 or raw[:2] != FRAME_HEAD:
        raise ValueError("帧必须以 AA 55 开头（总长6字节协议）")
    if len(raw) < FRAME_LEN:
        raw = raw + bytes([0x00] * (FRAME_LEN - len(raw)))
    elif len(raw) > FRAME_LEN:
        raw = raw[:FRAME_LEN]
    return raw

# ========== iOS 风格拨动开关（开=灰，关=绿）==========
class ToggleSwitch(tk.Canvas):
    """
    iOS 风格开关：
      - value: 0=关，1=开（仅 UI 值；设备开关逻辑请用 active_high 处理）
      - command(new_value): 切换后回调
    """
    def __init__(self, master, width=58, height=30, padding=2, value=0, command=None):
        parent_bg = self._safe_bg(master)
        super().__init__(master, width=width, height=height,
                         highlightthickness=0, bd=0, bg=parent_bg)
        self.width = width
        self.height = height
        self.pad = padding
        self.radius = (height - 2*padding) / 2      # 轨道圆角半径
        self.command = command
        self._anim_job = None
        self._value = 1 if value else 0
        self._anim_steps = 6
        # 颜色反转：现在 ON=灰(#d0d4da)，OFF=绿(#34c759)
        self._track_on  = "#d0d4da"   # 开=灰
        self._track_off = "#34c759"   # 关=绿
        self._knob_fill = "#ffffff"
        self._shadow    = "#000000"

        self.configure(takefocus=1)
        self.bind("<Button-1>", self._toggle_click)
        self.bind("<Key-Return>", self._toggle_key)
        self.bind("<Key-space>",  self._toggle_key)

        self._draw_static()

    @staticmethod
    def _safe_bg(widget):
        for key in ("background", "bg"):
            try:
                return widget.cget(key)
            except tk.TclError:
                pass
        try:
            top = widget.winfo_toplevel()
            for key in ("background", "bg"):
                try:
                    return top.cget(key)
                except tk.TclError:
                    pass
        except Exception:
            pass
        return "SystemButtonFace"

    def _draw_static(self):
        self.delete("all")
        # 轨道（圆角矩形）
        self._track = self._round_rect(self.pad, self.pad,
                                       self.width - self.pad, self.height - self.pad,
                                       radius=self.radius, fill=self._color_for_value(self._value), outline="")
        # knob 与轨道同弧度（半径一致）
        x0, y0, x1, y1 = self._knob_bbox_for_value(self._value)
        self._shadow_item = self.create_oval(x0+1, y0+2, x1+1, y1+3, fill=self._shadow, outline="", stipple="gray50")
        self._knob = self.create_oval(x0, y0, x1, y1, fill=self._knob_fill, outline="#e0e0e0")

    def _round_rect(self, x1, y1, x2, y2, radius=10, **kwargs):
        points = [
            x1+radius, y1,
            x2-radius, y1,
            x2, y1,
            x2, y1+radius,
            x2, y2-radius,
            x2, y2,
            x2-radius, y2,
            x1+radius, y2,
            x1, y2,
            x1, y2-radius,
            x1, y1+radius,
            x1, y1,
            ]
        return self.create_polygon(points, smooth=True, **kwargs)

    def _color_for_value(self, val):
        return self._track_on if val == 1 else self._track_off

    def _knob_bbox_for_value(self, val):
        # knob 半径 = 轨道圆角半径（弧度完全一致）
        r = self.radius
        left_x  = self.pad + r
        right_x = self.width - self.pad - r
        cx = right_x if val == 1 else left_x
        cy = self.height / 2
        return (cx - r, cy - r, cx + r, cy + r)

    def value(self):
        return self._value

    def set(self, val, animate=False):
        val = 1 if val else 0
        if val == self._value:
            return
        if animate:
            self._animate_to(val)
        else:
            self._value = val
            self._draw_static()

    def toggle(self):
        self.set(0 if self._value == 1 else 1, animate=True)
        if callable(self.command):
            self.command(self._value)

    def _toggle_click(self, _):
        self.focus_set()
        self.toggle()

    def _toggle_key(self, _):
        self.toggle()

    def _animate_to(self, target):
        if self._anim_job:
            self.after_cancel(self._anim_job)
            self._anim_job = None
        steps = self._anim_steps

        def lerp(a, b, t): return a + (b - a) * t

        lx0, ly0, lx1, ly1 = self._knob_bbox_for_value(0)
        rx0, ry0, rx1, ry1 = self._knob_bbox_for_value(1)

        def step(i=0):
            t = i / steps
            self.itemconfig(self._track, fill=self._color_for_value(target))
            if target == 1:
                x0 = lerp(lx0, rx0, t); y0 = lerp(ly0, ry0, t)
                x1 = lerp(lx1, rx1, t); y1 = lerp(ly1, ry1, t)
            else:
                x0 = lerp(rx0, lx0, t); y0 = lerp(ry0, ly0, t)
                x1 = lerp(rx1, lx1, t); y1 = lerp(ry1, ly1, t)
            self.coords(self._shadow_item, x0+1, y0+2, x1+1, y1+3)
            self.coords(self._knob, x0, y0, x1, y1)

            if i < steps:
                self._anim_job = self.after(12, lambda: step(i+1))
            else:
                self._value = target
                self._anim_job = None

        step()

# ========== 串口客户端 ==========
class SerialClient:
    def __init__(self):
        self.ser = None
        self._rx_thread = None
        self._stop = threading.Event()

    def open(self, port, baud, on_bytes_callback):
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
        self.geometry("760x700")
        self.resizable(False, False)

        self.serial = SerialClient()
        self.rx_q = queue.Queue()
        self.rx_buf = bytearray()

        self._rx_chunk_counter = 0

        self._build_conn_frame()
        self._build_controls()   # 使用 iOS 风格开关
        self._build_hex_send()
        self._build_log()

        self.after(50, self._poll_rx)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # 连接
    def _build_conn_frame(self):
        frm = ttk.LabelFrame(self, text="串口连接")
        frm.pack(fill="x", padx=10, pady=8)

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

    # 控制（门 / 灯 / 防丢 / 护眼 使用 iOS 风格拨动开关）
    def _build_controls(self):
        frm = ttk.LabelFrame(self, text="控制面板")
        frm.pack(fill="x", padx=10, pady=6)

        def make_switch_row(parent, title, on_key, off_key, default_on=False, active_high=True):
            """
            active_high:
              True  = UI的1代表设备“开”，发送 on_key
              False = UI的0代表设备“开”（逻辑反向）
            """
            row = ttk.Frame(parent)
            row.pack(fill="x", padx=8, pady=12)

            ttk.Label(row, text=f"{title}：").pack(side="left", padx=(0, 8))

            ui_val = 1 if default_on else 0
            device_is_on = (ui_val == 1) if active_high else (ui_val == 0)
            state_var = tk.StringVar(value="当前：开" if device_is_on else "当前：关")

            sw = ToggleSwitch(row, width=58, height=30, value=ui_val,
                              command=lambda v, ah=active_high, sv=state_var: self._on_ios_switch(v, on_key, off_key, ah, sv))
            sw.pack(side="left")

            ttk.Label(row, textvariable=state_var, foreground="#555").pack(side="left", padx=(12,0))
            return sw

        self.door_switch  = make_switch_row(frm, "门",  "DOOR 1",  "DOOR 0", default_on=False, active_high=True)
        self.light_switch = make_switch_row(frm, "灯", "LIGHT 1", "LIGHT 0", default_on=False, active_high=True)
        self.anti_switch  = make_switch_row(frm, "防忘带系统", "ANTI 1", "ANTI 0", default_on=False, active_high=True)
        self.eye_switch   = make_switch_row(frm, "护眼模式", "EYE 1", "EYE 0", default_on=False, active_high=True)

    def _on_ios_switch(self, ui_val, on_key, off_key, active_high=True, state_var=None):
        """
        ui_val: 0/1（UI 的值）
        active_high: True=UI的1表示设备开；False=UI的0表示设备开（反逻辑）
        """
        try:
            ui_is_on = bool(ui_val)          # UI 是否处于右侧（1）
            device_is_on = ui_is_on if active_high else (not ui_is_on)
            key = on_key if device_is_on else off_key
            self._send_cmd_hex(key)
            if state_var is not None:
                state_var.set("当前：开" if device_is_on else "当前：关")
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            self._log(f"[ERR] 开关发送失败: {e}")

    # HEX发送
    def _build_hex_send(self):
        frm = ttk.LabelFrame(self, text="HEX 发送")
        frm.pack(fill="x", padx=10, pady=6)

        self.hex_var = tk.StringVar(value="AA 55 01 01")
        ttk.Entry(frm, textvariable=self.hex_var, width=52).pack(side="left", padx=(10,6), pady=6)
        ttk.Button(frm, text="发送", command=self._send_hex).pack(side="left", padx=6)

    # 日志
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

    # 校验
    def _validate_0_255(self, P):
        if P == "":
            return True
        if not P.isdigit():
            return False
        v = int(P)
        return 0 <= v <= 255

    # 发送控制（固定命令）
    def _send_cmd_hex(self, key: str):
        try:
            if not self.serial.is_open():
                raise RuntimeError("请先连接串口")
            if key not in CMD_HEX:
                raise RuntimeError(f"未配置的命令映射：{key}")
            raw = parse_hex_string(CMD_HEX[key])
            data = build_frame_total6(raw)
            self.serial.send_bytes(data)
            self._log(f"[TX HEX] {bytes_to_hex(data)}  ({key})")
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            self._log(f"[ERR] 发送失败: {e}")

    # 手动 HEX 发送（要求输入必须包含 AA 55 开头；其余自动补0/截断至总长6）
    def _send_hex(self):
        s = self.hex_var.get()
        try:
            if not self.serial.is_open():
                raise RuntimeError("请先连接串口")
            raw = parse_hex_string(s)
            data = build_frame_total6(raw)
            self.serial.send_bytes(data)
            self._log(f"[TX HEX] {bytes_to_hex(data)}  (HEX手动发送)")
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            self._log(f"[ERR] 发送失败: {e}")

    # 串口接收回调（在IO线程）
    def _on_serial_bytes(self, data: bytes):
        self.rx_q.put(data)

    # GUI线程轮询
    def _poll_rx(self):
        try:
            while True:
                data = self.rx_q.get_nowait()

                # （可选）日志限流
                if LOG_EVERY_N_RX and LOG_EVERY_N_RX > 0:
                    self._rx_chunk_counter += 1
                    if self._rx_chunk_counter % LOG_EVERY_N_RX == 0:
                        self._log(f"[RX HEX] {bytes_to_hex(data)}  (每{LOG_EVERY_N_RX}条打印一次)")
                else:
                    self._log(f"[RX HEX] {bytes_to_hex(data)}")

                self.rx_buf.extend(data)
                self._extract_and_handle_frames()
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

    def _extract_and_handle_frames(self):
        """
        固定长度帧解析：总长 6 字节（AA 55 + 4）
        仅记录帧，不做 UMB/KEY/CNT 相关处理（已移除）
        """
        while True:
            start = self.rx_buf.find(FRAME_HEAD)
            if start == -1:
                self.rx_buf.clear()
                return

            if start > 0:
                del self.rx_buf[:start]

            if len(self.rx_buf) < FRAME_LEN:
                return  # 等待更多数据

            frame = bytes(self.rx_buf[:FRAME_LEN])
            del self.rx_buf[:FRAME_LEN]

            self._log(f"[RX FRAME] {bytes_to_hex(frame)}")
            # 不再区分 code；全部当作普通帧记录

    # 关闭
    def on_close(self):
        try:
            self.serial.close()
        except Exception:
            pass
        self.destroy()

if __name__ == "__main__":
    App().mainloop()