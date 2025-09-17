import tkinter as tk
from tkinter import ttk, messagebox
import threading, time, queue, serial, serial.tools.list_ports, re

CLASS_NAME   = "智能23xx班"
STUDENT_NAME = "组长：xxx   组员：xxx"

DEFAULT_PORT = "None"
DEFAULT_BAUD = 115200

CMD_HEX = {
    "DOOR 1":   "AA 55 01 01",
    "DOOR 0":   "AA 55 01 00",
    "LIGHT 1":  "AA 55 02 01",
    "LIGHT 0":  "AA 55 02 00",
    "EYE 1":    "AA 55 03 01",
    "EYE 0":    "AA 55 03 00",
    "ANTI 1":   "AA 55 04 01",
    "ANTI 0":   "AA 55 04 00",
}

FRAME_HEAD = bytes([0xAA, 0x55])
FRAME_LEN  = 6
LOG_EVERY_N_RX = 0

def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

def parse_hex_string(s: str) -> bytes:
    s = s.strip().replace(",", " ")
    if not s: return b""
    toks = [t for t in s.split() if t]
    if len(toks) == 1:
        hexstr = re.sub(r"^0x", "", toks[0], flags=re.IGNORECASE)
        if len(hexstr) % 2 != 0: raise ValueError("HEX 串长度应为偶数")
        return bytes(int(hexstr[i:i+2],16) for i in range(0,len(hexstr),2))
    out = bytearray()
    for t in toks:
        t = re.sub(r"^0x","",t,flags=re.IGNORECASE)
        if not re.fullmatch(r"[0-9a-fA-F]{1,2}", t): raise ValueError(f"非法HEX字节: {t}")
        out.append(int(t,16))
    return bytes(out)

def build_frame_total6(raw: bytes) -> bytes:
    if len(raw)<2 or raw[:2]!=FRAME_HEAD:
        raise ValueError("帧必须以 AA 55 开头（总长6字节协议）")
    if len(raw)<FRAME_LEN: raw = raw + bytes([0x00]*(FRAME_LEN-len(raw)))
    elif len(raw)>FRAME_LEN: raw = raw[:FRAME_LEN]
    return raw

class ToggleSwitch(tk.Canvas):
    def __init__(self, master, width=58, height=30, padding=2, value=0, command=None):
        super().__init__(master, width=width, height=height, highlightthickness=0, bd=0,
                         bg=self._safe_bg(master))
        self.width, self.height, self.pad = width, height, padding
        self.radius = (height-2*padding)/2
        self.command = command
        self._anim_job = None
        self._value = 1 if value else 0
        self._anim_steps = 6
        self._track_on, self._track_off = "#d0d4da", "#34c759"
        self._knob_fill, self._shadow = "#ffffff", "#000000"
        self.configure(takefocus=1)
        self.bind("<Button-1>", self._toggle_click)
        self.bind("<Key-Return>", self._toggle_key)
        self.bind("<Key-space>",  self._toggle_key)
        self._draw_static()

    @staticmethod
    def _safe_bg(w):
        for k in ("background","bg"):
            try: return w.cget(k)
            except tk.TclError: pass
        try:
            top=w.winfo_toplevel()
            for k in ("background","bg"):
                try: return top.cget(k)
                except tk.TclError: pass
        except: pass
        return "SystemButtonFace"

    def _round_rect(self,x1,y1,x2,y2,radius=10,**kw):
        pts=[x1+radius,y1, x2-radius,y1, x2,y1, x2,y1+radius, x2,y2-radius, x2,y2,
             x2-radius,y2, x1+radius,y2, x1,y2, x1,y2-radius, x1,y1+radius, x1,y1]
        return self.create_polygon(pts, smooth=True, **kw)

    def _color_for_value(self,val): return self._track_on if val==1 else self._track_off

    def _knob_bbox_for_value(self,val):
        r=self.radius; left=self.pad+r; right=self.width-self.pad-r
        cx = right if val==1 else left; cy=self.height/2
        return (cx-r,cy-r,cx+r,cy+r)

    def _draw_static(self):
        self.delete("all")
        self._track = self._round_rect(self.pad,self.pad,self.width-self.pad,self.height-self.pad,
                                       radius=self.radius, fill=self._color_for_value(self._value), outline="")
        x0,y0,x1,y1 = self._knob_bbox_for_value(self._value)
        self._shadow_item = self.create_oval(x0+1,y0+2,x1+1,y1+3, fill=self._shadow, outline="", stipple="gray50")
        self._knob = self.create_oval(x0,y0,x1,y1, fill=self._knob_fill, outline="#e0e0e0")

    def set(self,val,animate=False):
        val=1 if val else 0
        if val==self._value: return
        if animate: self._animate_to(val)
        else: self._value=val; self._draw_static()

    def toggle(self):
        self.set(0 if self._value==1 else 1, animate=True)
        if callable(self.command): self.command(self._value)

    def _toggle_click(self,_): self.focus_set(); self.toggle()
    def _toggle_key(self,_): self.toggle()

    def _animate_to(self,target):
        if self._anim_job: self.after_cancel(self._anim_job); self._anim_job=None
        steps=self._anim_steps
        def lerp(a,b,t): return a+(b-a)*t
        lx0,ly0,lx1,ly1 = self._knob_bbox_for_value(0)
        rx0,ry0,rx1,ry1 = self._knob_bbox_for_value(1)
        def step(i=0):
            t=i/steps; self.itemconfig(self._track, fill=self._color_for_value(target))
            if target==1: x0=lerp(lx0,rx0,t); y0=lerp(ly0,ry0,t); x1=lerp(lx1,rx1,t); y1=lerp(ly1,ry1,t)
            else:         x0=lerp(rx0,lx0,t); y0=lerp(ry0,ly0,t); x1=lerp(rx1,lx1,t); y1=lerp(ry1,ly1,t)
            self.coords(self._shadow_item,x0+1,y0+2,x1+1,y1+3); self.coords(self._knob,x0,y0,x1,y1)
            if i<steps: self._anim_job=self.after(12, lambda: step(i+1))
            else: self._value=target; self._anim_job=None
        step()

class SerialClient:
    def __init__(self):
        self.ser=None; self._rx_thread=None; self._stop=threading.Event()
    def open(self,port,baud,on_bytes_callback):
        if self.ser and self.ser.is_open: return
        self.ser=serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self._stop.clear()
        self._rx_thread=threading.Thread(target=self._rx_loop, args=(on_bytes_callback,), daemon=True); self._rx_thread.start()
    def _rx_loop(self,on_bytes_callback):
        while not self._stop.is_set():
            try:
                if not self.ser or not self.ser.is_open: time.sleep(0.05); continue
                n=self.ser.in_waiting
                if n: data=self.ser.read(n);
                else: time.sleep(0.01); continue
                if data: on_bytes_callback(data)
            except Exception: time.sleep(0.05)
    def close(self):
        self._stop.set()
        if self.ser:
            try: self.ser.close()
            except Exception: pass
        self.ser=None
    def is_open(self): return bool(self.ser and self.ser.is_open)
    def send_bytes(self,data:bytes):
        if not self.ser or not self.ser.is_open: raise RuntimeError("串口未连接")
        self.ser.write(data)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STC 串口控制面板")
        self.geometry("760x560")
        self.resizable(False, False)

        # 先隐藏主窗体；等开机动画结束后再显示并构建面板
        self.withdraw()
        self.after(50, lambda: self._show_splash(on_done=self._build_main_ui))

    # —— 在这里统一构建主界面（开机动画结束后调用）——
    def _build_main_ui(self):
        self.serial = SerialClient()
        self.rx_q = queue.Queue()
        self.rx_buf = bytearray()
        self._rx_chunk_counter = 0

        self._build_top_header()
        self._build_conn_frame()
        self._build_controls()
        self._build_hex_send()
        self._build_log()

        self.after(50, self._poll_rx)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.deiconify()  # 显示主窗体

    def _build_top_header(self):
        top=ttk.Frame(self); top.pack(fill="x", padx=10, pady=(8,0))
        # ttk.Label(top, text=f"{CLASS_NAME}  —  {STUDENT_NAME}",
        #           font=("Segoe UI",14,"bold"), foreground="#444").pack(side="left")

    def _build_conn_frame(self):
        frm=ttk.LabelFrame(self,text="串口连接"); frm.pack(fill="x", padx=10, pady=8)
        ttk.Label(frm,text="端口:").grid(row=0,column=0,padx=5,pady=6,sticky="e")
        self.port_var=tk.StringVar(value=DEFAULT_PORT)
        self.port_cb=ttk.Combobox(frm,textvariable=self.port_var,width=28); self.port_cb.grid(row=0,column=1,padx=5,pady=6,sticky="w")
        self._refresh_ports()
        ttk.Button(frm,text="刷新端口",command=self._refresh_ports).grid(row=0,column=2,padx=5,pady=6)
        ttk.Label(frm,text="波特率:").grid(row=1,column=0,padx=5,pady=6,sticky="e")
        self.baud_var=tk.IntVar(value=DEFAULT_BAUD)
        self.baud_cb=ttk.Combobox(frm,textvariable=self.baud_var,values=[9600,19200,38400,57600,115200,230400],width=10)
        self.baud_cb.grid(row=1,column=1,padx=5,pady=6,sticky="w")
        self.conn_btn=ttk.Button(frm,text="连接",command=self.toggle_conn); self.conn_btn.grid(row=0,column=3,rowspan=2,padx=10,pady=6,sticky="ns")
        self.status_var=tk.StringVar(value="未连接")
        ttk.Label(frm,textvariable=self.status_var,foreground="#666").grid(row=0,column=4,rowspan=2,padx=5,pady=6)
        frm.grid_columnconfigure(1,weight=1)

    def _refresh_ports(self):
        ports=[p.device for p in serial.tools.list_ports.comports()]
        if DEFAULT_PORT not in ports: ports.insert(0,DEFAULT_PORT)
        self.port_cb["values"]=ports

    def toggle_conn(self):
        if self.serial.is_open():
            self.serial.close(); self.status_var.set("未连接"); self.conn_btn.config(text="连接"); self._log("[SYS] 串口已断开")
        else:
            port=self.port_var.get().strip(); baud=int(self.baud_var.get())
            try:
                self.serial.open(port, baud, on_bytes_callback=self._on_serial_bytes)
            except Exception as e:
                messagebox.showerror("连接失败", f"无法打开串口 {port} @ {baud}\n{e}"); self._log(f"[ERR] 打开失败: {e}"); return
            self.status_var.set(f"已连接 {port} @ {baud}"); self.conn_btn.config(text="断开"); self._log(f"[SYS] 已连接: {port} @ {baud}")

    def _build_controls(self):
        wrapper=ttk.LabelFrame(self,text="控制面板"); wrapper.pack(fill="x", padx=10, pady=6)
        grid=ttk.Frame(wrapper); grid.pack(fill="x", padx=6, pady=6)

        def make_cell(parent,title,on_key,off_key,default_on=False,active_high=True):
            cell=ttk.Frame(parent)
            ttk.Label(cell,text=title+"：",font=("Segoe UI",10,"bold")).pack(anchor="w")
            row=ttk.Frame(cell); row.pack(fill="x", pady=(6,2))
            ui_val=1 if default_on else 0
            device_is_on=(ui_val==1) if active_high else (ui_val==0)
            state_var=tk.StringVar(value="当前：开" if device_is_on else "当前：关")
            sw=ToggleSwitch(row,width=66,height=34,value=ui_val,
                            command=lambda v:self._on_ios_switch(v,on_key,off_key,active_high,state_var))
            sw.pack(side="left")
            ttk.Label(row,textvariable=state_var,foreground="#555").pack(side="left",padx=(12,0))
            return cell, sw

        cell_door,  self.door_switch  = make_cell(grid,"门","DOOR 1","DOOR 0")
        cell_light, self.light_switch = make_cell(grid,"灯","LIGHT 1","LIGHT 0")
        cell_anti,  self.anti_switch  = make_cell(grid,"防忘带系统","ANTI 1","ANTI 0")
        cell_eye,   self.eye_switch   = make_cell(grid,"护眼模式","EYE 1","EYE 0")

        cell_door.grid(row=0,column=0,padx=8,pady=8,sticky="nsew")
        cell_light.grid(row=0,column=1,padx=8,pady=8,sticky="nsew")
        cell_anti.grid(row=1,column=0,padx=8,pady=8,sticky="nsew")
        cell_eye.grid(row=1,column=1,padx=8,pady=8,sticky="nsew")
        grid.grid_columnconfigure(0,weight=1); grid.grid_columnconfigure(1,weight=1)

    def _on_ios_switch(self, ui_val, on_key, off_key, active_high=True, state_var=None):
        try:
            ui_is_on=bool(ui_val); device_is_on = ui_is_on if active_high else (not ui_is_on)
            key = on_key if device_is_on else off_key
            self._send_cmd_hex(key)
            if state_var is not None: state_var.set("当前：开" if device_is_on else "当前：关")
        except Exception as e:
            messagebox.showerror("发送失败", str(e)); self._log(f"[ERR] 开关发送失败: {e}")

    def _build_hex_send(self):
        frm=ttk.LabelFrame(self,text="HEX 发送"); frm.pack(fill="x", padx=10, pady=6)
        self.hex_var=tk.StringVar(value="AA 55 01 01")
        ttk.Entry(frm,textvariable=self.hex_var,width=52).pack(side="left", padx=(10,6), pady=6)
        ttk.Button(frm,text="发送",command=self._send_hex).pack(side="left", padx=6)

    def _build_log(self):
        frm=ttk.LabelFrame(self,text="日志"); frm.pack(fill="both", expand=True, padx=10, pady=10)
        self.log_text=tk.Text(frm,height=12,wrap="word"); self.log_text.pack(fill="both",expand=True,padx=6,pady=6)
        self.log_text.config(state="disabled")

    def _log(self,msg:str):
        self.log_text.config(state="normal")
        ts=time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _send_cmd_hex(self,key:str):
        try:
            if not self.serial.is_open(): raise RuntimeError("请先连接串口")
            if key not in CMD_HEX: raise RuntimeError(f"未配置的命令映射：{key}")
            raw=parse_hex_string(CMD_HEX[key]); data=build_frame_total6(raw)
            self.serial.send_bytes(data); self._log(f"[TX HEX] {bytes_to_hex(data)}  ({key})")
        except Exception as e:
            messagebox.showerror("发送失败", str(e)); self._log(f"[ERR] 发送失败: {e}")

    def _send_hex(self):
        s=self.hex_var.get()
        try:
            if not self.serial.is_open(): raise RuntimeError("请先连接串口")
            raw=parse_hex_string(s); data=build_frame_total6(raw)
            self.serial.send_bytes(data); self._log(f"[TX HEX] {bytes_to_hex(data)}  (HEX手动发送)")
        except Exception as e:
            messagebox.showerror("发送失败", str(e)); self._log(f"[ERR] 发送失败: {e}")

    def _on_serial_bytes(self,data:bytes): self.rx_q.put(data)

    def _poll_rx(self):
        try:
            while True:
                data=self.rx_q.get_nowait()
                if LOG_EVERY_N_RX and LOG_EVERY_N_RX>0:
                    self._rx_chunk_counter+=1
                    if self._rx_chunk_counter % LOG_EVERY_N_RX == 0:
                        self._log(f"[RX HEX] {bytes_to_hex(data)}  (每{LOG_EVERY_N_RX}条打印一次)")
                else:
                    self._log(f"[RX HEX] {bytes_to_hex(data)}")
                if not hasattr(self,'rx_buf'): self.rx_buf=bytearray()
                self.rx_buf.extend(data); self._extract_and_handle_frames()
        except queue.Empty: pass
        self.after(50,self._poll_rx)

    def _extract_and_handle_frames(self):
        while True:
            start=self.rx_buf.find(FRAME_HEAD)
            if start==-1: self.rx_buf.clear(); return
            if start>0: del self.rx_buf[:start]
            if len(self.rx_buf)<FRAME_LEN: return
            frame=bytes(self.rx_buf[:FRAME_LEN]); del self.rx_buf[:FRAME_LEN]
            self._log(f"[RX FRAME] {bytes_to_hex(frame)}")

    # —— 开机动画（结束后调用 on_done）——
    def _show_splash(self, on_done=None):
        splash=tk.Toplevel(self); splash.overrideredirect(True)
        try: splash.attributes("-topmost",True); splash.attributes("-alpha",0.0)
        except tk.TclError: pass
        w,h=420,220; self.update_idletasks()
        sw,sh=self.winfo_screenwidth(), self.winfo_screenheight()
        x=(sw-w)//2; y=(sh-h)//3; splash.geometry(f"{w}x{h}+{x}+{y}")
        frm=ttk.Frame(splash,padding=18); frm.pack(fill="both",expand=True)
        ttk.Label(frm,text="STC 串口控制面板",font=("Segoe UI",16,"bold")).pack(pady=(6,4))
        ttk.Label(frm,text=f"{CLASS_NAME}  —  {STUDENT_NAME}",font=("Segoe UI",11)).pack(pady=(0,16))
        pbar=ttk.Progressbar(frm,orient="horizontal",mode="determinate",length=320,maximum=100); pbar.pack(pady=(6,0))
        start_ts=time.time(); duration=1.6
        def tick():
            el=time.time()-start_ts; pbar["value"]=min(100,int(el/duration*100))
            try: splash.attributes("-alpha", min(1.0, el/0.35))
            except tk.TclError: pass
            if el<duration: splash.after(16,tick)
            else:
                try: splash.destroy()
                finally:
                    if callable(on_done): on_done()
        tick()

    def on_close(self):
        try: self.serial.close()
        except Exception: pass
        self.destroy()

if __name__=="__main__":
    App().mainloop()