""" 
========================================
Python GUI + Arduino Button Task System + AMTI Force Plate + Xsens Awinda IMUs
========================================

FUNCTION:
- Displays a GUI with a start button, fixation cross, and next-condition button.
- Coordinates timing and presentation of trials for 8 executive-function conditions.
- Sends commands to Arduino to light up buttons in specific color patterns.
- Receives button press data from Arduino.
- Plays beeps between conditions to signal start/end.
- Sends LSL markers to Aurora fNIRS and BrainVision for three events:
    Marker 1 = button commanded to light up
    Marker 2 = button confirmed lit (hardware feedback)
    Marker 4 = button pressed
- Records AMTI AccuSway force plate data at 1000 Hz per condition.
- Records Xsens Awinda IMU data per condition via the AwindaMarkerRecorder C# app.
- Force plate AND IMUs start when spacebar is pressed, and stop together after the
  last trial finishes.
- Saves one force plate .txt per condition to C:\\AMTI\\PythonResults\\
- Saves one .mtb + .csv + _events.csv per condition to C:\\AMTI\\PythonResults\\IMU\\
- Marker columns (numeric + text label) embedded in force plate data and in the
  IMU events file, using the same codes and labels.

HARDWARE CONNECTIONS (Arduino side):
Buttons: Pins 2-5
LEDs (NeoPixels): Pins 6-9

DEPENDENCIES:
  pip install pyserial pylsl pandas openpyxl
  AMTI AMTIUSBDevice.dll must be installed (comes with NetForce)
  AwindaMarkerRecorder.exe must be built and RUNNING before this script starts
  (it listens on 127.0.0.1:5555). Set IMU_EXE_PATH below to auto-launch it.
"""

import os
import sys
import serial
import socket
import time
import random
import threading
import winsound
import tkinter as tk
import pandas as pd
from tkinter import ttk, messagebox, font
from datetime import datetime
from ctypes import cdll, c_float, sizeof
import subprocess
from pylsl import StreamInfo, StreamOutlet


# =====================================================
#                TERMINAL LOG CAPTURE
# =====================================================

class _TeeLogger:
    """Writes to both the real stdout and an internal list for later saving."""
    def __init__(self, original_stdout):
        self._original = original_stdout
        self._lines    = []
        self._lock     = threading.Lock()

    def write(self, msg):
        self._original.write(msg)
        self._original.flush()
        if msg.strip():
            with self._lock:
                self._lines.append({
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                    "message":   msg.rstrip("\n")
                })

    def flush(self):
        self._original.flush()

    def get_lines(self):
        with self._lock:
            return list(self._lines)

_tee_logger = _TeeLogger(sys.stdout)
sys.stdout  = _tee_logger


# =====================================================
#                FORCE PLATE SETUP
# =====================================================

FORCE_PLATE_SAVE_FOLDER = r'C:\AMTI\PythonResults'
os.makedirs(FORCE_PLATE_SAVE_FOLDER, exist_ok=True)

SAMPLING_RATE = 1000   # Hz — hard coded, no prompt needed
SAMPLE_SIZE   = 8      # values per sample from DLL
BLOCK_SIZE    = 16     # DLL always returns 16 samples at a time
BUF_SIZE      = SAMPLE_SIZE * BLOCK_SIZE

amti = cdll.AMTIUSBDevice

def init_force_plate():
    """Initialize AMTI DLL and select device. Call once at startup."""
    amti.fmDLLInit()
    countdown = 20
    while amti.fmDLLIsDeviceInitComplete() == 0:
        time.sleep(0.250)
        countdown -= 1
        if countdown < 0:
            raise RuntimeError("AMTI force plate connection timed out.")
    res = amti.fmDLLSetupCheck()
    assert res == 1, "AMTI setup check failed."
    n_devices = amti.fmDLLGetDeviceCount()
    assert n_devices > 0, "No AMTI devices found."
    amti.fmDLLSelectDeviceIndex(0)   # Always use first device
    amti.fmBroadcastAcquisitionRate(SAMPLING_RATE)
    amti.fmBroadcastRunMode(1)       # Metric, fully conditioned
    amti.fmDLLSetDataFormat(1)       # 8 values per sample (incl. counter)
    print(f"AMTI force plate initialized at {SAMPLING_RATE} Hz.")

def shutdown_force_plate():
    """Cleanly stop and shut down AMTI DLL."""
    try:
        amti.fmBroadcastStop()
        amti.fmDLLShutDown()
        print("AMTI force plate shut down.")
    except Exception as e:
        print(f"Error shutting down force plate: {e}")


# =====================================================
#        XSENS AWINDA IMU CLIENT  (NEW SECTION)
# =====================================================
# Talks to AwindaMarkerRecorder.exe over a localhost socket.
# Command set (one line each, one reply line each):
#   PING / SETDIR <folder> / CONNECT <n> / PREPARE <name> / GO / MARK <code> <label>
#   / STOP / QUIT
# Every call is failure-tolerant: if the IMU app is missing or errors, the rest of
# the experiment (force plate, LSL, Arduino) keeps running normally.

IMU_ENABLED      = True                                  # set False to run without IMUs
IMU_HOST         = "127.0.0.1"
IMU_PORT         = 5555                                  # must match CommandPort in RecorderForm.cs
IMU_SENSOR_COUNT = 2                                     # how many MTw sensors must join
IMU_SAVE_FOLDER  = r'C:\AMTI\PythonResults\IMU'
IMU_EXE_PATH     = r'C:\Users\rpier12\AwindaMarkerRecorder\bin\Debug\net10.0-windows\AwindaMarkerRecorder.exe'
IMU_AUTOLAUNCH   = True                                  # launch the .exe if it isn't already running

_imu_sock         = None
_imu_lock         = threading.Lock()
_imu_proc         = None
_imu_prepare_done = threading.Event()
_imu_prepare_ok   = False


def _imu_send(cmd, timeout=15.0):
    """Send one command, return the reply string, or None on failure."""
    global _imu_sock
    if not IMU_ENABLED or _imu_sock is None:
        return None
    with _imu_lock:
        try:
            _imu_sock.settimeout(timeout)
            _imu_sock.sendall((cmd + "\n").encode("ascii", "replace"))
            resp = b""
            while not resp.endswith(b"\n"):
                chunk = _imu_sock.recv(4096)
                if not chunk:
                    raise ConnectionError("IMU recorder closed the connection")
                resp += chunk
            reply = resp.decode("ascii", "replace").strip()
        except Exception as e:
            print(f"IMU command failed ('{cmd}'): {e}")
            try:
                _imu_sock.close()
            except Exception:
                pass
            _imu_sock = None
            return None
    if reply.startswith("ERR"):
        print(f"IMU error on '{cmd}': {reply}")
    return reply


def _imu_open_socket(timeout=20.0):
    """Open the TCP connection to AwindaMarkerRecorder.exe."""
    global _imu_sock
    deadline = time.time() + timeout
    last_err = None
    while time.time() < deadline:
        try:
            s = socket.create_connection((IMU_HOST, IMU_PORT), timeout=5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            _imu_sock = s
            return True
        except OSError as e:
            last_err = e
            time.sleep(0.5)
    print(f"Could not reach IMU recorder on {IMU_HOST}:{IMU_PORT} — {last_err}")
    return False


def imu_startup():
    """Launch (optionally), connect to, and radio-link the Awinda system. Call once."""
    global _imu_proc
    if not IMU_ENABLED:
        print("IMU recording DISABLED (IMU_ENABLED = False).")
        return False

    if not _imu_open_socket(timeout=3.0) and IMU_AUTOLAUNCH:
        if os.path.exists(IMU_EXE_PATH):
            print("Launching AwindaMarkerRecorder.exe ...")
            try:
                _imu_proc = subprocess.Popen([IMU_EXE_PATH],
                                             cwd=os.path.dirname(IMU_EXE_PATH))
            except Exception as e:
                print(f"Failed to launch IMU app: {e}")
            _imu_open_socket(timeout=30.0)
        else:
            print(f"IMU app not found at {IMU_EXE_PATH} — start it manually.")

    if _imu_sock is None:
        print("*** IMU SYSTEM UNAVAILABLE — experiment will run WITHOUT IMU data. ***")
        return False

    _imu_send(f"SETDIR {IMU_SAVE_FOLDER}", timeout=10)
    print("Connecting Awinda station and waiting for sensors "
          "(undock / power on the MTw sensors now) ...")
    reply = _imu_send(f"CONNECT {IMU_SENSOR_COUNT}", timeout=120)
    if reply and reply.startswith("OK"):
        print(f"IMU system ready: {reply}")
        return True
    print("*** IMU system failed to connect — experiment will run WITHOUT IMU data. ***")
    return False


def imu_prepare_async(session_name):
    """Arm the IMUs for one condition (config + file creation) in the background."""
    global _imu_prepare_ok
    _imu_prepare_done.clear()
    _imu_prepare_ok = False

    if not IMU_ENABLED or _imu_sock is None:
        _imu_prepare_done.set()
        return

    def _worker():
        global _imu_prepare_ok
        reply = _imu_send(f"PREPARE {session_name}", timeout=90)
        _imu_prepare_ok = bool(reply and reply.startswith("OK"))
        if _imu_prepare_ok:
            print(f"IMU armed for: {session_name}")
        else:
            print("IMU failed to arm — no IMU data for this condition.")
        _imu_prepare_done.set()

    threading.Thread(target=_worker, daemon=True).start()


def imu_wait_ready(timeout=90):
    """Block until arming finishes (normally already done by spacebar time)."""
    _imu_prepare_done.wait(timeout)
    return _imu_prepare_ok


def imu_go():
    """Start IMU recording. Returns the recorder's unix start time, or None."""
    if not _imu_prepare_ok:
        return None
    reply = _imu_send("GO", timeout=15)
    if reply and reply.startswith("OK GO"):
        t = float(reply.split()[2])
        print(f"IMU recording started (unix {t:.6f} = "
              f"{datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]})")
        return t
    return None


def imu_mark(code, label):
    """Stamp the IMU events file with the same code/label as the force plate."""
    if not _imu_prepare_ok:
        return
    _imu_send(f"MARK {int(code)} {label}", timeout=5)


def imu_stop():
    """Stop IMU recording and close this condition's files."""
    global _imu_prepare_ok
    if not IMU_ENABLED or _imu_sock is None:
        return
    reply = _imu_send("STOP", timeout=30)
    if reply and reply.startswith("OK"):
        print(f"IMU recording stopped — files in {IMU_SAVE_FOLDER}")
    _imu_prepare_ok = False
    _imu_prepare_done.clear()


def imu_shutdown():
    """Release the radio and close the connection. Call once on exit."""
    global _imu_sock
    if _imu_sock is None:
        return
    try:
        _imu_send("QUIT", timeout=15)
    except Exception:
        pass
    try:
        _imu_sock.close()
    except Exception:
        pass
    _imu_sock = None
    print("IMU system shut down.")


# =====================================================
#         FORCE PLATE RECORDING — PER CONDITION
# =====================================================

_fp_marker_lock   = threading.Lock()
_fp_current_marker_code  = 0
_fp_current_marker_label = ""

_fp_stop_event = threading.Event()
_fp_thread     = None
_fp_data_rows  = []


def session_name(condition_name, order_number, is_repeat):
    """Single source of truth for file naming — used by BOTH the force plate
    and the IMU recorder so the two files for a condition always match."""
    suffix = "Repeat" if is_repeat else str(order_number)
    return f"{condition_name} ({suffix})"


def set_fp_marker(code, label):
    global _fp_current_marker_code, _fp_current_marker_label
    with _fp_marker_lock:
        _fp_current_marker_code  = code
        _fp_current_marker_label = label


def _fp_recording_loop():
    global _fp_data_rows, _fp_current_marker_code, _fp_current_marker_label

    _fp_data_rows = []
    buf = (c_float * BUF_SIZE)()
    amti.fmBroadcastStart()

    start_time = datetime.now()
    print('Force plate recording started:  ' +
          start_time.strftime('%H:%M:%S.') +
          f'{start_time.microsecond // 1000:03d}')

    while not _fp_stop_event.is_set():
        res = amti.fmDLLGetTheFloatDataLBVStyle(buf, sizeof(buf))
        if res != 0:
            with _fp_marker_lock:
                stamp_code  = _fp_current_marker_code
                stamp_label = _fp_current_marker_label
                _fp_current_marker_code  = 0
                _fp_current_marker_label = ""

            for a in range(0, BUF_SIZE, SAMPLE_SIZE):
                sample = list(buf[a:(a + SAMPLE_SIZE)])
                _fp_data_rows.append(sample + [stamp_code, stamp_label])
                stamp_code  = 0
                stamp_label = ""
        else:
            time.sleep(0.001)

    amti.fmBroadcastStop()

    end_time = datetime.now()
    print('Force plate recording ended:    ' +
          end_time.strftime('%H:%M:%S.') +
          f'{end_time.microsecond // 1000:03d}')


def start_fp_recording():
    """Starts the force plate AND the IMUs together."""
    global _fp_thread
    _fp_stop_event.clear()
    _fp_thread = threading.Thread(target=_fp_recording_loop, daemon=True)
    _fp_thread.start()
    # ---- IMU STARTS HERE, same instant as the force plate ----
    imu_go()


def stop_fp_recording():
    """Stops the force plate AND the IMUs together."""
    _fp_stop_event.set()
    if _fp_thread is not None:
        _fp_thread.join(timeout=5)
    # ---- IMU STOPS HERE, same instant as the force plate ----
    imu_stop()


def save_fp_data(condition_name, order_number, is_repeat):
    safe_name = session_name(condition_name, order_number, is_repeat)
    filename  = safe_name + ".txt"
    full_path = os.path.join(FORCE_PLATE_SAVE_FOLDER, filename)

    with open(full_path, 'wt') as fp:
        fp.write(
            'counter\tFx (N)\tFy (N)\tFz (N)\t'
            'Mx (Nm)\tMy (Nm)\tMz (Nm)\t'
            'trigger state\tmarker\tmarker_label\n'
        )
        for row in _fp_data_rows:
            numeric_part = row[:8]
            marker_code  = row[8]
            marker_label = row[9]
            line = '\t'.join('%g' % x for x in numeric_part)
            line += f'\t{marker_code}\t{marker_label}\n'
            fp.write(line)

    print(f"Force plate data saved to: {full_path}")


# =====================================================
#                LSL SETUP
# =====================================================

_lsl_info = StreamInfo(
    name='Trigger',
    type='Markers',
    channel_count=1,
    channel_format='int32',
    source_id='fNIRS'
)
lsl_outlet = StreamOutlet(_lsl_info)
print("LSL Trigger stream created — start recording in Aurora before beginning task")

_bv_info = StreamInfo(
    name='BV_Markers',
    type='Markers',
    channel_count=1,
    channel_format='int32',
    source_id='BrainVision'
)
bv_outlet = StreamOutlet(_bv_info)
print("BrainVision LSL Marker stream created")

MARKER_BUTTON_COMMANDED = 1
MARKER_BUTTON_LIT       = 2
MARKER_BUTTON_PRESSED   = 4

def send_marker(marker_value, label=""):
    """Send LSL marker to Aurora + BrainVision, stamp the force plate file,
    AND stamp the IMU events file."""
    lsl_outlet.push_sample([marker_value])
    bv_outlet.push_sample([marker_value])
    set_fp_marker(marker_value, label)
    print(f"LSL Marker sent: {marker_value} ({label})")
    # ---- IMU marker last, so it can never delay the LSL/force-plate stamps ----
    imu_mark(marker_value, label)


# =====================================================
#                TIME SYNC
# =====================================================

def sync_windows_time():
    try:
        subprocess.run(["w32tm", "/resync"], check=True, capture_output=True, text=True)
        print("Windows time synced.")
    except subprocess.CalledProcessError as e:
        print("Time sync failed:", e.stdout, e.stderr)
    except Exception as e:
        print("Unexpected error during time sync:", e)
    print("This is okay — time sync only works as administrator on WiFi.")

sync_windows_time()


# =====================================================
#                ARDUINO SERIAL SETUP
# =====================================================

arduino = serial.Serial('COM3', 115200, timeout=1)
time.sleep(2)


# =====================================================
#                GUI SETUP (Tkinter)
# =====================================================

root = tk.Tk()
root.title("Button Task")
root.geometry("600x400")
root.configure(bg="white")

cross_font = font.Font(size=80, weight="bold")
fixation_label = tk.Label(root, text="+", font=cross_font, fg="black", bg="white")
fixation_label.place(relx=0.5, rely=0.5, anchor="center")
fixation_label.lower()

start_btn = tk.Button(root, text="Start", font=("Arial", 20))
start_btn.pack(pady=10)
stop_btn = tk.Button(root, text="Stop", font=("Arial", 20), state="disabled")
stop_btn.pack(pady=10)
next_btn = tk.Button(root, text="Next Condition", font=("Arial", 20), state="disabled")
next_btn.pack(pady=10)

override_var = tk.StringVar()
condition_dropdown = ttk.Combobox(root, textvariable=override_var, state="readonly", font=("Arial", 14))
condition_dropdown.pack(pady=10)

redo_btn = tk.Button(root, text="Redo Current Condition", font=("Arial", 16), state="disabled")
redo_btn.pack(pady=10)

status_frame = tk.Frame(root, bg="white")
status_frame.pack(pady=10, fill="x")
condition_label = tk.Label(status_frame, text="Condition: ",     font=("Arial", 14), bg="white")
condition_label.pack(anchor="w")
trial_label     = tk.Label(status_frame, text="Trial: ",         font=("Arial", 14), bg="white")
trial_label.pack(anchor="w")
pattern_label   = tk.Label(status_frame, text="Pattern: ",       font=("Arial", 14), bg="white")
pattern_label.pack(anchor="w")
buttons_label   = tk.Label(status_frame, text="Active Button(s): ", font=("Arial", 14), bg="white")
buttons_label.pack(anchor="w")

stop_requested        = False
canonical_conditions  = []
remaining_conditions  = []
current_condition_name = None
condition_history     = {}
event_log             = []

condition_order_counter = 0


# =====================================================
#                HELPER FUNCTIONS
# =====================================================

PATTERN_INSTRUCTIONS = {
    "GO_BLUE":   "Go-Blue: Press the button when a single blue light appears.",
    "STOP_RED":  "Stop-Red: Do not press when a red light appears.",
    "ONLY_BLUE": "Only-Blue: When three red lights and one blue light appear, press only the blue button.",
    "ONLY_RED":  "Only-Red: When three blue lights and one red light appear, press only the red button.",
}

def show_instructions(cond_name, trials, is_repeat=False):
    trial_names  = [t["pattern"] for t in trials]
    display_name = f"{cond_name} (REPEAT)" if is_repeat else cond_name

    # Build deduplicated instruction lines for patterns present in this condition
    seen_patterns = []
    for p in trial_names:
        if p not in seen_patterns:
            seen_patterns.append(p)
    instruction_lines = "\n".join(PATTERN_INSTRUCTIONS[p] for p in seen_patterns)

    instr_text   = (
        f"Condition: {display_name}\n\nTrials:\n" +
        "\n".join(trial_names) +
        "\n\n" + instruction_lines +
        "\n\nAfter instructing participant, press SPACEBAR to begin "
        "10-second fixation cross and trials.\n\n"
        "Markers will be placed in fNIRS data, EEG, force plate, IMU data, and button log "
        "corresponding to buttons being lit and pressed.\n\n"
        "NOTE: Force plate AND IMU recording will START when you press SPACEBAR."
    )

    instr_window = tk.Toplevel(root)
    instr_window.title("Instructions")
    instr_window.attributes("-fullscreen", True)
    instr_window.configure(bg="white")
    instr_window.focus_force()
    tk.Label(instr_window, text=instr_text, font=("Arial", 14), justify="left").pack(pady=20, padx=20)

    space_pressed = threading.Event()

    def on_space(event):
        space_pressed.set()
        instr_window.destroy()
        # ---- MAKE SURE THE IMUs ARE ARMED (normally already done) ----
        imu_wait_ready()
        # ---- START FORCE PLATE + IMU RECORDING HERE ----
        start_fp_recording()
        # ---- LOOK-DOWN BEEP at 8.5 seconds into fixation ----
        def _delayed_lookdown_beep():
            time.sleep(8.5)
            winsound.Beep(500, 500)
            print("Look-down beep played (8.5s after spacebar)")
        threading.Thread(target=_delayed_lookdown_beep, daemon=True).start()

    instr_window.bind("<space>", on_space)
    instr_window.grab_set()
    root.wait_window(instr_window)
    space_pressed.wait()


def show_fixation():
    fixation_label.lift()
    root.update()


def stop_experiment():
    global stop_requested
    log_gui_event("stop_button_clicked")
    stop_requested = True
    stop_fp_recording()          # also stops the IMUs
    send_arduino("ALL_OFF")
    next_btn.config(state="normal")


def beep():
    winsound.Beep(500, 500)


def send_arduino(cmd):
    arduino.write((cmd + "\n").encode('utf-8'))
    log_gui_event("arduino_command_sent", cmd)


def update_status(cond_name, trial_num, pattern, active_buttons, target_button=None):
    def _update():
        condition_label.config(text=f"Condition: {cond_name}")
        trial_label.config(text=f"Trial: {trial_num + 1}")
        pattern_label.config(text=f"Pattern: {pattern}")
        btn_texts = []
        for idx in active_buttons:
            display_idx = idx + 1
            btn_texts.append(f"[{display_idx}]" if idx == target_button else str(display_idx))
        buttons_label.config(text=f"Active Button(s): {', '.join(btn_texts)}")
    root.after(0, _update)


def clear_status():
    condition_label.config(text="Condition: ")
    trial_label.config(text="Trial: ")
    pattern_label.config(text="Pattern: ")
    buttons_label.config(text="Active Button(s): ")
    root.update()


def select_override(event):
    global override_condition_name, is_redo_run, is_manual_selection
    selected = override_var.get()
    if confirm_manual_selection(selected):
        override_condition_name = selected
        is_manual_selection     = True
        is_redo_run             = False
        next_btn.config(state="disabled")
        threading.Thread(target=run_current_condition).start()
    else:
        if remaining_conditions:
            override_var.set(remaining_conditions[0][0])


def confirm_manual_selection(cond_name):
    return messagebox.askyesno(
        "Confirm Condition Selection",
        f"Run condition next:\n\n{cond_name}\n\nThis will advance the experiment.\n\n"
        "Don't forget to manually Start/Stop the Aurora fNIRS recording.\n"
        "Markers will be placed in fNIRS data corresponding to buttons being lit and pressed."
    )


def confirm_override(cond_name):
    return messagebox.askyesno(
        "Confirm Condition Override",
        f"Redo condition:\n\n{cond_name}\n\n"
        "This will NOT advance the experiment.\n\nProceed?\n\n"
        "Don't forget to manually Start/Stop the Aurora fNIRS recording.\n"
        "Markers will be placed in fNIRS data corresponding to buttons being lit and pressed."
    )


def redo_current_condition():
    global override_condition_name, is_redo_run, is_manual_selection
    log_gui_event("redo_condition_clicked")
    cond_name = current_condition_name
    if confirm_override(cond_name):
        override_condition_name = cond_name
        is_redo_run             = True
        is_manual_selection     = False
        next_btn.config(state="disabled")
        threading.Thread(target=run_current_condition, daemon=True).start()


def update_history():
    def _update():
        history_text.config(state="normal")
        history_text.delete(1.0, tk.END)
        for cond_name, runs in condition_history.items():
            for run in runs:
                history_text.insert(tk.END, f"{run['label']}: {run['completed']}/{run['total']}\n")
        history_text.config(state="disabled")
    root.after(0, _update)


def log_event(event_type, cond_name, trial_num, trial_type,
              target_button=None, pressed_button=None, active_buttons=None, is_repeat=False):
    event_log.append({
        "timestamp":      datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
        "event":          event_type,
        "condition":      cond_name,
        "trial_num":      trial_num + 1 if trial_num is not None else "",
        "trial_type":     trial_type,
        "target_button":  target_button  if target_button  is not None else "",
        "pressed_button": pressed_button if pressed_button is not None else "",
        "active_buttons": ",".join(map(str, active_buttons)) if active_buttons else "",
        "is_repeat":      is_repeat
    })


def log_gui_event(event_type, extra=None):
    event_log.append({
        "timestamp":  datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
        "event":      event_type,
        "details":    extra if extra else "",
        "condition":  current_condition_name,
        "trial_num":  "",
        "trial_type": "",
        "button":     ""
    })


# =====================================================
#                TRIAL PRESENTATION LOGIC
# =====================================================

NUM_BUTTONS  = 4
TOTAL_TRIALS = 10

def run_trials(trials, cond_name):
    last_blue     = None
    last_only_red = None

    for trial_num, trial in enumerate(trials):
        pattern       = trial["pattern"]
        target_button = None
        active_buttons = []
        arduino_cmd   = ""

        if stop_requested:
            current_run = condition_history[cond_name][-1]
            current_run['completed'] = trial_num
            update_history()
            return

        # ---------- Build trial parameters ----------
        if pattern == "GO_BLUE":
            options        = [i for i in range(NUM_BUTTONS) if i != last_blue]
            target_button  = random.choice(options)
            active_buttons = [target_button]
            arduino_cmd    = f"GO_BLUE {target_button}"

        elif pattern == "STOP_RED":
            target_button  = random.choice(range(NUM_BUTTONS))
            active_buttons = [target_button]
            arduino_cmd    = f"STOP_RED {target_button}"

        elif pattern == "ONLY_BLUE":
            options        = [i for i in range(NUM_BUTTONS) if i != last_blue]
            target_button  = random.choice(options)
            active_buttons = [i for i in range(NUM_BUTTONS) if i != target_button] + [target_button]
            arduino_cmd    = f"ONLY_BLUE {','.join(map(str, active_buttons))}"

        elif pattern == "ONLY_RED":
            options        = [i for i in range(NUM_BUTTONS) if i != last_only_red]
            target_button  = random.choice(options)
            active_buttons = [i for i in range(NUM_BUTTONS) if i != target_button] + [target_button]
            arduino_cmd    = f"ONLY_RED {','.join(map(str, active_buttons))}"

        # ---------- Light up buttons + Marker 1 ----------
        update_status(cond_name, trial_num, pattern, active_buttons, target_button=target_button)
        send_arduino(arduino_cmd)
        send_marker(MARKER_BUTTON_COMMANDED, f"button_commanded: {arduino_cmd}")

        log_event(
            event_type="button_lit",
            cond_name=cond_name,
            trial_num=trial_num,
            trial_type=pattern,
            target_button=target_button,
            pressed_button=None,
            active_buttons=active_buttons,
            is_repeat=is_redo_run
        )

        # ---------- Wait for press OR 10s timeout ----------
        start_time     = time.time()
        pressed_button = None
        while (time.time() - start_time) < 10.0:
            if stop_requested:
                return
            if arduino.in_waiting > 0:
                line = arduino.readline().decode('utf-8').strip()
                if line.startswith("LIT"):
                    lit_btn = int(line.split()[1])
                    send_marker(MARKER_BUTTON_LIT, f"button_lit: button {lit_btn}")
                    log_event(
                        event_type="button_lit_actual",
                        cond_name=cond_name,
                        trial_num=trial_num,
                        trial_type=pattern,
                        target_button=lit_btn,
                        pressed_button=None,
                        active_buttons=active_buttons,
                        is_repeat=is_redo_run
                    )
                if line.startswith("PRESSED"):
                    pressed_button = int(line.split()[1])
                    send_marker(MARKER_BUTTON_PRESSED, f"button_pressed: button {pressed_button}")
                    log_event(
                        event_type="button_pressed",
                        cond_name=cond_name,
                        trial_num=trial_num,
                        trial_type=pattern,
                        target_button=target_button,
                        pressed_button=pressed_button,
                        active_buttons=active_buttons,
                        is_repeat=is_redo_run
                    )
                    break
            time.sleep(0.005)

        # ---------- Turn off lights ----------
        send_arduino("ALL_OFF")


        # ---------- Randomized ITI: 2.5–3.5 seconds ----------
        iti = random.uniform(2.5, 3.5)
        time.sleep(iti)

        if pattern in ("GO_BLUE", "ONLY_BLUE"):
            last_blue = target_button
        elif pattern == "ONLY_RED":
            last_only_red = target_button

    # All trials done — update history
    current_run = condition_history[cond_name][-1]
    current_run['completed'] = trial_num + 1
    update_history()


# =====================================================
#                CONDITION SEQUENCES
# =====================================================

def build_conditions():
    base_conditions = [
        ("Go - Feet Apart",          [{"pattern": "GO_BLUE"}   for _ in range(10)]),
        ("No Go - Feet Apart",       [{"pattern": "GO_BLUE"}]*8  + [{"pattern": "STOP_RED"}]*2),
        ("No Shift - Feet Apart",    [{"pattern": "ONLY_BLUE"} for _ in range(10)]),
        ("Shift - Feet Apart",       [{"pattern": "ONLY_BLUE"}]*8 + [{"pattern": "ONLY_RED"}]*2),
        ("Go - Feet Together",       [{"pattern": "GO_BLUE"}   for _ in range(10)]),
        ("No Go - Feet Together",    [{"pattern": "GO_BLUE"}]*8  + [{"pattern": "STOP_RED"}]*2),
        ("No Shift - Feet Together", [{"pattern": "ONLY_BLUE"} for _ in range(10)]),
        ("Shift - Feet Together",    [{"pattern": "ONLY_BLUE"}]*8 + [{"pattern": "ONLY_RED"}]*2),
    ]

    for i in [1, 3, 5, 7]:
        trials = base_conditions[i][1]
        while True:
            random.shuffle(trials)
            consecutive_red = any(
                (trials[j]["pattern"] in ["STOP_RED", "ONLY_RED"]) and
                (trials[j+1]["pattern"] in ["STOP_RED", "ONLY_RED"])
                for j in range(len(trials) - 1)
            )
            if not consecutive_red:
                break
        base_conditions[i] = (base_conditions[i][0], trials)

    random.shuffle(base_conditions)
    return base_conditions


# =====================================================
#                EXPERIMENT THREAD
# =====================================================

override_condition_name = None
is_redo_run             = False
is_manual_selection     = False


def start_experiment():
    global canonical_conditions, remaining_conditions, condition_order_counter
    start_btn.config(state="disabled")
    next_btn.config(state="disabled")
    fixation_label.lift()
    clear_status()
    root.update()

    condition_order_counter = 0


    canonical_conditions  = build_conditions()
    remaining_conditions  = canonical_conditions.copy()
    stop_btn.config(state="normal")

    condition_names = [c[0] for c in canonical_conditions]
    condition_dropdown["values"] = condition_names
    override_var.set(condition_names[0])

    redo_btn.config(state="normal")
    run_current_condition()


def run_current_condition():
    global remaining_conditions, override_condition_name, is_redo_run
    global stop_requested, current_condition_name, condition_order_counter

    if not remaining_conditions and not override_condition_name:
        start_btn.config(state="normal")
        next_btn.config(state="disabled")
        redo_btn.config(state="disabled")
        stop_btn.config(state="disabled")
        fixation_label.lower()
        return

    # ---------------- DETERMINE CONDITION ----------------
    if override_condition_name:
        cond_name = override_condition_name
        trials    = next(c[1] for c in canonical_conditions if c[0] == cond_name)
    else:
        cond_name, trials = remaining_conditions[0]

    current_condition_name = cond_name

    # ---------------- ARM THE IMUs FOR THIS CONDITION ----------------
    # Runs in the background while the operator reads the instructions, so that
    # SPACEBAR can start the IMUs and force plate at the same instant.
    expected_order = condition_order_counter if is_redo_run else condition_order_counter + 1
    imu_prepare_async(session_name(cond_name, expected_order, is_redo_run))

    # ---------------- SHOW INSTRUCTIONS + SPACEBAR ----------------
    show_instructions(cond_name, trials, is_redo_run)

    # ---------------- 10 SECOND FIXATION ----------------
    fixation_label.lift()
    root.update()
    time.sleep(10)
    fixation_label.lower()

    # ---------------- HISTORY SETUP ----------------
    if not is_redo_run:
        condition_order_counter += 1

    display_name = f"{cond_name} (REPEAT)" if is_redo_run else cond_name
    total_trials = len(trials)

    if cond_name not in condition_history:
        condition_history[cond_name] = []

    condition_history[cond_name].append({
        'label':     display_name,
        'completed': 0,
        'total':     total_trials
    })

    update_history()
    override_var.set(display_name)
    stop_requested = False

    print(f"RUNNING CONDITION: {cond_name}  (order #{condition_order_counter}, repeat={is_redo_run})")

    # ---------------- RUN TRIALS ----------------
    run_trials(trials, cond_name)

    # ---------------- STOP FORCE PLATE + IMUs, THEN SAVE ----------------
    stop_fp_recording()          # also stops the IMUs and closes their files
    save_fp_data(cond_name, condition_order_counter, is_redo_run)

    if stop_requested:
        stop_requested = False
        return

    send_arduino("ALL_OFF")
    log_gui_event("condition_end_all_off")
    beep()

    # ---------------- REMOVE FROM REMAINING ----------------
    if not override_condition_name:
        remaining_conditions.pop(0)
    else:
        remaining_conditions = [
            (name, t) for name, t in remaining_conditions
            if name != cond_name
        ]

    override_condition_name = None
    is_redo_run             = False
    next_btn.config(state="normal")


def next_condition():
    global override_condition_name, is_manual_selection, is_redo_run
    log_gui_event("next_condition_clicked")

    override_condition_name = None
    is_manual_selection     = False
    is_redo_run             = False

    if not remaining_conditions:
        messagebox.showinfo("Experiment Complete", "All conditions have been presented.")
        return

    idx = random.randint(0, len(remaining_conditions) - 1)
    cond_name, _ = remaining_conditions[idx]
    override_condition_name = cond_name

    threading.Thread(target=run_current_condition, daemon=True).start()


# =====================================================
#                HISTORY PANEL
# =====================================================

history_frame = tk.Frame(root, bg="white")
history_text  = tk.Text(history_frame, height=8, font=("Arial", 12), bg="lightgray")
history_text.pack(fill="both", expand=True)
history_frame.place(relx=0.0, rely=0.6, relwidth=1.0, relheight=0.35)


# =====================================================
#                SAVE LOG ON EXIT
# =====================================================

def save_terminal_log():
    """Save all captured terminal (stdout) messages to an Excel file."""
    try:
        lines = _tee_logger.get_lines()
        if lines:
            df = pd.DataFrame(lines)
            save_folder = r"C:\Users\rpier12\Python Results"
            os.makedirs(save_folder, exist_ok=True)
            file_path = os.path.join(
                save_folder,
                f"terminal_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
            )
            df.to_excel(file_path, index=False)
            _tee_logger._original.write(f"Terminal log saved to {file_path}\n")
        else:
            _tee_logger._original.write("No terminal messages to save.\n")
    except Exception as e:
        _tee_logger._original.write(f"Error saving terminal log: {e}\n")


def save_log_on_exit():
    global stop_requested
    stop_requested = True

    try:
        stop_fp_recording()      # also stops the IMUs
    except Exception as e:
        print("Error stopping force plate / IMUs on exit:", e)

    try:
        send_arduino("ALL_OFF")
        log_gui_event("gui_closed_all_off")
    except Exception as e:
        print("Error turning off Arduino LEDs:", e)

    try:
        if event_log:
            df = pd.DataFrame(event_log)
            df.sort_values("timestamp", inplace=True)
            save_folder = r"C:\Users\rpier12\Python Results"
            os.makedirs(save_folder, exist_ok=True)
            file_path = os.path.join(
                save_folder,
                f"button_task_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
            )
            df.to_excel(file_path, index=False)
            print(f"Log saved to {file_path}")
        else:
            print("No events to save.")
    except Exception as e:
        print("Error saving Excel log:", e)

    # ---- SAVE TERMINAL LOG LAST (captures everything above) ----
    save_terminal_log()

    try:
        arduino.close()
    except Exception as e:
        _tee_logger._original.write(f"Error closing Arduino serial: {e}\n")

    imu_shutdown()
    shutdown_force_plate()
    root.destroy()


# =====================================================
#                BUTTON CALLBACKS + BINDINGS
# =====================================================

start_btn.config(command=lambda: threading.Thread(target=start_experiment).start())
next_btn.config(command=lambda: threading.Thread(target=next_condition).start())
stop_btn.config(command=stop_experiment)
condition_dropdown.bind("<<ComboboxSelected>>", select_override)
redo_btn.config(command=redo_current_condition)


# =====================================================
#                STARTUP + MAIN LOOP
# =====================================================

init_force_plate()   # Connect to plate once at startup
imu_startup()        # Connect to Awinda station + sensors once at startup

root.protocol("WM_DELETE_WINDOW", save_log_on_exit)
root.mainloop()
