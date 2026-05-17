import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import serial
import time
import math

# --- CONFIGURATION ---
L1 = 0.110356
L2 = 0.143077
SERIAL_PORT = '/dev/cu.usbserial-0001'
BAUD_RATE = 115200
SEND_RATE_HZ = 1     # Limit how fast we spam the serial port (manual drag)
LOOP_DELAY_MS = 2000  # How long to wait between points in automatic loop mode (1000 ms = 1 sec)

# Initialize Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"✅ Connected to {SERIAL_PORT}")
except Exception as e:
    ser = None
    print(f"⚠️ Could not open serial port: {e}. Running in simulation mode only.")

# --- WORKSPACE LIMITS ---
R_MAX = L1 + L2
R_MIN = abs(L1 - L2)

# --- MATPLOTLIB SETUP ---
fig, ax = plt.subplots(figsize=(7, 8))
plt.subplots_adjust(bottom=0.2) # Make room for buttons at the bottom
ax.set_title("Robot Arm Workspace Commander\n(Click to move or select points)", fontsize=14)
ax.set_aspect('equal')
ax.set_xlim(-R_MAX * 1.2, R_MAX * 1.2)
ax.set_ylim(-R_MAX * 1.2, R_MAX * 1.2)
ax.grid(True, linestyle='--', alpha=0.6)

# Draw Workspace Boundaries
outer_circle = plt.Circle((0, 0), R_MAX, color='lightblue', alpha=0.3)
inner_circle = plt.Circle((0, 0), R_MIN, color='white')
ax.add_patch(outer_circle)
ax.add_patch(inner_circle)

# Plot elements for the arm and saved points
arm_line, = ax.plot([], [], 'o-', lw=4, markersize=8, color='black')
target_point, = ax.plot([], [], 'rx', markersize=10)
saved_points_plot, = ax.plot([], [], 'go', markersize=8, label="Saved Points") # Green dots for loop
ax.legend(loc="upper right")

# --- STATE VARIABLES ---
last_send_time = 0
current_mode = 'MANUAL'  # Modes: 'MANUAL', 'SELECTING', 'LOOPING'
saved_points = []
loop_idx = 0

def solve_analytical_ik(x, y):
    """Simple IK just to visualize the arm in Python"""
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if D > 1 or D < -1: return None # Unreachable
    
    q2 = math.acos(D) 
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    
    elbow_x = L1 * math.cos(q1)
    elbow_y = L1 * math.sin(q1)
    return (elbow_x, elbow_y)

def move_arm_to(x, y, ignore_rate_limit=False):
    """Handles math, drawing, and serial sending"""
    global last_send_time
    
    dist = math.hypot(x, y)
    if R_MIN <= dist <= R_MAX:
        elbow = solve_analytical_ik(x, y)
        if elbow:
            # Update UI
            arm_line.set_data([0, elbow[0], x], [0, elbow[1], y])
            target_point.set_data([x], [y])
            fig.canvas.draw_idle()
            
            # Send to Serial
            current_time = time.time()
            if ser and (ignore_rate_limit or (current_time - last_send_time) > (1.0 / SEND_RATE_HZ)):
                command = f"<{x:.5f},{y:.5f}>\n"
                ser.write(command.encode('utf-8'))
                print(f"Sent: {command.strip()}")
                last_send_time = current_time

def process_mouse(event):
    global current_mode
    
    # Only act if left mouse button is clicked/dragged inside the main plot
    if event.button != 1 or not event.inaxes or event.inaxes != ax:
        return
        
    x, y = event.xdata, event.ydata
    
    if current_mode == 'MANUAL':
        move_arm_to(x, y)
        
    elif current_mode == 'SELECTING' and event.name == 'button_press_event':
        # Only save point on click, not drag
        dist = math.hypot(x, y)
        if R_MIN <= dist <= R_MAX:
            if len(saved_points) < 4:
                saved_points.append((x, y))
                # Update visual dots
                xs = [p[0] for p in saved_points]
                ys = [p[1] for p in saved_points]
                saved_points_plot.set_data(xs, ys)
                fig.canvas.draw_idle()
                print(f"Point {len(saved_points)} saved: ({x:.4f}, {y:.4f})")
                
                if len(saved_points) == 4:
                    print("✅ 4 points selected! Click 'Run Loop' to start.")
                    current_mode = 'MANUAL'

def loop_step():
    """Timer callback function for the automatic loop"""
    global loop_idx
    if current_mode == 'LOOPING' and len(saved_points) == 4:
        x, y = saved_points[loop_idx]
        move_arm_to(x, y, ignore_rate_limit=True)
        loop_idx = (loop_idx + 1) % 4

# Setup automatic loop timer
loop_timer = fig.canvas.new_timer(interval=LOOP_DELAY_MS)
loop_timer.add_callback(loop_step)

# --- BUTTON CALLBACKS ---
def on_btn_select(event):
    global current_mode, saved_points
    current_mode = 'SELECTING'
    saved_points = []
    saved_points_plot.set_data([], []) # Clear UI dots
    loop_timer.stop()
    fig.canvas.draw_idle()
    print("\n--- SELECT MODE ---")
    print("Click 4 locations on the plot to save them.")

def on_btn_run(event):
    global current_mode, loop_idx
    if len(saved_points) == 4:
        current_mode = 'LOOPING'
        loop_idx = 0
        loop_timer.start()
        print("\n--- LOOPING MODE ---")
        print("Running saved loop. Click 'Stop' to end.")
    else:
        print(f"\n⚠️ Need exactly 4 points to loop. You have {len(saved_points)}.")

def on_btn_stop(event):
    global current_mode
    current_mode = 'MANUAL'
    loop_timer.stop()
    print("\n--- MANUAL MODE ---")

# --- UI BUTTON DEFINITIONS ---
# [left, bottom, width, height]
ax_btn_select = plt.axes([0.15, 0.05, 0.2, 0.075])
ax_btn_run = plt.axes([0.40, 0.05, 0.2, 0.075])
ax_btn_stop = plt.axes([0.65, 0.05, 0.2, 0.075])

btn_select = Button(ax_btn_select, 'Select 4 Pts')
btn_run = Button(ax_btn_run, 'Run Loop')
btn_stop = Button(ax_btn_stop, 'Stop / Manual')

btn_select.on_clicked(on_btn_select)
btn_run.on_clicked(on_btn_run)
btn_stop.on_clicked(on_btn_stop)

# Bind mouse events
fig.canvas.mpl_connect('button_press_event', process_mouse)
fig.canvas.mpl_connect('motion_notify_event', process_mouse)

plt.show()
if ser: ser.close()