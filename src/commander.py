import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import math

# --- CONFIGURATION ---
L1 = 0.110356
L2 = 0.143077
SERIAL_PORT = '/dev/cu.usbserial-0001'
BAUD_RATE = 115200
SEND_RATE_HZ = 15     # Limit how fast we spam the serial port (15 times per sec)

# Initialize Serial (Fails gracefully if not plugged in)
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
fig, ax = plt.subplots(figsize=(7, 7))
ax.set_title("Robot Arm Workspace Commander\n(Click and drag to move)", fontsize=14)
ax.set_aspect('equal')
ax.set_xlim(-R_MAX * 1.2, R_MAX * 1.2)
ax.set_ylim(-R_MAX * 1.2, R_MAX * 1.2)
ax.grid(True, linestyle='--', alpha=0.6)

# Draw Workspace Boundaries
outer_circle = plt.Circle((0, 0), R_MAX, color='lightblue', alpha=0.3)
inner_circle = plt.Circle((0, 0), R_MIN, color='white')
ax.add_patch(outer_circle)
ax.add_patch(inner_circle)

# Plot elements for the arm
arm_line, = ax.plot([], [], 'o-', lw=4, markersize=8, color='black')
target_point, = ax.plot([], [], 'rx', markersize=10)

last_send_time = 0

def solve_analytical_ik(x, y):
    """Simple IK just to visualize the arm in Python"""
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if D > 1 or D < -1: return None # Unreachable
    
    q2 = math.acos(D) 
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    
    # Calculate elbow joint position
    elbow_x = L1 * math.cos(q1)
    elbow_y = L1 * math.sin(q1)
    return (elbow_x, elbow_y)

def process_mouse(event):
    global last_send_time
    
    # Only act if left mouse button is clicked/dragged inside the plot
    if event.button != 1 or not event.inaxes:
        return
        
    x, y = event.xdata, event.ydata
    dist = math.hypot(x, y)
    
    # Check if target is physically reachable
    if R_MIN <= dist <= R_MAX:
        # Update UI
        elbow = solve_analytical_ik(x, y)
        if elbow:
            arm_line.set_data([0, elbow[0], x], [0, elbow[1], y])
            target_point.set_data([x], [y])
            fig.canvas.draw_idle()
            
            # Send to Arduino over Serial, limited by SEND_RATE_HZ
            current_time = time.time()
            if ser and (current_time - last_send_time) > (1.0 / SEND_RATE_HZ):
                # Send format: <X,Y>
                command = f"<{x:.5f},{y:.5f}>\n"
                ser.write(command.encode('utf-8'))
                print(f"Sent: {command.strip()}")
                last_send_time = current_time

# Bind mouse events
fig.canvas.mpl_connect('button_press_event', process_mouse)
fig.canvas.mpl_connect('motion_notify_event', process_mouse)

plt.show()
if ser: ser.close()