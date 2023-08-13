import tkinter as tk
from tkinter import ttk
import requests

# Define the Quadcopter IP address
quadcopter_ip = "192.168.1.100"  # Replace with your actual IP address

def update_parameters():
    # Update parameters on the quadcopter
    params = {
        'Throttle': throttle.get(),
        'kp': kp.get(),
        'ki': ki.get(),
        'kd': kd.get(),
        'kp_outerloop': kp_outerloop.get()
        # Add other parameters here
    }
    print(params)
    response = requests.get(f'http://{quadcopter_ip}/update_parameters', params=params)
    # Handle response if needed

def send_command(direction):
    # Send control commands based on the given direction
    if direction == "up":
        # Send the throttle increase command
        print("up")
    elif direction == "down":
        # Send the throttle decrease command
        print("down")
    elif direction == "right":
        # Send the roll right command
        print("right")
    elif direction == "left":
        # Send the roll left command
        print("left")

def on_key_press(event):
    # Map arrow keys to respective directions and send commands
    key = event.keysym.lower()
    if key == "up":
        send_command("up")
    elif key == "down":
        send_command("down")
    elif key == "right":
        send_command("right")
    elif key == "left":
        send_command("left")

def is_connected():
    # Check if connected to the drone
    try:
        response = requests.get(f'http://{quadcopter_ip}/ping')
        if response.status_code == 200:
            return True
        else:
            return False
    except requests.ConnectionError:
        return False

app = tk.Tk()
app.title("Quadcopter Control")

# Create frames for layout
sliders_frame = ttk.Frame(app)
direction_buttons_frame = ttk.Frame(app)
remaining_frame = ttk.Frame(app)
connection_status_frame = ttk.Frame(app)

# Create labels and sliders for parameter adjustment
throttle = tk.DoubleVar()
kp = tk.DoubleVar()
ki = tk.DoubleVar()
kd = tk.DoubleVar()
kp_outerloop = tk.DoubleVar()

throttle_label = ttk.Label(sliders_frame, text="Throttle")
kp_label = ttk.Label(sliders_frame, text="kp")
ki_label = ttk.Label(sliders_frame, text="ki")
kd_label = ttk.Label(sliders_frame, text="kd")
kp_outerloop_label = ttk.Label(sliders_frame, text="kp_outerloop")

throttle_slider = tk.Scale(sliders_frame, variable=throttle, from_=0, to=100, orient=tk.HORIZONTAL)
kp_slider = tk.Scale(sliders_frame, variable=kp, from_=0, to=10, resolution=0.1, orient=tk.HORIZONTAL)
ki_slider = tk.Scale(sliders_frame, variable=ki, from_=0, to=10, resolution=0.1, orient=tk.HORIZONTAL)
kd_slider = tk.Scale(sliders_frame, variable=kd, from_=0, to=10, resolution=0.1, orient=tk.HORIZONTAL)
kp_outerloop_slider = tk.Scale(sliders_frame, variable=kp_outerloop, from_=0, to=10, resolution=0.1, orient=tk.HORIZONTAL)

# Create buttons for arming and disarming
update_button = ttk.Button(remaining_frame, text="Update Parameters", command=update_parameters)

# Create arrow buttons for control
up_button = ttk.Button(direction_buttons_frame, text="Up")
down_button = ttk.Button(direction_buttons_frame, text="Down")
left_button = ttk.Button(direction_buttons_frame, text="Left")
right_button = ttk.Button(direction_buttons_frame, text="Right")

# Create label for connection status
connection_status = tk.StringVar(value="Connection Status: NOT CONNECTED - No MPU found")
connection_label = ttk.Label(connection_status_frame, textvariable=connection_status, foreground='red')

# Layout the widgets in the frames
throttle_label.grid(row=0, column=0, padx=10, pady=5)
kp_label.grid(row=1, column=0, padx=10, pady=5)
ki_label.grid(row=2, column=0, padx=10, pady=5)
kd_label.grid(row=3, column=0, padx=10, pady=5)
kp_outerloop_label.grid(row=4, column=0, padx=10, pady=5)

throttle_slider.grid(row=0, column=1, padx=10, pady=5)
kp_slider.grid(row=1, column=1, padx=10, pady=5)
ki_slider.grid(row=2, column=1, padx=10, pady=5)
kd_slider.grid(row=3, column=1, padx=10, pady=5)
kp_outerloop_slider.grid(row=4, column=1, padx=10, pady=5)

update_button.pack(side=tk.LEFT, padx=10)

up_button.grid(row=0, column=1, padx=10, pady=5)
down_button.grid(row=2, column=1, padx=10, pady=5)
left_button.grid(row=1, column=0, padx=10, pady=5)
right_button.grid(row=1, column=2, padx=10, pady=5)

connection_label.pack(side=tk.LEFT, padx=10)

# Pack the frames using .grid()
sliders_frame.grid(row=0, column=0)
direction_buttons_frame.grid(row=0, column=1)
remaining_frame.grid(row=1, column=0, columnspan=2)
connection_status_frame.grid(row=2, column=0, columnspan=2)

# Update connection status text and color
if is_connected():
    connection_status.set("Connection Status: CONNECTED - MPU found")
    connection_label.configure(foreground='green')
else:
    connection_status.set("Connection Status: NOT CONNECTED - No MPU found")
    connection_label.configure(foreground='red')

# Bind arrow key events to the key press function
app.bind("<Up>", on_key_press)
app.bind("<Down>", on_key_press)
app.bind("<Right>", on_key_press)
app.bind("<Left>", on_key_press)

# Update parameters when there is any change
throttle.trace("w", lambda *args: update_parameters())
kp.trace("w", lambda *args: update_parameters())
ki.trace("w", lambda *args: update_parameters())
kd.trace("w", lambda *args: update_parameters())
kp_outerloop.trace("w", lambda *args: update_parameters())

app.mainloop()
