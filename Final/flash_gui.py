import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import pickle

# Load the binary file
file_path = r"C:\MyFolder\Abhyuday\Codes\f4.dat"  # Update file location

with open(file_path, 'rb') as file:
    flight_data = pickle.load(file)

# Function to create plots
def plot_data(param):
    fig = Figure(figsize=(10, 6), dpi=100)
    ax = fig.add_subplot(111)
    ax.plot(flight_data['time'], flight_data[param], label=param, color='cyan')
    ax.set_xlabel('Time (μs)', color='black')
    ax.set_ylabel(param, color='black')
    ax.legend()
    ax.grid(True, color='gray')
    ax.tick_params(axis='x', colors='black')
    ax.tick_params(axis='y', colors='black')
    ax.spines['top'].set_color('black')
    ax.spines['bottom'].set_color('black')
    ax.spines['left'].set_color('black')
    ax.spines['right'].set_color('black')
    ax.title.set_color('black')

    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.draw()
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    canvas.get_tk_widget().config(bg='black')


def clear_plot():
    for widget in plot_frame.winfo_children():
        widget.destroy()


root = tk.Tk()
root.title("Flight Data Viewer")
root.configure(bg='black')

# Frame for the labels
label_frame = ttk.LabelFrame(root, text="Flight Data Summary", style="Custom.TLabelframe")
label_frame.pack(fill="both", expand=True, padx=10, pady=10)

ttk.Label(label_frame, text=f"Initial Pressure: {flight_data['initial_press']} Pa", style="Custom.TLabel").pack(anchor=tk.W)
ttk.Label(label_frame, text=f"Initial Temperature: {flight_data['initial_temp']} °C", style="Custom.TLabel").pack(anchor=tk.W)
ttk.Label(label_frame, text=f"Apogee: {flight_data['apogee']} m", style="Custom.TLabel").pack(anchor=tk.W)
ttk.Label(label_frame, text=f"Apogee Time: {flight_data['apogee_time']} μs", style="Custom.TLabel").pack(anchor=tk.W)

# Frame for plot options
option_frame = ttk.LabelFrame(root, text="Plot Options", style="Custom.TLabelframe")
option_frame.pack(fill="both", expand=True, padx=10, pady=10)

ttk.Label(option_frame, text="Select parameter to plot against time:", style="Custom.TLabel").pack(anchor=tk.W)
params = ["temp", "press", "filter_alt", "gyro_x", "gyro_y", "gyro_z", "mag_x", "mag_y", "mag_z", "accel_x", "accel_y", "accel_z", "bat_volt"]
selected_param = tk.StringVar()
param_menu = ttk.Combobox(option_frame, textvariable=selected_param, values=params, state="readonly", style="Custom.TCombobox")
param_menu.pack(anchor=tk.W, fill="x", padx=5, pady=5)

plot_button = ttk.Button(option_frame, text="Plot", command=lambda: [clear_plot(), plot_data(selected_param.get())], style="Custom.TButton")
plot_button.pack(anchor=tk.W, padx=5, pady=5)

plot_frame = ttk.LabelFrame(root, text="Plot", style="Custom.TLabelframe")
plot_frame.pack(fill="both", expand=True, padx=10, pady=10)

# Style configuration
style = ttk.Style()
style.theme_use('clam')
style.configure("Custom.TLabelframe", background='black', foreground='cyan')
style.configure("Custom.TLabel", background='black', foreground='cyan')
style.configure("Custom.TButton", background='black', foreground='cyan')
style.configure("Custom.TCombobox", background='black', foreground='cyan', fieldbackground='black', selectbackground='black', selectforeground='cyan')

root.mainloop()
