import tkinter as tk 
from urdfpy.urdf import URDF
import tkinter.ttk as ttk
import matplotlib.pyplot as plt
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def forward_kinematics():
    pass

def inverse_kinematics():
    pass

robot = URDF.load('/home/gokul/ROS/iroc_ws/src/galileo_description/urdf/rover.urdf')
robot.show()

window = tk.Tk()
window.minsize(960, 720)
window.title("Robotic Arm Kinematics")

style = ttk.Style(window)
style.configure('lefttab.TNotebook', tabposition='wn', tabmargins=[0, 5, 0, 0])
style.configure('lefttab.TNotebook.Tab', width=16, font=('Arial', 11), padding=[20, 5], background="white")
style.map('lefttab.TNotebook.Tab', background=[('selected', 'coral1')], foreground=[('selected', 'black')])
style.configure('lefttab.TNotebook', background='slate blue')

notebook = ttk.Notebook(window, style='lefttab.TNotebook')
notebook.pack(fill='both', expand=True)

tab1_frame = tk.Frame(notebook, width=960, height=720, bg='IndianRed2')
tab2_frame = tk.Frame(notebook, width=960, height=720, bg='IndianRed2')
tab3_frame = tk.Frame(notebook, width=960, height=720, bg='IndianRed2')

notebook.add(tab1_frame, text="Visualizer")
notebook.add(tab2_frame, text="Forward Kinematics")
notebook.add(tab3_frame, text="Inverse Kinematics")

joint_first3_frame = tk.Frame(master=tab2_frame)
joint_last3_frame = tk.Frame(master=tab2_frame)

heading = tk.Label(
    text="Forward Kinematics Solver",
    fg="white",
    bg="black",
    width=30,
    master=tab2_frame,
    font=("Arial", 20)
)

heading.pack(pady = 10)

image_path = "fk_image.png"
original_image = Image.open(image_path)
resized_image = original_image.resize((540, 320))
image = ImageTk.PhotoImage(resized_image)

image_label = tk.Label(tab2_frame, image=image)
image_label.pack(pady=10)

joint_first3_labels = ["Joint Angle 1:", "Joint Angle 2:", "Joint Angle 3:"]
joint_last3_labels = ["Joint Angle 4:", "Joint Angle 5:", "Joint Angle 6:"]
joint_entries = []

for i, label_text in enumerate(joint_first3_labels):
    label = tk.Label(
        text=label_text,
        master=joint_first3_frame
    )
    label.pack(side="left", padx=10)
    entry = tk.Entry(
        master=joint_first3_frame,
        width=10
    )
    entry.pack(side="left", padx=10)
    joint_entries.append(entry)

for i, label_text in enumerate(joint_last3_labels):
    label = tk.Label(
        text=label_text,
        master=joint_last3_frame
    )
    label.pack(side="left", padx=10)
    entry = tk.Entry(
        master=joint_last3_frame,
        width=10
    )
    entry.pack(side="left", padx=10)
    joint_entries.append(entry)

joint_first3_frame.pack(pady=10)
joint_last3_frame.pack()

fk_button = tk.Button(tab2_frame, text="Calculate Pose", command=forward_kinematics)
fk_button.pack(pady=10)

positions_frame = tk.Frame(master=tab3_frame)
quaternions_frame = tk.Frame(master=tab3_frame)

heading = tk.Label(
    text="Inverse Kinematics Solver",
    fg="white",
    bg="black",
    width=30,
    master=tab3_frame,
    font=("Arial", 20)
)

heading.pack(pady = 10)

image_path_2 = "ik_image.jpeg"
original_image_2 = Image.open(image_path_2)
resized_image_2 = original_image_2.resize((540, 320))
image_2 = ImageTk.PhotoImage(resized_image_2)

image_label_2 = tk.Label(tab3_frame, image=image_2)
image_label_2.pack(pady=10)

position_labels = ["Pos X:", "Pos Y:", "Pos Z:"]
position_entries = []

for i, label_text in enumerate(position_labels):
    label = tk.Label(
        text=label_text,
        master=positions_frame
    )
    label.pack(side="left")
    entry = tk.Entry(
        master=positions_frame,
        width=20
    )
    entry.pack(side="left", padx=4)
    position_entries.append(entry)

quaternion_labels = ["Quat X:", "Quat Y:", "Quat Z:", "Quat W:"]
quaternion_entries = []

for i, label_text in enumerate(quaternion_labels):
    label = tk.Label(
        text=label_text, 
        master=quaternions_frame
    )
    label.pack(side="left")
    entry = tk.Entry(
        master=quaternions_frame,
        width=13, 
    )
    entry.pack(side="left")

positions_frame.pack(pady=10)
quaternions_frame.pack(pady=10)

ik_button = tk.Button(tab3_frame, text="Calculate Joints", command=inverse_kinematics)
ik_button.pack(pady=10)

window.mainloop()