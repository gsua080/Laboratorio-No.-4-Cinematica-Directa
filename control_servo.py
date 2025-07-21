import rclpy
from tkinter import Tk, Button, Label, Entry, PhotoImage, Frame, StringVar

from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
import os
from dynamixel_sdk import PortHandler, PacketHandler
import matplotlib.pyplot as plt
import numpy as np

# Configurar rutas
script_dir = os.path.dirname(os.path.abspath(__file__))
logo_path = os.path.join(script_dir, "logo.png")

# IDs motores y límites
MOTOR_IDS = [1, 2, 3, 4, 5]
LIMITS_DEG = {
    1: (-150, 150),
    2: (-90, 90),
    3: (-90, 90),
    4: (-90, 90),
    5: (-90, 90),
}

# Conversiones
def deg_to_bits(deg):
    return int((deg / 150) * 511 + 512)

def bits_to_deg(bits):
    return round(((bits - 512) / 511) * 150, 1)

def graficar_robot(q):
    l1, l2, l3, l4 = 1, 1, 1, 0.5
    q_rad = np.radians(q)
    x0, y0 = 0, 0
    x1 = x0 + l1 * np.cos(q_rad[0])
    y1 = y0 + l1 * np.sin(q_rad[0])
    x2 = x1 + l2 * np.cos(q_rad[0] + q_rad[1])
    y2 = y1 + l2 * np.sin(q_rad[0] + q_rad[1])
    x3 = x2 + l3 * np.cos(q_rad[0] + q_rad[1] + q_rad[2])
    y3 = y2 + l3 * np.sin(q_rad[0] + q_rad[1] + q_rad[2])
    x4 = x3 + l4 * np.cos(q_rad[0] + q_rad[1] + q_rad[2] + q_rad[3])
    y4 = y3 + l4 * np.sin(q_rad[0] + q_rad[1] + q_rad[2] + q_rad[3])

    xs = [x0, x1, x2, x3, x4]
    ys = [y0, y1, y2, y3, y4]
    plt.figure()
    plt.plot(xs, ys, '-o')
    plt.xlim(-4, 4)
    plt.ylim(-4, 4)
    plt.title("Configuración del manipulador")
    plt.grid(True)
    plt.show()

POSES = {
    'Pose 1': [0, 0, 0, 0, 0],
    'Pose 2': [25, 25, 20, -20, 0],
    'Pose 3': [-35, 35, -30, 30, 0],
    'Pose 4': [85, -20, 55, 25, 0],
    'Pose 5': [80, -35, 55, -45, 0],
}



class ArticulationController(Node):
    def __init__(self):
        super().__init__('articulation_controller')
        self.port = PortHandler('/dev/ttyUSB0')
        self.packet = PacketHandler(1.0)
        self.port.openPort()
        self.port.setBaudRate(1000000)

        for dxl_id in MOTOR_IDS:
            self.packet.write1ByteTxRx(self.port, dxl_id, 24, 1)  

        self.joint_positions = [0] * 5
        self.create_gui()

    def move_to_pose(self, pose_name):
        angles = POSES[pose_name]
        for i, dxl_id in enumerate(MOTOR_IDS):
            ang = angles[i]
            if not (LIMITS_DEG[dxl_id][0] <= ang <= LIMITS_DEG[dxl_id][1]):
                self.get_logger().warn(f"ID {dxl_id}: {ang} fuera de límites")
                continue
            goal = deg_to_bits(ang)
            self.packet.write2ByteTxRx(self.port, dxl_id, 30, goal)
            time.sleep(0.5)
        graficar_robot(angles)

    def read_angles(self):
        for i, dxl_id in enumerate(MOTOR_IDS):
            pos, _, _ = self.packet.read2ByteTxRx(self.port, dxl_id, 36)
            self.joint_positions[i] = bits_to_deg(pos)
        self.update_joint_labels()
        graficar_robot(self.joint_positions)

    def update_joint_labels(self):
        for i, val in enumerate(self.joint_positions):
            self.joint_labels[i].set(f"Servo {i+1}: {val:.1f} grados")


    def create_gui(self):
        self.window = Tk()
        self.window.title("Interfaz Lab 4")
        self.window.geometry("1400x1200")
        self.window.configure(bg="#DBFFCB") 

        # Encabezado
        header = Frame(self.window, bg="#ffffff")
        header.pack(pady=10)

        Label(header, text="Robótica 2025-I", font=("Helvetica", 18, "bold"), bg="#f0f0f0", fg="#333").pack()
        Label(header, text="Grupo: Gerhaldine Suárez, Juliana Góngora", font=("Helvetica", 14), bg="#f0f0f0", fg="#555").pack()

        # Logo
        try:
            logo_path = os.path.join(os.path.dirname(__file__), "lab_sir.png")
            logo_img = PhotoImage(file=logo_path)
            logo_label = Label(self.window, image=logo_img, bg="#f0f0f0")
            logo_label.image = logo_img  # Previene que se borre
            logo_label.pack(pady=10)
        except Exception as e:
            Label(self.window, text="[Logo no cargado]").pack()
            print("Error cargando logo:", e)

        # Selección de poses
        pose_frame = Frame(self.window, bg="#f0f0f0")
        pose_frame.pack(pady=20)

        Label(pose_frame, text="Selecciona una pose:", font=("Helvetica", 14), bg="#f0f0f0").pack(pady=5)

        for name in POSES:
            Button(
                pose_frame, text=name, width=20,
                font=("Helvetica", 12), bg="#FF8282", fg="white",
                relief="flat", command=lambda n=name: self.move_to_pose(n)
            ).pack(pady=5)

        # Botón para leer posición actual
        Button(
            self.window, text="Leer Posición Actual",
            font=("Helvetica", 12), bg="#FF6363", fg="white",
            relief="flat", command=self.read_angles
        ).pack(pady=15)

        # Lectura de valores articulares
        angle_frame = Frame(self.window, bg="#BEE4D0", bd=1, relief="solid")
        angle_frame.pack(pady=10)

        Label(angle_frame, text="Ángulos actuales de los motores:", font=("Helvetica", 12, "bold"), bg="#ffffff").pack(pady=5)

        self.joint_labels = [StringVar() for _ in MOTOR_IDS]
        for i, lbl in enumerate(self.joint_labels):
            Label(angle_frame, textvariable=lbl, bg="#ffffff", font=("Helvetica", 11)).pack()

        self.window.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ArticulationController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()