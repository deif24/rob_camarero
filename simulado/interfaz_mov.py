#!/usr/bin/env python3
import tkinter as tk
from tkinter import scrolledtext
import rospy
from std_msgs.msg import String, Int32
import subprocess
import time

# Inicializar ROS
rospy.init_node('Interfaz', anonymous=True)

# Variables para contar los clics
ultimo_click = 0
tiempo_limite = 1  # Tiempo límite en segundos para considerar dos clics seguidos

# Función para manejar el botón de emergencia
def emergencia_handler():
    global ultimo_click, tiempo_limite
    # Obtener el tiempo actual
    tiempo_actual = time.time()

    # Comprobar si se ha pulsado dos veces en 1s
    if tiempo_actual - ultimo_click < tiempo_limite:
        # Bloquear todos los botones
        mesa1.config(state=tk.DISABLED)
        mesa2.config(state=tk.DISABLED)
        mesa3.config(state=tk.DISABLED)
        mesa4.config(state=tk.DISABLED)
        limpiar_btn.config(state=tk.DISABLED)
        emergencia_btn.config(state=tk.DISABLED)

        # Mostrar mensaje de emergencia
        emergencia_label.config(text="¡EMERGENCIA ACTIVADA!", fg="red")

        # Detener ROS completamente
        rospy.loginfo("Emergencia activada. Deteniendo ROS...")
        rospy.signal_shutdown("Emergencia activada, cerrando ROS.")
        subprocess.call(["killall", "roscore"])
        subprocess.call(["rosnode", "kill", "-a"])

    else:
        # Si el tiempo entre clics es mayor al límite, reiniciar el contador
        ultimo_click = tiempo_actual
        print("Presiona de nuevo para activar la emergencia.")

# Función para manejar los botones de las mesas
def mesa_handler(mesa):
    rospy.loginfo(f"Botón de la Mesa {mesa} presionado")
    pub_mesas.publish(mesa)
    print(f"Número de mesa {mesa} publicado en /mesa_seleccionada")

# Función para limpiar los pedidos
def limpiar_pedidos():
    pub_pedidos.publish("")  # Publicamos un mensaje vacío
    pub_boton_pedidos.publish(1)
    pedidos_text.delete('1.0', tk.END)
    print("Pedidos limpiados y mensaje publicado en /pedidos y /boton_pedidos")

# Función que actualiza el área de texto con los nuevos pedidos
def actualizar_pedidos(msg):
    pedidos_text.delete('1.0', tk.END)
    pedidos_text.insert(tk.END, msg.data)

# Crear los publicadores para los topics correspondientes
pub_mesas = rospy.Publisher('/seleccion_mesas', Int32, queue_size=10)
pub_pedidos = rospy.Publisher('/pedidos', String, queue_size=10)
pub_boton_pedidos = rospy.Publisher('/boton_pedidos', Int32, queue_size=10)

# Crear el suscriptor para escuchar los mensajes del topic /pedidos
def start_subscriber():
    rospy.Subscriber('/pedidos', String, actualizar_pedidos)

# Ventana principal
root = tk.Tk()
root.title("Interfaz restaurante")

# Configuración de las mesas
mesas_frame = tk.Frame(root)
mesas_frame.grid(row=0, column=0, padx=10, pady=10)

mesa1 = tk.Button(mesas_frame, text="Mesa 1", width=10, height=3, command=lambda: mesa_handler(1))
mesa2 = tk.Button(mesas_frame, text="Mesa 2", width=10, height=3, command=lambda: mesa_handler(2))
mesa3 = tk.Button(mesas_frame, text="Mesa 3", width=10, height=3, command=lambda: mesa_handler(3))
mesa4 = tk.Button(mesas_frame, text="Mesa 4", width=10, height=3, command=lambda: mesa_handler(4))

mesa1.grid(row=0, column=0, padx=5, pady=5)
mesa2.grid(row=0, column=1, padx=5, pady=5)
mesa3.grid(row=1, column=0, padx=5, pady=5)
mesa4.grid(row=1, column=1, padx=5, pady=5)

# Botón de emergencia
emergencia_btn = tk.Button(root, text="EMERGENCIA", bg="red", fg="white", width=20, height=2, command=emergencia_handler)
emergencia_btn.grid(row=1, column=0, padx=10, pady=10)

# Configuración de la tabla de pedidos
pedidos_frame = tk.Frame(root)
pedidos_frame.grid(row=0, column=1, rowspan=3, padx=10, pady=10)

tk.Label(pedidos_frame, text="Pedidos").pack()
pedidos_text = scrolledtext.ScrolledText(pedidos_frame, width=30, height=20)
pedidos_text.pack(pady=5)

# Botón para limpiar pedidos
limpiar_btn = tk.Button(pedidos_frame, text="Limpiar Pedidos", bg="blue", fg="white", width=20, height=2, command=limpiar_pedidos)
limpiar_btn.pack(pady=10)

# Etiqueta para mostrar el estado de emergencia
emergencia_label = tk.Label(root, text="", font=("Helvetica", 16), fg="red")
emergencia_label.grid(row=3, column=0, columnspan=2)

# Iniciar el suscriptor después de que la interfaz esté lista
start_subscriber()

# Inicia el loop principal
root.mainloop()

