#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String, Int32
from vosk import Model, KaldiRecognizer
import pyaudio
import json
import re

# USAGE
# Este codigo espera a que se publique un 1 en el topic /pedidos_start para iniciar
# Una vez iniciado procesa la entrada de audio hasta que se dice "quiero pedir", 
# entonces pasa al modo de leer pedidos hasta que lea "ya está"
# Una vez terminado la lista se publican los pedidos en /pedidos y se envia un 1 a /pedidos_end

# Variables iniciales
pedidos = []
modo_camarero = False
escuchando = False  
calibrado = False

# Inicializar nodo y publicadores de ROS
rospy.init_node('reconocimiento_voz')
pub_pedidos = rospy.Publisher('/pedidos', String, queue_size=10)
pub_pedidos_end = rospy.Publisher('/pedidos_end', Int32, queue_size=10)

# Lista de productos disponibles
productos_disponibles = ["agua", "cerveza", "papas", "café", "tostada", "bocadillo"]

# Cargar modelo Vosk
MODEL_PATH = "/home/david/vosk-model-es-0.42" 
 
if not os.path.exists(MODEL_PATH):
    rospy.logerr(f"Modelo de Vosk no encontrado en: {MODEL_PATH}")
    exit(1)

# Configurar el modelo
model = Model(MODEL_PATH)
recognizer = KaldiRecognizer(model, 16000, "hotword")

# Configurar PyAudio
audio = pyaudio.PyAudio()
stream = audio.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
stream.start_stream()

# Funcion para separar los pedidos en nº objetos y tipo objeto que se ha pedido
def procesar_pedido(texto):
    patron_numeros = r"\b(\d+|un|uno|una|unos|unas|dos|tres|cuatro|cinco|seis|siete|ocho|nueve|diez)\b"
    texto = texto.lower()

    # Diccionario para convertir números escritos en palabras a dígitos
    palabras_a_numeros = {
        "un": 1, "uno": 1, "una": 1, "unos": 1, "unas": 1, "dos": 2, "tres": 3, "cuatro": 4,
        "cinco": 5, "seis": 6, "siete": 7, "ocho": 8, "nueve": 9, "diez": 10
    }

    # Dividir el texto en palabras y buscar productos disponibles
    for producto in productos_disponibles:
        if producto in texto:
            # Buscar el número más cercano al producto
            palabras = texto.split()
            for i, palabra in enumerate(palabras):
                if producto in palabra:
                    # Buscar hacia atrás por un número
                    for j in range(max(0, i - 3), i):
                        if re.match(patron_numeros, palabras[j]):
                            cantidad = palabras[j]
                            # Convertir cantidad a número si está en palabras
                            if cantidad in palabras_a_numeros:
                                cantidad = palabras_a_numeros[cantidad]
                            else:
                                cantidad = int(cantidad.replace(",", "").replace(".", ""))
                            pedidos.append((cantidad, producto))
                            break
    return pedidos

# Inicio del codigo cuando se recibe un "1" por el topic 
def callback_start(msg):
    global escuchando
    if msg.data == 1:
        escuchando = True
        rospy.loginfo("Modo de escucha activado")

sub_pedidos = rospy.Subscriber('/pedidos_start', Int32, callback_start)

# Bucle principal del programa
while not rospy.is_shutdown():
    # Si se ha activado el nodo
    if escuchando:
        try:
            # Leer datos de audio desde el micrófono
            data = stream.read(4096, exception_on_overflow=False)
            if recognizer.AcceptWaveform(data):
                resultado = json.loads(recognizer.Result())
                texto = resultado.get("text", "").lower()
                if texto:
                    rospy.loginfo(f"Has dicho: {texto}")

                    # Activar modo camarero si se detecta "quiero pedir"
                    if "quiero pedir" in texto:
                        modo_camarero = True

                    # Si se activa el modo camarero procesamos los pedidos
                    if modo_camarero:
                        procesar_pedido(texto)
                        rospy.loginfo(f"Pedidos actuales: {pedidos}")

                    # Publicar lista de pedidos y finalizar si se detecta "ya está"
                    if "ya está" in texto:
                        modo_camarero = False
                        escuchando = False
                        lista_pedidos = ', '.join([f"{cantidad} {producto}" for cantidad, producto in pedidos])
                        pub_pedidos.publish(lista_pedidos)
                        rospy.loginfo(f"Pedidos publicados: {lista_pedidos}")
                        pub_pedidos_end.publish(1)
                        pedidos.clear()
                        lista_pedidos = None
                        
        except Exception as e:
            rospy.logerr(f"Error en la captura de audio: {e}")

