# Laboratorio No 4 Cinematica Directa
**Juliana Gongora Rasmussen**


_Ingeniería Mecatrónica_

Correo: jugongorar@unal.edu.co

---


**Gerhaldine Alejandra Suárez Bernal**
  
  _Ingeniería Mecatrónica_

Correo: gesuarezb@unal.edu.co
## Descripción de la solución planteada

```mermaid
graph TD
    Base([Base])
    J1((J1))
    J2((J2))
    J3((J3))
    J4((J4))
    J5((J5))

    Base -->|11.04 cm| J1
    J1 -->|10.3 cm| J2
    J2 -->|10.33 cm| J3
    J3 -->|9.12 cm| J4
    J4 -->|9.12 cm| J5

```
Para  la solución plantada se combinaron herramientas de control de hardware, visualización gráfica e interacción de usuario para operar el manipulador Phantom X Pincher utilizando ROS 2, Python y servomotores Dynamixel. El objetivo fue desarrollar un sistema completo que permitiera enviar comandos de posición, visualizar el estado articular del robot y graficar su configuración desde una interfaz intuitiva.

### Componentes principales
Control directo con Dynamixel SDK
Se utilizó la biblioteca oficial de Dynamixel (dynamixel_sdk) para establecer comunicación con los servomotores AX-12 a través del puerto /dev/ttyUSB0. El script inicializa cada motor, activa el torque y permite enviar comandos en tiempo real.

### Definición de poses articulares
Se establecieron cinco configuraciones articulares predefinidas. Estas posiciones se expresan en grados y representan distintas poses del brazo robótico. Antes de ejecutar cualquier movimiento, el sistema valida que los ángulos se encuentren dentro de los límites permitidos por cada junta.

### Conversión entre grados y bits
Se implementaron funciones para traducir los valores de ángulos humanos (grados) a valores en bits utilizados por los motores, y viceversa, permitiendo mostrar y recibir información interpretable por el usuario.

### Visualización de la configuración del robot
Se desarrolló una herramienta gráfica usando matplotlib que genera una vista lateral del robot en la pose seleccionada o actual. Esta representación ayuda a verificar visualmente la validez y similitud entre la simulación y la configuración física.

### Interfaz gráfica con Tkinter
Se diseñó una GUI amigable que permite:

- Seleccionar y ejecutar las cinco poses predefinidas.

- Leer la posición actual del robot.

- Visualizar los ángulos de cada articulación.

- Mostrar información del grupo y el logo del laboratorio.
Esta interfaz también puede usarse para tomar capturas o grabar videos de prueba de forma más sencilla.

### Integración con ROS 2
La solución se encapsula en una clase que extiende de rclpy.node.Node, permitiendo su ejecución como un nodo ROS. Aunque el enfoque fue directo y no se usaron controladores avanzados como joint_trajectory_controller, el diseño modular permite escalar fácilmente a una integración más completa en ROS.

## Diagrama de flujo de acciones del robot 

## Plano de planta de la ubicaci´on de cada uno de los elementos.
## Descripción de las funciones utilizadas.
## Código del script utilizado para el desarrollo de la pr´actica.
## Vídeo del brazo alcanzando cada posición solicitada.
## Vídeo demostración de uso de la interfaz de usuario.
## Los videos debe comenzar con la introducci´on oficial del laboratorio LabSIR Intro LabSIR.
## Gráfica digital de las poses compar´andola con la fotograf´ıa del brazo real en la misma configuraci´on.

