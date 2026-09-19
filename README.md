# Desafío Gazebo ROS2 — Cobertura Autónoma con TurtleBot3

Paquete ROS2 (Jazzy) para el proyecto final de "Proyecto de Robots II": un
TurtleBot3 Burger explora de forma autónoma el escenario `stage4` de Gazebo,
maximizando el área monitoreada mientras evita colisiones.

## Arquitectura

El nodo `turtlebot_ctrl` (paquete base provisto por el profesor, extendido
con la lógica de navegación) implementa:

1. **Percepción**: se suscribe a `/scan` (LaserScan) para detectar
   obstáculos y a `/odom` (Odometry) para conocer la posición del robot.
2. **Mapa discreto de cobertura**: una grilla 20x20 (resolución 0.25m)
   provista por el profesor, que marca cada celda como obstáculo (0), no
   visitada (1) o visitada (2). Cada vez que el robot pisa una celda nueva,
   se loguea el % de cobertura total.
3. **Decisión y control**: máquina de estados con dos modos:
   - `EXPLORE`: avanza en línea recta con un sesgo de giro aleatorio que
     cambia cada 8-15 segundos, para cubrir distintas zonas del mapa en
     vez de quedar dando vueltas en un mismo sector.
   - `AVOID`: al detectar un obstáculo dentro de la distancia de seguridad
     en el sector frontal (±45°), el robot retrocede brevemente para
     despegarse físicamente y luego gira hacia el lado con más espacio
     libre, comprometiéndose a esa dirección por un tiempo mínimo (evita
     quedar oscilando indeciso entre izquierda y derecha).

El cálculo del sector frontal usa el ángulo real de cada medición del láser
(`angle_min + i * angle_increment`, normalizado), en vez de asumir la
posición del índice central del array — necesario porque el LiDAR de este
robot barre de 0 a 2π, no de -π a π.

## Tópicos utilizados

| Tópico     | Tipo                          | Uso                              |
|------------|--------------------------------|------------------------------------|
| `/scan`    | `sensor_msgs/msg/LaserScan`    | Detección de obstáculos (entrada)  |
| `/odom`    | `nav_msgs/msg/Odometry`        | Posición del robot (entrada)       |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Comando de velocidad (salida)    |

## Escenario

`turtlebot3_dqn_stage4` (paquete `turtlebot3_gazebo`, repo
`turtlebot3_simulations`, rama `jazzy`): arena cuadrada con paredes
internas y dos obstáculos móviles (`obstacle1`, `obstacle2`).

## Dependencias

- ROS2 Jazzy + Gazebo Harmonic (`ros-jazzy-ros-gz`, `gz-harmonic`)
- Repos en el workspace: `turtlebot3`, `turtlebot3_msgs`,
  `turtlebot3_simulations` (rama `jazzy`)

## Compilación

Dentro del workspace `turtlebot3_ws` (que incluye este paquete junto a
`turtlebot3`, `turtlebot3_msgs` y `turtlebot3_simulations`, rama `jazzy`):

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select turtlebot3_control_ros2
source install/setup.bash
```

## Cómo ejecutar

Terminal 1 — levantar el simulador:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
```

Terminal 2 — correr el nodo de control:
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_control_ros2 turtlebot_ctrl
```

El % de cobertura se imprime en consola cada vez que el robot visita una
celda nueva del mapa discreto.

## Parámetros ajustables (en `turtlebot_ctrl.py`)

- `forward_speed`, `turn_speed`: velocidades de avance y giro.
- `safe_distance`: distancia frontal mínima antes de esquivar.
- `front_angle_deg`: ancho del sector frontal analizado.
- `avoid_ticks_left`: duración del comportamiento de esquive
  (retroceso + giro comprometido).
