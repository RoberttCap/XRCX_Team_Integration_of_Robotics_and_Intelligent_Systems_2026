# Puzzlebot Sim

Paquete de ROS 2 Humble para simular dos Puzzlebot diferenciales sin Gazebo. El movimiento se calcula con cinemática diferencial, odometría por dead reckoning, TF, URDF y visualización en RViz.

El launch principal crea dos robots con namespace:

```text
/robot1
/robot2
```

Cada robot tiene su propio simulador, localización, publicador de joint states, controlador y `robot_state_publisher`.

## Estructura

```text
puzzlebot_sim/
├── launch/
│   └── minichallenge4.launch.py
├── meshes/
│   ├── Puzzlebot_Caster_Wheel.stl
│   ├── Puzzlebot_Jetson_Lidar_Edition_Base(1).stl
│   └── Puzzlebot_Wheel.stl
├── puzzlebot_sim/
│   ├── control.py
│   ├── goal_input.py
│   ├── joint_states.py
│   ├── localisation.py
│   └── puzzlebot_sim.py
├── rviz/
│   └── puzzlebot_rviz.rviz
├── urdf/
│   └── puzzlebot.urdf
├── package.xml
├── setup.py
└── setup.cfg
```

## Nodos

### `puzzlebot_sim`

Simula la cinemática diferencial del robot.

Por cada robot:

```text
sub: cmd_vel
pub: wr
pub: wl
pub: sim_x
pub: sim_y
pub: sim_theta
pub: pose_sim
```

En el launch de dos robots estos tópicos quedan namespaced:

```text
/robot1/cmd_vel
/robot1/wr
/robot1/wl

/robot2/cmd_vel
/robot2/wr
/robot2/wl
```

### `localisation`

Reconstruye la odometría usando `wr` y `wl`.

```text
sub: wr
sub: wl
pub: odom
```

Ejemplos:

```text
/robot1/odom
/robot2/odom
```

### `control`

Controlador go-to-goal. Lee la odometría y una meta `Pose2D`, y publica velocidades.

```text
sub: odom
sub: goal_pose
pub: cmd_vel
```

Ejemplo para `robot1`:

```text
/robot1/goal_pose -> /robot1/control -> /robot1/cmd_vel
```

Ejemplo para `robot2`:

```text
/robot2/goal_pose -> /robot2/control -> /robot2/cmd_vel
```

### `joint_states`

Publica TF y estados de las llantas para que RViz pueda mostrar el URDF.

```text
sub: odom
sub: wr
sub: wl
pub: joint_states
pub: /tf
pub: /tf_static
```

La cadena esperada es:

```text
map
├── robot1/odom
│   └── robot1/base_footprint
│       └── robot1/base_link
│           ├── robot1/wheel_l_link
│           ├── robot1/wheel_r_link
│           └── robot1/caster_link
└── robot2/odom
    └── robot2/base_footprint
        └── robot2/base_link
            ├── robot2/wheel_l_link
            ├── robot2/wheel_r_link
            └── robot2/caster_link
```

### `goal_input`

Nodo interactivo para mandar metas desde una sola terminal a `robot1`, `robot2` o ambos.

Publica en:

```text
/robot1/goal_pose
/robot2/goal_pose
```

También se suscribe a:

```text
/robot1/odom
/robot2/odom
```

para saber cuándo avanzar al siguiente punto de un path.

## Compilar

Desde el workspace:

```bash
cd ~/ros2_ws
PYTHONNOUSERSITE=1 colcon build --packages-select puzzlebot_sim
source install/setup.bash
```

`PYTHONNOUSERSITE=1` evita que Python use una versión de `setuptools` instalada en `~/.local` que puede romper el build en ROS 2 Humble.

## Correr la simulación

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch puzzlebot_sim minichallenge4.launch.py
```

Este launch abre:

```text
robot1 stack
robot2 stack
RViz
rqt_tf_tree
rqt_graph
```

## Mandar metas

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run puzzlebot_sim goal_input
```

Comandos disponibles:

```text
r1 x y theta
r2 x y theta
both x1 y1 th1 x2 y2 th2
all x y theta
r1 path x y th; x y th; ...
r2 path x y th; x y th; ...
all path x y th; x y th; ...
r1 square [lado]
r2 pentagon [lado]
all square [lado]
all pentagon [lado]
q
```

`theta` está en grados.

Ejemplos:

```text
r1 1.0 -0.6 0
r2 -1.0 0.6 180
```

Mandar metas diferentes a los dos robots:

```text
both 1.0 -0.6 0 -1.0 0.6 180
```

Mandar la misma meta a ambos:

```text
all 0.0 0.0 90
```

Mandar un path solo a `robot1`:

```text
r1 path 1.0 -0.6 0; 1.0 0.0 90; 0.0 0.0 180
```

Mandar el mismo path a ambos:

```text
all path 0.5 0.0 0; 0.5 0.5 90
```

Hacer una figura desde la pose actual:

```text
r1 square 0.5
r2 pentagon 0.4
all square 0.5
all pentagon 0.4
```

## Cambiar posiciones iniciales

Las poses iniciales se modifican en:

[launch/minichallenge4.launch.py](launch/minichallenge4.launch.py)

Busca estas líneas:

```python
robot_group('robot1', robot_desc, 1.0, 1.0, 0.0),
robot_group('robot2', robot_desc, 1.0, 0.5, 0.0),
```

La firma es:

```python
robot_group(namespace, robot_desc, x0, y0, theta0)
```

Donde:

```text
x0      posición inicial en X
y0      posición inicial en Y
theta0  orientación inicial en radianes
```

Ejemplo:

```python
robot_group('robot1', robot_desc, -0.8, -0.6, 0.0),
robot_group('robot2', robot_desc, 0.8, 0.6, 3.1416),
```

## RViz

RViz carga:

```text
rviz/puzzlebot_rviz.rviz
```

Puntos importantes:

- El `Fixed Frame` debe ser `map`.
- `RobotModel robot1` usa `/robot1/robot_description`.
- `RobotModel robot2` usa `/robot2/robot_description`.
- El `TF Prefix` en RViz debe ser `robot1` y `robot2`, sin diagonal final.

Correcto:

```text
robot1
robot2
```

Incorrecto:

```text
robot1/
robot2/
```

## Comandos útiles

Ver nodos:

```bash
ros2 node list
```

Ver tópicos:

```bash
ros2 topic list
```

Ver odometría:

```bash
ros2 topic echo /robot1/odom
ros2 topic echo /robot2/odom
```

Ver metas:

```bash
ros2 topic echo /robot1/goal_pose
ros2 topic echo /robot2/goal_pose
```

Ver TF:

```bash
ros2 run tf2_ros tf2_echo map robot1/base_footprint
ros2 run tf2_ros tf2_echo map robot2/base_footprint
```

## Archivos ignorados por Git

El `.gitignore` ignora archivos generados o locales que no son necesarios para correr el proyecto:

```text
build/
install/
log/
__pycache__/
.pytest_cache/
.vscode/
.codex
```

Los archivos importantes que sí se deben versionar son:

```text
launch/
meshes/
puzzlebot_sim/
rviz/
urdf/
package.xml
setup.py
setup.cfg
README.md
```
