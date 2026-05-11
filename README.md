# Lab-1: Planificación de Trayectorias Cartesianas

## Introducción

## Introducción

En esta práctica se aborda la planificación de trayectorias cartesianas para el manipulador robótico propuesto utilizando ROS 2 y cinemática inversa basada en KDL. El objetivo principal es generar trayectorias suaves del efector final en el espacio operacional mediante la interpolación de posición y orientación entre distintas poses cartesianas. La práctica queda dividida en dos bloques principales:

1. Interpolación cartesiana entre poses:
   - Interpolación lineal de la posición.
   - Interpolación de la orientación mediante cuaternios, calculando el cuaternio relativo entre poses y escalando el ángulo de rotación.

2. Generación suave de trayectorias:
   - Suavizado de la trayectoria en puntos intermedios utilizando el método de Taylor.
   - Eliminación de discontinuidades en velocidad y reducción de aceleraciones bruscas.

Finalmente, las referencias cartesianas generadas se transforman en trayectorias articulares mediante cinemática inversa y se ejecutan utilizando un controlador de posición articular.

## Tarea 1: Interpolación Cartesiana

En este ejercicio se solicita la implementación de una función que, a partir de una pose inicial y final, calcule poses intermedias entre estas: "PoseInterpolation(start_pose, end_pose, lambda)". Siendo "lambda" el parámetro que indica cuánto ha avanzado el movimiento: Será "0" para el incio y "1" para el final de este.

### Fundamentos teóricos

1) Cálculo de la posición:

Se trata de una interpolación lineal entre la posición final e incial, mediante el parámetro "lambda". Para ello, se extrae estas posiciones de las matrices de transformación homogénea:

$$
p_0 =
\begin{bmatrix}
T_0(0,3) \\
T_0(1,3) \\
T_0(2,3)
\end{bmatrix}
,\qquad
p_1 =
\begin{bmatrix}
T_1(0,3) \\
T_1(1,3) \\
T_1(2,3)
\end{bmatrix}
$$

A continuación, se aproxima las posiciones intermedias según:

$$
p(\lambda) = p_0 + \lambda (p_1 - p_0)
$$

2) Cálculo de la orientación:

Análogamente, se procede a extraer las matrices de orientación de las poses iniciales y finales desde sus matrices de transformación homogénea

$$
R_0 =
\begin{bmatrix}
T_0(0,0) & T_0(0,1) & T_0(0,2) \\
T_0(1,0) & T_0(1,1) & T_0(1,2) \\
T_0(2,0) & T_0(2,1) & T_0(2,2)
\end{bmatrix}
$$

$$
R_1 =
\begin{bmatrix}
T_1(0,0) & T_1(0,1) & T_1(0,2) \\
T_1(1,0) & T_1(1,1) & T_1(1,2) \\
T_1(2,0) & T_1(2,1) & T_1(2,2)
\end{bmatrix}
$$

Posteriormente, se realizan sus conversiones a cuaternios mediante la función otorgada:

$$
q = \text{rot2Quat}(R)
$$

Además, se calcula el cuaternio inverso de la orientación inicial mediante otra función proporcionada:

$$
q_A^{-1} = \text{InverseQuaternion}(q_A)
$$

Con el objetivo de obtener la rotación necesaria para transicionar entre la orientación inicial y la final, se calcula el cuaternio relativo:

$$
q_C = q_A^{-1} q_B
$$

El ejercicio continua expresando el cuaternio relativo en formato eje-ángulo, útil para cálculos posteriores:

$$
\theta = 2 \arccos(w)
$$

$$
n_c = \frac{v}{\sin(\theta/2)}
$$

Se procede a interpolar el ángulo mediante el parámetro $\lambda$, consiguiendo una transición suave entre movimientos (y evitando un cambio abrupto de orientación):

$$
\theta_\lambda = \lambda \theta
$$

A partir de este ángulo se genera un cuaternio parcial, que representa una "parte" del movimiento:

$$
w_{rot} = \cos\left(\frac{\theta_\lambda}{2}\right)
$$

$$
v_{rot} = n_c \sin\left(\frac{\theta_\lambda}{2}\right)
$$

Finalmente, el cuaternio interpolado se obtiene aplicando la rotación parcial sobre la orientación inicial:

$$
q_{interp} = q_A q_{rot}
$$

### Resolución

texto

## Tarea 2: Generación Suave de Trayectorias

### Fundamentos teóricos

### Resolución

archivos con los plots con los datos
data_20260505_171605.csv -> tau = 1 ; T = 10
data_20260505_172846.csv -> tau = 10; T = 10
data_20260505_175915.csv -> tau = 10; T = 100

What happens when you change the value of τ? (se ve resultado comparando las gráficas) 
al aumentar tau, el paso por el punto es más suave pero pasa más lejos del punto. ya que tau es el momento en el cual comienza y finaliza de "el arco". (No sé como explicarlo mejor)

What happens when you change the value of T? La velocidad del robot cambia ya que T es el tiempo que debe tardar el robot en completar un camino

Can you change the velovity of the robot? How? Si, cambiando T



