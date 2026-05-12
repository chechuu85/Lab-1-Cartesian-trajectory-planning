# Lab-1: Planificación de Trayectorias Cartesianas

## Introducción

En esta práctica se aborda la planificación de trayectorias cartesianas para el manipulador robótico propuesto utilizando ROS 2 y cinemática inversa basada en KDL. El objetivo principal es generar trayectorias suaves del efector final en el espacio operacional mediante la interpolación de posición y orientación entre distintas poses cartesianas. La práctica queda dividida en dos bloques principales:

1. Interpolación cartesiana entre poses:
   - Interpolación lineal de la posición.
   - Interpolación de la orientación mediante cuaternios, calculando el cuaternio relativo entre poses y escalando el ángulo de rotación.

2. Generación suave de trayectorias:
   - Suavizado de la trayectoria en puntos intermedios utilizando el método de Taylor.
   - Eliminación de discontinuidades en velocidad y reducción de aceleraciones bruscas.

Finalmente, las referencias cartesianas generadas se transforman en trayectorias articulares mediante cinemática inversa y se ejecutan utilizando un controlador de posición articular.

---

## Tarea 1: Interpolación Cartesiana

En este ejercicio se solicita la implementación de una función ("PoseInterpolation(start_pose, end_pose, lambda)") que, a partir de una pose inicial y final, calcule una pose intermedia entre estas en función del parámetro $\lambda$: Indica cuánto ha avanzado el movimiento: Será "0" para el incio y "1" para el final de este.

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

### Resultados

Al implementarse el algoritmo previamente descrito, se reciben los siguientes datos, coincidiendo con los esperados:

FOTO*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Se trata de la comprobación de la función, devolviendo las tres poses originales. Se observa en la solución que la pose 1 coincide con la número 2. Esto es debido a que se han interpolado dos procesos:

    · 0 -> 1
    · 1 -> 2

En ambos se repite la pose intermedia y se introducen los valores de $\lambda$ "1" para el primer trayecto y "0" para el segundo, provocando que ambas líneas devuelvan las mismas poses (Ver el siguiente código).

```cpp
const auto [p0, q0] = PoseInterpolation(pose0, pose1, 0.0);
const auto [p1, q1] = PoseInterpolation(pose0, pose1, 1.0);
const auto [p2, q2] = PoseInterpolation(pose1, pose2, 0.0);
const auto [p3, q3] = PoseInterpolation(pose1, pose2, 1.0);
```

---

## Tarea 2: Generación Suave de Trayectorias

### Fundamentos teóricos

Una trayectoria suave (entre dos puntos) es un tipo de trayectoria en la que el movimiento cambia de forma continua, evitando saltos bruscos en posición, velocidad u orientación, apoyándose en un punto intermedio. Para ello, se divide la trayectoria en tres tramos:

FOTO++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

- $t < -\tau$ : Primer tramo lineal entre $pose_0$ y $pose_1$. Se realiza la interpolación de poses con $\lambda = (t + T)/T$.

- $t > \tau$ : Segundo tramo lineal entre $pose_1$ y $pose_2$. Se realiza la interpolación de poses con $\lambda = t/T$.

-**NOTA**: El parámetro $\lambda$ sigue las anteriores expresiones para garantizar que al principio del recorrido su valor sea "0" (t = -T) y al final del mismo sea "1" (t = T).

- $-\tau \leq t \leq \tau$ : Región de suavizado alrededor de la pose intermedia. En este procedimiento, la orientación se calcula a partir de la orientación intermedia $q_1$, aplicando dos rotaciones parciales. La primera tiene en cuenta la orientación del tramo anterior, mientras que la segunda incorpora progresivamente la orientación del tramo siguiente. De esta forma, se evita un cambio brusco de orientación en la pose intermedia. Se procede a desarrollar este algoritmo:

En primer lugar, se procede al estudio de la orientación. Para ello, se extraen las rotaciones de cada matriz de transformación homogénea, se transforman a cuaternios, se calculan cuaternios inversos de los dos primeros y los cuaternios relativos $q_0$ y $q_1$. Además, se extrae el eje y ángulo de estos últimos (Procedimientos análogos a los del primer ejercicio del informe).

Posteriormente, se calculan los ángulos parciales de la rotación, los cuales, representan la rotación continua a lo largo de la trayectoria suavizada.

$$
\theta_{k1} =
-\frac{(\tau-t)^2}{4\tau T}\theta_{01}
$$

$$
\theta_{k2} =
\frac{(\tau+t)^2}{4\tau T}\theta_{12}
$$

El siguiente paso es calcular el cuaternio asociado al primer tramo del suavizado.

$$
w_{k1} = \cos\left(\frac{\theta_{k1}}{2}\right)
$$

$$
v_{k1} = n_{01}\sin\left(\frac{\theta_{k1}}{2}\right)
$$

$$
q_{k1} =
\begin{bmatrix}
v_{k1} \\
w_{k1}
\end{bmatrix}
$$

Por otra parte, se repite el proceso para la segunda mitad del suavizado, la trayectoria más cercana al final de este.

$$
w_{k2} = \cos\left(\frac{\theta_{k2}}{2}\right)
$$

$$
v_{k2} = n_{12}\sin\left(\frac{\theta_{k2}}{2}\right)
$$

$$
q_{k2} =
\begin{bmatrix}
v_{k2} \\
w_{k2}
\end{bmatrix}
$$

Finalmente, se obtiene el cuaternio resultante del suavizado como producto de ambos.

$$
q_{interp} = q_1 q_{k1} q_{k2}
$$

El cálculo de la posición se realiza calculando los vectores de desplazamiento asociados a los dos tramos de la trayectoria:

$$
\Delta P_{01}=P_1-P_0
$$

$$
\Delta P_{12}=P_2-P_1
$$

Se finaliza el apartado aplicando la ecuación principal de suavizado estudiada en clases de teoría: 

$$
p(t)=
P_1
-\frac{(\tau-t)^2}{4\tau T}\Delta P_{01}
+\frac{(\tau+t)^2}{4\tau T}\Delta P_{12}
$$

### Resultados

archivos con los plots con los datos
data_20260505_171605.csv -> tau = 1 ; T = 10
data_20260505_172846.csv -> tau = 10; T = 10
data_20260505_175915.csv -> tau = 10; T = 100

What happens when you change the value of τ? (se ve resultado comparando las gráficas) 
al aumentar tau, el paso por el punto es más suave pero pasa más lejos del punto. ya que tau es el momento en el cual comienza y finaliza de "el arco". (No sé como explicarlo mejor)

What happens when you change the value of T? La velocidad del robot cambia ya que T es el tiempo que debe tardar el robot en completar un camino

Can you change the velovity of the robot? How? Si, cambiando T



