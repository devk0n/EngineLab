# Particle Constraints

### Particle Distance Constraints

Removes 1 degree of freedom.

$$
d = r_2 - r_1 
\quad
\dot{d} = \dot{r_2} - \dot{r_1}
\quad
\ddot{d} = \ddot{r_2} - \ddot{r_1}
$$

$$
\Phi = d^T d - l^2 
$$

$$
J = 
\begin{bmatrix} -2d^T & 2d^T \end{bmatrix}
\begin{bmatrix} \dot{r_1} \cr \dot{r_2} \end{bmatrix}
$$

$$
\gamma = -2\dot{d}^T \dot{d}
$$