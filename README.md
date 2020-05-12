# PSDM

Pseudo-Symbolic Dynamic Modelling matlab package.

PSDM is a numerical method of deriving the equations of motion of an arbitrary rigid body chain, in regressor form. It is an alternative means of deriving the equations of motion for a rigid, serial kinematic chain. The result is a numerically represented model in a highly organized form, which allows for many symbolic manipulations through numerical methods. This allows for

 - The generation of very fast real time code.
 - Automatic model simplification (to achieve faster evaluation times)
 - Both forward and inverse dynamic modelling in a single derivation and with the same inertial parameter set.

To use PSDM, you need to first derive a PSDM model. This will return an _exponent matrix_ $\mathbf{E}$ and a _reduction page matrix_ $\mathbf{P}$. These matrices contain the full information to compute the forward and inverse dynamic model.

## Derivation

Derivation of the PSDM model can be accomplished in a single line of code with one of two calling syntaxes. 

```matlab
	% Can derive model from a DH table and a gravity vector.
	[E, P] = PSDM.deriveModel(DH, g);
	
	% Or, can instead give an anonymous function which evaluates the 
	% inverse dynamics of the manipulator.
	[E, P] = PSDM.deriveModel(@inverseDynamicsFunc, jointTypes);
```

### Kinematic calling syntax

If the DH / g syntax is used, the DH table should be given as a DOF x 6 matrix:
| ![a_1](https://render.githubusercontent.com/render/math?math=a_1) | ![alpha_1](https://render.githubusercontent.com/render/math?math=\alpha_1) | ![d_1](https://render.githubusercontent.com/render/math?math=d_1) | ![theta_1](https://render.githubusercontent.com/render/math?math=\theta_1) | ![t_1](https://render.githubusercontent.com/render/math?math=t_1) |![s_1](https://render.githubusercontent.com/render/math?math=s_1)
|:--:|:--:|:--:|:--:|:--:|:--:|
| : | : | : | : | : | : |
| ![a_n](https://render.githubusercontent.com/render/math?math=a_n) | ![alpha_n](https://render.githubusercontent.com/render/math?math=\alpha_n) | ![d_n](https://render.githubusercontent.com/render/math?math=d_n) | ![theta_n](https://render.githubusercontent.com/render/math?math=\theta_n) | ![t_n](https://render.githubusercontent.com/render/math?math=t_n) |![s_n](https://render.githubusercontent.com/render/math?math=s_n)

where the DH variables are as defined by Spong et al (2008):

 - $a_i$ is the distance along the x axis from frame $o_{i-1}$ to $o_i$;
 - $\alpha_i$ is the angular rotation about $x_i$ from $o_{i-1}$ to $o_i$;
 - $d_i$ is the linear displacement along the $z$-axis from $o_{i-1}$ to $o_i$;
 - $\theta_i$ the angular rotation about the $z$-axis from $o_{i-1}$ to $o_i$;
 - $t_i$ is a number representing the joint type -- 0 indicates a revolute joint, 1 indicates a prismatic joint
 - $s_i$ is either -1 or 1, denoting the direction of the joint.

The variables $t_i$ and $s_i$ are combined such that, for each joint, we have
$$
d_i^* = d_i + t_i s_i q_i\\
\theta_i^* = \theta_i + (1-t_i)s_i q_i
$$

The gravity vector is a unit vector which points "upwards" (i.e. against gravity). If omitted, a gravity vector of
$$
\mathbf{g} = \begin{bmatrix} 0 & 0 & 1\end{bmatrix}^T
$$
is assumed, i.e. that the $z$-axis points directly upwards against gravity.

### Implicit model simplifications

A third parameter $\mathbf{X}$ can also be supplied, which represents the inertial parameters of the robot, in the following form:
| $m_1$ | $r_{x,1}$ | $r_{y,1}$ | $r_{z,1}$ | $I_{xx,1}$ | $I_{yy,1}$ | $I_{zz,1}$ | $I_{xy,1}$ | $I_{xz,1}$ | $I_{yz,1}$
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| $\vdots$ | $\vdots$ | $\vdots$ | $\vdots$ | $\vdots$ | $\vdots$ |$\vdots$ |$\vdots$ |$\vdots$ |$\vdots$ |
| $m_n$ | $r_{x,n}$ | $r_{y,n}$ | $r_{z,n}$ | $I_{xx,n}$ | $I_{yy,n}$ | $I_{zz,n}$ | $I_{xy,n}$ | $I_{xz,n}$ | $I_{yz,n}$

where
 - $m_i$ is the mass of link $i$;
 - $r_{x,i}$, $r_{y,i}$, $r_{z,i}$ are the (x,y,z) position of the center of gravity of the link;
 - $I_{xx,i}$, $I_{yy,i}$, $I_{zz,i}$, $I_{xy,i}$, $I_{xz,i}$, $I_{yz,i}$ are the principle and cross inertia terms of the link, centered at the center of gravity and aligned with the main coordinate system of the link.

Note, if supplied, the numerical values of this variable are not actually used. However, any parameter which is given as _identically_ zero will be implicitely numerically ignored by the algorithm, resulting in a simpler equation.

If omitted, the algorithm will simply assume all parameters are nonzero.

### Function calling syntax
If you do not have a DH table of the robot, but instead have access to some dynamic simulation of your manipulator, you can still use PSDM if you can define an anonymous function `@inverseDynamicsFunc` which allows the algorithm to sample the joint torques for any joint states and inertial parameters, as
```matlab
tau = inverseDynamicsFunc(Q, Qd, Qdd, X)
```
where `tau`, `Q`, `Qd`, `Qdd` are $n \times N$ matrices ($n$ degrees of freedom, $N$ samples), and $\mathbf{X}$ is an inertia parameter matrix as defined above.

### Additional Options

Additionally, the following name/value pairs can be supplied:

 - ` tolerance`: A tolerance used in the ID. Any factors below this tolerance (in estimation of the torques) is ignored. Defaut: `1e-10`.
- `verbose`: A verbosity flag. Set to false to suppress output. Default: `true`.
- `gravity_only:` If true, the algorithm will ignore all acceleration  and velocity effects. Default: `false`.

## Examples
For usage, see the several examples in the +examples folder. The livescripts give images and full explanation of the process. Equivalent M-file scripts are also available but do not have images or equations.

## Credit
If you use our work, we ask that you cite us appropriately in any work. This work is in the process of being published and citation information will be posted when it becomes available.

## Contact
Author: Steffan Lloyd (Steffan.Lloyd@carleton.ca).
