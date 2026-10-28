# Chassis Kinematics

**LucSys Robotics | Planar Motion and Wheel-Level Commands**

## 1. Reference frame

For planar motion, define the robot body velocity as

$$\boldsymbol{\xi}=\begin{bmatrix}v_x & v_y & \omega\end{bmatrix}^{\mathsf{T}},$$

where $v_x$ points forward, $v_y$ points left, and $\omega$ is counterclockwise yaw rate. Linear speeds are in m/s and angular speed is in rad/s.

**Forward kinematics** estimates vehicle motion from measured wheel motion. **Inverse kinematics** converts a desired vehicle motion into wheel-speed and, for steering chassis, steering-related targets. Their formulas depend on wheel geometry and physical constraints.

## 2. Differential drive

For two driven wheels with effective wheel separation $b$, wheel radius $r$, and left/right wheel angular rates $\dot\phi_L,\dot\phi_R$:

$$v_x=\frac{r}{2}(\dot\phi_R+\dot\phi_L),\qquad
\omega=\frac{r}{b}(\dot\phi_R-\dot\phi_L).$$

The inverse mapping is

$$\dot\phi_L=\frac{v_x-\omega b/2}{r},\qquad
\dot\phi_R=\frac{v_x+\omega b/2}{r}.$$

The ideal no-slip differential model has no independently commanded lateral body velocity ($v_y=0$).

## 3. Four-wheel skid steering

A conventional four-wheel-drive chassis with fixed wheel orientations may turn by commanding different longitudinal speeds on its left and right sides. This resembles differential drive at the command level, but turns require lateral tire slip. Therefore, a differential approximation with an *effective* track width may need calibration for the operating surface and tire configuration.

Four driven wheels do not by themselves imply lateral-motion capability or independent wheel steering.

## 4. Ackermann steering

Ackermann steering changes vehicle heading through steering geometry rather than differential left/right wheel skid. For the ideal bicycle approximation referenced at the rear-axle center,

$$\omega=\frac{v_x}{L}\tan\delta,$$

where $L$ is wheelbase and $\delta$ is the equivalent front steering angle. The left and right physical wheel steering angles are generally different in a real Ackermann linkage.

This relation does not describe zero-radius turning; steering travel and the chassis geometry constrain minimum turning radius. The steering-command interpretation must be confirmed for the particular vehicle controller.

## 5. Mecanum and omnidirectional wheels

Mecanum wheels use angled rollers; other omnidirectional wheel layouts use passive rollers with different contact geometry. Suitable wheel arrangements can generate body motion in $v_x$, $v_y$, and $\omega$.

Their inverse kinematic matrices depend on wheel order, mounting location, roller orientation, drive direction, and wheel radius. A generic matrix should **not** be copied into an application without checking these physical conventions against the selected platform's wheel layout.

## 6. Model selection checklist

To apply any of the above equations, verify the wheel radius, wheelbase and track, drive/steer allocation, wheel order, coordinate signs, gearing, encoder scaling, and controller reference point of the exact robot. The platform's specification and software configuration remain authoritative for its numerical parameters.

For Ackermann steering, the mathematical conversion from steering geometry to body yaw rate is applicable to LucSys Robotics Ackermann configurations; use the **actual wheelbase, track and steering reference** for the selected product. A generic example or a table calculated for another vehicle must not be reused as a model-specific command table.
