# Supplementary Material for "Application of Pseudo-Symbolic Dynamic Modeling (PSDM) in the Modeling & Calibration of a 6-DOF Articulated Robot"

This folder provides supplementary files for the conference paper "Application of Pseudo-Symbolic Dynamic Modeling (PSDM) in the Modeling & Calibration of a 6-DOF Articulated Robot". 

The data is provided in two formats, a matlab ".mat" format (note, this format can be read by python as well through the scipy library), and a '.csv' file for each individual file, in the zip-file.

The data contains the following information:
	- E: The integer exponent matrix of the PSDM model
	- P: The reduction matrix of the PSDM model.
	- theta_b: The identified calibration vector for the model.
	- DH: The DH table used for generation of the model.
	- identifiedJointParameters: A table of the joint-level variables for the motor and friction calibration, including
		+ Kt: The motor constants [Nm/A].
		+ Fc: The coulomb friction constant [Nm].
		+ Fs: The static friction constant [Nm].
		+ vs: The Stribeck transition velocity [rad/s].
		+ Fv: The viscous friction constant [Nms/rad].
		+ dv: The viscous friction shape factor.
		+ sigma_0: The LuGre bristle stiffness parameter [Nm/rad].
		+ sigma_1: The LuGre bristle damping parameter [Nms/rad].