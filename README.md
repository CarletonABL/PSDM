# PSDM

Pseudo-Symbolic Dynamic Modelling matlab package.

### [**The README for this codebase is posted as a PDF, please see the file README.pdf.**](README.pdf)

PSDM is a numerical method of deriving the equations of motion of an arbitrary rigid body chain, in regressor form. It is an alternative means of deriving the equations of motion for a rigid, serial kinematic chain. The result is a numerically represented model in a highly organized form, which allows for many symbolic manipulations through numerical methods. This allows for

 - The generation of very fast real time code.
 - Automatic model simplification (to achieve faster evaluation times)
 - Both forward and inverse dynamic modelling in a single derivation and with the same inertial parameter set.

## Requirements
This codebase requires a Matlab environment of R2018a, or newer. Additionally:

 - The Matlab *Symbolic Toolbox* is used in some of the example live scripts to illustrate some of the derivation results. This toolbox must be installed to run these code snippets.
 - *Matlab Coder* is required to leverage the code-generation capabilities built into this toolbox (see Section 3 the PDF README).
 - The derivation process can also leverage parallel processing, if the Matlab *Parallel Computing Toolbox* is installed. See Section 3 of the PDF README for details on this. 

## Credit
If you use our work, we ask that you cite us appropriately in any work.

``[1] S. Lloyd, R. Irani, and M. Ahmadi, “A numeric derivation for fast regressive modeling of manipulator dynamics,” Mech. Mach. Theory, vol. 156, p. 104149, Feb. 2021.``

This work can be accessed via its doi at [10.1016/j.mechmachtheory.2020.104149](https://doi.org/10.1016/j.mechmachtheory.2020.104149). A preprint version of this work is included in this repository, see [SLloydEtAl2021_PSDM.pdf](SLloydEtAl2021_PSDM.pdf)

## Contact
Author: Steffan Lloyd (Steffan.Lloyd@carleton.ca).
