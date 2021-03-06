/*
% _MEXFUNCTIONNAME_.c Computes the inverse dynamic for a model, as per the E and P
% matrices which were used to generate it.
%
% Calling syntax is:
% 	_MEXFUNCTIONNAME_(Q, Qd, tau, Theta, N, Qdd);
% where
%   - Q, Qd, Qdd are the joint variables and their first and second
%     derivatives, respectively, in a _1DOF_xN matrix.
%   - Theta is the regression vector, a _ell_ column vector.
%   - tau is the joint torques, in a _1DOF_xN matrix.
%
% See also PSDM.makeForwardDynamics.
 */

#include "mex.h"
#include <math.h>
#include "_FUNCTIONNAME_.h"

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
	double *Q;
	double *Qd;
	double *tau;
	double *Theta;
	double *Qdd;
	int N;


	/* Error checking */
	if(nrhs!=4) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","Wrong number of inputs.");
    }
    if(nlhs>1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","Too many outputs requested!");
    }
    int k;
    for (k=0;k<4;k++){
	    if( (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ) ||
	        (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) ) ||
	        (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) ) ||
	        (!mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) )) {
	        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notDouble","Input matrices must be type double.");
	    }
	}
	/* check that number of rows in second input argument is 1 */
    if(mxGetM(prhs[0])!=_1DOF_ || mxGetM(prhs[1])!=_1DOF_ || mxGetM(prhs[2])!=_1DOF_) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notRowVector","Wrong number of rows in input matrices.");
    }
	/* check that number of rows in second input argument is 1 */
    if(mxGetM(prhs[3])!=_ell_ || mxGetN(prhs[3])!=1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notRowVector","Theta is the wrong size.");
    }

	/* Read in inputs */
	Q = mxGetDoubles(prhs[0]);
	Qd = mxGetDoubles(prhs[1]);
	tau = mxGetDoubles(prhs[2]);
	Theta = mxGetDoubles(prhs[3]);
	N = mxGetN(prhs[0]);

	/* Initialize outputs */
 	plhs[0] = mxCreateDoubleMatrix(_1DOF_, N, mxREAL);
	Qdd = mxGetPr(plhs[0]);

	/* Call routine */
	_FUNCTIONNAME_(Q, Qd, tau, Theta, N, Qdd);
}