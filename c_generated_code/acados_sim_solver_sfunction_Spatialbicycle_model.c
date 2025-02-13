/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#define S_FUNCTION_NAME   acados_sim_solver_sfunction_Spatialbicycle_model
#define S_FUNCTION_LEVEL  2

#define MDL_START

// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "Spatialbicycle_model_model/Spatialbicycle_model_model.h"
#include "acados_sim_solver_Spatialbicycle_model.h"

#include "simstruc.h"

#define SAMPLINGTIME 0.02


static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

          

    // specify the number of input ports
    if ( !ssSetNumInputPorts(S, 3) )
        return;

    // specify the number of output ports
    if ( !ssSetNumOutputPorts(S, 1) )
        return;

    // specify dimension information for the input ports
    // x0
    ssSetInputPortVectorDimension(S, 0, 6);
    // u0
    ssSetInputPortVectorDimension(S, 1, 2);
    // parameters
    ssSetInputPortVectorDimension(S, 2, 1);

    // specify dimension information for the output ports
    ssSetOutputPortVectorDimension(S, 0, 6 ); // xnext

    // specify the direct feedthrough status
    // should be set to 1 for all inputs used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    // one sample time
    ssSetNumSampleTimes(S, 1);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

#endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    Spatialbicycle_model_sim_solver_capsule *capsule = Spatialbicycle_model_acados_sim_solver_create_capsule();
    Spatialbicycle_model_acados_sim_create(capsule);

    ssSetUserData(S, (void*)capsule);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    Spatialbicycle_model_sim_solver_capsule *capsule = ssGetUserData(S);

    sim_config *acados_sim_config = Spatialbicycle_model_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = Spatialbicycle_model_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = Spatialbicycle_model_acados_get_sim_out(capsule);
    void *acados_sim_dims = Spatialbicycle_model_acados_get_sim_dims(capsule);
    // sim_opts * Spatialbicycle_model_acados_get_sim_opts(capsule);
    // sim_solver * Spatialbicycle_model_acados_get_sim_solver(capsule);

    InputRealPtrsType in_sign;
    

    // local buffer
    real_t buffer[6];


    /* go through inputs */
    // initial condition
    in_sign = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 6; i++)
        buffer[i] = (double)(*in_sign[i]);

    sim_in_set(acados_sim_config, acados_sim_dims,
               acados_sim_in, "x", buffer);


    // ssPrintf("\nin acados sim:\n");
    // for (int i = 0; i < 6; i++) ssPrintf("x0[%d] = %f\n", i, buffer[i]);
    //     ssPrintf("\n");


    // control input - u
    in_sign = ssGetInputPortRealSignalPtrs(S, 1);

    for (int i = 0; i < 2; i++)
        buffer[i] = (double)(*in_sign[i]);

    sim_in_set(acados_sim_config, acados_sim_dims,
               acados_sim_in, "u", buffer);



    // parameters
    in_sign = ssGetInputPortRealSignalPtrs(S, 2);

    for (int i = 0; i < 1; i++)
        buffer[i] = (double)(*in_sign[i]);

    // update value of parameters
    Spatialbicycle_model_acados_sim_update_params(capsule, buffer, 1);


    /* call solver */
    int acados_status = Spatialbicycle_model_acados_sim_solve(capsule);


    /* set outputs */
    real_t *out_x = ssGetOutputPortRealSignal(S, 0);

    // get simulated state
    sim_out_get(acados_sim_config, acados_sim_dims, acados_sim_out,
                "xn", (void *) out_x);

    // ssPrintf("\nacados sim solve: returned %d\n", acados_status);
    // for (int i = 0; i < 6; i++) ssPrintf("x_sim[%d] = %f\n", i, out_x[i]);
    //     ssPrintf("\n");

}


static void mdlTerminate(SimStruct *S)
{
    Spatialbicycle_model_sim_solver_capsule *capsule = ssGetUserData(S);

    Spatialbicycle_model_acados_sim_free(capsule);
    Spatialbicycle_model_acados_sim_solver_free_capsule(capsule);
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
