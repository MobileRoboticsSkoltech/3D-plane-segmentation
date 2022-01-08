/*
 * Software License Agreement (BSD License)
 *
 *  Technical Aspects of Multimodal Systems (TAMS) - http://tams-www.informatik.uni-hamburg.de/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TAMS, nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Junhao Xiao
 * Email  : junhao.tams@ieee.org, tams@informatik.uni-hamburg.de
 *
 */

#include <math.h>
#include <stdlib.h>

#include "fftw_correlate/softFFTWCorrelateReal.h"
#include "fftw3.h"
#include "csecond.h"
#include "makeweights.h"
#include "so3_correlate_fftw.h"
#include "soft_fftw.h"
#include "s2_cospmls.h"
#include "s2_legendreTransforms.h"
#include "s2_semi_memo.h"


#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )

/****************************************

 softFFTWCor2: simple wrapper for correlating two functions defined on the sphere;
              if efficiency is important to you, or want more control,
              e.g. want to correlate lots and lots of times without having
	      reallocate tmp workspace, or change the bandwidth
	      you want to correlate at, or correlate complex-valued
              functions, you should look at

	      test_soft_fftw_correlate2.c

	      as an example of how to do it. softFFTWCor2() is basically
	      test_soft_fftw_correlate2.c turned into a wrapper, with
	      some simplifying assumptions.
	      
  bw: bandwidth of signal and pattern

  isReal: int defining whether or not the signal and pattern are
          strictly real, or interleaved (complex)
          = 1 -> strictly real
          = 0 -> complex/interleaved

  sig: double ptr to SIGNAL function samples;
       for bandwidth bw, then, is a pointer to a
       double array of size (2*bw)^3 + (isReal*(2*bw)^3)

  pat: double ptr to PATTERN function samples
       for bandwidth bw, then, is a pointer to a
       double array of size (2*bw)^3 + (isReal*(2*bw)^3)

  alpha, beta, gamma: ptrs to doubles; at the end of the routine,
               will "contain" the angles alpha, beta, and gamma needed
	       in order to rotate the SIGNAL to match the PATTERN; the
	       order of rotation is:

                   1) rotate by gamma about the z-axis
                   2) rotate by beta about the y-axis
                   3) rotate by alpha about the z-axis.
		   
	       where
             
	           0 <= alpha, gamma < 2*pi
	           0 <= beta <= pi


***********************************/
void softFFTWCorrelate( int bw,
                    float *sig,
                    float *pat,
                    float *alpha,
                    float *beta,
                    float *gamma,
                   	int isReal)
{
  int i ;
  int n, bwIn, bwOut, degLim ;
  fftw_complex *workspace1, *workspace2  ;
  double *workspace3 ;
  double *tmpR, *tmpI ;
  double *sigCoefR, *sigCoefI ;
  double *patCoefR, *patCoefI ;
  fftw_complex *so3Sig, *so3Coef ;
  fftw_plan p1 ;
  int na[2], inembed[2], onembed[2] ;
  int rank, howmany, istride, idist, ostride, odist ;
  int tmp, maxloc, ii, jj, kk ;
  double tmpval, maxval ;
  double *weights ;
  double *seminaive_naive_tablespace  ;
  double **seminaive_naive_table ;
  fftw_plan dctPlan, fftPlan ;
  int howmany_rank ;
  fftw_iodim dims[1], howmany_dims[1];
  
  bwIn = bw ;
  bwOut = bw ;
  degLim = bw - 1 ;
  n = 2 * bwIn ;

  tmpR = (double *) malloc( sizeof(double) * ( n * n ) );
  tmpI = (double *) malloc( sizeof(double) * ( n * n ) );
  so3Sig = fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
  workspace1 = fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
  workspace2 = fftw_malloc( sizeof(fftw_complex) * ((14*bwIn*bwIn) + (48 * bwIn)));
  workspace3 = (double *) malloc( sizeof(double) * (12*n + n*bwIn));
  sigCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  sigCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  patCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  patCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  so3Coef = fftw_malloc( sizeof(fftw_complex) * ((4*bwOut*bwOut*bwOut-bwOut)/3) ) ;
  seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bwIn,bwIn) +
		       Reduced_SpharmonicTableSize(bwIn,bwIn)));

  weights = (double *) malloc(sizeof(double) * (4*bwIn));

  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (seminaive_naive_tablespace == NULL) || (weights == NULL) ||
       (tmpR == NULL) || (tmpI == NULL) ||
       (so3Coef == NULL) ||
       (workspace1 == NULL) || (workspace2 == NULL) ||
       (workspace3 == NULL) ||
       (sigCoefR == NULL) || (sigCoefI == NULL) ||
       (patCoefR == NULL) || (patCoefI == NULL) ||
	   (so3Sig == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  /* create fftw plans for the S^2 transforms */
  /* first for the dct */
  dctPlan = fftw_plan_r2r_1d( 2*bwIn, weights, workspace3,
			      FFTW_REDFT10, FFTW_ESTIMATE ) ;

  /* now for the fft */
  /* 
     IMPORTANT NOTE!!! READ THIS!!!

     Now to make the fft plans.

     Please note that the planning-rigor flag *must be* FFTW_ESTIMATE!
     Why? Well, to try to keep things simple. I am using some of the
     pointers to arrays in rotateFct's arguments in the fftw-planning
     routines. If the planning-rigor is *not* FFTW_ESTIMATE, then
     the arrays will be written over during the planning stage.

     Therefore, unless you are really really sure you know what
     you're doing, keep the rigor as FFTW_ESTIMATE !!!
  */

  /*
    fftw "preamble" ;
    note  that this places in the transposed array
  */

  rank = 1 ;
  dims[0].n = 2*bwIn ;
  dims[0].is = 1 ;
  dims[0].os = 2*bwIn ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bwIn ;
  howmany_dims[0].is = 2*bwIn ;
  howmany_dims[0].os = 1 ;

  fftPlan = fftw_plan_guru_split_dft( rank, dims,
				      howmany_rank, howmany_dims,
				      tmpR, tmpI,
				      (double *) workspace2,
				      (double *) workspace2 + (n*n),
				      FFTW_ESTIMATE );

  /* create plan for inverse SO(3) transform */
  n = 2 * bwOut ;
  howmany = n*n ;
  idist = n ;
  odist = n ;
  rank = 2 ;
  inembed[0] = n ;
  inembed[1] = n*n ;
  onembed[0] = n ;
  onembed[1] = n*n ;
  istride = 1 ;
  ostride = 1 ;
  na[0] = 1 ;
  na[1] = n ;

  p1 = fftw_plan_many_dft( rank, na, howmany,
			   workspace1, inembed,
			   istride, idist,
			   so3Sig, onembed,
			   ostride, odist,
			   FFTW_FORWARD, FFTW_ESTIMATE );


  seminaive_naive_table = SemiNaive_Naive_Pml_Table(bwIn, bwIn,
						    seminaive_naive_tablespace,
						    (double *) workspace2);


  /* make quadrature weights for the S^2 transform */
  makeweights( bwIn, weights ) ;

  n = 2 * bwIn ;
  /* load SIGNAL samples into temp array */
  if ( isReal )
    for ( i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = sig[i];
	tmpI[i] = 0. ;
      }
  else
    for ( i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = sig[2*i];
	tmpI[i] = sig[2*i+1] ;
      }

  /* spherical transform of SIGNAL */
  FST_semi_memo( tmpR, tmpI,
		 sigCoefR, sigCoefI,
		 bwIn, seminaive_naive_table,
		 (double *) workspace2, isReal, bwIn,
		 &dctPlan, &fftPlan,
		 weights );

  /* load PATTERN samples into temp array; note that I'm
     also providing 0s in the imaginary part */
  if ( isReal )
    for (i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = pat[i] ;
	tmpI[i] = 0.  ;
      }
  else
    for (i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = pat[2*i] ;
	tmpI[i] = pat[2*i+1] ;
      }

  /* spherical transform of PATTERN */
  FST_semi_memo( tmpR, tmpI,
		 patCoefR, patCoefI,
		 bwIn, seminaive_naive_table,
		 (double *) workspace2, isReal, bwIn,
		 &dctPlan, &fftPlan,
		 weights ) ;

  /* all done with the spherical transform, so free up
     some memory before continuing */
  free( seminaive_naive_table ) ;
  free( seminaive_naive_tablespace ) ;


  /* combine coefficients */
  so3CombineCoef_fftw( bwIn, bwOut, degLim,
		       sigCoefR, sigCoefI,
		       patCoefR, patCoefI,
		       so3Coef ) ;

  /* now inverse so(3) */
  Inverse_SO3_Naive_fftw( bwOut,
			  so3Coef,
			  so3Sig,
			  workspace1,
			  workspace2,
			  workspace3,
			  &p1,
			  isReal ) ;

  /* now find max value */
  maxval = 0.0 ;
  maxloc = 0 ;
  for ( i = 0 ; i < 8*bwOut*bwOut*bwOut; i ++ )
    {
      tmpval = NORM( so3Sig[i] );
      if ( tmpval > maxval )
	{
	  maxval = tmpval;
	  maxloc = i ;
	}
    }


  ii = floor( maxloc / (4.*bwOut*bwOut) );
  tmp = maxloc - (ii*4.*bwOut*bwOut);
  jj = floor( tmp / (2.*bwOut) );
  tmp = maxloc - (ii *4*bwOut*bwOut) - jj*(2*bwOut);
  kk = tmp ;



  *alpha = M_PI*jj/((double) bwOut) ;
  *beta =  M_PI*(2*ii+1)/(4.*bwOut) ;
  *gamma = M_PI*kk/((double) bwOut) ;

  /* clean up */

  fftw_destroy_plan( p1 );
  fftw_destroy_plan( fftPlan );
  fftw_destroy_plan( dctPlan );

  free( weights );
  fftw_free( so3Coef ) ;
  free( patCoefI );
  free( patCoefR );
  free( sigCoefI );
  free( sigCoefR );
  free( workspace3 );
  fftw_free( workspace2 );
  fftw_free( workspace1 );
  fftw_free( so3Sig ) ;
  free( tmpI );
  free( tmpR );

}

