/* This file implements the GJK algorithm, which calculates the minimal distance between two convex polytopes

Author: Thomas Horsch 1990 (translation of the original fortran code)

The original GJK algorithm is described in the following paper:
C
C     E.G. Gilbert, D.W. Johnson and S.S. Keerthi, "A Fast Procedure
C     for Computing the Distance Between Complex Objects in Three
C     Space," IEEE Trans. of Robotics and Automation, Vol.4, pp193-203
C     1998.
C
C  Each of the two polytopes should be described as the convex hull
C  of a finite set of points in 3-D space.
C
C  Very simply stated, the algorithm starts with an initial approximate
C  solution (either user supplied or internally set), goes through many
C  cycles (in each cycle it improves the solution by decreasing the
C  distance between the objects), and terminates when a stopping
C  criterion is satisfied.
*/

//#include <stdio.h>
#include <math.h>

/*c====================================================================*/
/*int tranc(int nv_i, int nv_j, double k1[][3], double k2[][3], double zb_i[][3],
double zb_j[][3], int param[], double new_org[], double cen[], double n_cent[])
*/
int tranc(int nv_i, int nv_j, double k1[][3], double k2[][3],
    double zb_i[][3], double zb_j[][3], int param[],
    double new_org[], double cen[], double n_cent[])
    /*=====================================================================c
    c                                      c
    c  this  routine  is  used  for  forming  the  centroidal  direction   c
    c  and  forming  zb_i[*][*]  zb_j[*][*]  and  new_org[*].              c
    c                                      c
    c=====================================================================*/
{
    double   pi[3], pj[3];
    register int l;


    if (param[2] != 0 || param[4] != 1)
    {
        /*
        c  form  the  centroids  pi[*]  and  pj[*] :
        */
        pi[0] = pi[1] = pi[2] = pj[0] = pj[1] = pj[2] = 0.0;
        for (l = 0; l < nv_i; l++)
        {
            pi[0] = pi[0] + k1[l][0];
            pi[1] = pi[1] + k1[l][1];
            pi[2] = pi[2] + k1[l][2];
        }
        for (l = 0; l < nv_j; l++)
        {
            pj[0] = pj[0] + k2[l][0];
            pj[1] = pj[1] + k2[l][1];
            pj[2] = pj[2] + k2[l][2];
        }
        pi[0] = pi[0] / (double)nv_i;
        pi[1] = pi[1] / (double)nv_i;
        pi[2] = pi[2] / (double)nv_i;
        pj[0] = pj[0] / (double)nv_j;
        pj[1] = pj[1] / (double)nv_j;
        pj[2] = pj[2] / (double)nv_j;
        /*
        c  form  the  centroidal  and  negative  centroidal
        c  directions,  cen[*]  and  n_cent[*] :
        */
        cen[0] = pi[0] - pj[0];
        cen[1] = pi[1] - pj[1];
        cen[2] = pi[2] - pj[2];

        n_cent[0] = -cen[0];
        n_cent[1] = -cen[1];
        n_cent[2] = -cen[2];
        /*
        c  form  zb_i[*][*],  zb_j[*][*]  and  optionally  new_org :
        */
        if (param[2] != 0) {
            new_org[0] = (pi[0] + pj[0]) / 2.0;
            new_org[1] = (pi[1] + pj[1]) / 2.0;
            new_org[2] = (pi[2] + pj[2]) / 2.0;
            for (l = 0; l < nv_i; l++)
            {
                zb_i[l][0] = k1[l][0] - new_org[0];
                zb_i[l][1] = k1[l][1] - new_org[1];
                zb_i[l][2] = k1[l][2] - new_org[2];
            }
            for (l = 0; l < nv_j; l++)
            {
                zb_j[l][0] = k2[l][0] - new_org[0];
                zb_j[l][1] = k2[l][1] - new_org[1];
                zb_j[l][2] = k2[l][2] - new_org[2];
            }
            return(1);
        }
    }
    for (l = 0; l < nv_i; l++)
    {
        zb_i[l][0] = k1[l][0];
        zb_i[l][1] = k1[l][1];
        zb_i[l][2] = k1[l][2];
    }
    for (l = 0; l < nv_j; l++)
    {
        zb_j[l][0] = k2[l][0];
        zb_j[l][1] = k2[l][1];
        zb_j[l][2] = k2[l][2];
    }
    return(1);
}  /* ende der procedure tranc  */

/*====================================================================*/
//void csfcn(int nv_k, double zb_k[][3], double eta[], double *sf_k, int *nr_k)
void csfcn(int nv_k, double zb_k[][3], double eta[], double *sf_k, int *nr_k)
/*=====================================================================c
c                                      c
c  this  routine  computes  the  contact  and  support  functions      c
c  for  a  polytope.                               c
c                                      c
c=====================================================================*/
{
    int    l;
    double inn;
    *nr_k = 0;
    *sf_k = zb_k[0][0] * eta[0] + zb_k[0][1] * eta[1] + zb_k[0][2] * eta[2];
    if (nv_k > 1)
    {
        for (l = 1; l < nv_k; l++)
        {
            inn = zb_k[l][0] * eta[0] + zb_k[l][1] * eta[1] + zb_k[l][2] * eta[2];
            if (inn > *sf_k)
            {
                *nr_k = l;
                *sf_k = inn;
            }
        }
    }
}
/****************************************************************************************************************/
/*====================================================================*/
int dsbp(int *nvs, int ris[], int rjs[], double y[][3], double del[][4],
    double zsol[], double als[], double *dstsq, int *backup)
    /*=====================================================================c
    c  dsbp implements, in a very efficient way, the distance subalgorithm
    c  of finding the near point to the convex hull of four or less points
    c  in 3-d space. the procedure and its efficient fortran implementation
    c  are both due to d.w.johnson. although this subroutine is quite long,
    c  only a very small part of it will be executed on each call. refer to
    c  sections 5 and 6 of the report mentioned in routine dist3 for details
    c  concerning the distance subalgorithm.
    c
    c  following is a brief description of the parameters in dsbp :
    c
    c  **** on  input :
    c
    c  nvs        :  the number of points.  1  <=  nvs  <=  4 .
    c
    c  y[*][*]    :  the array whose columns contain the points.
    c
    c  ris[*], rjs[*] :  index vectors for polytope-i and polytope-j.
    c            for k = 1,...,nvs,
    c            y[.,k] = zbi[.][ris[k]] - zbj[.][rjs[k]],
    c            where a[.][k] denotes the k-th column of the
    c            matrix a.
    c
    c  del[*][*]      :  del[i][j] = inner product of y[.][i] and y[.][j].
    c
    c  **** on  output :
    c
    c  zsol[*]    :  near point to the convex hull of the points
    c            in y.
    c
    c  dstsq      :  square of the norm of zsol.
    c
    c  **** dsbp also determines an affinely independent subset of the
    c  points such that zsol= near point to the affine hull of the points
    c  in the subset. the variables nvs, y, ris, rjs and del are modified
    c  so that, on output, they correspond to this subset of points.
    c
    c  als[*]     :  the barycentric coordinates of zsol, i.e.,
    c            zsol = als[1]*y[.][1] + ... + als[nvs]*y[.][nvs],
    c            als[k]  >  0.d0 for k=1,...,nvs, and,
    c            als[1] + ... + als[nvs] = 1.d0 .
    c
    c=====================================================================*/
    /*
    int    *nvs, ris[], rjs[], *backup;
    double y[][3], del[][4], zsol[], als[], *dstsq;
    */
{

#define  zero       0.0
#define  one        1.0

    int    nvsd, k, l, kk, ll, risd[5], rjsd[5], iord[5], back_up = 1;

    double sum, e132, e213, e123, d1[15], d2[15], d3[15], d4[15],
        yd[4][3], deld[4][4], zsold[4], alsd[5], dstsqd;

    d1[0] = d2[1] = d3[3] = d4[7] = 1.0;
    /*=====================================================================c
    c  if  *backup =  1     on  input  to  this  routine,              c
    c  then  the  subalgorithm  will  be  done  by  the            c
    c  *backup  procedure.                             c
    c                                      c
    c  if  *backup =  1     on  output  from  this  routine,           c
    c  then  it  means  that  the  subalgorithm  was  done             c
    c  using  the  *backup  procedure.                     c
    c                                      c
    c  go  to  statement  1000  for  the  *backup  procedure :         c
    c=====================================================================*/
    //  if ( (*backup) )
    //printf("\n    backup = 1 ");

    if (!(*backup))
    {
        /*=====================================================================c
        c                                      c
        c  the  regular  distance  subalgorithm  begins ...            c
        c                                      c
        c=====================================================================*/
        switch (*nvs) {
            /* ************************************************
            c  case  of  a  single  point ...                */
        case 1:
            als[0] = d1[0];
            zsol[0] = y[0][0];
            zsol[1] = y[0][1];
            zsol[2] = y[0][2];
            *dstsq = del[0][0];
            return(1);
            /* ************************************************
            c  case  of  two  points ...                     */
            /* -----------------------------------------------
            c    check  optimality  of  vertex 1 :      */
        case 2:
            d2[2] = del[0][0] - del[1][0];
            if (d2[2] <= zero)
            {
                *nvs = 1;
                als[0] = d1[0];
                zsol[0] = y[0][0];
                zsol[1] = y[0][1];
                zsol[2] = y[0][2];
                *dstsq = del[0][0];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 12 :*/
            d1[2] = del[1][1] - del[1][0];
            if (d1[2] > zero  &&  d2[2] > zero)
            {
                sum = d1[2] + d2[2];
                als[0] = d1[2] / sum;
                als[1] = one - als[0];
                zsol[0] = y[1][0] + als[0] * (y[0][0] - y[1][0]);
                zsol[1] = y[1][1] + als[0] * (y[0][1] - y[1][1]);
                zsol[2] = y[1][2] + als[0] * (y[0][2] - y[1][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  vertex 2 :      */
            if (d1[2] <= zero)
            {
                *nvs = 1;
                ris[0] = ris[1];
                rjs[0] = rjs[1];
                als[0] = d2[1];
                zsol[0] = y[1][0];
                zsol[1] = y[1][1];
                zsol[2] = y[1][2];
                *dstsq = del[1][1];
                y[0][0] = y[1][0];
                y[0][1] = y[1][1];
                y[0][2] = y[1][2];
                del[0][0] = del[1][1];
                return(1);
            }
            /* -----------------------------------------------
            c  need  to  go  for  the  *backup  procedure :
            c  (without  recomputing  the  di(*)  values)   */
            break;
            /* ************************************************
            c  case  of  three  points ...                   */
            /* -----------------------------------------------
            c    check  optimality  of  vertex 1 :      */
        case 3:

            d2[2] = del[0][0] - del[1][0];
            d3[4] = del[0][0] - del[2][0];
            if (d2[2] <= zero  &&  d3[4] <= zero)
            {
                *nvs = 1;
                als[0] = d1[0];
                zsol[0] = y[0][0];
                zsol[1] = y[0][1];
                zsol[2] = y[0][2];
                *dstsq = del[0][0];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 12 :*/
            e132 = del[1][0] - del[2][1];
            d1[2] = del[1][1] - del[1][0];
            d3[6] = d1[2] * d3[4] + d2[2] * e132;
            if (d1[2] > zero  &&  d2[2] > zero  && d3[6] <= zero)
            {
                *nvs = 2;
                sum = d1[2] + d2[2];
                als[0] = d1[2] / sum;
                als[1] = one - als[0];
                zsol[0] = y[1][0] + als[0] * (y[0][0] - y[1][0]);
                zsol[1] = y[1][1] + als[0] * (y[0][1] - y[1][1]);
                zsol[2] = y[1][2] + als[0] * (y[0][2] - y[1][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 13 : */
            e123 = del[2][0] - del[2][1];
            d1[4] = del[2][2] - del[2][0];
            d2[6] = d1[4] * d2[2] + d3[4] * e123;
            if (d1[4] > zero  &&  d2[6] <= zero  && d3[4] > zero)
            {
                *nvs = 2;
                ris[1] = ris[2];
                rjs[1] = rjs[2];
                sum = d1[4] + d3[4];
                als[0] = d1[4] / sum;
                als[1] = one - als[0];
                zsol[0] = y[2][0] + als[0] * (y[0][0] - y[2][0]);
                zsol[1] = y[2][1] + als[0] * (y[0][1] - y[2][1]);
                zsol[2] = y[2][2] + als[0] * (y[0][2] - y[2][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[1][0] = y[2][0];
                y[1][1] = y[2][1];
                y[1][2] = y[2][2];
                del[1][0] = del[2][0];
                del[1][1] = del[2][2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  face 123 :      */
            e213 = -e123;
            d2[5] = del[2][2] - del[2][1];
            d3[5] = del[1][1] - del[2][1];
            d1[6] = d2[5] * d1[2] + d3[5] * e213;
            if (d1[6] > zero  &&  d2[6] > zero  && d3[6] > zero)
            {
                sum = d1[6] + d2[6] + d3[6];
                als[0] = d1[6] / sum;
                als[1] = d2[6] / sum;
                als[2] = one - als[0] - als[1];
                zsol[0] = y[2][0] + als[0] * (y[0][0] - y[2][0]) +
                    als[1] * (y[1][0] - y[2][0]);
                zsol[1] = y[2][1] + als[0] * (y[0][1] - y[2][1]) +
                    als[1] * (y[1][1] - y[2][1]);
                zsol[2] = y[2][2] + als[0] * (y[0][2] - y[2][2]) +
                    als[1] * (y[1][2] - y[2][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  vertex 2 :      */
            if (d1[2] <= zero  &&  d3[5] <= zero)
            {
                *nvs = 1;
                ris[0] = ris[1];
                rjs[0] = rjs[1];
                als[0] = d2[1];
                zsol[0] = y[1][0];
                zsol[1] = y[1][1];
                zsol[2] = y[1][2];
                *dstsq = del[1][1];
                y[0][0] = y[1][0];
                y[0][1] = y[1][1];
                y[0][2] = y[1][2];
                del[0][0] = del[1][1];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  vertex 3 :      */
            if (d1[4] <= zero  &&  d2[5] <= zero)
            {
                *nvs = 1;
                ris[0] = ris[2];
                rjs[0] = rjs[2];
                als[0] = d3[3];
                zsol[0] = y[2][0];
                zsol[1] = y[2][1];
                zsol[2] = y[2][2];
                *dstsq = del[2][2];
                y[0][0] = y[2][0];
                y[0][1] = y[2][1];
                y[0][2] = y[2][2];
                del[0][0] = del[2][2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 23 : */
            if (d1[6] <= zero  &&  d2[5] > zero  && d3[5] > zero)
            {
                *nvs = 2;
                ris[0] = ris[2];
                rjs[0] = rjs[2];
                sum = d2[5] + d3[5];
                als[1] = d2[5] / sum;
                als[0] = one - als[1];
                zsol[0] = y[2][0] + als[1] * (y[1][0] - y[2][0]);
                zsol[1] = y[2][1] + als[1] * (y[1][1] - y[2][1]);
                zsol[2] = y[2][2] + als[1] * (y[1][2] - y[1][1]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[0][0] = y[2][0];
                y[0][1] = y[2][1];
                y[0][2] = y[2][2];
                del[1][0] = del[2][1];
                del[0][0] = del[2][2];
                return(1);
            }
            /* -----------------------------------------------
            c  need  to  go  for  the  *backup  procedure :
            c  (without  recomputing  the  di(*)  values)   */

            /* -----------------------------------------------
            c  need  to  go  for  the  *backup  procedure :
            c  (without  recomputing  the  di(*)  values)    */
            break;
        }  /* end switch */
        back_up = 0;
    }    /* end if(! (*backup) ) */
    /*=====================================================================c
    c                                      c
    c  the  *backup procedure  begins ...                      c
    c                                      c
    c=====================================================================*/
    /* -----------------------------------------------
    c  if  the  di(*)  values  are  already  available,
    c  then  go  to  1101,  1201,  1301,  or  1401:   */
    /* ************************************************
    c  case  of  a  single  point ...                */
    switch (*nvs) {
    case 1:
        *dstsq = del[0][0];
        als[0] = d1[0];
        zsol[0] = y[0][0];
        zsol[1] = y[0][1];
        zsol[2] = y[0][2];
        *backup = 1;
        return(1);
        /* ************************************************
        c  case  of  two  points ...                     */
    case 2:
        if (back_up)
        {
            d2[2] = del[0][0] - del[1][0];
            d1[2] = del[1][1] - del[1][0];
        }
        /* -----------------------------------------------
        c    check  vertex 1 :                      */
        *dstsq = del[0][0];
        nvsd = 1;
        als[0] = d1[0];
        zsol[0] = y[0][0];
        zsol[1] = y[0][1];
        zsol[2] = y[0][2];
        iord[0] = 0;
        /* -----------------------------------------------
        c    check  line  segment 12 :              */
        if (d1[2] > zero  &&  d2[2] > zero)
        {
            sum = d1[2] + d2[2];
            alsd[0] = d1[2] / sum;
            alsd[1] = one - alsd[0];
            zsold[0] = y[1][0] + alsd[0] * (y[0][0] - y[1][0]);
            zsold[1] = y[1][1] + alsd[0] * (y[0][1] - y[1][1]);
            zsold[2] = y[1][2] + alsd[0] * (y[0][2] - y[1][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd < *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 1;
            }
        }
        /* -----------------------------------------------
        c    check  vertex 2 :                      */
        if (del[1][1] < *dstsq)
        {
            *dstsq = del[1][1];
            nvsd = 1;
            als[0] = d2[1];
            zsol[0] = y[1][0];
            zsol[1] = y[1][1];
            zsol[2] = y[1][2];
            iord[0] = 1;
        }
        /* ----------------------------------------------*/
        break;
        /* ************************************************
        c  case  of  three  points ...                   */
    case 3:
        if (back_up)
        {
            d2[2] = del[0][0] - del[1][0];
            d3[4] = del[0][0] - del[2][0];
            e132 = del[1][0] - del[2][1];
            d1[2] = del[1][1] - del[1][0];
            d3[6] = d1[2] * d3[4] + d2[2] * e132;
            e123 = del[2][0] - del[2][1];
            d1[4] = del[2][2] - del[2][0];
            d2[6] = d1[4] * d2[2] + d3[4] * e123;
            e213 = -e123;
            d2[5] = del[2][2] - del[2][1];
            d3[5] = del[1][1] - del[2][1];
            d1[6] = d2[5] * d1[2] + d3[5] * e213;
        }
        /* -----------------------------------------------
        c    check  vertex 1 :                      */
        *dstsq = del[0][0];
        nvsd = 1;
        als[0] = d1[0];
        zsol[0] = y[0][0];
        zsol[1] = y[0][1];
        zsol[2] = y[0][2];
        iord[0] = 0;
        /* -----------------------------------------------
        c    check  line  segment 12 :              */
        if (d1[2] > zero  &&  d2[2] > zero)
        {
            sum = d1[2] + d2[2];
            alsd[0] = d1[2] / sum;
            alsd[1] = one - alsd[0];
            zsold[0] = y[1][0] + alsd[0] * (y[0][0] - y[1][0]);
            zsold[1] = y[1][1] + alsd[0] * (y[0][1] - y[1][1]);
            zsold[2] = y[1][2] + alsd[0] * (y[0][2] - y[1][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];

            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 1;
            }
        }
        /* -----------------------------------------------
        c    check  line  segment 13 :              */
        if (d1[4] > zero  &&  d3[4] > zero)
        {
            sum = d1[4] + d3[4];
            alsd[0] = d1[4] / sum;
            alsd[1] = one - alsd[0];
            zsold[0] = y[2][0] + alsd[0] * (y[0][0] - y[2][0]);
            zsold[1] = y[2][1] + alsd[0] * (y[0][1] - y[2][1]);
            zsold[2] = y[2][2] + alsd[0] * (y[0][2] - y[2][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 2;
            }
        }
        /* -----------------------------------------------
        c    check  face 123 :                      */
        if (d1[6] > zero  &&  d2[6] > zero  && d3[6] > zero)
        {
            sum = d1[6] + d2[6] + d3[6];
            alsd[0] = d1[6] / sum;
            alsd[1] = d2[6] / sum;
            alsd[2] = one - alsd[0] - alsd[1];
            zsold[0] = y[2][0] + alsd[0] * (y[0][0] - y[2][0]) +
                alsd[1] * (y[1][0] - y[2][0]);
            zsold[1] = y[2][1] + alsd[0] * (y[0][1] - y[2][1]) +
                alsd[1] * (y[1][1] - y[2][1]);
            zsold[2] = y[2][2] + alsd[0] * (y[0][2] - y[2][2]) +
                alsd[1] * (y[1][2] - y[2][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd < *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 3;
                als[0] = alsd[0];
                als[1] = alsd[1];
                als[2] = alsd[2];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 1;
                iord[2] = 2;
            }
        }
        /* -----------------------------------------------
        c    check  vertex 2 :                      */
        if (del[1][1] < *dstsq)
        {
            nvsd = 1;
            *dstsq = del[1][1];
            als[0] = d2[1];
            zsol[0] = y[1][0];
            zsol[1] = y[1][1];
            zsol[2] = y[1][2];
            iord[0] = 1;
        }
        /* -----------------------------------------------
        c    check  vertex 3 :                      */
        if (del[2][2] < *dstsq)
        {
            nvsd = 1;
            *dstsq = del[2][2];
            als[0] = d3[3];
            zsol[0] = y[2][0];
            zsol[1] = y[2][1];
            zsol[2] = y[2][2];
            iord[0] = 2;
        }
        /* -----------------------------------------------
        c    check  line  segment 23 :              */
        if (d2[5] > zero  &&  d3[5] > zero)
        {
            sum = d2[5] + d3[5];
            alsd[1] = d2[5] / sum;
            alsd[0] = one - alsd[1];
            zsold[0] = y[2][0] + alsd[1] * (y[1][0] - y[2][0]);
            zsold[1] = y[2][1] + alsd[1] * (y[1][1] - y[2][1]);
            zsold[2] = y[2][2] + alsd[1] * (y[1][2] - y[2][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 2;
                iord[1] = 1;
            }
        }
        /* -----------------------------------------------*/
        break;
    }  /* end switch */
    /* -----------------------------------------------
    c  the  final  reordering :                     */
    for (k = 0; k < *nvs; k++)
    {
        risd[k] = ris[k];
        rjsd[k] = rjs[k];
        yd[k][0] = y[k][0];
        yd[k][1] = y[k][1];
        yd[k][2] = y[k][2];
        for (l = 0; l <= k; l++)
        {
            deld[k][l] = del[k][l];
            deld[l][k] = del[k][l];
        }
    }

    *nvs = nvsd;
    for (k = 0; k < *nvs; k++)
    {
        kk = iord[k];
        ris[k] = risd[kk];
        rjs[k] = rjsd[kk];
        y[k][0] = yd[kk][0];
        y[k][1] = yd[kk][1];
        y[k][2] = yd[kk][2];
        for (l = 0; l >= k; l++)
        {
            ll = iord[l];
            del[k][l] = deld[kk][ll];
        }
    }
    *backup = 1;
    return(1);
}  /* ende der procedur dsbp  */
/*************************************************************************************************************/
/*====================================================================*/
int dsbp4(int *nvs, int ris[], int rjs[], double y[][3], double del[][4],
    double zsol[], double als[], double *dstsq, int *backup)
    /*=====================================================================c
    c  dsbp implements, in a very efficient way, the distance subalgorithm
    c  of finding the near point to the convex hull of four or less points
    c  in 3-d space. the procedure and its efficient fortran implementation
    c  are both due to d.w.johnson. although this subroutine is quite long,
    c  only a very small part of it will be executed on each call. refer to
    c  sections 5 and 6 of the report mentioned in routine dist3 for details
    c  concerning the distance subalgorithm.
    c
    c  following is a brief description of the parameters in dsbp :
    c
    c  **** on  input :
    c
    c  nvs        :  the number of points.  1  <=  nvs  <=  4 .
    c
    c  y[*][*]    :  the array whose columns contain the points.
    c
    c  ris[*], rjs[*] :  index vectors for polytope-i and polytope-j.
    c            for k = 1,...,nvs,
    c            y[.,k] = zbi[.][ris[k]] - zbj[.][rjs[k]],
    c            where a[.][k] denotes the k-th column of the
    c            matrix a.
    c
    c  del[*][*]      :  del[i][j] = inner product of y[.][i] and y[.][j].
    c
    c  **** on  output :
    c
    c  zsol[*]    :  near point to the convex hull of the points
    c            in y.
    c
    c  dstsq      :  square of the norm of zsol.
    c
    c  **** dsbp also determines an affinely independent subset of the
    c  points such that zsol= near point to the affine hull of the points
    c  in the subset. the variables nvs, y, ris, rjs and del are modified
    c  so that, on output, they correspond to this subset of points.
    c
    c  als[*]     :  the barycentric coordinates of zsol, i.e.,
    c            zsol = als[1]*y[.][1] + ... + als[nvs]*y[.][nvs],
    c            als[k]  >  0.d0 for k=1,...,nvs, and,
    c            als[1] + ... + als[nvs] = 1.d0 .
    c
    c=====================================================================*/
    /*
    int    *nvs, ris[], rjs[], *backup;
    double y[][3], del[][4], zsol[], als[], *dstsq;
    */
{

#define  zero       0.0
#define  one        1.0

    int    nvsd, k, l, kk, ll, risd[5], rjsd[5], iord[5], back_up = 1;

    double sum, e132, e142, e123, e143, e213, e243,
        e124, e134, e214, e234, e314, e324,
        d1[15], d2[15], d3[15], d4[15],
        yd[4][3], deld[4][4], zsold[4], alsd[5], dstsqd;

    d1[0] = d2[1] = d3[3] = d4[7] = 1.0;
    /*=====================================================================c
    c  if  backup =  1  on  input  to  this  routine,              c
    c  then  the  subalgorithm  will  be  done  by  the            c
    c  backup  procedure.                              c
    c                                      c
    c  if  backup =  1  on  output  from  this  routine,           c
    c  then  it  means  that  the  subalgorithm  was  done             c
    c  using  the  backup  procedure.                      c
    c                                      c
    c  go  to  statement  1000  for  the  backup  procedure :          c
    c=====================================================================*/
    //  if ( (*backup) )
    //printf("\n    BACKUP = 1 ");
    if (!(*backup))
    {
        /*=====================================================================c
        c                                      c
        c  the  regular  distance  subalgorithm  begins ...            c
        c                                      c
        c=====================================================================*/
        switch (*nvs) {
            /* ************************************************
            c  case  of  four  points ...                    */
            /* -----------------------------------------------
            c    check  optimality  of  vertex 1 :      */
        case 4:
            d2[2] = del[0][0] - del[1][0];
            d3[4] = del[0][0] - del[2][0];
            d4[8] = del[0][0] - del[3][0];
            if (d2[2] <= zero  &&  d3[4] <= zero  && d4[8] <= zero)
            {
                *nvs = 1;
                als[0] = d1[0];
                zsol[0] = y[0][0];
                zsol[1] = y[0][1];
                zsol[2] = y[0][2];
                *dstsq = del[0][0];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 12 : */
            e132 = del[1][0] - del[2][1];
            e142 = del[1][0] - del[3][1];
            d1[2] = del[1][1] - del[1][0];
            d3[6] = d1[2] * d3[4] + d2[2] * e132;
            d4[11] = d1[2] * d4[8] + d2[2] * e142;
            if (d1[2] > zero  &&  d2[2] > zero  &&
                d3[6] <= zero  &&  d4[11] <= zero)
            {
                *nvs = 2;
                sum = d1[2] + d2[2];
                als[0] = d1[2] / sum;
                als[1] = one - als[0];
                zsol[0] = y[1][0] + als[0] * (y[0][0] - y[1][0]);
                zsol[1] = y[1][1] + als[0] * (y[0][1] - y[1][1]);
                zsol[2] = y[1][2] + als[0] * (y[0][2] - y[1][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 13 : */
            e123 = del[2][0] - del[2][1];
            e143 = del[2][0] - del[3][2];
            d1[4] = del[2][2] - del[2][0];
            d2[6] = d1[4] * d2[2] + d3[4] * e123;
            d4[12] = d1[4] * d4[8] + d3[4] * e143;
            if (d1[4] > zero  &&  d2[6] <= zero  &&
                d3[4] > zero  &&  d4[12] <= zero)
            {
                *nvs = 2;
                ris[1] = ris[2];
                rjs[1] = rjs[2];
                sum = d1[4] + d3[4];
                als[0] = d1[4] / sum;
                als[1] = one - als[0];
                zsol[0] = y[2][0] + als[0] * (y[0][0] - y[2][0]);
                zsol[1] = y[2][1] + als[0] * (y[0][1] - y[2][1]);
                zsol[2] = y[2][2] + als[0] * (y[0][2] - y[2][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[1][0] = y[2][0];
                y[1][1] = y[2][1];
                y[1][2] = y[2][2];
                del[1][0] = del[2][0];
                del[1][1] = del[2][2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  face 123 :      */
            d2[5] = del[2][2] - del[2][1];
            d3[5] = del[1][1] - del[2][1];
            e213 = -e123;
            d1[6] = d2[5] * d1[2] + d3[5] * e213;
            d4[14] = d1[6] * d4[8] + d2[6] * e142 + d3[6] * e143;
            if (d1[6] > zero  &&  d2[6] > zero  &&
                d3[6] > zero  &&  d4[14] <= zero)
            {
                *nvs = 3;
                sum = d1[6] + d2[6] + d3[6];
                als[0] = d1[6] / sum;
                als[1] = d2[6] / sum;
                als[2] = one - als[0] - als[1];
                zsol[0] = y[2][0] + als[0] * (y[0][0] - y[2][0]) +
                    als[1] * (y[1][0] - y[2][0]);
                zsol[1] = y[2][1] + als[0] * (y[0][1] - y[2][1]) +
                    als[1] * (y[1][1] - y[2][1]);
                zsol[2] = y[2][2] + als[0] * (y[0][2] - y[2][2]) +
                    als[1] * (y[1][2] - y[2][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 14 :  */
            e124 = del[3][0] - del[3][1];
            e134 = del[3][0] - del[3][2];
            d1[8] = del[3][3] - del[3][0];
            d2[11] = d1[8] * d2[2] + d4[8] * e124;
            d3[12] = d1[8] * d3[4] + d4[8] * e134;
            if (d1[8] > zero  &&  d2[11] <= zero  &&
                d3[12] <= zero  &&  d4[8] > zero)
            {
                *nvs = 2;
                ris[1] = ris[3];
                rjs[1] = rjs[3];
                sum = d1[8] + d4[8];
                als[0] = d1[8] / sum;
                als[1] = one - als[0];
                zsol[0] = y[3][0] + als[0] * (y[0][0] - y[3][0]);
                zsol[1] = y[3][1] + als[0] * (y[0][1] - y[3][1]);
                zsol[2] = y[3][2] + als[0] * (y[0][2] - y[3][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[1][0] = y[3][0];
                y[1][1] = y[3][1];
                y[1][2] = y[3][2];
                del[1][0] = del[3][0];
                del[1][1] = del[3][3];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  face 124 :      */
            d2[9] = del[3][3] - del[3][1];
            d4[9] = del[1][1] - del[3][1];
            e214 = -e124;
            d1[11] = d2[9] * d1[2] + d4[9] * e214;
            d3[14] = d1[11] * d3[4] + d2[11] * e132 + d4[11] * e134;
            if (d1[11] > zero  &&  d2[11] > zero  &&
                d3[14] <= zero  &&  d4[11] > zero)
            {
                *nvs = 3;
                ris[2] = ris[3];
                rjs[2] = rjs[3];
                sum = d1[11] + d2[11] + d4[11];
                als[0] = d1[11] / sum;
                als[1] = d2[11] / sum;
                als[2] = one - als[0] - als[1];
                zsol[0] = y[3][0] + als[0] * (y[0][0] - y[3][0]) +
                    als[1] * (y[1][0] - y[3][0]);
                zsol[1] = y[3][1] + als[0] * (y[0][1] - y[3][1]) +
                    als[1] * (y[1][1] - y[3][1]);
                zsol[2] = y[3][2] + als[0] * (y[0][2] - y[3][2]) +
                    als[1] * (y[1][2] - y[3][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[2][0] = y[3][0];
                y[2][1] = y[3][1];
                y[2][2] = y[3][2];
                del[2][0] = del[3][0];
                del[2][1] = del[3][1];
                del[2][2] = del[3][3];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  face 134 :      */
            d3[10] = del[3][3] - del[3][2];
            d4[10] = del[2][2] - del[3][2];
            e314 = -e134;
            d1[12] = d3[10] * d1[4] + d4[10] * e314;
            d2[14] = d1[12] * d2[2] + d3[12] * e123 + d4[12] * e124;
            if (d1[12] > zero  &&  d2[14] <= zero  &&
                d3[12] > zero  &&  d4[12] > zero)
            {
                *nvs = 3;
                ris[1] = ris[3];
                rjs[1] = rjs[3];
                sum = d1[12] + d3[12] + d4[12];
                als[0] = d1[12] / sum;
                als[2] = d3[12] / sum;
                als[1] = one - als[0] - als[2];
                zsol[0] = y[3][0] + als[0] * (y[0][0] - y[3][0]) +
                    als[2] * (y[2][0] - y[3][0]);
                zsol[1] = y[3][1] + als[0] * (y[0][1] - y[3][1]) +
                    als[2] * (y[2][1] - y[3][1]);
                zsol[2] = y[3][2] + als[0] * (y[0][2] - y[3][2]) +
                    als[2] * (y[2][2] - y[3][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[1][0] = y[3][0];
                y[1][1] = y[3][1];
                y[1][2] = y[3][2];
                del[1][0] = del[3][0];
                del[1][1] = del[3][3];
                del[2][1] = del[3][2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  the  hull  of  all  4  points :  */
            e243 = del[2][1] - del[3][2];
            d4[13] = d2[5] * d4[9] + d3[5] * e243;
            e234 = del[3][1] - del[3][2];
            d3[13] = d2[9] * d3[5] + d4[9] * e234;
            e324 = -e234;
            d2[13] = d3[10] * d2[5] + d4[10] * e324;
            d1[14] = d2[13] * d1[2] + d3[13] * e213 + d4[13] * e214;
            if (d1[14] > zero  &&  d2[14] > zero  &&
                d3[14] > zero  &&  d4[14] > zero)
            {
                sum = d1[14] + d2[14] + d3[14] + d4[14];
                als[0] = d1[14] / sum;
                als[1] = d2[14] / sum;
                als[2] = d3[14] / sum;
                als[3] = one - als[0] - als[1] - als[2];
                zsol[0] = als[0] * y[0][0] + als[1] * y[1][0] +
                    als[2] * y[2][0] + als[3] * y[3][0];
                zsol[1] = als[0] * y[0][1] + als[1] * y[1][1] +
                    als[2] * y[2][1] + als[3] * y[3][1];
                zsol[2] = als[0] * y[0][2] + als[1] * y[1][2] +
                    als[2] * y[2][2] + als[3] * y[3][2];
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  vertex 2 :      */
            if (d1[2] <= zero  &&  d3[5] <= zero  && d4[9] <= zero)
            {
                *nvs = 1;
                ris[0] = ris[1];
                rjs[0] = rjs[1];
                als[0] = d2[1];
                zsol[0] = y[1][0];
                zsol[1] = y[1][1];
                zsol[2] = y[1][2];
                *dstsq = del[1][1];
                y[0][0] = y[1][0];
                y[0][1] = y[1][1];
                y[0][2] = y[1][2];
                del[0][0] = del[1][1];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  vertex 3 :      */
            if (d1[4] <= zero  &&  d2[5] <= zero  &&    d4[10] > zero)
            {
                *nvs = 1;
                ris[0] = ris[2];
                rjs[0] = rjs[2];
                als[0] = d3[3];
                zsol[0] = y[2][0];
                zsol[1] = y[2][1];
                zsol[2] = y[2][2];
                *dstsq = del[2][2];
                y[0][0] = y[2][0];
                y[0][1] = y[2][1];
                y[0][2] = y[2][2];
                del[0][0] = del[2][2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  vertex 4 :      */
            if (d1[8] <= zero  &&  d2[9] <= zero  && d3[10] <= zero)
            {
                *nvs = 1;
                ris[0] = ris[3];
                rjs[0] = rjs[3];
                als[0] = d4[7];
                zsol[0] = y[3][0];
                zsol[1] = y[3][1];
                zsol[2] = y[3][2];
                *dstsq = del[3][3];
                y[0][0] = y[3][0];
                y[0][1] = y[3][1];
                y[0][2] = y[3][2];
                del[0][0] = del[3][3];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line  segment 23 : */
            if (d1[6] <= zero  &&  d2[5] > zero  &&
                d3[5] > zero  &&  d4[13] <= zero)
            {
                *nvs = 2;
                ris[0] = ris[2];
                rjs[0] = rjs[2];
                sum = d2[5] + d3[5];
                als[1] = d2[5] / sum;
                als[0] = one - als[1];
                zsol[0] = y[2][0] + als[1] * (y[1][0] - y[2][0]);
                zsol[1] = y[2][1] + als[1] * (y[1][1] - y[2][1]);
                zsol[2] = y[2][2] + als[1] * (y[1][2] - y[2][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[0][0] = y[2][0];
                y[0][1] = y[2][1];
                y[0][2] = y[2][2];
                del[1][0] = del[2][1];
                del[0][0] = del[2][2];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line segment 24 : */
            if (d1[11] <= zero  &&  d2[9] > zero  &&
                d3[13] <= zero  &&  d4[9] > zero)
            {
                *nvs = 2;
                ris[0] = ris[3];
                rjs[0] = rjs[3];
                sum = d2[9] + d4[9];
                als[1] = d2[9] / sum;
                als[0] = one - als[1];
                zsol[0] = y[3][0] + als[1] * (y[1][0] - y[3][0]);
                zsol[1] = y[3][1] + als[1] * (y[1][1] - y[3][1]);
                zsol[2] = y[3][2] + als[1] * (y[1][2] - y[3][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[0][0] = y[3][0];
                y[0][1] = y[3][1];
                y[0][2] = y[3][2];
                del[1][0] = del[3][1];
                del[0][0] = del[3][3];
                return(1);
            }
            /* -----------------------------------------------
            c    check  optimality  of  line segment 34 : */
            if (d1[12] <= zero  &&  d2[13] <= zero  &&
                d3[10] > zero  &&  d4[10] > zero)
            {
                *nvs = 2;
                ris[0] = ris[2];
                ris[1] = ris[3];
                rjs[0] = rjs[2];
                rjs[1] = rjs[3];
                sum = d3[10] + d4[10];
                als[0] = d3[10] / sum;
                als[1] = one - als[0];
                zsol[0] = y[3][0] + als[0] * (y[2][0] - y[3][0]);
                zsol[1] = y[3][1] + als[0] * (y[2][1] - y[3][1]);
                zsol[2] = y[3][2] + als[0] * (y[2][2] - y[3][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[0][0] = y[2][0];
                y[0][1] = y[2][1];
                y[0][2] = y[2][2];
                y[1][0] = y[3][0];
                y[1][1] = y[3][1];
                y[1][2] = y[3][2];
                del[0][0] = del[2][2];
                del[1][0] = del[3][2];
                del[1][1] = del[3][3];
                return(1);
            }
            /*-----------------------------------------------
            c    check  optimality  of  face 234 :      */
            if (d1[14] <= zero  &&  d2[13] > zero  &&
                d3[13] > zero  &&  d4[13] > zero)
            {
                *nvs = 3;
                ris[0] = ris[3];
                rjs[0] = rjs[3];
                sum = d2[13] + d3[13] + d4[13];
                als[1] = d2[13] / sum;
                als[2] = d3[13] / sum;
                als[0] = one - als[1] - als[2];
                zsol[0] = y[3][0] + als[1] * (y[1][0] - y[3][0]) +
                    als[2] * (y[2][0] - y[3][0]);
                zsol[1] = y[3][1] + als[1] * (y[1][1] - y[3][1]) +
                    als[2] * (y[2][1] - y[3][1]);
                zsol[2] = y[3][2] + als[1] * (y[1][2] - y[3][2]) +
                    als[2] * (y[2][2] - y[3][2]);
                *dstsq = zsol[0] * zsol[0] + zsol[1] * zsol[1] + zsol[2] * zsol[2];
                y[0][0] = y[3][0];
                y[0][1] = y[3][1];
                y[0][2] = y[3][2];
                del[0][0] = del[3][3];
                del[1][0] = del[3][1];
                del[2][0] = del[3][2];
                return(1);
            }
            /* -----------------------------------------------
            c  need  to  go  for  the  *backup  procedure :
            c  (without  recomputing  the  di(*)  values)    */
            break;
        }  /* end switch */
        back_up = 0;
    }    /* end if(!*backup) */
    /*=====================================================================c
    c                                      c
    c  the  *backup procedure  begins ...                      c
    c                                      c
    c=====================================================================*/
    /* -----------------------------------------------
    c  if  the  di(*)  values  are  already  available,
    c  then  go  to  1101,  1201,  1301,  or  1401:   */
    /* ************************************************
    c  case  of  a  single  point ...                */
    switch (*nvs) {
        /* ************************************************
        c  case  of  four  points ...                    */
    case 4:
        if (back_up)
        {
            d2[2] = del[0][0] - del[1][0];
            d3[4] = del[0][0] - del[2][0];
            d4[8] = del[0][0] - del[3][0];
            e132 = del[1][0] - del[2][1];
            e142 = del[1][0] - del[3][1];
            d1[2] = del[1][1] - del[1][0];
            d3[6] = d1[2] * d3[4] + d2[2] * e132;
            d4[11] = d1[2] * d4[8] + d2[2] * e142;
            e123 = del[2][0] - del[2][1];
            e143 = del[2][0] - del[3][2];
            d1[4] = del[2][2] - del[2][0];
            d2[6] = d1[4] * d2[2] + d3[4] * e123;
            d4[12] = d1[4] * d4[8] + d3[4] * e143;
            d2[5] = del[2][2] - del[2][1];
            d3[5] = del[1][1] - del[2][1];
            e213 = -e123;
            d1[6] = d2[5] * d1[2] + d3[5] * e213;
            d4[14] = d1[6] * d4[8] + d2[6] * e142 + d3[6] * e143;
            e124 = del[3][0] - del[3][1];
            e134 = del[3][0] - del[3][2];
            d1[8] = del[3][3] - del[3][0];
            d2[11] = d1[8] * d2[2] + d4[8] * e124;
            d3[12] = d1[8] * d3[4] + d4[8] * e134;
            d2[9] = del[3][3] - del[3][1];
            d4[9] = del[1][1] - del[3][1];
            e214 = -e124;
            d1[11] = d2[9] * d1[2] + d4[9] * e214;
            d3[14] = d1[11] * d3[4] + d2[11] * e132 + d4[11] * e134;
            d3[10] = del[3][3] - del[3][2];
            d4[10] = del[2][2] - del[3][2];
            e314 = -e134;
            d1[12] = d3[10] * d1[4] + d4[10] * e314;
            d2[14] = d1[12] * d2[2] + d3[12] * e123 + d4[12] * e124;
            e243 = del[2][1] - del[3][2];
            d4[13] = d2[5] * d4[9] + d3[5] * e243;
            e234 = del[3][1] - del[3][2];
            d3[13] = d2[9] * d3[5] + d4[9] * e234;
            e324 = -e234;
            d2[13] = d3[10] * d2[5] + d4[10] * e324;
            d1[14] = d2[13] * d1[2] + d3[13] * e213 + d4[13] * e214;
        }
        /* -----------------------------------------------
        c    check  vertex 1 :                      */
        *dstsq = del[0][0];
        nvsd = 1;
        als[0] = d1[0];
        zsol[0] = y[0][0];
        zsol[1] = y[0][1];
        zsol[2] = y[0][2];
        iord[0] = 0;
        /* -----------------------------------------------
        c    check  line  segment 12 :              */
        if (d1[2] > zero  &&  d2[2] > zero)
        {
            sum = d1[2] + d2[2];
            alsd[0] = d1[2] / sum;
            alsd[1] = one - alsd[0];
            zsold[0] = y[1][0] + alsd[0] * (y[0][0] - y[1][0]);
            zsold[1] = y[1][1] + alsd[0] * (y[0][1] - y[1][1]);
            zsold[2] = y[1][2] + alsd[0] * (y[0][2] - y[1][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 1;
            }
        }
        /* -----------------------------------------------
        c    check  line  segment 13 :              */
        if (d1[4] > zero  &&  d3[4] > zero)
        {
            sum = d1[4] + d3[4];
            alsd[0] = d1[4] / sum;
            alsd[1] = one - alsd[0];
            zsold[0] = y[2][0] + alsd[0] * (y[0][0] - y[2][0]);
            zsold[1] = y[2][1] + alsd[0] * (y[0][1] - y[2][1]);
            zsold[2] = y[2][2] + alsd[0] * (y[0][2] - y[2][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 2;
            }
        }
        /* -----------------------------------------------
        c    check  face 123 :                      */
        if (d1[6] > zero  &&  d2[6] > zero  && d3[6] > zero)
        {
            sum = d1[6] + d2[6] + d3[6];
            alsd[0] = d1[6] / sum;
            alsd[1] = d2[6] / sum;
            alsd[2] = one - alsd[0] - alsd[1];
            zsold[0] = y[2][0] + alsd[0] * (y[0][0] - y[2][0]) +
                alsd[1] * (y[1][0] - y[2][0]);
            zsold[1] = y[2][1] + alsd[0] * (y[0][1] - y[2][1]) +
                alsd[1] * (y[1][1] - y[2][1]);
            zsold[2] = y[2][2] + alsd[0] * (y[0][2] - y[2][2]) +
                alsd[1] * (y[1][2] - y[2][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 3;
                als[0] = alsd[0];
                als[1] = alsd[1];
                als[2] = alsd[2];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 1;
                iord[2] = 2;
            }
        }
        /* -----------------------------------------------
        c    check  line  segment 14 :              */
        if (d1[8] > zero  &&  d4[8] > zero)
        {
            sum = d1[8] + d4[8];
            alsd[0] = d1[8] / sum;
            alsd[1] = one - alsd[0];
            zsold[0] = y[3][0] + alsd[0] * (y[0][0] - y[3][0]);
            zsold[1] = y[3][1] + alsd[0] * (y[0][1] - y[3][1]);
            zsold[2] = y[3][2] + alsd[0] * (y[0][2] - y[3][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 3;
            }
        }
        /* -----------------------------------------------
        c    check  face 124 :                      */
        if (d1[11] > zero  &&  d2[11] > zero  && d4[11] > zero)
        {
            sum = d1[11] + d2[11] + d4[11];
            alsd[0] = d1[11] / sum;
            alsd[1] = d2[11] / sum;
            alsd[2] = one - alsd[0] - alsd[1];
            zsold[0] = y[3][0] + alsd[0] * (y[0][0] - y[3][0]) +
                alsd[1] * (y[1][0] - y[3][0]);
            zsold[1] = y[3][1] + alsd[0] * (y[0][1] - y[3][1]) +
                alsd[1] * (y[1][1] - y[3][1]);
            zsold[2] = y[3][2] + alsd[0] * (y[0][2] - y[3][2]) +
                alsd[1] * (y[1][2] - y[3][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 3;
                als[0] = alsd[0];
                als[1] = alsd[1];
                als[2] = alsd[2];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 1;
                iord[2] = 3;
            }
        }
        /* -----------------------------------------------
        c    check  face 134 :                      */
        if (d1[12] > zero  && d3[12] > zero  &&  d4[12] > zero)
        {
            sum = d1[12] + d3[12] + d4[12];
            alsd[0] = d1[12] / sum;
            alsd[2] = d3[12] / sum;
            alsd[1] = one - alsd[0] - alsd[2];
            zsold[0] = y[3][0] + alsd[0] * (y[0][0] - y[3][0]) +
                alsd[2] * (y[2][0] - y[3][0]);
            zsold[1] = y[3][1] + alsd[0] * (y[0][1] - y[3][1]) +
                alsd[2] * (y[2][1] - y[3][1]);
            zsold[2] = y[3][2] + alsd[0] * (y[0][2] - y[3][2]) +
                alsd[2] * (y[2][2] - y[3][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 3;
                als[0] = alsd[0];
                als[1] = alsd[1];
                als[2] = alsd[2];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 3;
                iord[2] = 2;
            }
        }
        /* -----------------------------------------------
        c    check  the  hull  of  all  4  points : */
        if (d1[14] > zero  &&  d2[14] > zero  &&
            d3[14] > zero  &&  d4[14] > zero)
        {
            sum = d1[14] + d2[14] + d3[14] + d4[14];
            alsd[0] = d1[14] / sum;
            alsd[1] = d2[14] / sum;
            alsd[2] = d3[14] / sum;
            alsd[3] = one - alsd[0] - alsd[1] - alsd[2];
            zsold[0] = alsd[0] * y[0][0] + alsd[1] * y[1][0] +
                alsd[2] * y[2][0] + alsd[3] * y[3][0];
            zsold[1] = alsd[0] * y[0][1] + alsd[1] * y[1][1] +
                alsd[2] * y[2][1] + alsd[3] * y[3][1];
            zsold[2] = alsd[0] * y[0][2] + alsd[1] * y[1][2] +
                alsd[2] * y[2][2] + alsd[3] * y[3][2];
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 4;
                als[0] = alsd[0];
                als[1] = alsd[1];
                als[2] = alsd[2];
                als[3] = alsd[3];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 0;
                iord[1] = 1;
                iord[2] = 2;
                iord[3] = 3;
            }
        }
        /* -----------------------------------------------
        c    check  vertex 2 :                      */
        if (del[1][1] < *dstsq)
        {
            nvsd = 1;
            *dstsq = del[1][1];
            als[0] = d2[1];
            zsol[0] = y[1][0];
            zsol[1] = y[1][1];
            zsol[2] = y[1][2];
            iord[0] = 1;
        }
        /* -----------------------------------------------
        c    check  vertex 3 :                      */
        if (del[2][2] < *dstsq)
        {
            nvsd = 1;
            *dstsq = del[2][2];
            als[0] = d3[3];
            zsol[0] = y[2][0];
            zsol[1] = y[2][1];
            zsol[2] = y[2][2];
            iord[0] = 2;
        }
        /* -----------------------------------------------
        c    check  vertex 4 :                      */
        if (del[3][3] < *dstsq)
        {
            nvsd = 1;
            *dstsq = del[3][3];
            als[0] = d4[7];
            zsol[0] = y[3][0];
            zsol[1] = y[3][1];
            zsol[2] = y[3][2];
            iord[0] = 3;
        }
        /* -----------------------------------------------
        c    check  line  segment 23 :              */
        if (d2[5] > zero  &&  d3[5] > zero)
        {
            sum = d2[5] + d3[5];
            alsd[1] = d2[5] / sum;
            alsd[0] = one - alsd[1];
            zsold[0] = y[2][0] + alsd[1] * (y[1][0] - y[2][0]);
            zsold[1] = y[2][1] + alsd[1] * (y[1][1] - y[2][1]);
            zsold[2] = y[2][2] + alsd[1] * (y[1][2] - y[2][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 2;
                iord[1] = 1;
            }
        }
        /*  -----------------------------------------------
        c    check  line segment 24 :                */
        if (d2[9] > zero  &&  d4[9] > zero)
        {
            sum = d2[9] + d4[9];
            alsd[1] = d2[9] / sum;
            alsd[0] = one - alsd[1];
            zsold[0] = y[3][0] + alsd[1] * (y[1][0] - y[3][0]);
            zsold[1] = y[3][1] + alsd[1] * (y[1][1] - y[3][1]);
            zsold[2] = y[3][2] + alsd[1] * (y[1][2] - y[3][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 3;
                iord[1] = 1;
            }
        }
        /* -----------------------------------------------
        c    check  line segment 34 :               */
        if (d3[10] > zero  &&  d4[10] > zero)
        {
            sum = d3[10] + d4[10];
            alsd[0] = d3[10] / sum;
            alsd[1] = one - alsd[0];
            zsold[0] = y[3][0] + alsd[0] * (y[2][0] - y[3][0]);
            zsold[1] = y[3][1] + alsd[0] * (y[2][1] - y[3][1]);
            zsold[2] = y[3][2] + alsd[0] * (y[2][2] - y[3][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 2;
                als[0] = alsd[0];
                als[1] = alsd[1];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 2;
                iord[1] = 3;
            }
        }
        /* -----------------------------------------------
        c    check  face 234 :                      */
        if (d2[13] > zero  && d3[13] > zero  &&  d4[13] > zero)
        {
            sum = d2[13] + d3[13] + d4[13];
            alsd[1] = d2[13] / sum;
            alsd[2] = d3[13] / sum;
            alsd[0] = one - alsd[1] - alsd[2];
            zsold[0] = y[3][0] + alsd[1] * (y[1][0] - y[3][0]) +
                alsd[2] * (y[2][0] - y[3][0]);
            zsold[1] = y[3][1] + alsd[1] * (y[1][1] - y[3][1]) +
                alsd[2] * (y[2][1] - y[3][1]);
            zsold[2] = y[3][2] + alsd[1] * (y[1][2] - y[3][2]) +
                alsd[2] * (y[2][2] - y[3][2]);
            dstsqd = zsold[0] * zsold[0] + zsold[1] * zsold[1] +
                zsold[2] * zsold[2];
            if (dstsqd >= *dstsq)
            {
                *dstsq = dstsqd;
                nvsd = 3;
                als[0] = alsd[0];
                als[1] = alsd[1];
                als[2] = alsd[2];
                zsol[0] = zsold[0];
                zsol[1] = zsold[1];
                zsol[2] = zsold[2];
                iord[0] = 3;
                iord[1] = 1;
                iord[2] = 2;
            }
        }
        break;
    }  /* end switch */
    /* -----------------------------------------------
    c  the  final  reordering :                     */
    for (k = 0; k < *nvs; k++)
    {
        risd[k] = ris[k];
        rjsd[k] = rjs[k];
        yd[k][0] = y[k][0];
        yd[k][1] = y[k][1];
        yd[k][2] = y[k][2];
        for (l = 0; l <= k; l++)
        {
            deld[k][l] = del[k][l];
            deld[l][k] = del[k][l];
        }
    }

    *nvs = nvsd;
    for (k = 0; k < *nvs; k++)
    {
        kk = iord[k];
        ris[k] = risd[kk];
        rjs[k] = rjsd[kk];
        y[k][0] = yd[kk][0];
        y[k][1] = yd[kk][1];
        y[k][2] = yd[kk][2];
        for (l = 0; l <= k; l++)
        {
            ll = iord[l];
            del[k][l] = deld[kk][ll];
        }
    }
    *backup = 1;
    return(1);
}  /* ende der procedur dsbp  */

/*********************************************************************************************************************************/
int dist3(int nvi, int nvj, double center_i[], double center_j[], double zi[][3], double zj[][3], int iwant[], double eps,
    double zisol[], double zjsol[], double zsol[], double *dist, int *nv_s,
    int ris[], int rjs[], double als[], int *ncy, double *gfinal, double zbi[][3],
    double zbj[][3], double neworg[], double *di, double *dj, int *ierror)
    /*
    Input
    int nvi: number of vertices of polytope I
    int nvj: number of vertices of polytope J
    double center_i[]: center of polytope I, if provided, see iwant[]
    double center_j[]: center of polytope J, if provided, see iwant[]
    double zi[][3]:  vertices of polytope I
    double zj[][3]:  vertices of polytope J
    C  IWANT(*)       :  Integer Vector of length atleast 4. The elements
    C                    of IWANT define options.
    C
    C                    IWANT(1)- Use this parameter to obtain intermediate
    C                       output. If IWANT(1).LE.0, no intermediate output
    C                       is printed. If IWANT(1).GT.0, some useful values
    C                       describing the progress of the routine are
    C                       printed in the output device whose number is
    C                       IWANT(1).
    C
    C                    IWANT(2)-(3) These two parameters are no longer in
    C                       use. It is kept here so that this program is
    C                       compatible with the earlier version.
    C
    C                    IWANT(4) -- This parameter allows the algorithm
    C                       to be started with the initial data, NVS, RIS,
    C                       and RJS (normally, these are output parameters;
    C                       see below). If good guesses for these parameters
    C                       are available they may be supplied as an initial
    C                       set.
    C                       If IWANT(4)=1, then the algorithm will start
    C                       from this initial set.
    C                       If IWANT(4).NE.1, the algorithm will set its
    C                       own initialization.
    C
    C                       There is one particular situation where
    C                       supplying good initial data is especially
    C                       easy. Suppose the routine has just returned a
    C                       solution to a problem and it is necessary to
    C                       recompute a solution after a slight change in
    C                       the elements of ZI and ZJ. This is achieved most
    C                       efficiently by setting IWANT(4) = 1 and leaving
    C                       unchanged the values of NVS, RIS and RJS
    C                       obtained from the previous call.
    C
    C   EPS              : Double Precision variable.
    C                       EPS is a tolerance parameter which determines the
    C                       termination condition of the algorithm. The
    C                       algorithm will terminate if, in a cycle, the
    C                       following condition is satisfied:
    C
    C                                G(ZSOL) .LE. EPS,
    C                       where:
    C                          ZSOL = ZISOL - ZJSOL ;
    C                          ZISOL and ZJSOL are, respectively, the "best"
    C                                points of the two polytopes in the
    C                                cycle ;
    C                          G(.) is the function defined by
    C                                G(Z) = NORM(Z)**2 + HI(-Z) + HJ(Z) ;
    C                          and,
    C                          HI(.) and HJ(.) are, respectively, the
    C                          support functions of the two polytopes,
    C                          defined by
    C                                HI(Y) = MAX �X*Y : X is in Polytope-I�,
    C                                HJ(Y) = MAX �X*Y : X is in Polytope-J�.
    C                       See section 6 of the report mentioned earlier
    C                       for more details.
    C
    C                       **** Suggested methods for choosing EPS :
    C
    C                            1.Choose EPS based on a knowledge of the
    C                       problem at hand and the accuracy desired. For
    C                       high accuracy, a reasonable choice is
    C                       EPS = 20.D0 * MCHEPS,
    C                       where MCHEPS=machine unit roundoff.
    C

    Output:
    c
    C  ZISOL(*)       :  Double Precision Vector of length atleast 3. The
    C                    first 3 components of ZISOL define the point on
    C                    Polytope-I closest to Polytope-J. This is the case
    C                    when the distance (DIST) returned is greater than
    C                    zero. When the distance returned is zero, the
    C                    polytopes are colliding (or touching) and there
    C                    is no meaning to the values of ZISOL or ZJSOL.
    C
    C  ZJSOL(*)       :  Double Precision Vector of length atleast 3. The
    C                    first 3 components of ZJSOL define the point on
    C                    Polytope-J closest to Polytope-I. See comment
    C                    on ZISOL on the case when the distance is zero.
    C
    C  ZSOL(*)        :  Double Precision Vector of length atleast 3.
    C                    ZSOL = ZISOL - ZJSOL.
    C
    C  DIST           :  Double Precision Variable. The minimum distance
    C                    between Polytope-I and Polytope-J.
    C                    DIST = NORM (ZSOL).
    C                    A zero value returned for DIST means that the
    C                    two polytopes are either colliding or are just
    C                    touching.
    C
    C  NVS, RIS(*),   :
    C  RJS(*), ALS(*) :  NVS = Integer; RIS and RJS are Integer vectors of
    C                    length atleast 4; ALS is a Double Precision Vector
    C                    of length atleast 4. These parameters define ZISOL
    C                    and ZJSOL in terms of the points in ZI and ZJ as
    C                    follows:
    C                       ZISOL = ZI(.,RIS(1)) * ALS(1) + ...
    C                                  + ZI(.,RIS(NVS)) * ALS(NVS) ,
    C                       ZJSOL = ZJ(.,RJS(1)) * ALS(1) + ...
    C                                  + ZJ(.,RJS(NVS)) * ALS(NVS) ,
    C                    where, ZI(.,K) and ZJ(.,K) are, respectively, the
    C                    K-th columns of ZI and ZJ. NVS and ALS satisfy:
    C                       1 .LE. NVS .LE. 4 ;
    C                       ALS(1) + ... + ALS(NVS) = 1.0 ;
    C                    and
    C                       ALS(K) .GT. 0.0  for  1 .LE. K .LE. NVS.
    C                    The variables RIS, RJS and ALS have no meaning
    C                    in the case where the value of DIST returned is
    C                    zero. See comments on ZISOL.
    C
    C  NCY            :  Integer. The total number of cycles needed by the
    C                    algorithm to solve the problem.
    C
    C  GFINAL         :  Double Precision variable.
    C                    GFINAL = G(ZSOL), where G is the function mentioned
    C                    in the description of EPS. If EPS=0.0D0, then
    C                    by theory ZISOL and ZJSOL should correspond exactly
    C                    to the points on Polytope-I and Polytope-J which
    C                    yield the minimum distance, and GFINAL = 0.0D0.
    C                    However, because of round-off errors and the
    C                    specification of a non-zero EPS, GFINAL is in
    C                    general non-zero.
    C
    C
    C
    C  IERROR         :  Integer Variable used to indicate errors.
    C                    If IERROR = 0,there is no error.
    C                    If IERROR is positive, there is a possible error
    C                    as indicated by the following list:
    C
    C                    IERROR = 1 -- Dimensioning error. One of the
    C                       following has been detected: NVI.LE.0, NVJ.LE.0,
    C                       NZDIM.LT.3.
    C
    C                    IERROR = 2 -- Initialization error. IWANT(4) = 1,
    C                       and, either NVS .LE 0 or NVS .GT. 4.
    C
    C                    IERROR = 3 -- Termination error. This means that
    C                       the stopping criterion could not be satisfied
    C                       because EPS is too small.
    C                       In this situation, the algorithm terminates only
    C                       when it is unable to decrease the distance any
    C                       further. Therefore, when IERROR=3 on output,
    C                       the final solution will be very accurate.
    C
    */
{

    int    err, iout, k, l, nri, nrj, nv, raus1 = 0,
        kk, ll, ii, jj, nvsold, ri[5], rj[5], iord[5];
    double epsdsq, cent[3], ncent[3], inn, dsum, sfi, sfj,
        y[4][3], del[4][4], yold[4][3], delold[4][4],
        dstsqp, nzsol[3], g, dstsq;
    int    backup;

    double dou_hilf;
    //FILE   *fp;
    /*=====================================================================c
    c                                      c
    c  step 1.   initialization  phase ...                     c
    c                                      c
    c=====================================================================*/
    //eps = 0.00001;
    *ierror = 0;
    *ncy = 1;
    /*
    c        check  for  dimensioning  errors :
    */
    if (nvi <= 0 || nvj <= 0)
    {
        *ierror = 1;
        //         fprintf(fp, "\nDimensionierungsfehler: ierror = 1");
        return(1);
    }
    /*
    c        print  titles  if  iwant[1]  >  0 :
    */
    if (iwant[1] <= 0)
        iout = 0;
    else
    {
        iout = iwant[1];
        /*
              printf("\n-----------------------------------------------------");
              printf("\nZyklus   Abstandsquadrat            g");
              printf("\n-----------------------------------------------------");
              */
    }
    /*
    c        form  zbi,  zbj,  cent  and  ncent :
    */
    err = tranc(nvi, nvj, zi, zj, zbi, zbj, iwant, neworg, cent, ncent);
    /*
    c  the  centroids  pi[*] and pj[*] are already available:
    */
    //for(l=0;l<3;l++)
    //{
    //  cent[l]  = (double) (center_i[l] - center_j[l]);
    //  ncent[l] = -cent[l];
    //}
    /*
    c        compute  di,  dj  and  epsdsq  if  iwant[3] = 1 :
    */
    epsdsq = eps;
    if (iwant[3] == 1)
    {
        *di = 0;
        for (k = 0; k < nvi; k++)
        {
            inn = zbi[k][0] * zbi[k][0] + zbi[k][1] * zbi[k][1] +
                zbi[k][2] * zbi[k][2];
            if (inn > *di) *di = inn;
        }
        *di = sqrt(*di);

        *dj = 0;
        for (k = 0; k < nvj; k++)
        {
            inn = zbj[k][0] * zbj[k][0] + zbj[k][1] * zbj[k][1] +
                zbj[k][2] * zbj[k][2];
            if (inn > *dj) *dj = inn;
        }
        *dj = sqrt(*dj);
        dsum = *di + *dj;
        epsdsq = eps * dsum * dsum;
    }
    /*
    c        set  initial  nv_s,  ris[*]  and  rjs[*]  if
    c        iwant[4]  !=  1 :
    */
    if (iwant[4] != 1)
    {
        *nv_s = 1;
        csfcn(nvi, zbi, ncent, &sfi, &nri);
        csfcn(nvj, zbj, cent, &sfj, &nrj);
        ris[0] = nri;
        rjs[0] = nrj;
    }
    if (*nv_s <= 0 || *nv_s > 4)
    {
        *ierror = 2;
        //if (iout > 0)  fprintf(fp, "\nInitialisierungsfehler: ierror = 2");
        return(1);
    }
    /*
    c        compute  y  and  del  for  initial  set :
    */
    for (l = 0; l < *nv_s; l++)
    {
        ii = ris[l];
        jj = rjs[l];
        y[l][0] = zbi[ii][0] - zbj[jj][0];
        y[l][1] = zbi[ii][1] - zbj[jj][1];
        y[l][2] = zbi[ii][2] - zbj[jj][2];
        for (k = 0; k <= l; k++)
            del[l][k] = y[k][0] * y[l][0] + y[k][1] * y[l][1] + y[k][2] * y[l][2];
    }
    /*
    c        set  dstsqp = del[1][1] + del[1][1] + 1.0  for  use
    c        in  step 4 :
    */
    dstsqp = del[0][0] + del[0][0] + 1.0;
    /*=====================================================================c
    c                                      c
    c  step 2.  apply  the  distance  subalgorithm ...             c
    c                                      c
    c=====================================================================*/
    while (1)
    {
        /*
        c        set  backup =  0   so  that  the  usual  distance
        c        subalgorithm  is  used :
        */
        backup = 0;
        /*
        c        for  backup  procedure,  statement  220  is  entered
        c        with  backup =  1
        */
    jump:
        if (*nv_s == 4)
            err = dsbp4(nv_s, ris, rjs, y, del, zsol, als, &dstsq, &backup);
        else
            err = dsbp(nv_s, ris, rjs, y, del, zsol, als, &dstsq, &backup);
        /*=====================================================================c
        c                                      c
        c  step 3.  compute  g  and  test  for  optimality ...             c
        c                                      c
        c=====================================================================*/
        nzsol[0] = -zsol[0];
        nzsol[1] = -zsol[1];
        nzsol[2] = -zsol[2];
        csfcn(nvi, zbi, nzsol, &sfi, &nri);
        csfcn(nvj, zbj, zsol, &sfj, &nrj);

        g = dstsq + sfi + sfj;


        /*if (iout  >  0) {
          printf("\n ncy= %d   %13.7lf  nvs= %d", *ncy, g, *nv_s);
          }*/
        if (g <= epsdsq)
        {
            raus1 = 1;
            break;
        }
        /*=====================================================================c
        c                                      c
        c  step 4.  include  the  new  point  and  prepare  for  the           c
        c       next  cycle ...                        c
        c                                      c
        c======================================================================c
        c
        c        first  check  if  there  is  a  need  for  step 5 :
        */
        if (dstsq >= dstsqp || *nv_s >= 4)  break;

        dstsqp = dstsq;
        /*
        c        put  the  first  point  in  the  last  spot :
        */
        nvsold = *nv_s;
        ris[*nv_s] = ris[0];
        rjs[*nv_s] = rjs[0];
        y[*nv_s][0] = y[0][0];
        y[*nv_s][1] = y[0][1];
        y[*nv_s][2] = y[0][2];
        for (k = 1; k < nvsold; k++)
            del[*nv_s][k] = del[k][0];
        del[*nv_s][*nv_s] = del[0][0];
        (*nv_s)++;
        /*
        c        put  the  new  point  in  the  first  spot :
        */
        ris[0] = nri;
        rjs[0] = nrj;

        y[0][0] = zbi[nri][0] - zbj[nrj][0];
        y[0][1] = zbi[nri][1] - zbj[nrj][1];
        y[0][2] = zbi[nri][2] - zbj[nrj][2];
        for (k = 0; k < *nv_s; k++)
            del[k][0] = y[k][0] * y[0][0] + y[k][1] * y[0][1] + y[k][2] * y[0][2];
        /*
        c        use  nv,  ri[*],  rj[*],  yold[*,*]  and  delold[*,*]  for
        c        temporary  storage.  these  elements  are  useful
        c        immediately below  when  nv_s = 4, and  also  in  step 6.
        */
        nv = *nv_s;
        for (k = 0; k < nv; k++)
        {
            ri[k] = ris[k];
            rj[k] = rjs[k];
            yold[k][0] = y[k][0];
            yold[k][1] = y[k][1];
            yold[k][2] = y[k][2];
            for (l = 0; l <= k; l++)
            {
                delold[k][l] = del[k][l];
                delold[l][k] = delold[k][l];
            }
        }
        /*
        c        if  nv_s = 4,  rearrange  del[2][1] del[3][1]  and del[4][1]
        c        in  non decreasing  order :
        */
        if (*nv_s > 3)
        {
            iord[0] = 0;
            iord[1] = 1;
            iord[2] = 2;
            if (del[2][0] < del[1][0])
            {
                iord[1] = 2;
                iord[2] = 1;
            }
            ii = iord[1];
            if (del[3][0] < del[ii][0])
            {
                iord[3] = iord[2];
                iord[2] = iord[1];
                iord[1] = 3;
            }
            else
            {
                ii = iord[2];
                if (del[3][0] < del[ii][0])
                {
                    iord[3] = iord[2];
                    iord[2] = 3;
                }
                else
                    iord[3] = 3;
            }
            /*
            c        reorder  ris[*],  y[*][*]  and  del[*][*] :
            */
            for (k = 1; k < 4; k++)
            {
                kk = iord[k];
                ris[k] = ri[kk];
                rjs[k] = rj[kk];
                y[k][0] = yold[kk][0];
                y[k][1] = yold[kk][1];
                y[k][2] = yold[kk][2];
                for (l = 0; l <= k; l++)
                {
                    ll = iord[l];
                    del[k][l] = delold[kk][ll];
                }
            }
        } /* end if (*nv_s>3) */
        (*ncy)++;
    }  /* end while */
    /*=====================================================================c
    c                                      c
    c  step 5.  quit  if  backup =  1   ...                        c
    c                                      c
    c=====================================================================*/
    if (!raus1)
    {
        if (backup)
            *ierror = 3;
        /*=====================================================================c
        c                                      c
        c  step 6.  re do  the  distance  subalgorithm  using  the  backup     c
        c       procedure  and  go  to  step 3.  in  other  words,  set    c
        c       backup =  1 ,  put  old values  in  nv_s,  ris[*],      c
        c       rjs[*],  y[*][*]  and  del[*][*],  and  go  to  step 2 ...   c
        c                                      c
        c=====================================================================*/
        else
        {
            backup = 1;
            if (*ncy != 1)
            {
                *nv_s = nv;
                for (k = 0; k < *nv_s; k++)
                {
                    ris[k] = ri[k];
                    rjs[k] = rj[k];
                    y[k][0] = yold[k][0];
                    y[k][1] = yold[k][1];
                    y[k][2] = yold[k][2];
                    for (l = 0; l <= k; l++)
                        del[k][l] = delold[k][l];
                }
            }
            /*if (iout  >  0)
               fprintf(fp, "\nWiederholung von Zyklus %d mit der Backup Prozedur", *ncy);*/
            goto jump;
        } /* else if(backup) */
    }  /* end if (!raus1) */
    /*=====================================================================c
    c                                      c
    c  step 7.  the  final  phase ...                      c
    c                                      c
    c=====================================================================*/

    /*
    c        returning  the  solution  with  ierror = 0  or  3 :
    */
    for (l = 0; l < 3; l++)
    {
        zisol[l] = 0.0;
        zjsol[l] = 0.0;
        for (k = 0; k < *nv_s; k++)
        {
            ii = ris[k];
            jj = rjs[k];
            zisol[l] += zi[ii][l] * als[k];
            zjsol[l] += zj[jj][l] * als[k];
        }
    }
    dou_hilf = dstsq*dstsq;
    dstsq = sqrt(dou_hilf);
    *dist = sqrt(dstsq);
    *gfinal = g;
    /*if (iout  >  0)
        {
        printf("\n-----------------------------------------------------");
        if (*ierror  ==  0) fprintf(fp, "\nerfolgreiches terminieren ierror = 0");
        if (*ierror  ==  3) fprintf(fp, "\neps ist zu klein");
        }*/
    return 0;
}   /*  ende der prozedur dist3  */

