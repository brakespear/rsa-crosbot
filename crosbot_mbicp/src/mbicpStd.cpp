#include "mbicpStd.h"
#include <unistd.h>
#include <algorithm>
#include <float.h>

/*************************************************************************************/
/*                                                                              */
/*  File:	   MbICP.h                                                      */
/*  Authors:       Luis Montesano and Javier Minguez                            */
/*  Modified:      1/3/2006                                                     */
/*                                                                              */
/*  This library implements the:                                                */
/*										*/
/*	J. Minguez, F. Lamiraux and L. Montesano				*/
/*	Metric-Based Iterative Closest Point, 					*/
/*  Scan Matching for Mobile Robot Displacement Estimation			*/
/*	IEEE Transactions on Roboticics (2006)					*/
/*                                                                                   */
/*************************************************************************************/


// #include "MbICP.h"
// #include "MBICP/MbICP2.h"
// #include "MBICP/calcul.h"
// #include "MBICP/sp_matrix.h"
#include <stdio.h>
#include <math.h>
// #include "MBICP/percolate.h"

namespace mbicp
{

// Initial error to compute error ratio
#define BIG_INITIAL_ERROR 1000000.0F

#define MROWS(m)             ((m).rows)
#define MCOLS(m)             ((m).cols)
#define MDATA(m,i,j)         ((m).data[i][j])

#define VELEMENTS(v)         ((v).elements)
#define VDATA(v,i)           ((v).data[i])

#define M_SQUARE(m)          ((m).rows == (m).cols)
#define M_COMPAT_DIM(m, n)   ((m).cols == (n).rows)
#define M_EQUAL_DIM(m, n)    (((m).rows == (n).rows)  && ((m).cols == (n).cols))
#define V_EQUAL_DIM(v, w)    (((v).elements == (w).elements))
#define MV_COMPAT_DIM(m, v)  ((m).cols == (v).elements)

#define FIRST(b)             ((b).mat[0])
#define SECOND(b)            ((b).mat[1])
#define THIRD(b)             ((b).mat[2])
#define RANGE(b)             ((b).range)

#define SQUARE(x)            ((x)*(x))

// Debugging flag. Print sm info in the screen.
// #define INTMATSM_DEB

// ************************
// Function that initializes the SM parameters
// ************************

void MBICPStd::Init_MbICP_ScanMatching(float max_laser_range,float Bw, float Br,
					  float L, int laserStep,
					  float MaxDistInter,
					  float filter,
					  int ProjectionFilter,
					  float AsocError,
					  int MaxIter, float error_ratio,
					  float error_x, float error_y, float error_t, int IterSmoothConv,
					float xmin, float xmax, float ymin, float ymax){

  #ifdef INTMATSM_DEB
	printf("-- Init EM params . . ");
  #endif

  MAXLASERRANGE = max_laser_range;
  MINLASERRANGE = 0.4;
  params.Bw = Bw;
  params.Br = Br*Br;
  params.error_th=error_ratio;
  params.MaxIter=MaxIter;
  params.LMET=L;
  params.laserStep=laserStep;
  params.MaxDistInter=MaxDistInter;
  params.filter=filter;
  params.ProjectionFilter=ProjectionFilter;
  params.AsocError=AsocError;
  params.errx_out=error_x;
  params.erry_out=error_y;
  params.errt_out=error_t;
  params.IterSmoothConv=IterSmoothConv;
  params.xmin = xmin;
  params.xmax = xmax;
  params.ymin = ymin;
  params.ymax = ymax;
  xmin_ = xmin;
  xmax_ = xmax;
  ymin_ = ymin;
  ymax_ = ymax;

  useAdvanced_ = false;

  #ifdef INTMATSM_DEB
	printf(". OK!\n");
  #endif

}


// ************************
// Function that initializes the SM parameters
// ************************

int MBICPStd::MbICPmatcher(Tpfp *laserK, Tpfp *laserK1,
				Tsc *sensorMotion, Tsc *solution){
	double x, y, th;
	preProcessingCreate(laserK,laserK1,sensorMotion);

	return MbICPMatchScans(x, y, th);

}

int MBICPStd::MbICPMatchScans(double &x, double &y, double &th) {
	int resEStep=1;
	int resMStep=1;
	int numIteration=0;
	Tsc solution;

//	motion2.x = 0.0;
//	motion2.y = 0.0;
	motion2.tita = 0.0;

	// Preprocess both scans
	preProcessingCalc();
	x = 0.0;
	y = 0.0;
	th = 0.0;
	solution.x = 0.0;
	solution.y = 0.0;
	solution.tita = 0.0;

	while (numIteration<params.MaxIter){

		// Compute the correspondences of the MbICP
		resEStep=EStep();;
		if (resEStep!=1) {
			return -1; // E failure
		}
		// Minize and compute the solution
		resMStep=MStep(&solution);
// cerr << solution.x << " " << solution.y << " " << solution.tita << endl;
		if (resMStep==1) {
			x = solution.x;
			y = solution.y;
			th = solution.tita;
			return 1;
		} else if (resMStep==-1) {
			return -2; // M failure
		} else {
			numIteration++;
		}
	}

	return 2; // too many iterations

}


// ---------------------------------------------------------------
// ---------------------------------------------------------------
// Inner functions
// ---------------------------------------------------------------
// ---------------------------------------------------------------

// ************************
// Function that does the association step of the MbICP
// ************************

int MBICPStd::EStep()
{
  int cnt;
  int i,J;

  static Tscan ptosNewRef;
  static int indexPtosNewRef[MAXLASERPOINTS];

  int L,R,Io;
  float dist;
  float cp_ass_ptX,cp_ass_ptY,cp_ass_ptD;
  float tmp_cp_indD;

  float q1x, q1y, q2x,q2y,p2x,p2y, dqx, dqy, dqpx, dqpy, qx, qy,dx,dy;
  float landaMin;
  float A,B,C,D;
  float LMET2;

  LMET2=params.LMET*params.LMET;


	// Transform the points according to the current pose estimation

	ptosNewRef.numPuntos=0;
	for (i=0; i<ptosNew.numPuntos; i++){
		transfor_directa_p ( ptosNew.laserC[i].x, ptosNew.laserC[i].y,
			&motion2, &ptosNewRef.laserC[ptosNewRef.numPuntos]);
		car2pol(&ptosNewRef.laserC[ptosNewRef.numPuntos],&ptosNewRef.laserP[ptosNewRef.numPuntos]);
		ptosNewRef.numPuntos++;
	}
// printf("initial: %d\n",ptosNewRef.numPuntos);
	// ----
	/* Projection Filter */
	/* Eliminate the points that cannot be seen */
	/* Furthermore it orders the points with the angle */

	cnt = 1; /* Becarefull with this filter (order) when the angles are big >90 */
	ptosNoView.numPuntos=0;
	if (params.ProjectionFilter==1){
		for (i=1;i<ptosNewRef.numPuntos;i++){
			if (ptosNewRef.laserP[i].t>=ptosNewRef.laserP[cnt-1].t){
				ptosNewRef.laserP[cnt]=ptosNewRef.laserP[i];
				ptosNewRef.laserC[cnt]=ptosNewRef.laserC[i];
				cnt++;
			}
			else{
				ptosNoView.laserP[ptosNoView.numPuntos]=ptosNewRef.laserP[i];
				ptosNoView.laserC[ptosNoView.numPuntos]=ptosNewRef.laserC[i];
				ptosNoView.numPuntos++;
			}
		}
		ptosNewRef.numPuntos=cnt;
	}
// printf("projection: %d\n",ptosNewRef.numPuntos);

	// ----
	/* Build the index for the windows (this is the role of the Bw parameter */
	/* The correspondences are searched between windows in both scans */
	/* Like this you speed up the algorithm */

	L=0; R=0; // index of the window for ptoRef
	Io=0; // index of the window for ptoNewRef

	if (ptosNewRef.laserP[Io].t<ptosRef.laserP[L].t) {
		if (ptosNewRef.laserP[Io].t + params.Bw < ptosRef.laserP[L].t){
			while (Io<ptosNewRef.numPuntos-1 && ptosNewRef.laserP[Io].t + params.Bw < ptosRef.laserP[L].t) {
				Io++;
			}
		}
		else{
			while (R<ptosRef.numPuntos-1 && ptosNewRef.laserP[Io].t + params.Bw > ptosRef.laserP[R+1].t)
				R++;
		}
	}
	else{
		while (L<ptosRef.numPuntos-1 && ptosNewRef.laserP[Io].t - params.Bw > ptosRef.laserP[L].t)
			L++;
		R=L;
		while (R<ptosRef.numPuntos-1 && ptosNewRef.laserP[Io].t + params.Bw > ptosRef.laserP[R+1].t)
			R++;
	}

	// ----
	/* Look for potential correspondences between the scans */
	/* Here is where we use the windows */

	cnt=0;
	for (i=Io;i<ptosNewRef.numPuntos;i++){

		// Keep the index of the original scan ordering
		cp_associations[cnt].index=indexPtosNewRef[i];

		// Move the window
		while  (L < ptosRef.numPuntos-1 && ptosNewRef.laserP[i].t - params.Bw > ptosRef.laserP[L].t)
			L = L + 1;
		while (R <ptosRef.numPuntos-1 && ptosNewRef.laserP[i].t + params.Bw > ptosRef.laserP[R+1].t)
			R = R + 1;

		cp_associations[cnt].L=L;
		cp_associations[cnt].R=R;

		if (L==R){
			// Just one possible correspondence

			// precompute stuff to speed up
			qx=ptosRef.laserC[R].x; qy=ptosRef.laserC[R].y;
			p2x=ptosNewRef.laserC[i].x;	p2y=ptosNewRef.laserC[i].y;
			dx=p2x-qx; dy=p2y-qy;
			dist=dx*dx+dy*dy-(dx*qy-dy*qx)*(dx*qy-dy*qx)/(qx*qx+qy*qy+LMET2);

			if (dist<params.Br){
				cp_associations[cnt].nx=ptosNewRef.laserC[i].x;
				cp_associations[cnt].ny=ptosNewRef.laserC[i].y;
				cp_associations[cnt].rx=ptosRef.laserC[R].x;
				cp_associations[cnt].ry=ptosRef.laserC[R].y;
				cp_associations[cnt].dist=dist;
				cnt++;
			}
		}
		else if (L<R)
		{
			// More possible correspondences

			cp_ass_ptX=0;
			cp_ass_ptY=0;
			cp_ass_ptD=100000;

			/* Metric based Closest point rule */
			for (J=L+1;J<=R;J++){

				// Precompute stuff to speed up
				q1x=ptosRef.laserC[J-1].x; q1y=ptosRef.laserC[J-1].y;
				q2x=ptosRef.laserC[J].x; q2y=ptosRef.laserC[J].y;
				p2x=ptosNewRef.laserC[i].x; p2y=ptosNewRef.laserC[i].y;

				dqx=refdqx[J-1]; dqy=refdqy[J-1];
				dqpx=q1x-p2x;  dqpy=q1y-p2y;
				A=1/(p2x*p2x+p2y*p2y+LMET2);
				B=(1-A*p2y*p2y);
				C=(1-A*p2x*p2x);
				D=A*p2x*p2y;

				landaMin=(D*(dqx*dqpy+dqy*dqpx)+B*dqx*dqpx+C*dqy*dqpy)/(B*refdqx2[J-1]+C*refdqy2[J-1]+2*D*refdqxdqy[J-1]);

				if (landaMin<0){ // Out of the segment on one side
					qx=q1x; qy=q1y;}
				else if (landaMin>1){ // Out of the segment on the other side
					qx=q2x; qy=q2y;}
				else if (distref[J-1]<params.MaxDistInter) { // Within the segment and interpotation OK
					qx=(1-landaMin)*q1x+landaMin*q2x;
					qy=(1-landaMin)*q1y+landaMin*q2y;
				}
				else{ // Segment too big do not interpolate
					if (landaMin<0.5){
						qx=q1x; qy=q1y;}
					else{
						qx=q2x; qy=q2y;}
				}

				// Precompute stuff to see if we save the association
				dx=p2x-qx;
				dy=p2y-qy;
				tmp_cp_indD=dx*dx+dy*dy-(dx*qy-dy*qx)*(dx*qy-dy*qx)/(qx*qx+qy*qy+LMET2);

				// Check if the association is the best up to now
				if (tmp_cp_indD < cp_ass_ptD){
					cp_ass_ptX=qx;
					cp_ass_ptY=qy;
					cp_ass_ptD=tmp_cp_indD;
				}
			}

			// Association compatible in distance (Br parameter)
			if (cp_ass_ptD< params.Br){
				cp_associations[cnt].nx=ptosNewRef.laserC[i].x;
				cp_associations[cnt].ny=ptosNewRef.laserC[i].y;
				cp_associations[cnt].rx=cp_ass_ptX;
				cp_associations[cnt].ry=cp_ass_ptY;
				cp_associations[cnt].dist=cp_ass_ptD;

				cnt++;
			}
		}
		else { // This cannot happen but just in case ...
			cp_associations[cnt].nx=ptosNewRef.laserC[i].x;
			cp_associations[cnt].ny=ptosNewRef.laserC[i].y;
			cp_associations[cnt].rx=0;
			cp_associations[cnt].ry=0;
			cp_associations[cnt].dist=params.Br;
			cnt++;
		}
	}  // End for (i=Io;i<ptosNewRef.numPuntos;i++){
// cerr << " B" << cnt << " " << ptosNewRef.numPuntos*params.AsocError << " " << ptosNewRef.numPuntos << " " << endl;
	cntAssociationsT=cnt;
// printf("Associations: %d\n", cnt);
	// Check if the number of associations is ok
	if (cntAssociationsT<ptosNewRef.numPuntos*params.AsocError){
//		printf("Number of associations too low <%d out of %d, %f>\n", cntAssociationsT,ptosNewRef.numPuntos, ptosNewRef.numPuntos*params.AsocError);
		#ifdef INTMATSM_DEB
			printf("Number of associations too low <%d out of %f>\n",
				cntAssociationsT,ptosNewRef.numPuntos*params.AsocError);
		#endif
		return 0;
	}

	return 1;
}


// ************************
// Function that does the minimization step of the MbICP
// ************************

int MBICPStd::MStep(Tsc *solucion){

  Tsc estim_cp;
  int i,cnt,res;
  float error_ratio, error;
  float cosw, sinw, dtx, dty, tmp1, tmp2;
  static TAsoc cp_tmp[MAXLASERPOINTS+1];

	// Filtering of the spurious data
	// Used the trimmed versions that orders the point by distance between associations

     if (params.filter<1){

		// Add Null element in array position 0	(this is because heapsort requirement)
		for (i=0;i<cntAssociationsT;i++){
			cp_tmp[i+1]=cp_associations[i];
		}
		cp_tmp[0].dist=-1;
		// Sort array
		heapsort(cp_tmp, cntAssociationsT);
		// Filter out big distances
		cnt=((int)(cntAssociationsT*100*params.filter))/100;
		// Remove Null element
		for (i=0;i<cnt;i++){
			cp_associationsTemp[i]=cp_tmp[i+1];
		}
	 }
	 else{ // Just build the Temp array to minimize
		cnt=0;
		for (i=0; i<cntAssociationsT;i++){
			if (cp_associations[i].dist<params.Br){
				cp_associationsTemp[cnt]=cp_associations[i];
				cnt++;
			}
		}
	}

	cntAssociationsTemp=cnt;

	#ifdef INTMATSM_DEB
		printf("All assoc: %d  Filtered: %d  Percentage: %f\n",
			cntAssociationsT, cntAssociationsTemp, cntAssociationsTemp*100.0/cntAssociationsT);
	#endif

	// ---
	/* Do de minimization Minimize Metric-based distance */
	/* This function is optimized to speed up */

	res=computeMatrixLMSOpt(cp_associationsTemp,cnt,&estim_cp);
// cerr << estim_cp.x << " " << estim_cp.y << " " << estim_cp.tita << endl;
	if (res==-1)
		return -1;

	#ifdef INTMATSM_DEB
		printf("estim_cp: <%f %f %f>\n",estim_cp.x, estim_cp.y,estim_cp.tita);
		printf("New impl: <%f %f %f>\n",estim_cp.x, estim_cp.y,estim_cp.tita);
	#endif

	cosw=(float)cos(estim_cp.tita); sinw=(float)sin(estim_cp.tita);
	dtx=estim_cp.x; dty=estim_cp.y;


	// ------
	/* Compute the error of the associations */

	error=0;
	for (i = 0; i<cnt;i++){
		tmp1=cp_associationsTemp[i].nx * cosw - cp_associationsTemp[i].ny * sinw + dtx - cp_associationsTemp[i].rx;tmp1*=tmp1;
		tmp2=cp_associationsTemp[i].nx * sinw + cp_associationsTemp[i].ny * cosw + dty - cp_associationsTemp[i].ry;tmp2*=tmp2;
		error = error+ tmp1+tmp2;
	}

	error_ratio = error / error_k1;

	#ifdef INTMATSM_DEB
		printf("<err,errk1,errRatio>=<%f,%f,%f>\n estim=<%f,%f,%f>\n",
			error,error_k1,error_ratio, estim_cp.x,estim_cp.y, estim_cp.tita);
	#endif

	// ----
	/* Check the exit criteria */
	/* Error ratio */
	if (fabs(1.0-error_ratio)<=params.error_th ||
		(fabs(estim_cp.x)<params.errx_out && fabs(estim_cp.y)<params.erry_out
		&& fabs(estim_cp.tita)<params.errt_out) ){
		numConverged++;
	}
	else
		numConverged=0;

	//--
	/* Build the solution */
	composicion_sis(&estim_cp, &motion2, solucion);
	motion2=*solucion;
	error_k1=error;

	/* Number of iterations doing convergence (smooth criterion of convergence) */
	if (numConverged>params.IterSmoothConv)
		return 1;
	else
		return 0;
}


// ************************
// Function to do the least-squares but optimized for the metric
// ************************

int MBICPStd::computeMatrixLMSOpt(TAsoc *cp_ass, int cnt, Tsc *estimacion) {

	int i;
	float fac = 1.0;
	float Ltmp;
	float LMETRICA2;
	float X1[MAXLASERPOINTS], Y1[MAXLASERPOINTS];
	float X2[MAXLASERPOINTS],Y2[MAXLASERPOINTS];
	float X2Y2[MAXLASERPOINTS],X1X2[MAXLASERPOINTS];
	float X1Y2[MAXLASERPOINTS], Y1X2[MAXLASERPOINTS];
	float Y1Y2[MAXLASERPOINTS];
	float K[MAXLASERPOINTS], DS[MAXLASERPOINTS];
	float DsD[MAXLASERPOINTS], X2DsD[MAXLASERPOINTS], Y2DsD[MAXLASERPOINTS];
	float Bs[MAXLASERPOINTS], BsD[MAXLASERPOINTS];
	float A1, A2, A3, B1, B2, B3, C1, C2, C3, D1, D2, D3;
	MATRIX matA,invMatA;
	VECTOR vecB,vecSol;

	A1=0;A2=0;A3=0;B1=0;B2=0;B3=0;
	C1=0;C2=0;C3=0;D1=0;D2=0;D3=0;


	LMETRICA2=params.LMET*params.LMET;

	for (i=0; i<cnt; i++){
		X1[i]=cp_ass[i].nx*cp_ass[i].nx;
		Y1[i]=cp_ass[i].ny*cp_ass[i].ny;
		X2[i]=cp_ass[i].rx*cp_ass[i].rx;
		Y2[i]=cp_ass[i].ry*cp_ass[i].ry;
		X2Y2[i]=cp_ass[i].rx*cp_ass[i].ry;

		X1X2[i]=cp_ass[i].nx*cp_ass[i].rx;
		X1Y2[i]=cp_ass[i].nx*cp_ass[i].ry;
		Y1X2[i]=cp_ass[i].ny*cp_ass[i].rx;
		Y1Y2[i]=cp_ass[i].ny*cp_ass[i].ry;

		if (useAdvanced_) {
			fac = sqrt(X1[i] + Y1[i]);
			if (fac < 1.0) fac = 1.0;
			if (fac > 4.0) fac = 4.0;
//			Ltmp = params.LMET * (6.0 - fac) / 6.0;
//			Ltmp = 2.0 * params.LMET * (fac) / 4.0;
//			LMETRICA2 = Ltmp*Ltmp;

//			fac = 1.0;
		}

		K[i]=X2[i]+Y2[i] + LMETRICA2;
		DS[i]=Y1Y2[i] + X1X2[i];
		DsD[i]=DS[i]/K[i];
		X2DsD[i]=cp_ass[i].rx*DsD[i];
		Y2DsD[i]=cp_ass[i].ry*DsD[i];

		Bs[i]=X1Y2[i]-Y1X2[i];
		BsD[i]=Bs[i]/K[i];

		A1=A1 + fac * (1-Y2[i]/K[i]);
		B1=B1 + fac * X2Y2[i]/K[i];
		C1=C1 + fac * (-cp_ass[i].ny + Y2DsD[i]);
		D1=D1 + fac * (cp_ass[i].nx - cp_ass[i].rx -cp_ass[i].ry*BsD[i]);

		A2=B1;
		B2=B2 + fac * (1-X2[i]/K[i]);
		C2=C2 + fac * (cp_ass[i].nx-X2DsD[i]);
		D2=D2 + fac * (cp_ass[i].ny -cp_ass[i].ry +cp_ass[i].rx*BsD[i]);

		A3=C1;
		B3=C2;
		C3=C3 + fac * (X1[i] + Y1[i] - DS[i]*DS[i]/K[i]);
		D3=D3 + fac * (Bs[i]*(-1+DsD[i]));
	}


	initialize_matrix(&matA,3,3);
	MDATA(matA,0,0)=A1;	MDATA(matA,0,1)=B1;	MDATA(matA,0,2)=C1;
	MDATA(matA,1,0)=A2;	MDATA(matA,1,1)=B2;	MDATA(matA,1,2)=C2;
	MDATA(matA,2,0)=A3;	MDATA(matA,2,1)=B3;	MDATA(matA,2,2)=C3;

	if (inverse_matrix (&matA, &invMatA)==-1)
		return -1;

#ifdef INTMATSM_DEB
	print_matrix("inverted matrix", &invMatA);
#endif

	initialize_vector(&vecB,3);
	VDATA(vecB,0)=D1; VDATA(vecB,1)=D2; VDATA(vecB,2)=D3;
	multiply_matrix_vector (&invMatA, &vecB, &vecSol);

	estimacion->x=-VDATA(vecSol,0);
	estimacion->y=-VDATA(vecSol,1);
	estimacion->tita=-VDATA(vecSol,2);

	return 1;
}


void MBICPStd::preProcessingCreate(Tpfp *laserK, Tpfp *laserK1,
					  Tsc *initialMotion)
{
	int i;
	double tx, ty;

	// ------------------------------------------------//
	// Compute xy coordinates of the points in laserK1
	ptosNew.numPuntos=0;
	for (i=0; i<MAXLASERPOINTS; i++) {
		if (laserK1[i].r < MAXLASERRANGE){
			tx = (float)(laserK1[i].r * cos(laserK1[i].t));
			ty = (float)(laserK1[i].r * sin(laserK1[i].t));

			ptosNew.laserP[ptosNew.numPuntos].r=laserK1[i].r;
			ptosNew.laserP[ptosNew.numPuntos].t=laserK1[i].t;
			ptosNew.laserC[ptosNew.numPuntos].x=tx;
			ptosNew.laserC[ptosNew.numPuntos].y=ty;
		    ptosNew.numPuntos++;
		}
	}

	// Compute xy coordinates of the points in laserK
	ptosRef.numPuntos=0;
	for (i=0; i<MAXLASERPOINTS; i++) {
 		if (laserK[i].r <MAXLASERRANGE){
			tx = (float)(laserK[i].r * cos(laserK1[i].t));
			ty = (float)(laserK[i].r * sin(laserK1[i].t));
			ptosRef.laserP[ptosRef.numPuntos].r=laserK[i].r;
			ptosRef.laserP[ptosRef.numPuntos].t=laserK[i].t;
			ptosRef.laserC[ptosRef.numPuntos].x=tx;
			ptosRef.laserC[ptosRef.numPuntos].y=ty;
		    ptosRef.numPuntos++;
		}
	}
	motion2=*initialMotion;
}

void MBICPStd::preProcessingCalc()
{
	int i,j, k1, k2;
	double tx, ty;

	k1 = 0; k2 = 0;

	// Choose one point out of params.laserStep points
//	j=0;
//	for (i=0; i<ptosNew.numPuntos; i+=params.laserStep) {
//		ptosNew.laserC[j]=ptosNew.laserC[i];
//		ptosNew.laserP[j]=ptosNew.laserP[i];
//		car2pol(&ptosNew.laserC[j],&ptosNew.laserP[j]);
//		j++;
//	}
//	ptosNew.numPuntos=j;

	// Choose one point out of params.laserStep points
	// XXX: MBICP: isn't this wrong?  Shouldn't we compare a reduced set to the entire set?

	j=0;
	for (i=0; i<ptosRef.numPuntos; i+=params.laserStep) {
		ptosRef.laserC[j]=ptosRef.laserC[i];
		ptosRef.laserP[j]=ptosRef.laserP[i];
		car2pol(&ptosRef.laserC[j],&ptosRef.laserP[j]);
		j++;
	}
	ptosRef.numPuntos=j;

	// Preprocess reference points
	for (i=0;i<ptosRef.numPuntos-1;i++) {
//		car2pol(&ptosRef.laserC[i],&ptosRef.laserP[i]);
		refdqx[i]=ptosRef.laserC[i].x - ptosRef.laserC[i+1].x;
		refdqy[i]=ptosRef.laserC[i].y - ptosRef.laserC[i+1].y;
		refdqx2[i]=refdqx[i]*refdqx[i];
		refdqy2[i]=refdqy[i]*refdqy[i];
		distref[i]=refdqx2[i] + refdqy2[i];
		refdqxdqy[i]=refdqx[i]*refdqy[i];
	}
//	car2pol(&ptosRef.laserC[ptosRef.numPuntos-1],&ptosRef.laserP[ptosRef.numPuntos-1]);

	error_k1=BIG_INITIAL_ERROR;
	numConverged=0;
}


void MBICPStd::preProcessingLib(Tpfp *laserK, Tpfp *laserK1,
					  Tsc *initialMotion)
{
	preProcessingCreate(laserK, laserK1, initialMotion);
	preProcessingCalc();
}


void MBICPStd::transfor_directa_p(float x, float y,
			Tsc *sistema, Tpf *sol){

  /* Esta funcion transforma el punto x,y en el sistema de coordenadas mas global*/
  /* Es decir las coordenadas x y son vistas desde el sistema de coordenadas sistema*/
  /* Y se las quiere transformar en el sistema de ref desde el que se sistema*/
  /* Es la transformacion directa */

  float SinT,CosT;

  SinT=(float)sin(sistema->tita);
  CosT=(float)cos(sistema->tita);

  sol->x=x*CosT-y*SinT+sistema->x;
  sol->y=x*SinT+y*CosT+sistema->y;

  //fprintf(stderr,"input:<%f,%f> sis:<%f %f %f> sol:<%f %f>\n",x,y,sistema->x, sistema->y, sistema->tita,sol->x, sol->y);

}

void MBICPStd::transfor_directa_pt0(float x, float y,
			Tsc *sistema, Tpf *sol){

  /* Esta funcion transforma el punto x,y en el sistema de coordenadas mas global*/
  /* Es decir las coordenadas x y son vistas desde el sistema de coordenadas sistema*/
  /* Y se las quiere transformar en el sistema de ref desde el que se sistema*/
  /* Es la transformacion directa */

  sol->x=x+sistema->x;
  sol->y=y+sistema->y;

}


void MBICPStd::transfor_inversa_p(float x,float y,
			Tsc *sistema, Tpf *sol){

  /* Esta funcion transforma el punto x,y en el sistema de coordenadas que entra*/
  /* Las coordenadas x y se ven desde el sistema de coordenadas desde el que se tienen las */
  /* las coordenadas de sistema */
  /* Es la transformacion directa */

  float a13, a23;
  float SinT,CosT;

  SinT=(float)sin(sistema->tita);
  CosT=(float)cos(sistema->tita);


  a13=-sistema->y*SinT-sistema->x*CosT;
  a23=-sistema->y*CosT+sistema->x*SinT;

  sol->x=x*CosT+y*SinT+a13;
  sol->y=-x*SinT+y*CosT+a23;
}

float MBICPStd::NormalizarPI(float ang){

  return (float)(ang+(2*M_PI)*floor((M_PI-ang)/(2*M_PI)));
}

void MBICPStd::inversion_sis(Tsc *sisIn, Tsc *sisOut){

  float c,s;

  c=(float)cos(sisIn->tita);
  s=(float)sin(sisIn->tita);
  sisOut->x =-c*sisIn->x-s*sisIn->y;
  sisOut->y = s*sisIn->x-c*sisIn->y;
  sisOut->tita = NormalizarPI(-sisIn->tita);
}

void MBICPStd::composicion_sis(Tsc *sis1,Tsc *sis2,Tsc *sisOut){

  Tpf sol;

  transfor_directa_p(sis2->x, sis2->y,
		     sis1, &sol);
  sisOut->x=sol.x;
  sisOut->y=sol.y;
  sisOut->tita = NormalizarPI(sis1->tita+sis2->tita);

}

void MBICPStd::car2pol(Tpf *in, Tpfp *out){

  out->r=(float)sqrt(in->x*in->x+in->y*in->y);
  out->t=(float)atan2(in->y,in->x);
//  out->r=(float)sqrt(in->x*in->x+in->y*in->y);
//  out->t=(float)atan2(in->x,in->y);
}

void MBICPStd::pol2car(Tpfp *in, Tpf *out){

  out->x=in->r*(float)cos(in->t);
  out->y=in->r*(float)sin(in->t);
//  out->y=in->r*(float)cos(in->t);
//  out->x=in->r*(float)sin(in->t);
}




int MBICPStd::corte_segmentos(float x1,float y1,float x2,float y2,
		    float x3,float y3,float x4,float y4,
		    Tpf *sol){
/* corte de segmentos */
/* TE DEVUELVE EL PUNTO DE CORTE EN EL SISTEMA QUE ESTEN LOS SEGMENTOS */

  float a1,a2,b1,b2,c1,c2,xm,ym,denominador,max1_x,max1_y,min1_x,min1_y;
  float xerr,yerr;
  int si1;
  float error_redondeo;

  error_redondeo=(float)0.00001F;

  /* primera recta */
  a1=y2-y1;
  b1=x1-x2;
  c1=y1*(-b1)-x1*a1;

  /* segunda recta */
  a2=y4-y3;
  b2=x3-x4;
  c2=y3*(-b2)-x3*a2;


  denominador=a1*b2-a2*b1;
  if (denominador==0)
    return 0;
  else{
    xm=(b1*c2-b2*c1)/denominador;
    ym=(c1*a2-c2*a1)/denominador;

    xerr=xm+error_redondeo;
    yerr=ym+error_redondeo;

    /* Comprobamos que cae entre los segmantos */
    if (x1>x2){
      max1_x=x1; min1_x=x2;
    }
    else{
      max1_x=x2; min1_x=x1;
    }
    if (y1>y2){
      max1_y=y1; min1_y=y2;
    }
    else{
      max1_y=y2; min1_y=y1;
    }
    si1=0;
    if (max1_x+error_redondeo>=xm && xerr>=min1_x &&  max1_y+error_redondeo>=ym && yerr>=min1_y)
      si1=1;


    if (si1){

      if (x3>x4){
	max1_x=x3; min1_x=x4;
      }
      else{
	max1_x=x4; min1_x=x3;
      }
      if (y3>y4){
	max1_y=y3; min1_y=y4;
      }
      else{
	max1_y=y4; min1_y=y3;
      }

      if (max1_x+error_redondeo>=xm && xerr>=min1_x &&  max1_y+error_redondeo>=ym && yerr>=min1_y){
	sol->x=xm;
	sol->y=ym;
	return 1;
      }
    }
    return 0;
  }
}

void MBICPStd::swapItem(TAsoc *a, TAsoc *b){
	TAsoc c;

	c=*a;
	*a=*b;
	*b=c;
}

void MBICPStd::perc_down(TAsoc a[], int i, int n) {
  int child; TAsoc tmp;
  for (tmp=a[i]; i*2 <= n; i=child) {
    child = i*2;
    if ((child != n) && (a[child+1].dist > a[child].dist))
      child++;
    if (tmp.dist < a[child].dist)
      a[i] = a[child];
    else
      break;
  }
  a[i] = tmp;
}

void MBICPStd::heapsort(TAsoc a[], int n) {
  int i, j;
  j = n;
  for (i=n/2; i>0; i--)  /* BuildHeap */
    perc_down(a,i,j);
  i = 1;
  for (j=n; j>=2; j--) {
    swapItem(&a[i],&a[j]);   /* DeleteMax */
    perc_down(a,i,j-1);
  }
}

/*****************************************************************************/
 MATRIX MBICPStd::create_matrix (int rows, int cols)

/*****************************************************************************
 Creates a MATRIX of dimensions (rows, cols) and initializaes it to zeros.
******************************************************************************/
{
  MATRIX m;

  MROWS (m) = rows;
  MCOLS (m) = cols;

  {
    int i, j;

    for (i = 0; i < MROWS (m); i++)
        for (j = 0; j < MCOLS (m); j++)
            MDATA (m, i, j) = 0;
  }

  return m;
}

/*****************************************************************************/
 void MBICPStd::initialize_matrix (MATRIX *m, int rows, int cols)

/*****************************************************************************
 Initializes a MATRIX to dimensions (rows, cols) and content zeros.
******************************************************************************/
{
  MROWS (*m) = rows;
  MCOLS (*m) = cols;

  {
    int i, j;

    for (i = 0; i < MROWS (*m); i++)
        for (j = 0; j < MCOLS (*m); j++)
            MDATA (*m, i, j) = 0;
  }

}


/*****************************************************************************/
 void MBICPStd::print_matrix (const char *message, MATRIX const *m)

/*****************************************************************************
 Print to stdout the contents of MATRIX m.
******************************************************************************/
{
  int i, j;

  printf ("%s\n",message);
  printf("%d %d \n",MROWS (*m),MCOLS (*m));
  if ((MROWS (*m) <= MAX_ROWS) && (MCOLS (*m) <= MAX_COLS))
    for (i = 0; i < MROWS (*m); i++)
    {
        for (j = 0; j < MCOLS (*m); j++)
            printf ("%10.5f ", MDATA (*m, i, j));
        printf ("\n");
    }
  else printf ("Dimension incorrecta!");
  printf ("\n");
}

/*****************************************************************************/
 VECTOR MBICPStd::create_vector (int elements)

/*****************************************************************************
 Initializes a VECTOR to dimension (elements) and its contents to zeros.
******************************************************************************/
{
  VECTOR v;

  VELEMENTS (v) = elements;

  {
    int i;

    for (i = 0; i < VELEMENTS (v); i++)
        VDATA (v, i) = 0;
  }

  return v;
}

/*****************************************************************************/
 void MBICPStd::initialize_vector (VECTOR *v, int elements)

/*****************************************************************************
 Initializes a VECTOR to dimension (elements) and its contents to zeros.
******************************************************************************/
{
  VELEMENTS (*v) = elements;

  {
    int i;

    for (i = 0; i < VELEMENTS (*v); i++)
        VDATA (*v, i) = 0;
  }
}

/*****************************************************************************/
 void MBICPStd::print_vector (const char *message, VECTOR const *v)

/*****************************************************************************
 Print to stdout the contents of VECTOR m.
******************************************************************************/
{
  int i;

  printf ("%s\n",message);
  if (VELEMENTS (*v) <= MAX_ROWS)
    for (i = 0; i < VELEMENTS (*v); i++)
    {
        printf ("%f ", VDATA (*v, i));
        printf ("\n");
    }
  else printf ("Dimension incorrecta!");
  printf ("\n");
}

/*****************************************************************************/
 float MBICPStd::cross_product (MATRIX const *m, int f1, int c1, int f2, int c2)

/*****************************************************************************
******************************************************************************/
{
  return MDATA (*m, f1, c1) * MDATA (*m, f2, c2) - MDATA (*m, f1, c2) * MDATA (*m, f2, c1);
}

/*****************************************************************************/
int MBICPStd::determinant (MATRIX const *m, float *result)
/*****************************************************************************
******************************************************************************/
{
  if (!M_SQUARE (*m))
  {
     printf ("ERROR (determinant): MATRIX must be square!\n");
     print_matrix ("MATRIX:", m);
	 return -1;
  }
  else
  {

    if (MROWS (*m) == 1)
       *result = MDATA (*m, 0, 0);
    else if (MROWS (*m) == 2)
       *result = cross_product (m, 0, 0, 1, 1);
    else
       *result = MDATA (*m, 0, 0) * cross_product (m, 1, 1, 2, 2)
              - MDATA (*m, 0, 1) * cross_product (m, 1, 0, 2, 2)
              + MDATA (*m, 0, 2) * cross_product (m, 1, 0, 2, 1);


    return 1;
  }
}

/*****************************************************************************/
 int MBICPStd::inverse_matrix (MATRIX const *m, MATRIX *n)

/*****************************************************************************
******************************************************************************/
{
  if (!M_SQUARE (*m))
  {
     printf ("ERROR (inverse_matrix): MATRIX must be square!\n");
     print_matrix ("MATRIX:", m);
	 n->cols=0; n->rows=0;
     return -1;
  }
  else
  {
    float det;
	int res;

    res = determinant (m,&det);

    if (res == -1)
    {
       printf ("ERROR (inverse_matrix): singular MATRIX!\n");
       print_matrix ("MATRIX:", m);
       return -1;
    }
    else
    {
      initialize_matrix (n, MROWS (*m), MCOLS (*m));
      if (MROWS (*m) == 1)
      {
        MDATA (*n, 0, 0) = 1 / det ;
      }
      else if (MROWS (*m) == 2)
      {
        MDATA (*n, 0, 0) = MDATA (*m, 1, 1) / det ;
        MDATA (*n, 0, 1) = -MDATA (*m, 0, 1) / det ;
        MDATA (*n, 1, 0) = -MDATA (*m, 1, 0) / det ;
        MDATA (*n, 1, 1) = MDATA (*m, 0, 0) / det ;
      }
      else
      {
        MDATA (*n, 0, 0) = cross_product (m, 1, 1, 2, 2) / det ;
        MDATA (*n, 0, 1) = -cross_product (m, 0, 1, 2, 2) / det ;
        MDATA (*n, 0, 2) = cross_product (m, 0, 1, 1, 2) / det ;
        MDATA (*n, 1, 0) = -cross_product (m, 1, 0, 2, 2) / det ;
        MDATA (*n, 1, 1) = cross_product (m, 0, 0, 2, 2) / det ;
        MDATA (*n, 1, 2) = -cross_product (m, 0, 0, 1, 2) / det ;
        MDATA (*n, 2, 0) = cross_product (m, 1, 0, 2, 1) / det ;
        MDATA (*n, 2, 1) = -cross_product (m, 0, 0, 2, 1) / det ;
        MDATA (*n, 2, 2) = cross_product (m, 0, 0, 1, 1) / det ;
      }
	}
  }
  return 1;
}

/*****************************************************************************/
 int MBICPStd::multiply_matrix_vector (MATRIX const *m, VECTOR const *v, VECTOR *r)

/*****************************************************************************
 Returns the VECTOR-MATRIX product of m and v in r.
******************************************************************************/
{
  if (! (MV_COMPAT_DIM (*m, *v)))
  {
     printf ("ERROR (multiply_matrix_vector): MATRIX  and VECTOR dimensions incompatible!\n");
     print_matrix ("MATRIX:", m);
     print_vector ("VECTOR:", v);
     return -1; /*added 1996-07*/
  }
  else
  {
    int i, j;
    float datum;

    VELEMENTS (*r) = MROWS (*m);

    for (i = 0; i < MROWS (*m); i++)
    {
        datum = 0;
        for (j = 0; j < VELEMENTS (*v); j++)
            datum = datum + MDATA (*m, i, j) * VDATA (*v, j);
        VDATA (*r, i) = datum;
    }
  }
  return 1;
}


} // namespace mbicp
