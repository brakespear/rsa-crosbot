#ifndef MBPOSTRACKSTD_H_
#define MBPOSTRACKSTD_H_

// #include "MBICP/TData.h"

using namespace std;

namespace mbicp
{

// AHM: this must be greater than or equal to the max possible number of points.
// it must be equal if using preProcessingCreate
//#define MAXLASERPOINTS 682
#define MAXLASERPOINTS 1081

#define MAX_ROWS 	(7)
#define MAX_COLS 	(7)

typedef struct {
  int   rows;
  int   cols;
  float data[MAX_ROWS][MAX_COLS];
} MATRIX;

typedef struct {
  int   elements;
  float data[MAX_ROWS];
} VECTOR;

#define DOF (3)

typedef struct {
  int mat[DOF];
  int range;
} BMAT;


// #define RADIO 0.4F  /* Radio del robot */

typedef struct {
  float x;
  float y;
}Tpf;


typedef struct {
  float r;
  float t;
}Tpfp;

typedef struct {
  int x;
  int y;
}Tpi;

typedef struct {
  float x;
  float y;
  float tita;
}Tsc;

typedef struct {
  int numPuntos;
  Tpf laserC[MAXLASERPOINTS];  // Cartesian coordinates
  Tpfp laserP[MAXLASERPOINTS]; // Polar coordinates
}Tscan;




// Associations information
typedef struct{
  float rx,ry,nx,ny,dist;				// Point (nx,ny), static corr (rx,ry), dist
  int numDyn;							// Number of dynamic associations
  float unknown;						// Unknown weight
  int index;							// Index within the original scan
  int L,R;
}TAsoc;


// ************************
// Scan inner matching parameters
typedef struct{
	/* --------------------- */
	/* --- Thresold parameters */
	/* Bw: maximum angle diference between points of different scans */
	/* Points with greater Bw cannot be correspondent (eliminate spurius asoc.) */
	/* This is a speed up parameter */
	float Bw;

	/* Br: maximum distance difference between points of different scans */
	/* Points with greater Br cannot be correspondent (eliminate spurius asoc.) */
	float Br;

	/* --------------------- */
	/* --- Inner parameters */

	/* L: value of the metric */
	/* When L tends to infinity you are using the standart ICP */
    /* When L tends to 0 you use the metric (more importance to rotation */
	float LMET;

	/* laserStep: selects points of each scan with an step laserStep  */
	/* When laserStep=1 uses all the points of the scans */
	/* When laserStep=2 uses one each two ... */
	/* This is an speed up parameter */
	int laserStep;

	/* ProjectionFilter: */
	/* Eliminate the points that cannot be seen given the two scans (see Lu&Millios 97) */
	/* It works well for angles < 45 \circ*/
	/* 1 : activates the filter */
	/* 0 : desactivates the filter */
	int ProjectionFilter;

	/* MaxDistInter: maximum distance to interpolate between points in the ref scan */
	/* Consecutive points with less Euclidean distance than MaxDistInter are considered to be a segment */
	float MaxDistInter;

	/* filtrado: in [0,1] sets the % of asociations NOT considered spurious */
	float filter;

	/* AsocError: in [0,1] */
	/* One way to check if the algorithm diverges if to supervise if the number of associatios goes below a thresold */
	/* When the number of associations is below AsocError, the main function will return error in associations step */
	float AsocError;

	/* --------------------- */
	/* --- Exit parameters */
	/* MaxIter: sets the maximum number of iterations for the algorithm to exit */
	/* More iterations more chance you give the algorithm to be more accurate   */
	int MaxIter;

	/* error_th: in [0,1] sets the maximum error ratio between iterations to exit */
	/* In each iteration, the error is the residual of the minimization */
	/* When error_th tends to 1 more precise is the solution of the scan matching */
	float error_th;

	/* errx_out,erry_out, errt_out: minimum error of the asociations to exit */
	/* In each iteration, the error is the residual of the minimization in each component */
	/* The condition is (lower than errx_out && lower than erry_out && lower than errt_out */
	/* When error_XXX tend to 0 more precise is the solution of the scan matching */
	float errx_out,erry_out, errt_out;

	/* IterSmoothConv: number of consecutive iterations that satisfity the error criteria */
	/* (error_th) OR (errorx_out && errory_out && errt_out) */
	/* With this parameter >1 avoids random solutions */
	int IterSmoothConv;

     float xmin;
     float xmax;
     float ymin;
     float ymax;


}TSMparams;

class MBICPStd {
public:
	void Init_MbICP_ScanMatching(
				     float max_laser_range,
				     float Bw,
				     float Br,
				     float L,
				     int   laserStep,
				     float MaxDistInter,
				     float filter,
				     int   ProjectionFilter,
				     float AsocError,
				     int   MaxIter,
				     float errorRatio,
				     float errx_out,
				     float erry_out,
				     float errt_out,
				     int IterSmoothConv,
				     float xmin,
				     float xmax,
				     float ymin,
				     float ymax
	);

	int MbICPmatcher(Tpfp *laserK, Tpfp *laserK1,
					 Tsc *sensorMotion, Tsc *solution);

	int MbICPMatchScans(double &x, double &y, double &th);

	// Original points to be aligned
	Tscan ptosRef;
	Tscan ptosNew;
	// Current motion estimation
	Tsc motion2;

	bool useAdvanced_;
protected:

	// ************************
	// Static structure to initialize the SM parameters
	TSMparams params;

	// At each step::

	// Those points removed by the projection filter (see Lu&Millios -- IDC)
	Tscan ptosNoView; // Only with ProjectionFilter=1;

	// Structure of the associations before filtering
	TAsoc cp_associations[MAXLASERPOINTS];
	int cntAssociationsT;

	// Filtered Associations
	TAsoc cp_associationsTemp[MAXLASERPOINTS];
	int cntAssociationsTemp;

	float MAXLASERRANGE;
	float MINLASERRANGE;
	float xmin_;
	float xmax_;
	float ymin_;
	float ymax_;

	// ************************
	// Some precomputations for each scan to speed up
	float refdqx[MAXLASERPOINTS];
	float refdqx2[MAXLASERPOINTS];
	float refdqy[MAXLASERPOINTS];
	float refdqy2[MAXLASERPOINTS];
	float distref[MAXLASERPOINTS];
	float refdqxdqy[MAXLASERPOINTS];


	// value of errors
	float error_k1;
	int numConverged;

	// Function for compatibility with the scans
	void preProcessingLib(Tpfp *laserK, Tpfp *laserK1,
						  Tsc *initialMotion);
	// separated functionality to avoid multiple conversions, this way the structures can be initialized elsewhere
	void preProcessingCreate(Tpfp *laserK, Tpfp *laserK1, Tsc *initialMotion);
	void preProcessingCalc();

	// Function that does the association step of the MbICP
	int EStep();

	// Function that does the minimization step of the MbICP
	int MStep(Tsc *solucion);

	// Function to do the least-squares but optimized for the metric
	int computeMatrixLMSOpt(TAsoc *cp_ass, int cnt, Tsc *estimacion);

	/* --------------------------------------------------------------------------------------- */
	/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
	/* --------------------------------------------------------------------------------------- */

	/* --------------------------------------------------------------------------------------- */
	/* transfor_directa_p                                                                      */
	/*  .... Hace la transformacion directa de un punto a un sistema a otro                    */
	/*  .... In: (x,y) las coordenadas del punto, sistema es el sistema de referencia          */
	/*  .... Out: en sol se devuelve las coordenadas del punto en el nuevo sistema             */

	void transfor_directa_p ( float x, float y, Tsc *sistema, Tpf *sol );

	/* --------------------------------------------------------------------------------------- */
	/* transfor_directa_p                                                                      */
	/*  .... Hace la transformacion directa de un punto a un sistema a otro                    */
	/*  .... La diferencia es que aqui el punto de entrada es el (0,0) (optimiza la anterior)  */
	/*  .... In: (x,y) las coordenadas del punto, sistema es el sistema de referencia          */
	/*  .... Out: en sol se devuelve las coordenadas del punto en el nuevo sistema             */

	void transfor_directa_pt0(float x, float y,
				  Tsc *sistema, Tpf *sol);

	/* --------------------------------------------------------------------------------------- */
	/* transfor_inversa_p                                                                      */
	/*  .... Hace la transformacion inversa de un punto a un sistema a otro                    */
	/*  .... In: (x,y) las coordenadas del punto, sistema es el sistema de referencia          */
	/*  .... Out: en sol se devuelve las coordenadas del punto en el nuevo sistema             */

	void transfor_inversa_p ( float x, float y, Tsc *sistema, Tpf *sol );

	/* --------------------------------------------------------------------------------------- */
	/* TRANSFORMACIONES DE COMPOSICION E INVERSION DE SISTEMAS DE REFERENCIA                   */
	/* --------------------------------------------------------------------------------------- */

	/* --------------------------------------------------------------------------------------- */
	/* composicion_sis                                                                         */
	/*  .... Realiza la composicion de sistemas de referencia en otro sistema                  */
	/*  .... In: compone sis1 y sis2                                                           */
	/*  .... Out: la salida sisOut es el resultado de la composicion de los sistemas           */
	/*  .... Nota: resulta muy importante el orden de las entradas en la composicion           */

	void composicion_sis(Tsc *sis1,Tsc *sis2,Tsc *sisOut);

	/* --------------------------------------------------------------------------------------- */
	/* inversion_sis                                                                           */
	/*  .... Realiza la inversion de un sistema de referencia                                  */
	/*  .... In: sisIn es el sistema a invertir                                                */
	/*  .... Out: sisOut es el sistema invertido                                               */

	void inversion_sis(Tsc *sisIn, Tsc *sisOut);

	/* --------------------------------------------------------------------------------------- */
	/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
	/* --------------------------------------------------------------------------------------- */

	/* --------------------------------------------------------------------------------------- */
	/* car2pol                                                                                 */
	/*  .... Transforma un punto de coordenadas cartesianas a polares                          */
	/*  .... In: el punto en coordenadas cartesianas a transformar                             */
	/*  .... Out: el punto salida en coordenadas polares                                       */

	void car2pol(Tpf *in, Tpfp *out);

	/* --------------------------------------------------------------------------------------- */
	/* pol2car                                                                                 */
	/*  .... Transforma un punto de coordenadas polares a cartesianas                          */
	/*  .... In: el punto entrada en coordenadas polares a transformar                         */
	/*  .... Out: el punto en coordenadas cartesianas transformado                             */

	void pol2car(Tpfp *in, Tpf *out);

	/* --------------------------------------------------------------------------------------- */
	/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
	/* --------------------------------------------------------------------------------------- */

	/* --------------------------------------------------------------------------------------- */
	/* corte_segmentos                                                                         */
	/*  .... Calcula el punto de corte entre dos segmentos                                     */
	/*  .... In: las coordenadas de los puntos extremos (x1,y1)-(x2,y2) y (x3,y3)-(x4,y4)      */
	/*  .... Out: sol son las coordenadas del punto de corte. return --> 1 si hay corte. -->0 no */

	int corte_segmentos ( float x1, float y1, float x2, float y2,
			      float x3, float y3, float x4, float y4,
			      Tpf *sol );


	/* Normaliza el angulo entre [-PI, PI] */
	float NormalizarPI(float ang);

	void heapsort(TAsoc a[], int n);
	void swapItem(TAsoc *a, TAsoc *b);
	void perc_down(TAsoc a[], int i, int n);

	MATRIX create_matrix (int rows, int cols);
	void initialize_matrix (MATRIX *m, int rows, int cols);
	void diagonal_matrix (MATRIX *m, int dim, float el1, float el2, float el3);
	void print_matrix (const char *message, MATRIX const *m);
	VECTOR create_vector (int elements);
	void initialize_vector (VECTOR *v, int elements);
	void print_vector (const char *message, VECTOR const *v);
	float cross_product (MATRIX const *m, int f1, int c1, int f2, int c2);
	int determinant (MATRIX const *m, float *result);
	int inverse_matrix (MATRIX const *m, MATRIX *n);
	int multiply_matrix_vector (MATRIX const *m, VECTOR const *v, VECTOR *r);

};

} // namespace mbicp



#endif /*MBPOSTRACKSTD_H_*/
