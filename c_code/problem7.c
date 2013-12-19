/* 
 * CI Class WSS2013-14
 * 
 * Fuzzy logic control homework
 * 
 * Mobile robot trajectory tracking
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

/* handy macros */
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define Kde	 {0.25,0.25,0.50,0.5}
#define Kte	 {0.12,0.25,0.12,0.25}

/* helper types */

/* specify the type of aggregator */
enum 
{
	min = 1,
	prod
};

typedef struct 
{
	double crisp_val;	/* crisp value */
	int membership_type;	/* type of membership function */
	int membership_num;	/* number of membership functions */
}fuzzy_in;

typedef struct
{
	double x_pos;
	double y_pos;
	double theta;
}trajectory;

/* enum specifying the type of membership function */
enum
{
	test = 0,
	triangle = 1,	/* f(x,a,b,c) = max( min( x-a/b-a  , c-x/c-b ), 0) */
	trapezoid,	/* f(x,a,b,c,d) = max( min( x-a/b-a, min( 1, d-x/d-c )), 0) */
	gauss		/* f(x, sigma, mean) = exp( -(x-mean)^2 / 2*sigma^2) */
};

/* compute translation error */
double compute_de(trajectory ref, trajectory cur)
{
	return (double)sqrt((pow(ref.x_pos - cur.x_pos, 2) + pow(ref.y_pos - cur.y_pos, 2)));
}

/* compute the heading error */
double compute_thetae(trajectory ref, trajectory cur)
{
	return (double)(ref.theta - cur.theta);
}

/* aggregate the antecedents and compute the weight (membership degree) */
double compute_aggregation(int type, double memb_val_in1, double memb_val_in2)
{
	return (type==min)?min(memb_val_in1, memb_val_in2):(memb_val_in1*memb_val_in2);
}

/* compute the membership vf for a crisp input */
double compute_membership(int type, fuzzy_in* in)
{
	double out = 0.0f;
	/* cherck the type of the membership */
	switch(type){
		case 0:
			switch(in->membership_type){
				case 11:
					out = min(max(0, -(in->crisp_val-80)/160), 1);
				break;
				case 12:
					out = 1 - min(max(0, -(in->crisp_val-80)/160), 1);
				break;
				case 21:
					out = (-abs(in->crisp_val)+180)/180;
				break;
				case 22:
					out = 1 -  (-abs(in->crisp_val)+180)/180;
				break;
			}
		break;
		case 1:
	
		break;

		case 2:


		break;


		case 3:

		break;
	}
	return out; 
}

/* entry point */
int main(int argc, char** argv)
{
	
	
	


	return (EXIT_SUCCESS);
}
