/* 
 * CI Class WSS2013-14
 * 
 * Fuzzy logic control homework
 * 
 * Mobile robot trajectory tracking - sample implementation of 2nd assignment
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

/* handy macros */
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define MAX_TRAJ_SIZE	1000
#define NUM_RULES	4

/* helper types */

/* membership wedges */
enum
{
	smalld = 11,
	larged = 12,
	smallt = 21,
	larget = 22
};

typedef struct 
{
	double crisp_val;	/* crisp value */
	int membership_func;    /* the specific function Small, Large ... */
}fuzzy_in;

typedef struct
{
	double x_pos;
	double y_pos;
	double theta;
}trajectory;

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
double compute_aggregation(double memb_val_in1, double memb_val_in2)
{
	return min(memb_val_in1, memb_val_in2);
}

/* compute the membership vf for a crisp input */
double compute_membership(fuzzy_in* in)
{
	double out = 0.0f;
		switch(in->membership_func){
			case smalld:
				out = (double)(min(max(0, -(in->crisp_val-80.0)/160.0), 1.0));
			break;
			case larged:
				out = (double)(1 - min(max(0, -(in->crisp_val-80.0)/160.0), 1.0));
			break;
			case smallt:
				out = (double)((-fabs(in->crisp_val)+180.0)/180.0);
			break;
			case larget:
				out = (double)(1.0 -  (-(fabs(in->crisp_val))+180.0)/180.0);	
			break;
		}
	return out; 
}

/* entry point */
int main(int argc, char** argv)
{
	
	/* create the reference trajectory */
	trajectory *ref_trajectory = (trajectory*)calloc(MAX_TRAJ_SIZE, sizeof(trajectory));
	/* create the robot trajectory */
	trajectory *robot_trajectory = (trajectory*)calloc(MAX_TRAJ_SIZE, sizeof(trajectory));	

	/* input data samples counter */
	int input_idx = 0;	

	/* fuzzy controller vars */
	double w1 = 0.0f, w2 = 0.0f, w3 = 0.0f, w4 = 0.0f;
	/* control signals for each rule */
	double *u1 = (double*)calloc(2,sizeof(double));
	double *u2 = (double*)calloc(2,sizeof(double));
	double *u3 = (double*)calloc(2,sizeof(double));
	double *u4 = (double*)calloc(2,sizeof(double));
	double **u_fin = (double**)calloc(MAX_TRAJ_SIZE, sizeof(double*));
	for (int i = 0; i< MAX_TRAJ_SIZE; i++)
		u_fin[i] = (double*) calloc(2, sizeof(double));

	/* the 2 fuzzified inputs, de and thetae */
	fuzzy_in *de = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
	fuzzy_in *thetae = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
	
	double Kde[NUM_RULES] = {0.25,0.25,0.50,0.5};
	double Kte[NUM_RULES] = {0.12,0.25,0.12,0.25};

        /* get input data from stdin */
        while(scanf("%lf,%lf,%lf,%lf,%lf,%lf\n", &(ref_trajectory[input_idx].x_pos),
                                                 &(ref_trajectory[input_idx].y_pos),
                                                 &(ref_trajectory[input_idx].theta),
                                                 &(robot_trajectory[input_idx].x_pos),
                                                 &(robot_trajectory[input_idx].y_pos),
                                                 &(robot_trajectory[input_idx].theta))>0){
                /* check if max size was reached */
                if(++input_idx == MAX_TRAJ_SIZE) break;
        }

	
	/* main control loop simulation */
	for(int t = 0; t<input_idx; ++t){
		/* the 2 inputs in the fuzzy controller are the 2 error values */
		de->crisp_val = compute_de(ref_trajectory[t], robot_trajectory[t]);
		thetae->crisp_val = compute_thetae(ref_trajectory[t], robot_trajectory[t]);
		
		/* fire the rules */
		
		/* first rule */
		de->membership_func = smalld;
		thetae->membership_func = smallt;
			
		w1 = compute_aggregation(compute_membership(de),
					 compute_membership(thetae)
					);
		u1[0] = Kde[0]*(de->crisp_val) + Kte[0]*(thetae->crisp_val);
		u1[1] = Kde[0]*(de->crisp_val) - Kte[0]*(thetae->crisp_val);


		/* second rule */
		de->membership_func = smalld;
		thetae->membership_func = larget;
	
		w2 = compute_aggregation(
					 compute_membership(de),
					 compute_membership(thetae)
					);
		u2[0] = Kde[1]*de->crisp_val + Kte[1]*thetae->crisp_val;
		u2[1] = Kde[1]*de->crisp_val - Kte[1]*thetae->crisp_val;


		/* third rule */
		de->membership_func = larged;
		thetae->membership_func = smallt;
	
		w3 = compute_aggregation(
					 compute_membership(de),
					 compute_membership(thetae)
					);
		u3[0] = Kde[2]*de->crisp_val + Kte[2]*thetae->crisp_val;
		u3[1] = Kde[2]*de->crisp_val - Kte[2]*thetae->crisp_val;


		/* fourth rule */
		de->membership_func = larged;
		thetae->membership_func = larget;
	
		w4 = compute_aggregation(
		 			 compute_membership(de),
					 compute_membership(thetae)
					);
		u4[0] = Kde[3]*de->crisp_val + Kte[3]*thetae->crisp_val;
		u4[1] = Kde[3]*de->crisp_val - Kte[3]*thetae->crisp_val;

		/* compute output */
	        u_fin[t][0] = (w1*u1[0] + w2*u2[0] + w3*u3[0] + w4*u4[0])/(w1+w2+w3+w4);
		u_fin[t][1] = (w1*u1[1] + w2*u2[1] + w3*u3[1] + w4*u4[1])/(w1+w2+w3+w4);
					
	}

        for(int i = 0;i<input_idx;i++){
                printf("%lf,%lf\n", u_fin[i][0], u_fin[i][1]);
        }

	/* free resources */
	free(ref_trajectory); 
	free(robot_trajectory);
	free(de);
	free(thetae);
	free(u1);
	free(u2);
	free(u3);
	free(u4);
	free(u_fin);


	return (EXIT_SUCCESS);
}
