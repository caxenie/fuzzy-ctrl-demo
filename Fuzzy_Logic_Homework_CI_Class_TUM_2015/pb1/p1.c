/* 
 * CI Class WSS2013-14
 * 
 * Fuzzy logic control homework
 * 
 * Mobile robot trajectory tracking - sample implementation 1st assignment
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

	/* the 2 fuzzified inputs, de and thetae */
	fuzzy_in *de = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
	fuzzy_in *thetae = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
	
	/* membership values */
	double* mde1  = (double*) calloc(MAX_TRAJ_SIZE, sizeof(double)); 
	double* mde2  = (double*) calloc(MAX_TRAJ_SIZE, sizeof(double)); 
	double* mthetae1  = (double*) calloc(MAX_TRAJ_SIZE, sizeof(double)); 
	double* mthetae2  = (double*) calloc(MAX_TRAJ_SIZE, sizeof(double)); 
	
	/* get input data from stdin */
	while(scanf("%lf,%lf,%lf,%lf,%lf,%lf\n", &(ref_trajectory[input_idx].x_pos), 
				     		 &(ref_trajectory[input_idx].y_pos),
				     		 &(ref_trajectory[input_idx].theta),
						 &(robot_trajectory[input_idx].x_pos),
						 &(robot_trajectory[input_idx].y_pos),  
						 &(robot_trajectory[input_idx].theta))!=EOF){
		input_idx++;
		/* check if max size was reached */
		if(input_idx == MAX_TRAJ_SIZE) break;
	}
	/* main control loop simulation */
	for(int t = 0; t<input_idx; t++){
		/* the 2 inputs in the fuzzy controller are the 2 error values */
		de->crisp_val = compute_de(ref_trajectory[t], robot_trajectory[t]);
		thetae->crisp_val = compute_thetae(ref_trajectory[t], robot_trajectory[t]);
		
		de->membership_func = smalld;
		thetae->membership_func = smallt;
		mde1[t] = compute_membership(de);
		mthetae1[t] = compute_membership(thetae);
		de->membership_func = larged;
		thetae->membership_func = larget;
	 	mde2[t] = compute_membership(de);
		mthetae2[t] = compute_membership(thetae);
	}

	for(int i = 0;i<input_idx;i++){
		printf("%lf,%lf,%lf,%lf\n", mde1[i], mde2[i], mthetae1[i], mthetae2[i]);
	}

	/* free resources */
	free(ref_trajectory); 
	free(robot_trajectory);
	free(mde1);
	free(mde2);
	free(mthetae1);
	free(mthetae2);
	
	return (EXIT_SUCCESS);
}
