#ifndef _PARTICLE_H_
#define _PARTICLE_H_



#include <stdio.h>
#include <stdlib.h>
#include <math.h>


//===============================================================================
#define	COS_MULTIPLIER_2	128
#define	COS_MULTIPLIER_3	1024

#define	FLOAT_REPRESENTATION		
//#define	INT_REPRESENTATION

#define	BEARING_SAMPLING			

#define	PARTICLE_COMPONENTS				3
#define	LINE_COMPONENTS					7
#define	ENVIRONMENT_COMPONENTS		5
#define	V									1
#define	H									0
#define	POSITIVE							0
#define	NEGATIVE							1

#define	PARTICLE_X							0
#define	PARTICLE_Y							1
#define	PARTICLE_THETA						2

#define	PARTICLE_NUM_SENSORS			5
#define 	PARTICLE_RANGE_MIN				0
#define 	PARTICLE_RANGE_MAX				0
#define	PARTICLE_INVALID_MEASURE			300

#define	LINE_ALPHA							0
#define	LINE_RHO							1
#define	LINE_TYPE							2
#define	LINE_P1_X							3
#define	LINE_P1_Y							4
#define	LINE_P2_X							5
#define	LINE_P2_Y							6

//#define	VERBOSE_POINT_TO_LINE_DIST
//#define	VERBOSE_ENVIRONMENT_INIT
//#define	VERBOSE_CHECK_ANGLE
//#define	VERBOSE_get_weight_from_sensors

//#define		VERBOSE_PARTICLE_EVOLUTION
//#define		VERBOSE_PARTICLE_RENORMALIZATION				
//#define		VERBOSE_PARTICLE_RESAMPLING
//#define		VERBOSE_PARTICLE_create_random_particles
//#define		VERBOSE_sir_resampling
//#define			VERBOSE_particles_freeze_log




//#define			PARTICLE_SAMPLING_UNIQUE_AMBIENT
#define			PARTICLE_SAMPLING_DUAL_AMBIENT


#define	PARTICLE_FITNESS_CONSTANT_SHORT			0.005
#define	PARTICLE_FITNESS_CONSTANT_LONG			0.004

#define	PARTICLE_FITNESS_RANGE_THRESHOLD		35.0
#define	PARTICLE_RESAMPLING_RAND_RATIO			0.8
#define 	PARTICLE_REPRODUCTOR_NUMBER			1
#define 	PARTICLE_PERIOD							0.25

#define	PARTICLE_INVALID_SOGLIA			-10.0





#define	PARTICLE_ENV_WIDTH						120
#define	PARTICLE_ENV_OFFSET_WIDTH				23

#define	PARTICLE_ENV_HEIGHT						160
#define	PARTICLE_ENV_OFFSET_HEIGHT				27



#define	LIB_PARTICLE_MODULE(a)((a>0)?a:-a)
#define	LIB_PARTICLE_SIGN(a)((a>0)?POSITIVE:NEGATIVE)
#define	PARTICLE_RAND_NUMBER(a)( a*((float)rand()/RAND_MAX-0.5))



//===============================================================================


//===============================================================================
	float 	*environment[ENVIRONMENT_COMPONENTS];
	#ifdef	FLOAT_REPRESENTATION	
	float	line_1[LINE_COMPONENTS]={-M_PI, 		23,		V,	 -23, -27,		-23,	133};
	float	line_2[LINE_COMPONENTS]={M_PI/2, 	133, 	H, 	-23,	133,		37,	133};
	float	line_3[LINE_COMPONENTS]={0, 			37, 		V, 	37, 133,		37,	71};		
	float	line_4[LINE_COMPONENTS]={M_PI/2, 	71,		H, 	37, 	71,		97,	71};
	float	line_5[LINE_COMPONENTS]={0,			97,		V, 	97, 	71,		97,	-27};		
	
	float	IR_POS[][3]={
				{ -7.5,	 9.5,		 M_PI/2  },
				{-7.5,	-9.5,		-M_PI/2  },
				{11,		 9.5,		 M_PI/18},
				{11,	        -9.5,		-M_PI/18},
				{11,	            0,			    0 },
			};
			
	#endif
	#ifdef	INT_REPRESENTATION	
	int	line_1[LINE_COMPONENTS]={-1, 23, V, -23, -27,-23,133};
	#endif
	#define	PARTICLE_FITNESS_CONSTANT_SHORT			0.005
#define	PARTICLE_FITNESS_CONSTANT_LONG			0.004

	unsigned 	int	num_particles;
	float			*particles;
	float			*particles_weights;
	float			*particles_iteration_weights;	
	
//	static	float	ambience_A_borders[4]=		{-23,37,-27,133};
//	static	float	ambience_B_borders[4]=		{37,97,-27,71};
	static	float	ambience_A_borders[4]=		{0,37,0,133};
	static	float	ambience_B_borders[4]=		{37,97,0,71};
//	static	float	ambience_A_borders[4]=		{30,40,30,40};
//	static	float	ambience_B_borders[4]=		{30,40,30,40};
	static	float	ambience_C_borders[4]=		{0,97,0,133};
	static 	float	particle_sampling_variance[3]=	{1,1,0.1};
	
	FILE	*particle_log;
	char	name_file_particle[]="particle.txt";
	
	unsigned	int 	real_distances[PARTICLE_NUM_SENSORS];
	float				 particle_accumulator;
	unsigned 	int		*particle_to_cancel_indexes;
	unsigned 	int		*particle_best_indexes;
	unsigned	int		 particle_best_min;
	unsigned	int		 particle_best_max;
	unsigned	int		particle_2_rand_resample;
	unsigned	int		particle_2_best_resample;
	unsigned	int		particle_faileds_number;	
	
	
	float		particle_death_precentage;
	float		particle_death_threshold;
	unsigned	int	particle_ir_current_measure[5];
	unsigned	int	particle_min_number_2_resample;	
	
//===============================================================================


//===============================================================================
//int	create_look_up_cos_table(float	angle, unsigned int levels, int **table, int precision);
//unsigned long julery_isqrt(unsigned long val) ;

#ifdef	FLOAT_REPRESENTATION
int		environment_init();
int		point_to_line_dist( float *sensor_pose, float *line, float *distance  );
int 	check_angle(float *angle);
int		get_weight_from_sensors(float	*particle, float *weights, unsigned int *ir_distances);
int		print_line(float* line);
int		print_environment(float* env[]);
int 	print_sensor_poses();
int 	get_weight( float estimate,  float distance, float *fitness_value);
int		particle_init(int n);
int		particle_destroy();
int		particles_prediction(float *particles, int n, float u, float w , float t );
int		print_particle(float *p);
int		create_random_particles(float *particles, int init_index, int N, float *environment);
int		sir_resampling( float *p, float *g, float *variance);
int		particles_evolution( float u, float w, float t, float *accumulator, unsigned int *measure);
int		particles_renormalization( float *weights, float accumulator, unsigned int *to_cancel_particles);
int		particles_resampling(float *particles, unsigned int low_n_particles);
int		particle_reinitialize_weights(float *weights, unsigned int n);
int		particles_freeze_log(char nome_file[]);
int		int_particles_evolution( float u, float w, float t, float *accumulator, unsigned int *measure);
int		int_get_weight_from_sensors(float	*particle, float *w,  unsigned int *ir_distances );
int		int_point_to_line_dist( float *sensor_pose, float *line, float *distance );

#endif

//===============================================================================
#endif
