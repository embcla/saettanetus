
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//===============================================================================


#define	NUMERO_PARTICLES	1


#define	PARTICLE_SENSOR_LEVELS	360



#define	COS_LEVELS_128	128


#define	COS_MULTIPLIER_1024	1024
#define	COS_MULTIPLIER_512	512


#define	PARTICLE_MULTIPLIER			COS_MULTIPLIER_1024
#define	PARTICLE_TRIG_LEVELS		COS_LEVELS_128-1

#define	FLOAT_REPRESENTATION		
#define	INT_REPRESENTATION

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


#define	PARTICLE_2_PI_2_MULTIPLIER		5340343
#define	PARTICLE_2_PI_2_SHIFTER			16

#define	PARTICLE_BEARING_TO_TABLE					15019744
#define	PARTICLE_BEARING_TO_TABLE_SHIFTER			18




//#define	PARTICLE_ROBOT_BEARING_TO_TABLE(a)(PARTICLE_GET_SIGN(a)*(((unsigned int)PARTICLE_GET_MODULE(a))*PARTICLE_BEARING_TO_TABLE)>>PARTICLE_2_PI_2_SHIFTER)

#define	PARTICLE_ROBOT_BEARING_TO_TABLE(a)(180+PARTICLE_GET_SIGN(a)*((unsigned int)(PARTICLE_GET_MODULE(a)*PARTICLE_BEARING_TO_TABLE)>>PARTICLE_BEARING_TO_TABLE_SHIFTER))


#define	PARTICLE_ROBOT_TABLE_TO_BEARING(a)(PARTICLE_GET_SIGN(a)*((float)(PARTICLE_GET_MODULE(a)<<PARTICLE_BEARING_TO_TABLE_SHIFTER)/PARTICLE_BEARING_TO_TABLE))



//#define	VERBOSE_POINT_TO_LINE_DIST
//#define	VERBOSE_ENVIRONMENT_INIT
//#define	VERBOSE_CHECK_ANGLE
//#define	VERBOSE_get_weight_from_sensors

//#define		VERBOSE_PARTICLE_EVOLUTION
//#define		VERBOSE_PARTICLE_RENORMALIZATION				
//#define		VERBOSE_PARTICLE_RESAMPLING
//#define		VERBOSE_PARTICLE_create_random_particles
//#define		VERBOSE_sir_resampling
//#define		VERBOSE_particles_freeze_log


//#define			VERBOSE_get_quadrante
//#define			VERBOSE_get_cos
//#define			VERBOSE_get_sin
//#define			VERBOSE_particle_create_lookup_sensor

//#define				VERBOSE_int_point_to_line_dist
//#define					VERBOSE_particle_int_get_weights

//#define					VERBOSE_crea_exp_table
//#define			PARTICLE_SAMPLING_UNIQUE_AMBIENT
#define			PARTICLE_SAMPLING_DUAL_AMBIENT


//#define	PARTICLE_FITNESS_CONSTANT_SHORT			0.005
//#define	PARTICLE_FITNESS_CONSTANT_LONG			0.004

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





//#define	PARTICLE_INT_RAND_NUMBER(a)(   ((rand()/((unsigned int)RAND_MAX>>7))-64)             )

//#define	PARTICLE_INT_RAND_NUMBER(a)(  (int)((int)(a*rand())/((int)RAND_MAX>>10)-512)                   )
#define		PARTICLE_INT_RAND_NUMBER(a)(((int)((int)a*((int)(rand())/((int)RAND_MAX>>10)-512)))>>9)

//((a*(int)((rand()/(RAND_MAX>>7))-64))>>7)

				



//#define	PARTICLE_INT_RAND_NUMBER(a)( a*((float)rand()/RAND_MAX-0.5))


#define	PARTICLE_RAD_2_INT(a)((unsigned int)(a*PARTICLE_2_PI_2_MULTIPLIER)>>PARTICLE_2_PI_2_SHIFTER)
#define	PARTICLE_INT_2_RAD(a)(PARTICLE_GET_SIGN(a)*((float)(PARTICLE_GET_MODULE(a)<<PARTICLE_2_PI_2_SHIFTER)/PARTICLE_2_PI_2_MULTIPLIER))



#define	PARTICLE_RESTO(a,b)(a-b)
#define	PARTICLE_GET_MODULE(a)((a>0)?a:-a)
#define	PARTICLE_GET_SIGN(a)((a>0)?1:-1)

#define	A_CAPO()(printf("\n"))
#define INTERLINEA()(printf("==========================================\n"))
#define	ASPETTA()(while(getchar()!='\n'))






#define	PARTICLE_PROJECTION(x,cos_val, sign_cos)((int)((((unsigned int)(PARTICLE_GET_MODULE(x)*2))*cos_val)>>11)*PARTICLE_GET_SIGN(x)*sign_cos)

#define	PARTICLE_ROTOTRASLA_SUP(x,cos_val, sign_cos,y,sin_val, sign_sin)(PARTICLE_PROJECTION(x,cos_val, sign_cos)-PARTICLE_PROJECTION(y,sin_val, sign_sin))

#define	PARTICLE_ROTOTRASLA_INF(x,cos_val, sign_cos,y,sin_val, sign_sin)(PARTICLE_PROJECTION(x,sin_val, sign_sin)+PARTICLE_PROJECTION(y,cos_val, sign_cos))


//#define	PARTICLE_ROTOTRASLA_SUP(a)(a)	





#define	FROM_UNIT_2_CM(a)(a>>2)
#define	FROM_CM_2_UNIT(a)(a<<2)




#define		PARTICLE_EXP_TABLE_CARDINALITY		128
#define		PARTICLE_EXP_TABLE_MAX				32768

#define		PARTICLE_EXP_TABLE_EQUIVALENT_SHIFT	15

#define	PARTICLE_FITNESS_CONSTANT_SHORT			0.0005
#define	PARTICLE_FITNESS_CONSTANT_LONG			0.0004


#define	PARTICLE_INT_GET_DISTANCE(meas_diff)(particle_fitness_table[meas_diff])

#define	PARTICLE_INT_WEIGHT_UPDATE(a,b)(a=(a*b)>>15)






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
				{ -7.5,	 9.5,  	M_PI/2  },
				{-7.5,	-9.5, 	-M_PI/2  },
				{11, 9.5,  	M_PI/18},
				{11,-9.5,	-M_PI/18},
				{11, 0, 0 },
			};
			
	#endif
//#define	PARTICLE_FITNESS_CONSTANT_SHORT			0.005
//#define	PARTICLE_FITNESS_CONSTANT_LONG			0.004

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
	unsigned	int		 particle_2_rand_resample;
	unsigned	int		 particle_2_best_resample;
	unsigned	int		 particle_faileds_number;	
	
	
	float		particle_death_precentage;
	float		particle_death_threshold;
	unsigned	int	particle_min_number_2_resample;	


	unsigned 	int 	particle_cos_lookup[COS_MULTIPLIER_1024];
	unsigned 	int 	particle_sin_lookup[COS_MULTIPLIER_1024];



	int					particle_sensor_table[PARTICLE_SENSOR_LEVELS][15];

	int					particle_sensor_distances[NUMERO_PARTICLES][ENVIRONMENT_COMPONENTS];

	int					current_sensor_distance[NUMERO_PARTICLES];
	int					particle_iteration_distances[NUMERO_PARTICLES][PARTICLE_NUM_SENSORS];

	int					particle_sensor_weights[NUMERO_PARTICLES][PARTICLE_NUM_SENSORS];
	int					particle_iteration_weights[NUMERO_PARTICLES];
	int					particle_weights[NUMERO_PARTICLES];


	unsigned 	int			particle_fitness_table[PARTICLE_EXP_TABLE_CARDINALITY];

	unsigned 	int			particles_int_weights[NUMERO_PARTICLES];
	unsigned 	int			particles_int_iteration_weights[NUMERO_PARTICLES];	
	unsigned 	int			particle_int_current_distances[PARTICLE_NUM_SENSORS];
	unsigned 	int			particle_int_best_indexes[NUMERO_PARTICLES];

	static 		int			particle_int_sampling_variance[3]=	{10,10,100};

	int						particle_int_death_threshold;

	unsigned	int	offset_pi;

	int 		particle_int_accumulator;


	struct timeval particle_start_time, particle_end_time; 
	long   particle_time;




	
//===============================================================================


//===============================================================================

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
int		particles_prediction(float *particles, int n, float u, float w, float t);
int		print_particle(float *p);
int		create_random_particles(float *particles, int init_index, int N, float *environment);
int		sir_resampling( float *p, float *g, float *variance);
int		particles_evolution( float u, float w, float t, float *accumulator, unsigned int *measure);
int		particles_renormalization( float *weights, float accumulator, unsigned int *to_cancel_particles);
int		particles_resampling(float *particles, unsigned int low_n_particles);
int		particle_reinitialize_weights(float *weights, unsigned int n);
int		particles_freeze_log(char nome_file[]);


#endif




#ifdef	INT_REPRESENTATION





//int	create_look_up_cos_table(float	angle, unsigned int levels, unsigned int **table, int precision);
unsigned long	julery_isqrt(unsigned long val) ;
int				particle_create_lookup_tables(float angle);
int				particle_create_lookup_sensor();
int				get_cos(float	angle, int *result, int *sign);
int				get_sin(float	angle, int *result, int *sign);
int				get_quadrante(unsigned int	converted_angle, int *quadrante, unsigned int *new_angle);
int				evo_get_sin(int conv_angle, int angle_sign, int *result, int *sign);
int				evo_get_cos(int conv_angle, int angle_sign, int *result, int *sign);
int				get_mod_and_sign(float	angle, unsigned int *mod, int *sign);
int				particle_lookup_initialization();
int				display_sensor_from_table(int level);
int				int_point_to_line_dist( int rob_bearing_level);
int				particle_int_get_iteration_weights(unsigned int real_distances[]);
int				particle_int_update_weights(int *accumulator);
int				crea_exp_table();
int				particle_int_create_random_particles(float *particles, int	init_index, int N, float *environment);
int				particle_int_resampling(unsigned int low_n_particles );
int				particle_int_particle_reinitialize_weights(int *weights, unsigned int n);
int				particle_int_reinitialize_weights(int *weights, unsigned int n);
int				particle_int_renormalization(int accumulator, unsigned int *to_cancel_particles);
int 			particle_get_elapsed_time(struct timeval *start_time_pt, struct timeval *end_time_pt, long *elapsed_utime);
#endif







//===============================================================================
