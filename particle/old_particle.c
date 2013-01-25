
#include "particle.h"

//===================================================================================
	int		environment_init(){
		int i;
		environment[0]=&(line_1[0]);
		environment[1]=&(line_2[0]);
		environment[2]=&(line_3[0]);
		environment[3]=&(line_4[0]);
		environment[4]=&(line_5[0]);
		#ifdef	VERBOSE_ENVIRONMENT_INIT
			print_environment(environment);
		#endif
		return 0;
	}
//===================================================================================


//===================================================================================
	int 	check_angle(float *angle){
		
		#ifdef VERBOSE_CHECK_ANGLE
			printf("INPUT: %f\n", *angle);
		#endif
		while(*angle>M_PI){
			*angle= *angle-2*M_PI;
		}
		while(*angle<-M_PI){
			*angle= *angle+2*M_PI;
		}
		return 0;
	}
//===================================================================================

//===================================================================================
	int	create_look_up_cos_table(float	angle, unsigned int levels, int **table, int precision){
		int i;
		float delta_alpha;
		int multiplicator;
		switch (precision){
			case  2:
				multiplicator=COS_MULTIPLIER_2;
				break;
			case  3:
				multiplicator=COS_MULTIPLIER_3;			
				break;
			default :
				multiplicator=0;
				return -1;
				break;
		}
		
		*table=malloc(sizeof(int)*levels);
		delta_alpha=angle/levels;
		for(i=0;i<levels;i++){*(*table+i)=(int)(cos(i*delta_alpha)*multiplicator);}
		return	0;
	}
//===================================================================================	
	
	unsigned long julery_isqrt(unsigned long val) {
		unsigned long temp, g=0, b = 0x8000, bshft = 15;
		do {
	  		if (val >= (temp = (((g << 1) + b)<<bshft--))) {
	      		g += b;
	        	val -= temp;
	      		}
		}
		while (b >>= 1);
	 	return g;
	}	
//===================================================================================


	
//===================================================================================
	int		point_to_line_dist( float *sensor_pose, float *line, float *distance ) {
	
		float	*p1_prime;
		float    	*p2_prime;
		float	angle;
		float	rho_prime;
		float	alpha_prime;
		int a, b;
		#ifdef	VERBOSE_POINT_TO_LINE_DIST
		printf("SENSOR: %f %f %f\n",*(sensor_pose),*(sensor_pose+1),*(sensor_pose+2));
		printf("LINE:  ");
		for(a=0;a<7;a++){
			printf(" %f  ", *(line+a));		
		}
		printf("\n");
		#endif
		alpha_prime=*(line+LINE_ALPHA)-*(sensor_pose+PARTICLE_THETA);
		angle=-*(sensor_pose+PARTICLE_THETA);
		check_angle(&alpha_prime);
		#ifdef	VERBOSE_POINT_TO_LINE_DIST
		printf("ALPHA PRIME: %f\n", alpha_prime);
		while(getchar()!='\n');
		#endif
		//==========	ROTATION OF SEGMENT ENDINGS =======================
		p1_prime=malloc(sizeof(float)*2);
		p2_prime=malloc(sizeof(float)*2);
		*(p1_prime+PARTICLE_X)= cos(angle)*(       *(line+LINE_P1_X) -*(sensor_pose+PARTICLE_X))-sin(angle)*(*(line+LINE_P1_Y) -*(sensor_pose		+PARTICLE_Y));
		*(p1_prime+PARTICLE_Y)= sin(angle)*(*(line+LINE_P1_X) -*(sensor_pose+PARTICLE_X))+cos(angle)*(*(line+LINE_P1_Y) -*(sensor_pose		+PARTICLE_Y));
		*(p2_prime+PARTICLE_X)= cos(angle)*(       *(line+LINE_P2_X) -*(sensor_pose+PARTICLE_X))-sin(angle)*(*(line+LINE_P2_Y) -*(sensor_pose		+PARTICLE_Y));
		*(p2_prime+PARTICLE_Y)= sin(angle)*(*(line+LINE_P2_X) -*(sensor_pose+PARTICLE_X))+cos(angle)*(*(line+LINE_P2_Y) -*(sensor_pose		+PARTICLE_Y));
		#ifdef	VERBOSE_POINT_TO_LINE_DIST
		printf("punto 1: %f %f\n", *(p1_prime+PARTICLE_X),*(p1_prime+PARTICLE_Y));
		printf("punto 2: %f %f\n", *(p2_prime+PARTICLE_X),*(p2_prime+PARTICLE_Y));
		while(getchar()!='\n');
		#endif

		if(LIB_PARTICLE_MODULE(alpha_prime) <M_PI/2 &&  (LIB_PARTICLE_SIGN(*(p1_prime+PARTICLE_Y)) !=LIB_PARTICLE_SIGN(*(p2_prime+PARTICLE_Y)))){
			// 	in the following I need the biased rho: in v or h lines it corresponds to the common coordinate
			if(*(line+LINE_TYPE)==V){	rho_prime=*(line+LINE_P1_X)-*(sensor_pose+PARTICLE_X);}
		
			if(*(line+LINE_TYPE)==H){	rho_prime=*(line+LINE_P1_Y)-*(sensor_pose+PARTICLE_Y);}
		
			*distance=LIB_PARTICLE_MODULE(rho_prime)/cos(alpha_prime);
			#ifdef	VERBOSE_POINT_TO_LINE_DIST
			printf("DISTANZA REALE: %f \n", *distance);
			while(getchar()!='\n');
			#endif
			
		}
		else{
			return -1;
		}
		return 0;
	}
	//===================================================================================



//===================================================================================
	int		int_point_to_line_dist( float *sensor_pose, float *line, float *distance ) {
	
		float	*p1_prime;
		float    	*p2_prime;
		float	angle;
		float	rho_prime;
		float	alpha_prime;
		int a, b;
		alpha_prime=*(line+LINE_ALPHA)-*(sensor_pose+PARTICLE_THETA);
		angle=-*(sensor_pose+PARTICLE_THETA);
		check_angle(&alpha_prime);
		//==========	ROTATION OF SEGMENT ENDINGS =======================
		p1_prime=malloc(sizeof(float)*2);
		p2_prime=malloc(sizeof(float)*2);
		*(p1_prime+PARTICLE_X)= cos(angle)*(       *(line+LINE_P1_X) -*(sensor_pose+PARTICLE_X))-sin(angle)*(*(line+LINE_P1_Y) -*(sensor_pose		+PARTICLE_Y));
		*(p1_prime+PARTICLE_Y)= sin(angle)*(*(line+LINE_P1_X) -*(sensor_pose+PARTICLE_X))+cos(angle)*(*(line+LINE_P1_Y) -*(sensor_pose		+PARTICLE_Y));
		*(p2_prime+PARTICLE_X)= cos(angle)*(       *(line+LINE_P2_X) -*(sensor_pose+PARTICLE_X))-sin(angle)*(*(line+LINE_P2_Y) -*(sensor_pose		+PARTICLE_Y));
		*(p2_prime+PARTICLE_Y)= sin(angle)*(*(line+LINE_P2_X) -*(sensor_pose+PARTICLE_X))+cos(angle)*(*(line+LINE_P2_Y) -*(sensor_pose		+PARTICLE_Y));

		if(LIB_PARTICLE_MODULE(alpha_prime) <M_PI/2 &&  (LIB_PARTICLE_SIGN(*(p1_prime+PARTICLE_Y)) !=LIB_PARTICLE_SIGN(*(p2_prime+PARTICLE_Y)))){
			// 	in the following I need the biased rho: in v or h lines it corresponds to the common coordinate
			if(*(line+LINE_TYPE)==V){	rho_prime=*(line+LINE_P1_X)-*(sensor_pose+PARTICLE_X);}
		
			if(*(line+LINE_TYPE)==H){	rho_prime=*(line+LINE_P1_Y)-*(sensor_pose+PARTICLE_Y);}
		
			*distance=LIB_PARTICLE_MODULE(rho_prime)/cos(alpha_prime);
			#ifdef	VERBOSE_POINT_TO_LINE_DIST
			printf("DISTANZA REALE: %f \n", *distance);
			while(getchar()!='\n');
			#endif
			
		}
		else{
			return -1;
		}
		return 0;
	}
	//===================================================================================
	

	//===================================================================================
	int	print_line(float *line){
		int a;
		printf("LINE:  ");
		for(a=0;a<LINE_COMPONENTS;a++){printf(" %f  ", *(line+a));}
		printf("\n");	
	}
	//===================================================================================


	//===================================================================================
	int		print_environment(float* env[]){
		int i;
		for(i=0;i<ENVIRONMENT_COMPONENTS;i++){	print_line(env[i]);	}
	}
	//===================================================================================
	
	//===================================================================================
	int 		print_sensor_poses(){
		int i;
		printf("================");
		for(i=0;i<PARTICLE_NUM_SENSORS;i++){printf("%f %f %f\n", IR_POS[i][0],IR_POS[i][1],IR_POS[i][2]);}
		printf("================");
	
	
	}
	//===================================================================================
	//===================================================================================	
	int	get_weight_from_sensors(float	*particle, float *w,  unsigned int *ir_distances ){
		
		int i,j;
		float	sensor_pose[3];
		//float 	distances_sensors[PARTICLE_NUM_SENSORS];
		float	true_distance;
		float	temp_distance;
		float weights[PARTICLE_NUM_SENSORS];
		unsigned int 	accumulatore;
		accumulatore=0;
		*w=0;

		for(i=0;i<5;i++){weights[i]=0;}


		#ifdef	VERBOSE_get_weight_from_sensors

		printf("MISURE IN ESAME\n");
		for(i=0;i<PARTICLE_NUM_SENSORS;i++){printf("%d ", *(ir_distances+i));}
		printf("\n");
		#endif


		for(i=0;i<PARTICLE_NUM_SENSORS;i++){
			sensor_pose[PARTICLE_X]=*(particle+PARTICLE_X)+cos( *(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_X]-sin( *(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_Y];
			sensor_pose[PARTICLE_Y]=*(particle+PARTICLE_Y)+sin( *(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_X]+cos(*(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_Y];
			sensor_pose[PARTICLE_THETA]=*(particle+PARTICLE_THETA)+IR_POS[i][PARTICLE_THETA];
			
			#ifdef	VERBOSE_get_weight_from_sensors
			printf("=============================\n");
			printf("ANGOLO: %f\n", *(particle+PARTICLE_THETA));
			printf("PARTICLE:	%f %f %f\n", *(particle+PARTICLE_X) ,*(particle+PARTICLE_Y) ,*(particle+PARTICLE_THETA) );
			printf("LOCALE: 	%f %f %f\n", IR_POS[i][PARTICLE_X],IR_POS[i][PARTICLE_Y],IR_POS[i][PARTICLE_THETA]);			
			printf("POSA SENSORE: %f %f %f\n", sensor_pose[PARTICLE_X],sensor_pose[PARTICLE_Y],sensor_pose[PARTICLE_THETA]);
			printf("=============================\n");			
			while(getchar()!='\n');
			#endif
			true_distance=PARTICLE_INVALID_MEASURE;
			for(j=0;j<ENVIRONMENT_COMPONENTS;j++){	
				#ifdef	VERBOSE_get_weight_from_sensors
				printf("=============================\n");							
				printf("LINEA :%d\n", j);
				#endif
				temp_distance=PARTICLE_INVALID_MEASURE;
				point_to_line_dist( sensor_pose, environment[j], &temp_distance  );
				if(temp_distance > 0 && (temp_distance<true_distance)){true_distance=temp_distance;}
				#ifdef	VERBOSE_get_weight_from_sensors
				printf("DISTANCES: TRUE %f   TEMP %f\n ", true_distance, temp_distance);
				printf("=============================\n");			
				while(getchar()!='\n');
				#endif
			}
			
			if(true_distance < PARTICLE_INVALID_MEASURE){
				#ifdef	VERBOSE_get_weight_from_sensors			
				printf("SENSORE: %d INPUT %d %f\n", i, *(ir_distances+i),  true_distance);
				//get_weight( *(ir_distances+i),  true_distance, &(weights[i]));
				#endif
				get_weight( true_distance, *(ir_distances+i) , &(weights[i]));


				
				#ifdef	VERBOSE_get_weight_from_sensors
				printf("RISULTATO: %f\n", weights[i]);
				if(weights[i]<0){

					printf("AZZ: ....... valore negativo, input: %f %f\n", true_distance, *(ir_distances+i));
				}


				while(getchar()!='\n');
				#endif
				++accumulatore;
			}
			else{
				if(*ir_distances== PARTICLE_RANGE_MIN || *ir_distances==PARTICLE_RANGE_MAX){
					weights[i]=1;
					++accumulatore;
				}
				else{weights[i]=0;}
			}
		}	
		for(i=0;i<5;i++){*w=*w+weights[i];	}
		*w=*w/accumulatore;
		#ifdef	VERBOSE_get_weight_from_sensors
		printf("PESI: ");
		for(i=0;i<5;i++){
			printf("%f  ", weights[i]   );
		}		
		printf("FINAL FITNESS VALUE: %f\n", *w);

		if(*w<0){printf("AZZ... AZZ...\n");		exit(1);}

		#endif

		
		return 0;
	}
	//===================================================================================




	//===================================================================================	
	int	int_get_weight_from_sensors(float	*particle, float *w,  unsigned int *ir_distances ){
		
		int i,j;
		float	sensor_pose[3];
		//float 	distances_sensors[PARTICLE_NUM_SENSORS];
		float	true_distance;
		float	temp_distance;
		float weights[PARTICLE_NUM_SENSORS];
		unsigned int 	accumulatore;
		accumulatore=0;
		*w=0;

		for(i=0;i<5;i++){weights[i]=0;}

		for(i=0;i<PARTICLE_NUM_SENSORS;i++){
			sensor_pose[PARTICLE_X]=*(particle+PARTICLE_X)+cos( *(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_X]-sin( *(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_Y];
			sensor_pose[PARTICLE_Y]=*(particle+PARTICLE_Y)+sin( *(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_X]+cos(*(particle+PARTICLE_THETA) )*IR_POS[i][PARTICLE_Y];
			sensor_pose[PARTICLE_THETA]=*(particle+PARTICLE_THETA)+IR_POS[i][PARTICLE_THETA];
			
			true_distance=PARTICLE_INVALID_MEASURE;
			for(j=0;j<ENVIRONMENT_COMPONENTS;j++){	
				temp_distance=PARTICLE_INVALID_MEASURE;
				point_to_line_dist( sensor_pose, environment[j], &temp_distance  );
				if(temp_distance > 0 && (temp_distance<true_distance)){true_distance=temp_distance;}
			}
			
			if(true_distance < PARTICLE_INVALID_MEASURE){
				get_weight( true_distance, *(ir_distances+i) , &(weights[i]));
				++accumulatore;
			}
			else{
				if(*ir_distances== PARTICLE_RANGE_MIN || *ir_distances==PARTICLE_RANGE_MAX){
					weights[i]=1;
					++accumulatore;
				}
				else{weights[i]=0;}
			}
		}	
		for(i=0;i<5;i++){*w=*w+weights[i];	}
		*w=*w/accumulatore;
		return 0;
	}
	//===================================================================================










	//===================================================================================
	int 		get_weight( float estimate,  float distance, float *fitness_value){
		*fitness_value=0;
		if(distance<PARTICLE_FITNESS_RANGE_THRESHOLD){
			//printf("-------------------------   SHORT\n");
			*fitness_value=exp(-PARTICLE_FITNESS_CONSTANT_SHORT*(distance-estimate)*(distance-estimate)	);}
		else{		
			//printf("-------------------------   LONG\n");
			*fitness_value=exp(-PARTICLE_FITNESS_CONSTANT_LONG*(distance-estimate)*(distance-estimate));
		}
		return 0;
	
	}
	//===================================================================================

	//===================================================================================
	int		particle_init(int n){


		environment_init();
		num_particles=n;
		particle_accumulator=0;
		particles=malloc(sizeof(float)*PARTICLE_COMPONENTS*num_particles);
		particles_weights=malloc(sizeof(float)*num_particles);
		particles_iteration_weights=malloc(sizeof(float)*num_particles);
		particle_to_cancel_indexes=malloc(sizeof(unsigned int)*num_particles);
		particle_best_indexes=malloc(sizeof(unsigned int)*num_particles);		
		particle_log=fopen(name_file_particle, "w+");
		particle_death_precentage=0.2;		
		particle_death_threshold=1.0/(10.0*(float)num_particles);		
		particle_min_number_2_resample=(unsigned int)(num_particles*particle_death_precentage);
		particle_reinitialize_weights(particles_weights, num_particles);
		
		
	}
	//===================================================================================

	//===================================================================================
	int		particle_destroy(){
		free(particles);
		free(particles_weights);
		free(particle_to_cancel_indexes);
		free(particle_best_indexes);
		fprintf(particle_log,"%s","fine\n");
		close(particle_log);
	}
	//===================================================================================



	//===================================================================================
	int		particles_prediction(float *particles, int n, float u, float w, float T){
	
		int	i;
		for(i=0;i<n;i++){
			#ifdef	VERBOSE_get_weight_from_sensors
			printf("A----------------------\n");
			print_particle(particles+i*PARTICLE_COMPONENTS);
			#endif
			*(particles+(i*PARTICLE_COMPONENTS)+PARTICLE_X)+=cos(*(particles+(i*PARTICLE_COMPONENTS)+PARTICLE_THETA))*u*T;
			*(particles+(i*PARTICLE_COMPONENTS)+PARTICLE_Y)+=+sin(*(particles+(i*PARTICLE_COMPONENTS)+PARTICLE_THETA))*u*T;
			*(particles+(i*PARTICLE_COMPONENTS)+PARTICLE_THETA)+=w*T;
			#ifdef	VERBOSE_get_weight_from_sensors
			printf("B----------------------\n");
			print_particle(particles+i*PARTICLE_COMPONENTS);
			#endif

		}
		return 0;
	}
	//===================================================================================	
	
	//===================================================================================	
	int		print_particle(float *p){
		printf("================================\n");
		printf("PARTICLE STATE: %f %f %f\n", *p, *(p+1), *(p+2));	
		printf("================================\n");
		printf("\n");
	
	}
	//===================================================================================	
	
	//===================================================================================	
	int		create_random_particles(float *particles, int	init_index, int N, float *environment){

		int i;
		float app;
		//srand(3);
		float	width;
		float	height;
		float	offset_x;
		float	offset_y;		
		//----------------------------------------------------------------------------
		width=(*(environment+1)-*(environment));
		height=(*(environment+3)-*(environment+2));
		offset_x=*(environment);//+*(environment))/2;
		offset_y=*(environment+2);//+*(environment+2))/2;
		//----------------------------------------------------------------------------	
		#ifdef	VERBOSE_PARTICLE_create_random_particles	
		printf(" W: %f  H: %f  O_X: %f O_Y: %f	\n", width, height, offset_x, offset_y);while(getchar()!='\n');
		#endif

		for(i=0;i<N;i++){
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_X)=(((float)rand()/RAND_MAX)*width)+offset_x;
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_Y)=(((float)rand()/RAND_MAX)*height)+offset_y;
			#ifdef	BEARING_SAMPLING
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA)=M_PI*(((float)rand()/RAND_MAX)*2-1);
			//((float)(rand()/RAND_MAX)*2*M_PI-M_PI;
			#endif
				
			if(*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_X) < PARTICLE_INVALID_SOGLIA   || *(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_Y) < PARTICLE_INVALID_SOGLIA ){
				printf("INGRESSO BUONO: %f %f %f\n", 
					*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_X),
					*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_Y),
					*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA)
				);
				exit(1);
			}



			#ifdef	VERBOSE_PARTICLE_create_random_particles	
			printf("PARTICLES: %f %f %f\n",
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_X),
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_Y),
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA) );
			while(getchar()!='\n');
			#endif
		}
		
	}
	//===================================================================================	

	//===================================================================================	
	int		sir_resampling( float *p, float *g, float *variance){
	
		#ifdef	VERBOSE_sir_resampling
		printf("INGRESSO CATTIVO: %f %f %f\n", *(p+PARTICLE_X),*(p+PARTICLE_Y),*(p+PARTICLE_THETA));
		printf("INGRESSO BUONO: %f %f %f\n", *(g+PARTICLE_X),*(g+PARTICLE_Y),*(g+PARTICLE_THETA));
		#endif
		*(p+PARTICLE_X)=*(g+PARTICLE_X)+PARTICLE_RAND_NUMBER((*variance));//	1;//(*(variance)*rand());
		*(p+PARTICLE_Y)=*(g+PARTICLE_Y)+PARTICLE_RAND_NUMBER((*(variance+PARTICLE_Y)));
		*(p+PARTICLE_THETA)=*(g+PARTICLE_THETA)+PARTICLE_RAND_NUMBER((*(variance+PARTICLE_THETA)));	


		if(*(p+PARTICLE_X) < PARTICLE_INVALID_SOGLIA   || *(p+PARTICLE_Y) < PARTICLE_INVALID_SOGLIA ){


			printf("USCITA CATTIVA: %f %f %f\n", *(p+PARTICLE_X),*(p+PARTICLE_Y),*(p+PARTICLE_THETA));
			printf("INGRESSO BUONO: %f %f %f\n", *(g+PARTICLE_X),*(g+PARTICLE_Y),*(g+PARTICLE_THETA));
			exit(1);



		}


	
		//*(p+PARTICLE_X)+=(*(variance+PARTICLE_THETA)*rand());		
		//*(p+1)=10.0;
		//*(p+PARTICLE_X)=1.0;
		check_angle((p+PARTICLE_THETA));
		#ifdef	VERBOSE_sir_resampling
		printf("OUTPUT: %f %f %f\n", *(p+PARTICLE_X), *(p+1), *(p+2));		
		#endif
	}
	//===================================================================================		

	//===================================================================================		
	void swap_float(float *a, float *b){
  		float t=*a; *a=*b; *b=t;
	}
	//===================================================================================		
	
	//===================================================================================		
	void swap_int(unsigned int *a, unsigned int *b){
  		unsigned int t=*a; *a=*b; *b=t;
	}
	//===================================================================================		
	
	//===================================================================================		
	void sort(float *arr, int beg, int end){
		float piv;	
		int l,r;
		if (end > beg + 1) {
			piv = *(arr+beg), l = beg + 1, r = end;
    			while (l < r) {
      				if (*(arr+l) <= piv){l++;}
        			else{
        				swap_float(arr+l, arr+(--r));
        			}
			}
    			swap_float(arr+(--l), arr+beg);
    			sort(arr, beg, l);
    			sort(arr, r, end);
  		}
	}
	//===================================================================================		

	//===================================================================================		
	void double_sort(float *arr, unsigned int *index_array, int beg, int end){
		float piv;	
		int l,r;
		if (end > beg + 1) {
			piv = *(arr+beg), l = beg + 1, r = end;
    			while (l < r) {
      				if (*(arr+l) <= piv){l++;}
        			else{
        				swap_float(arr+l, arr+(--r));
        				swap_int(index_array+l, index_array+r);
        			}
			}
			swap_float(arr+(--l), arr+beg);
    			swap_int(index_array+l, index_array+beg);
    			double_sort(arr, index_array, beg, l);
    			double_sort(arr, index_array,r, end);
  		}
	}
	//===================================================================================		


	//===================================================================================		
	int		particles_evolution( float u, float w, float T, float *accumulator, unsigned int *measure){
	
		int i;
		particles_prediction(particles, num_particles, u,w, T);
		*accumulator=0;
		for(i=0;i<num_particles;i++){

			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			print_particle(particles+i*PARTICLE_COMPONENTS);
			#endif
			get_weight_from_sensors(particles+i*PARTICLE_COMPONENTS, particles_iteration_weights+i, measure);

			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			printf("PARTICLE %d: ITERATION WEIGHT: %f\n", i, *(particles_iteration_weights+i));
			printf("PARTICLE %d: OLD WEIGHT: %f\n", i, *(particles_weights+i));
			#endif
			*(particles_weights+i)= *(particles_weights+i) * (*(particles_iteration_weights+i));

			if(isnan(*(particles_weights+i))!=0){*(particles_weights+i)=0.0;}
			
			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			printf("PARTICLE %d: NEW WEIGHT: %f\n", i, *(particles_weights+i));
			printf("L'accumulatore era: %f\n", *accumulator);
			while(getchar()!='\n');
			#endif
			*accumulator=*accumulator+*(particles_weights+i);
			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			printf("Ora è: %f\n", *accumulator);
			#endif

		}
		#ifdef VERBOSE_PARTICLE_EVOLUTION			
		printf("ACCUMULATOR: %f\n", *accumulator);
		#endif
	}
	//===================================================================================



	//===================================================================================		
	int		int_particles_evolution( float u, float w, float T, float *accumulator, unsigned int *measure){
	
		int i;
		particles_prediction(particles, num_particles, u,w, T);
		*accumulator=0;
		for(i=0;i<num_particles;i++){

			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			print_particle(particles+i*PARTICLE_COMPONENTS);
			#endif
			get_weight_from_sensors(particles+i*PARTICLE_COMPONENTS, particles_iteration_weights+i, measure);

			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			printf("PARTICLE %d: ITERATION WEIGHT: %f\n", i, *(particles_iteration_weights+i));
			printf("PARTICLE %d: OLD WEIGHT: %f\n", i, *(particles_weights+i));
			#endif
			*(particles_weights+i)= *(particles_weights+i) * (*(particles_iteration_weights+i));

			if(isnan(*(particles_weights+i))!=0){*(particles_weights+i)=0.0;}
			
			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			printf("PARTICLE %d: NEW WEIGHT: %f\n", i, *(particles_weights+i));
			printf("L'accumulatore era: %f\n", *accumulator);
			while(getchar()!='\n');
			#endif
			*accumulator=*accumulator+*(particles_weights+i);
			#ifdef VERBOSE_PARTICLE_EVOLUTION			
			printf("Ora è: %f\n", *accumulator);
			#endif

		}
		#ifdef VERBOSE_PARTICLE_EVOLUTION			
		printf("ACCUMULATOR: %f\n", *accumulator);
		#endif
	}
	//===================================================================================










	
	//===================================================================================				

	int		particles_renormalization( float *weights, float accumulator, unsigned int *to_cancel_particles){
		int i;
		*to_cancel_particles=0;


		#ifdef	VERBOSE_PARTICLE_RENORMALIZATION
		float	cesso;
		cesso=0;
		printf("ACCUMULATORE: %f\n", accumulator);
		#endif

		for(i=0;i<num_particles;i++){
			#ifdef	VERBOSE_PARTICLE_RENORMALIZATION
			printf("Actual particle: %d WEIGHT %f\n", i ,*(weights+i));
			#endif
			*(weights+i)=*(weights+i)/accumulator;
			#ifdef	VERBOSE_PARTICLE_RENORMALIZATION
			cesso+=*(weights+i);
			printf("Actual particle: %d RENORMALIZED %f\n", i ,*(weights+i));
			#endif

			if( *(weights+i) < particle_death_threshold ) {
				#ifdef	VERBOSE_PARTICLE_RENORMALIZATION
				printf("THRESHOLD: %f PESO ATTUALE %f\n",particle_death_threshold,*(weights+i));
				#endif
				*(particle_to_cancel_indexes+*to_cancel_particles)=i;
				*to_cancel_particles=*to_cancel_particles+1;
				#ifdef	VERBOSE_PARTICLE_RENORMALIZATION
				printf("AGGIUNTO IN POS: %d IL PARTICLE %d\n",*to_cancel_particles, i);
				#endif
			}

		}
		#ifdef	VERBOSE_PARTICLE_RENORMALIZATION
		if(fabs(cesso-1.0)<0.0001){
			printf("LA SOMMA E'	%f\n", cesso);
		}
		else{printf("ATTENTION: NORMALIZATION FAILED. SUM is %f\n", cesso);exit(1);}
		#endif

	}
	//===================================================================================			
	
	
	//===================================================================================				
	int		particles_resampling(float *particles, unsigned int low_n_particles ){
	
		int i,j;
		unsigned	int	A_a;
		unsigned	int	A_b;

		float		*app_weights;

		#ifdef	VERBOSE_PARTICLE_RESAMPLING
		printf("PARTICLES CORROTTI: %d NUM_SOGLIA %d\n", low_n_particles, particle_min_number_2_resample);
		printf("PESI: ");
		for(i=0;i<num_particles;i++){printf("%f ", *(particles_weights+i));}
		printf("\n");
		while(getchar()!='\n');
		#endif
		if(low_n_particles > particle_min_number_2_resample-1){

			#ifdef	VERBOSE_PARTICLE_RESAMPLING
				printf("ENTRO NEL PROCESSO DI RICAMPIONAMENTO\n");
				printf("PESI: ");
				for(i=0;i<num_particles;i++){printf("%f ",*(particles_weights+i));}
				printf("\n");
				while(getchar()!='\n');
			#endif


			for(i=0;i<num_particles;i++){*(particle_best_indexes+i)=i;}

			app_weights=malloc(sizeof(float)*num_particles);

			for(i=0;i<num_particles;i++){*(app_weights+i)=*(particles_weights+i);}

			double_sort(app_weights, particle_best_indexes, 0, num_particles);

			#ifdef	VERBOSE_PARTICLE_RESAMPLING
				printf("PESI ORDINATI: ");
				for(i=0;i<num_particles;i++){printf("%f ", *(particles_weights+i));}
				printf("\n");
				printf("INDICI RELATIVI: ");
				for(i=0;i<num_particles;i++){printf("%d ", *(particle_best_indexes+i));}
				printf("\n");
				while(getchar()!='\n');
			#endif

			particle_2_rand_resample=PARTICLE_RESAMPLING_RAND_RATIO*low_n_particles;

			#ifdef	VERBOSE_PARTICLE_RESAMPLING
			printf("CAMPIONO %d PARTICLES RANDOM\n",particle_2_rand_resample);
			#endif
			A_a=particle_2_rand_resample>>1;
			#ifdef	VERBOSE_PARTICLE_RESAMPLING
			printf("CAMPIONO %d PARTICLES NELL'AMBIENTE A SU %d\n",A_a,particle_2_rand_resample);
			#endif
			A_b=particle_2_rand_resample-A_a;
			#ifdef	VERBOSE_PARTICLE_RESAMPLING
			printf("CAMPIONO %d PARTICLES NELL'AMBIENTE B SU %d\n",A_b,particle_2_rand_resample);
			#endif
			particle_2_best_resample=low_n_particles-particle_2_rand_resample;
			#ifdef	VERBOSE_PARTICLE_RESAMPLING
			printf("CAMPIONO %d PARTICLES DAI MIGLIORI SU UN TOTALE DI  %d\n",particle_2_best_resample,low_n_particles);
			while(getchar()!='\n');
			#endif


			#ifdef PARTICLE_SAMPLING_DUAL_AMBIENT
			//	sample in the A ambient
			for(i=0;i<A_a;i++){

				#ifdef	VERBOSE_PARTICLE_RESAMPLING
				printf("1- SOSTITUISCO IL CAMPIONE %d\n",*(particle_best_indexes+i));
				printf("VALORE DI i: %d\n",i);
				while(getchar()!='\n');
				#endif
				create_random_particles(particles, *(particle_best_indexes+i), 1, ambience_A_borders);
			}


			//	sample in the B ambient
			for(i=A_a;i<particle_2_rand_resample;i++){ 
				#ifdef	VERBOSE_PARTICLE_RESAMPLING
				printf("2- SOSTITUISCO IL CAMPIONE %d\n",*(particle_best_indexes+i));
				printf("VALORE DI i: %d\n",i);
				while(getchar()!='\n');
				#endif
				create_random_particles(particles, *(particle_best_indexes+i), 1, ambience_B_borders);
			}		
			#endif


			#ifdef PARTICLE_SAMPLING_UNIQUE_AMBIENT
			for(i=0;i<particle_2_rand_resample;i++){ 
				#ifdef	VERBOSE_PARTICLE_RESAMPLING
				printf("unique- SOSTITUISCO IL CAMPIONE %d\n",*(particle_best_indexes+i));
				printf("VALORE DI i: %d\n",i);
				while(getchar()!='\n');
				#endif
				create_random_particles(particles, *(particle_best_indexes+i), 1, ambience_C_borders);
			}		
			#endif



	
			//	sample in the neighborhood of best particles
			for(i=particle_2_rand_resample;i<low_n_particles;i++){
				#ifdef	VERBOSE_PARTICLE_RESAMPLING
				printf("3- ELIMINO IL PARTICLE %d\n", (*(particle_best_indexes+i)));
				printf("ELEMENTO i-esimo %d\n", i);
				printf("INDICE BEST: %d\n",(num_particles-1-((i-particle_2_rand_resample)%PARTICLE_REPRODUCTOR_NUMBER)));
				printf("LO RICAMPIONO NELL'INTORNO DEL PARTICLE %d\n", *(particle_best_indexes+4));
				while(getchar()!='\n');
				#endif

				sir_resampling( 
					particles+PARTICLE_COMPONENTS*(*(particle_best_indexes+i)),
					particles+PARTICLE_COMPONENTS*(*( particle_best_indexes+(num_particles-1-(i%PARTICLE_REPRODUCTOR_NUMBER)))), 					particle_sampling_variance);
			}
			particle_reinitialize_weights(particles_weights, num_particles);
		}
		#ifdef	VERBOSE_PARTICLE_RESAMPLING	
			printf("INDICI RELATIVI: ");
			for(i=0;i<num_particles;i++){printf("%d ", *(particle_best_indexes+i));}
			printf("\n");
			while(getchar()!='\n');
		#endif

	
	}
	//===================================================================================				
	
	//===================================================================================					
	int		particle_reinitialize_weights(float *weights, unsigned int n){

		int i;
		for(i=0;i<n;i++){*(weights+i)=1.0/(float)n;}

	}
	//===================================================================================				
	
	//===================================================================================					
	int		particles_freeze_log(char nome_file[]){

		int i;
		FILE *fp;

		#ifdef VERBOSE_particles_freeze_log
		printf("NOME: %s, PARTICELLE: %d\n", nome_file, num_particles);
		#endif
		fp=fopen(nome_file, "w+");
		for(i=0;i<num_particles;i++){

			fprintf(fp, "%f %f %f;\n", *(particles+i*PARTICLE_COMPONENTS),*(particles+i*PARTICLE_COMPONENTS+1),*(particles+i*PARTICLE_COMPONENTS+2));
		}
		fclose(fp);



	}


	//===================================================================================				
	
	
