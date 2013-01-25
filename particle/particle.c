
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
	int	point_to_line_dist( float *sensor_pose, float *line, float *distance ) {
	
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
		for(i=0;i<ENVIRONMENT_COMPONENTS;i++){	print_line(environment[i]);	}
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
	int 		int_get_weight( int estimate,  int distance, int *fitness_value){

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
		//particle_log=fopen(name_file_particle, "w+");
		particle_death_precentage=0.2;		
		particle_death_threshold=PARTICLE_EXP_TABLE_MAX/(10*num_particles);
		//particle_int_death_threshold=1.0/(10.0*(float)num_particles);		
			
		particle_min_number_2_resample=(unsigned int)(num_particles*particle_death_precentage);
		//particle_reinitialize_weights(particles_weights, num_particles);
		particle_int_reinitialize_weights(particles_int_weights, num_particles);
		particle_lookup_initialization();
		//printf("PW: %d %d %d\n", particles_int_weights[0], particles_int_weights[1], particles_int_weights[2]);while(getchar()!='\n');
		
		
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
			get_weight_from_sensors(particles+i*PARTICLE_COMPONENTS, particles_iteration_weights+i,  measure );

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
	
	//===================================================================================
	/*int	create_look_up_cos_table(float	angle, unsigned int levels, unsigned int **table, int precision){
		unsigned int i;
		float delta_alpha;
		int multiplicator;
		float app;

		multiplicator=precision;
		
		*table=malloc(sizeof(int)*levels);
		delta_alpha=angle/levels;


		printf("DELTA: %f LIVELLI: %d\n", delta_alpha, levels);

		//*((*table))=0;	
		//*((*table)+1)=1;

		for(i=0;i<levels;i++){
			*((*table)+i)=(unsigned int) ((float)cos(delta_alpha*i)*multiplicator);
		}

		return	0;
	}*/
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
int	get_cos(float	angle, int *result, int *sign){

		
		#ifdef	VERBOSE_get_cos
		printf("INPUT: %f\n", angle);
		#endif

		unsigned int	conv_angle	=	PARTICLE_RAD_2_INT(PARTICLE_GET_MODULE(angle));
		int				angle_sign	=	PARTICLE_GET_SIGN(angle);
		unsigned int 	new_angle;
		int				quadrante;

		get_quadrante(conv_angle, &quadrante, &new_angle);

		quadrante*=angle_sign;
		#ifdef	VERBOSE_get_cos
		printf("OUTPUT: LEVEL: %d QUAD: %d\n", new_angle, quadrante);
		#endif

		switch(quadrante){
			case 1:
				*result=particle_cos_lookup[new_angle];
				*sign=1;
				break;
			case 2:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];
				*sign=-1;
				break;
			case 3:
				*result=particle_cos_lookup[new_angle];
				*sign=-1;
				break;
			case 4:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=1;
				break;
			case -1:
				*result=particle_cos_lookup[new_angle];
				*sign=1;
				break;
			case -2:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=-1;
				break;
			case -3:
				*result=particle_cos_lookup[new_angle];
				*sign=-1;
				break;
			case -4:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=1;
				break;
			default:
				break;
		}
		#ifdef	VERBOSE_get_cos
		printf("COS: %f APPROX: %f errore su PROJ 200 CM: %f\n", cos(angle), (float)(*sign*(*result))/PARTICLE_MULTIPLIER,200*(cos(angle)-(float)(*sign*(*result))/PARTICLE_MULTIPLIER));	
		#endif
		return	0;
}
//===================================================================================	

//===================================================================================
int		evo_get_cos(int conv_angle, int angle_sign, int *result, int *sign){

		
		#ifdef	VERBOSE_get_cos
		printf("INPUT: %f\n", angle);
		#endif

		//unsigned int	conv_angle	=	PARTICLE_RAD_2_INT(PARTICLE_GET_MODULE(angle));
		//int				angle_sign	=	PARTICLE_GET_SIGN(angle);
		unsigned int 	new_angle;
		int				quadrante;

		get_quadrante(conv_angle, &quadrante, &new_angle);

		quadrante*=angle_sign;
		#ifdef	VERBOSE_get_cos
		printf("OUTPUT: LEVEL: %d QUAD: %d\n", new_angle, quadrante);
		#endif

		switch(quadrante){
			case 1:
				*result=particle_cos_lookup[new_angle];
				*sign=1;
				break;
			case 2:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];
				*sign=-1;
				break;
			case 3:
				*result=particle_cos_lookup[new_angle];
				*sign=-1;
				break;
			case 4:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=1;
				break;
			case -1:
				*result=particle_cos_lookup[new_angle];
				*sign=1;
				break;
			case -2:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=-1;
				break;
			case -3:
				*result=particle_cos_lookup[new_angle];
				*sign=-1;
				break;
			case -4:
				*result=particle_cos_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=1;
				break;
			default:
				break;
		}
		#ifdef	VERBOSE_get_cos
		printf("COS: %f APPROX: %f errore su PROJ 200 CM: %f\n", cos(angle), (float)(*sign*(*result))/PARTICLE_MULTIPLIER,200*(cos(angle)-(float)(*sign*(*result))/PARTICLE_MULTIPLIER));	
		#endif
		return	0;
}
//===================================================================================	



//===================================================================================	
int		get_mod_and_sign(float	angle, unsigned int *mod, int *sign){

		
		#ifdef	VERBOSE_get_sin
		printf("INPUT: %f\n", angle);
		#endif

		*mod 	=	PARTICLE_RAD_2_INT(PARTICLE_GET_MODULE(angle));
		*sign	=	PARTICLE_GET_SIGN(angle);
		return 0;
}
//===================================================================================	



//===================================================================================
int		evo_get_sin(int conv_angle, int angle_sign, int *result, int *sign){

		
		#ifdef	VERBOSE_get_sin
		printf("INPUT: %f\n", angle);
		#endif

		//unsigned int	conv_angle	=	PARTICLE_RAD_2_INT(PARTICLE_GET_MODULE(angle));
		//int				angle_sign	=	PARTICLE_GET_SIGN(angle);
		unsigned int 	new_angle;
		int				quadrante;

		get_quadrante(conv_angle, &quadrante, &new_angle);

		quadrante*=angle_sign;
		#ifdef	VERBOSE_get_sin
		printf("OUTPUT: LEVEL: %d QUAD: %d\n", new_angle, quadrante);
		#endif

		switch(quadrante){
			case 1:
				*result=particle_sin_lookup[new_angle];
				*sign=1;
				break;
			case 2:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];
				*sign=1;
				break;
			case 3:
				*result=particle_sin_lookup[new_angle];
				*sign=-1;
				break;
			case 4:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=-1;
				break;
			case -1:
				*result=particle_sin_lookup[new_angle];
				*sign=-1;
				break;
			case -2:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=-1;
				break;
			case -3:
				*result=particle_sin_lookup[new_angle];
				*sign=1;
				break;
			case -4:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=1;
				break;
			default:
				break;
		}
		#ifdef	VERBOSE_get_sin
		printf("SIN: %f APPROX: %f errore su PROJ 200 CM: %f\n", sin(angle), (float)(*sign*(*result))/PARTICLE_MULTIPLIER,200*(sin(angle)-(float)(*sign*(*result))/PARTICLE_MULTIPLIER));	
		while(getchar()!='\n');
		#endif
		return	0;
}
//===================================================================================	











//===================================================================================
int	get_sin(float	angle, int *result, int *sign){

		
		#ifdef	VERBOSE_get_sin
		printf("INPUT: %f\n", angle);
		#endif

		unsigned int	conv_angle	=	PARTICLE_RAD_2_INT(PARTICLE_GET_MODULE(angle));
		int				angle_sign	=	PARTICLE_GET_SIGN(angle);
		unsigned int 	new_angle;
		int				quadrante;

		get_quadrante(conv_angle, &quadrante, &new_angle);

		quadrante*=angle_sign;
		#ifdef	VERBOSE_get_sin
		printf("OUTPUT: LEVEL: %d QUAD: %d\n", new_angle, quadrante);
		#endif

		switch(quadrante){
			case 1:
				*result=particle_sin_lookup[new_angle];
				*sign=1;
				break;
			case 2:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];
				*sign=1;
				break;
			case 3:
				*result=particle_sin_lookup[new_angle];
				*sign=-1;
				break;
			case 4:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=-1;
				break;
			case -1:
				*result=particle_sin_lookup[new_angle];
				*sign=-1;
				break;
			case -2:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=-1;
				break;
			case -3:
				*result=particle_sin_lookup[new_angle];
				*sign=1;
				break;
			case -4:
				*result=particle_sin_lookup[PARTICLE_TRIG_LEVELS-new_angle];				
				*sign=1;
				break;
			default:
				break;
		}
		#ifdef	VERBOSE_get_sin
		printf("SIN: %f APPROX: %f errore su PROJ 200 CM: %f\n", sin(angle), (float)(*sign*(*result))/PARTICLE_MULTIPLIER,200*(sin(angle)-(float)(*sign*(*result))/PARTICLE_MULTIPLIER));	
		while(getchar()!='\n');
		#endif
		return	0;
}
//===================================================================================	



//===================================================================================
int	get_quadrante(unsigned int	converted_angle, int *quadrante, unsigned int *new_angle){

		unsigned int resto;
		//*quadrante=0;
		//resto=PARTICLE_RESTO(converted_angle,COS_LEVELS_128);
		resto=converted_angle;
		*quadrante=1;
		#ifdef	VERBOSE_get_quadrante
		printf("prima del ciclo: resto %d conv_angle: %d\n", resto, converted_angle );
		#endif
		while(resto>COS_LEVELS_128-1){
			resto=PARTICLE_RESTO(resto,COS_LEVELS_128);
			*quadrante=*quadrante+1;
		}
		*new_angle=resto;
		*quadrante%=5;
		#ifdef	VERBOSE_get_quadrante
		printf("DOPO IL CICLO: nuovo angolo %d quadrante: %d\n", *new_angle, *quadrante );
		#endif

		return	0;
}
//===================================================================================	










//===================================================================================	
int		particle_create_lookup_tables(float angle){

		unsigned int i;
		float delta_alpha;
		int multiplicator;
		float app;

		multiplicator=COS_MULTIPLIER_1024;
		delta_alpha=angle/COS_LEVELS_128;



		for(i=0;i<COS_LEVELS_128;i++){
			particle_cos_lookup[i]=(unsigned int) ((float)cos(delta_alpha*i)*multiplicator);
			particle_sin_lookup[i]=(unsigned int) ((float)sin(delta_alpha*i)*multiplicator);
		}

		return	0;






}
//===================================================================================	


//===================================================================================	
int		particle_create_lookup_sensor(){

	float delta;
	int i, k;	

	float x,y,theta;
	float ang_aux;

	unsigned int	ang_val,	sin_val,	cos_val;
	int		sign_ang,	sign_sin,	sign_cos;
	

	//delta=2*M_PI/PARTICLE_SENSOR_LEVELS;
	delta=2*M_PI/PARTICLE_SENSOR_LEVELS;

	float	appoggio;



	for(i=0;i<PARTICLE_SENSOR_LEVELS;i++){
	


		
		for(k=0;k<PARTICLE_NUM_SENSORS;k++){



			

			//get_mod_and_sign(i*delta+IR_POS[k][2], &ang_val, &sign_ang);
			get_mod_and_sign((i-PARTICLE_SENSOR_LEVELS/2)*delta, &ang_val, &sign_ang);

			
		
			#ifdef VERBOSE_particle_create_lookup_sensor
			printf("=======================================\n");
			printf("angolo convertito: %d %f\n", ang_val, (float)ang_val/127);
			printf("=======================================\n");
			#endif
			evo_get_cos(ang_val, sign_ang, &cos_val, &sign_cos);				
			evo_get_sin(ang_val, sign_ang, &sin_val, &sign_sin);
			particle_sensor_table[i][k*3]=PARTICLE_ROTOTRASLA_SUP(IR_POS[k][0],cos_val, sign_cos,IR_POS[k][1],sin_val, sign_sin);
			particle_sensor_table[i][k*3+1]=PARTICLE_ROTOTRASLA_INF(IR_POS[k][0],cos_val, sign_cos,IR_POS[k][1],sin_val, sign_sin);				
			get_mod_and_sign((i-PARTICLE_SENSOR_LEVELS/2)*delta+IR_POS[k][2], &ang_val, &sign_ang);
			particle_sensor_table[i][k*3+2]=sign_ang*(int)ang_val;




			theta=i*delta;//+IR_POS[k][2];

			x=IR_POS[k][0]*(cos(theta))-IR_POS[k][1]*(sin(theta));
			y=IR_POS[k][0]*(sin(theta))+IR_POS[k][1]*(cos(theta));

			//printf("NON SEGNATO: %d\n",(unsigned int)PARTICLE_GET_MODULE(ang_val));
			//printf("SHIFTATO: %d\n",((unsigned int)PARTICLE_GET_MODULE(ang_val)<<1));

			//printf("ANGOLOAPPROX: %f\n", ((float)(()>>PARTICLE_2_PI_2_SHIFTER))/PARTICLE_2_PI_2_MULTIPLIER);			


			
			#ifdef VERBOSE_particle_create_lookup_sensor
			printf("ANGOLO: %f\n", theta);
			printf("POSA: %f %f %f\n",  x,y,theta  );
			printf("POSA APPROX: %d %d %d   |      %f\n",  particle_sensor_table[i][k*3],particle_sensor_table[i][k*3+1],particle_sensor_table[i][k*3+2],PARTICLE_INT_2_RAD(particle_sensor_table[i][k*3+2]));
			#endif
			appoggio=PARTICLE_INT_2_RAD(particle_sensor_table[i][k*3+2]);
	
			#ifdef VERBOSE_particle_create_lookup_sensor
			printf("ERRORE: %f %f %f\n", x-particle_sensor_table[i][k*3],y-particle_sensor_table[i][k*3+1],theta-appoggio);
			while(getchar()!='\n');
			#endif





		}

		


	}
}
//===================================================================================	


//===================================================================================	
int		particle_lookup_initialization(){

		offset_pi=PARTICLE_RAD_2_INT(M_PI);
		//printf("OFFSET SENSORE: %d\n", offset_pi);

		particle_create_lookup_tables(M_PI/2);
		particle_create_lookup_sensor();
		crea_exp_table();
		return 0;
}
//===================================================================================	




//===================================================================================	
int		display_sensor_from_table(int level){

	int i;


	printf("LINEA: %d\n", level);
	for(i=0;i<5;i++){


//		printf("POSA S%d: %d %d %d  |  %f\n", i,particle_sensor_table[level][i*3],particle_sensor_table[level][i*3+1],PARTICLE_INT_2_RAD(particle_sensor_table[level][i*3+2]),PARTICLE_INT_2_RAD(particle_sensor_table[level][i*3+2])*180/M_PI);



		printf("POSA S%d: %d %d %d   |      %f\n", i, particle_sensor_table[level][i*3+0],particle_sensor_table[level][i*3+1],particle_sensor_table[level][2],PARTICLE_INT_2_RAD(particle_sensor_table[level][i*3+2])*180/M_PI);






	}



}
//===================================================================================	



//===================================================================================
int		int_point_to_line_dist( int rob_bearing_level) {
	
		int	p1_prime[2];
		int   	p2_prime[2];
		int	p1_prime_aux[2];
		int   	p2_prime_aux[2];

		float	angle;
		int	rho_prime;
		int i,j,k, ii;


		unsigned int 	cos_val, sin_val, mod_rho_prime;
		int 		sign_sin, sign_cos, sign_rho_prime;

		unsigned int line_mod[ENVIRONMENT_COMPONENTS];
		unsigned int line_sign[ENVIRONMENT_COMPONENTS];

		int 		alpha_prime, sign_alpha_prime, sign_cos_alpha_prime;
		unsigned int	mod_alpha_prime, cos_alpha_prime;




		unsigned int 		dist_aux;
		#ifdef VERBOSE_int_point_to_line_dist
		float angolo;
		int   ang_aux;
		ang_aux=rob_bearing_level-180;
		float		delta_x, delta_y, theta;
		#endif


		#ifdef VERBOSE_int_point_to_line_dist
		angolo=PARTICLE_ROBOT_TABLE_TO_BEARING(ang_aux);
		printf("INPUT: %d INIDICE TABELLA: %d ANGOLO CORRISPONDENTE: %f\n",rob_bearing_level, ang_aux, angolo);
		while(getchar()!='\n');
		#endif




		for(i=0;i<ENVIRONMENT_COMPONENTS;i++){
			//printf("LINEA: %d COEFF: %f   \n",i,*(environment[i]+LINE_ALPHA));
			get_mod_and_sign(*(environment[i]+LINE_ALPHA), &(line_mod[i]), &(line_sign[i]));
		}

		


		//	FOR EVERY SENSOR ...
		for(i=0;i<PARTICLE_NUM_SENSORS;i++){

			for(k=0;k<num_particles;k++){particle_iteration_distances[k][i]=300;}


			gettimeofday(&particle_start_time,NULL);

			evo_get_cos(
				PARTICLE_GET_MODULE(particle_sensor_table[rob_bearing_level][i*3+2]), 
				PARTICLE_GET_SIGN(particle_sensor_table[rob_bearing_level][i*3+2]), 
				&cos_val, 
				&sign_cos
			);				
			evo_get_sin(
				PARTICLE_GET_MODULE(particle_sensor_table[rob_bearing_level][i*3+2]), 
				PARTICLE_GET_SIGN(particle_sensor_table[rob_bearing_level][i*3+2]), 
				&sin_val, 
				&sign_sin
			);



			// FOR EVERY WALL ...

			for(j=0;j<ENVIRONMENT_COMPONENTS;j++){

				#ifdef VERBOSE_int_point_to_line_dist
				printf("ORIGINALE -----------------------------------------\n");
				print_line(environment[j]);		
				printf("P1: %f %f\n",*(environment[j]+LINE_P1_X),*(environment[j]+LINE_P1_Y));
				printf("P2: %f %f\n",*(environment[j]+LINE_P2_X),*(environment[j]+LINE_P2_Y));
				printf("\n");
				printf("==========================================\n");
				delta_x=(float)*(environment[j]+LINE_P1_X)-particle_sensor_table[rob_bearing_level][i*3];
				delta_y=(float)*(environment[j]+LINE_P1_Y)-particle_sensor_table[rob_bearing_level][i*3+1];
				theta=-((float)PARTICLE_INT_2_RAD(particle_sensor_table[rob_bearing_level][i*3+2]));
				printf("ESATTO P1: %f %f\n",delta_x*cos(theta)-delta_y*sin(theta), delta_x*sin(theta)+delta_y*cos(theta));
				delta_x=(float)*(environment[j]+LINE_P2_X)-particle_sensor_table[rob_bearing_level][i*3];
				delta_y=(float)*(environment[j]+LINE_P2_Y)-particle_sensor_table[rob_bearing_level][i*3+1];
				theta=-((float)PARTICLE_INT_2_RAD(particle_sensor_table[rob_bearing_level][i*3+2]));
				printf("ESATTO P2: %f %f\n",delta_x*cos(theta)-delta_y*sin(theta), delta_x*sin(theta)+delta_y*cos(theta));
				printf("==========================================\n");
				while(getchar()!='\n');
				#endif
	


				// the wall is represented in the frame oriented like the current sensor bearing, centeredat origin

				p1_prime[0]=PARTICLE_ROTOTRASLA_SUP(
					(int)(*(environment[j]+LINE_P1_X)-particle_sensor_table[rob_bearing_level][i*3]),
					cos_val,
					sign_cos,
					(int)(*(environment[j]+LINE_P1_Y)-particle_sensor_table[rob_bearing_level][i*3+1]),
					sin_val, 
					-sign_sin	
				);
				p1_prime[1]=PARTICLE_ROTOTRASLA_INF(
					(int)(*(environment[j]+LINE_P1_X)-particle_sensor_table[rob_bearing_level][i*3]),
					cos_val, 
					sign_cos,
					(int)(*(environment[j]+LINE_P1_Y)-particle_sensor_table[rob_bearing_level][i*3+1]),
					sin_val, 
					-sign_sin
				);	
	
				p2_prime[0]=PARTICLE_ROTOTRASLA_SUP(
					(int)(*(environment[j]+LINE_P2_X)-particle_sensor_table[rob_bearing_level][i*3]),
					cos_val, 
					sign_cos,
					(int)(*(environment[j]+LINE_P2_Y)-particle_sensor_table[rob_bearing_level][i*3+1]),
					sin_val, 
					-sign_sin
				);


				p2_prime[1]=PARTICLE_ROTOTRASLA_INF(
					(int)(*(environment[j]+LINE_P2_X)-particle_sensor_table[rob_bearing_level][i*3]),
					cos_val, 
					sign_cos,
					(int)(*(environment[j]+LINE_P2_Y)-particle_sensor_table[rob_bearing_level][i*3+1]),
					sin_val, 
					-sign_sin
				);	

				#ifdef VERBOSE_int_point_to_line_dist
				printf("\n");
				printf("****************************************************\n");
				printf("APPROSSIMATO P1: %d %d\n",p1_prime[0],p1_prime[1]);
				printf("APPROSSIMATO P2: %d %d\n",p2_prime[0],p2_prime[1]);
				printf("****************************************************\n");
				while(getchar()!='\n');
				#endif


				


				// the frame i translated to the current particle position

				for(k=0;k<num_particles;k++){


					#ifdef VERBOSE_int_point_to_line_dist
					printf("\n");
					printf("POSA DEL PARTICLE %d: %f %f %f:\n",k,*(particles+k*3),*(particles+k*3+1),*(particles+k*3+2));
					printf("\n");
					while(getchar()!='\n');
					#endif


					p1_prime_aux[0]=p1_prime[0]-*(particles+k*3);		
					p1_prime_aux[1]=p1_prime[1]-*(particles+k*3+1);

					p2_prime_aux[0]=p2_prime[0]-*(particles+k*3);		
					p2_prime_aux[1]=p2_prime[1]-*(particles+k*3+1);

					#ifdef VERBOSE_int_point_to_line_dist
					printf("\n");
					printf("PUNTO 1 ESPRESSO NEL FRAME DEL PARTICLE: %d %d\n", p1_prime_aux[0],p1_prime_aux[1]);
					printf("PUNTO 2 ESPRESSO NEL FRAME DEL PARTICLE: %d %d\n", p2_prime_aux[0],p2_prime_aux[1]);
					while(getchar()!='\n');
					#endif


					// 	the wall is detected by the current sensor under the following conditions:
					//	almost one point has the x-coordinate greater than zero
					//	y-coordinate of the points are opposite each other


					if(	(p1_prime_aux[0]>0 || p2_prime_aux[0]>0) &&  
						(PARTICLE_GET_SIGN(p1_prime_aux[1]) != PARTICLE_GET_SIGN(p2_prime_aux[1]))){
	

	
						//	vertical line...
						if(*(environment[j]+LINE_TYPE)==V){

							rho_prime=*(environment[j]+LINE_P1_X)-particle_sensor_table[rob_bearing_level][i*3];

							#ifdef VERBOSE_int_point_to_line_dist
							A_CAPO();
							INTERLINEA();
							printf("LINEA VERTICALE: rho passa da %d a %d\n", *(environment[j]+LINE_P1_X),rho_prime);
							INTERLINEA();
							A_CAPO();
							#endif

							
						}
						//	horizontal line...
						if(*(environment[j]+LINE_TYPE)==H){
							rho_prime=*(environment[j]+LINE_P1_Y)-particle_sensor_table[rob_bearing_level][i*3+1];
							#ifdef VERBOSE_int_point_to_line_dist
							A_CAPO();
							INTERLINEA();
							printf("LINEA ORIZZONTALE: rho passa da %d a %d\n", *(environment[j]+LINE_P1_Y),rho_prime);
							INTERLINEA();
							A_CAPO();
							#endif


						}


						alpha_prime=((int)line_sign[j]*line_mod[j])-particle_sensor_table[rob_bearing_level][i*3+2];
						
						
						evo_get_cos(
							PARTICLE_GET_MODULE(alpha_prime), 
							PARTICLE_GET_SIGN(alpha_prime), 
							&cos_alpha_prime, 
							&sign_cos_alpha_prime
						);

						dist_aux=((unsigned int)PARTICLE_GET_MODULE(rho_prime)<<10)/cos_alpha_prime;
						#ifdef VERBOSE_int_point_to_line_dist						
						printf("====================================\n");
						printf("DISTANZA STIMATA: %d\n",dist_aux);
						A_CAPO();
						printf("LA DISTANZA PRECEDENTE ERA: %d\n",particle_iteration_distances[k][i] );
						A_CAPO();
						#endif

						if(dist_aux< particle_iteration_distances[k][i]){particle_iteration_distances[k][i]=dist_aux;}
						//particle_sensor_distances[k][j]=
						#ifdef VERBOSE_int_point_to_line_dist			
						A_CAPO();			
						printf("LA NUOVA DISTAZA E': %d\n", particle_iteration_distances[k][i]);
						INTERLINEA();
						while(getchar()!='\n');
						#endif

					}
					#ifdef VERBOSE_int_point_to_line_dist
					else{INTERLINEA();printf("NESSUNA INTERSEZIONE\n");INTERLINEA();while(getchar()!='\n');}
					#endif

				}

			}
			gettimeofday(&particle_end_time,NULL);
			particle_get_elapsed_time(&particle_start_time, &particle_end_time, &particle_time);
			printf("*********** TEMPO %d ******************\n", particle_time);

		}

		#ifdef VERBOSE_int_point_to_line_dist					
		INTERLINEA();
		for(k=0;k<num_particles;k++){
			printf("DISTANZA PARTICLE %d: \t ", k);
			for(i=0;i<PARTICLE_NUM_SENSORS;i++){
				printf("%d %c ",particle_iteration_distances[k][i], 0x09);
			}	
			A_CAPO();
		}
		INTERLINEA();
		#endif

			


		return 0;
}
//================================================================================================================================




//================================================================================================================================


int		crea_exp_table(){
		
		int i;
		float aux;
		

		for(i=0;i<PARTICLE_FITNESS_RANGE_THRESHOLD;i++){ 
			aux=exp(-PARTICLE_FITNESS_CONSTANT_SHORT*(i*i))*PARTICLE_EXP_TABLE_MAX;
			particle_fitness_table[i]=(unsigned int)aux;
			#ifdef VERBOSE_crea_exp_table
			printf("INPUT: %d\n", i);
			printf("ESATTO: %f    APPROSSIMATO: %f\n", aux, (float)particle_fitness_table[i]);
			while(getchar()!='\n');
			#endif
		}


		

		for(i=PARTICLE_FITNESS_RANGE_THRESHOLD;i<PARTICLE_EXP_TABLE_CARDINALITY;i++){
			aux=exp(-PARTICLE_FITNESS_CONSTANT_LONG*(float)(i*i))*PARTICLE_EXP_TABLE_MAX;
			particle_fitness_table[i]=(unsigned int)aux;
			#ifdef VERBOSE_crea_exp_table
			printf("INPUT: %d\n", i);
			printf("ESATTO: %f    APPROSSIMATO: %f\n", aux, (float)particle_fitness_table[i]);
			while(getchar()!='\n');
			#endif

		}
}
//================================================================================================================================



//================================================================================================================================
int		particle_int_update_weights(int *accumulator){

		int i;
		for(i=0;i<num_particles;i++){

			//printf("w: %d i_w: %d\n",particles_int_weights[i],particle_iteration_weights[i]);
			particle_weights[i]=(particles_int_weights[i]*particle_iteration_weights[i])>>PARTICLE_EXP_TABLE_EQUIVALENT_SHIFT;
			*accumulator+=particles_int_weights[i];
		}
}
//================================================================================================================================






//================================================================================================================================
int		particle_int_get_iteration_weights(unsigned int real_distances[]){


		int	i,j,ii;
		unsigned int 	errore;
		unsigned int aux[PARTICLE_NUM_SENSORS];




		

		for(i=0;i<num_particles;i++){

			#ifdef	VERBOSE_particle_int_get_weights	
				INTERLINEA();
				printf("DISTANZE SIMULATE: ");
				for(ii=0;ii<PARTICLE_NUM_SENSORS;ii++){
					printf(" %d ", particle_iteration_distances[i][ii]);
				}
				A_CAPO();
				INTERLINEA();
				A_CAPO();

				INTERLINEA();
				printf("DISTANZE REALI: ");
				for(ii=0;ii<PARTICLE_NUM_SENSORS;ii++){
					printf(" %d ", real_distances[ii]);
				}
				A_CAPO();
				INTERLINEA();
				A_CAPO();
				while(getchar()!='\n');
			#endif


			for(j=0;j<PARTICLE_NUM_SENSORS;j++){
		



				if(particle_iteration_distances[i][j]>real_distances[j]){errore=particle_iteration_distances[i][j]-real_distances[j];}
				else{errore=(real_distances[j]<<1)-(particle_iteration_distances[i][j]<<1);}

				#ifdef	VERBOSE_particle_int_get_weights
				printf("IN ESAME: sim %d reale %d\n", particle_iteration_distances[i][j], real_distances[j]);
				printf("L'ERRORE e' %d\n", errore);
				#endif

				if((errore)< PARTICLE_EXP_TABLE_CARDINALITY){
					particle_iteration_weights[i]+=PARTICLE_INT_GET_DISTANCE(errore);
				}

				#ifdef	VERBOSE_particle_int_get_weights
				printf("PESO: %d    PESO ITERAZIONE: %d\n",PARTICLE_INT_GET_DISTANCE(errore),particle_iteration_weights[i] );
				while(getchar()!='\n');
				#endif				


			}
			
			particle_iteration_weights[i]/=5;
			#ifdef	VERBOSE_particle_int_get_weights
			printf("PESO FINE ITERAZIONE: %d\n",particle_iteration_weights[i]);
			#endif				



		}

		return 0;

}

//================================================================================================================================



//===================================================================================				
	int		particle_int_resampling(unsigned int low_n_particles ){
	
		int i,j;
		unsigned	int	A_a;
		unsigned	int	A_b;

		//float		*app_weights;

		if(low_n_particles > particle_min_number_2_resample-1){


			for(i=0;i<num_particles;i++){particle_int_best_indexes[i]=i;}

			double_sort(particles_int_weights, particle_best_indexes, 0, num_particles);
			particle_2_rand_resample=PARTICLE_RESAMPLING_RAND_RATIO*low_n_particles;

			A_a=particle_2_rand_resample>>1;
			A_b=particle_2_rand_resample-A_a;
			particle_2_best_resample=low_n_particles-particle_2_rand_resample;


			#ifdef PARTICLE_SAMPLING_DUAL_AMBIENT
			//	sample in the A ambient
			for(i=0;i<A_a;i++){							particle_int_create_random_particles(particles, *(particle_best_indexes+i), 1, ambience_A_borders);}
			//	sample in the B ambient
			for(i=A_a;i<particle_2_rand_resample;i++){	particle_int_create_random_particles(particles, *(particle_best_indexes+i), 1, ambience_B_borders);}		
			#endif


			#ifdef PARTICLE_SAMPLING_UNIQUE_AMBIENT
			for(i=0;i<particle_2_rand_resample;i++){particle_int_create_random_particles(particles, *(particle_best_indexes+i), 1, ambience_C_borders);}		
			#endif

			//	sample in the neighborhood of best particles
			for(i=particle_2_rand_resample;i<low_n_particles;i++){
				sir_resampling( 
					particles+PARTICLE_COMPONENTS*(*(particle_best_indexes+i)),
					particles+PARTICLE_COMPONENTS*(*( particle_best_indexes+(num_particles-1-(i%PARTICLE_REPRODUCTOR_NUMBER)))), 					particle_sampling_variance);
			}
			particle_int_reinitialize_weights(particles_int_weights, num_particles);
		}

	
	}
	//===================================================================================				




	//===================================================================================	
	int		particle_int_create_random_particles(float *particles, int	init_index, int N, float *environment){

		int i;
		float app;
		//srand(3);
		unsigned int	width;
		unsigned int	height;
		unsigned int	offset_x;
		unsigned int	offset_y;		
		unsigned int 	appoggio;
		//----------------------------------------------------------------------------
		width=	(unsigned int)(*(environment+1)-*(environment));
		height=	(unsigned int)(*(environment+3)-*(environment+2));
		offset_x=*(environment);
		offset_y=*(environment+2);
		//----------------------------------------------------------------------------	

		for(i=0;i<N;i++){


			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_X)=((rand()/(unsigned int)(RAND_MAX>>7))*height)>>7+offset_x;
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_Y)=((rand()/(unsigned int)(RAND_MAX>>7))*height)>>7+offset_x;
			#ifdef	BEARING_SAMPLING
			//*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA)=M_PI*(((float)rand()/RAND_MAX)*2-1);
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA)=(float)	PARTICLE_INT_RAND_NUMBER(1024)/325.95;//325.9493234522017;
			#endif
			//printf("PARTICLE %d: %f %f %f\n",i,*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_X),*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_Y),*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA));

			if(*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA)>M_PI || 
			*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA) < -M_PI){printf("errore\n"); printf("ANG: %f\n",*(particles+(init_index+i)*PARTICLE_COMPONENTS+PARTICLE_THETA) );exit(1);}			


		}
		
	}
	//===================================================================================	


	//===================================================================================	
	int		particle_int_sir_resampling( float *p, float *g, int *variance){
	

		*(p+PARTICLE_X)=		*(g+PARTICLE_X)			+			PARTICLE_INT_RAND_NUMBER(*variance);
		*(p+PARTICLE_Y)=		*(g+PARTICLE_Y)			+			PARTICLE_INT_RAND_NUMBER(*(variance+PARTICLE_Y));
		*(p+PARTICLE_THETA)=	*(g+PARTICLE_THETA)	+(float)	PARTICLE_INT_RAND_NUMBER(*(variance+PARTICLE_THETA))/1000;
		printf("NUOVO PARTICLE: %f %f %f\n", *(p+PARTICLE_X),*(p+PARTICLE_Y),*(p+PARTICLE_THETA));
	}
	//===================================================================================		




	//===================================================================================					
	int		particle_int_reinitialize_weights(int *weights, unsigned int n){

		int i;
		for(i=0;i<n;i++){

				*(weights+i)=PARTICLE_EXP_TABLE_MAX/n;
				//printf("peso: %d   n: %d\n", *(weights+i), n); while(getchar()!='\n');

		}

	}
	//===================================================================================				




	//===================================================================================				
	int		particle_int_renormalization(int accumulator, unsigned int *to_cancel_particles){
		int i;
		*to_cancel_particles=0;

		for(i=0;i<num_particles;i++){

			//printf("p %d %d\n",i,particles_int_weights[0]);
			particle_weights[i]=(particles_int_weights[i]/accumulator);
			//printf("step1\n");
			if( particles_int_weights[i] < particle_int_death_threshold ) {
				//printf("step2\n");
				*(particle_to_cancel_indexes+*to_cancel_particles)=i;
				//printf("step3\n");
				*to_cancel_particles=*to_cancel_particles+1;
				//printf("step4\n");
			}
		}
	}
	//===================================================================================			






	//===================================================================================		
	int		particle_int_evolution( int robot_bearing, int *accumulator, unsigned int *measure){
	
		int i;
		//particles_prediction(particles, num_particles, u,w, T);
		*accumulator=0;
		int_point_to_line_dist(robot_bearing);
		//printf("step1\n");while(getchar()!='\n');
		particle_int_get_iteration_weights(measure);
		//printf("step2\n");while(getchar()!='\n');
		particle_int_update_weights(accumulator);
		//printf("step3\n");while(getchar()!='\n');
		//if(isnan(*(particles_weights+i))!=0){*(particles_weights+i)=0.0;}

		
	}
	//===================================================================================


	//===================================================================================
	int particle_get_elapsed_time(struct timeval *start_time_pt, struct timeval *end_time_pt, long *elapsed_utime)
	{
	
		//long elapsed_utime;    /* elapsed time in microseconds */
		//long elapsed_mtime;    /* elapsed time in milliseconds */
		long elapsed_seconds;	 /* diff between seconds counter */
		long elapsed_useconds;	 /* diff between microseconds counter */
		
		elapsed_seconds  = end_time_pt->tv_sec  - start_time_pt->tv_sec;
		elapsed_useconds = end_time_pt->tv_usec - start_time_pt->tv_usec;
		
		*elapsed_utime = (elapsed_seconds) * 1000000 + elapsed_useconds;
		//elapsed_mtime = ((elapsed_seconds) * 1000 + elapsed_useconds/1000.0) + 0.5;
		
		//printf("%s : Elapsed time = %ld microseconds\n", step_name, *elapsed_utime);
		//printf("%s : Elapsed time = %ld milliseconds\n", step_name, elapsed_mtime);
		
		return 0;	
	}
	//===================================================================================

