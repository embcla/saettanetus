#include "probstavoid.h"

int sign_int(int argument)
{
  int result=(argument<0)?-1:1;
  return result;
}

void apply_fuzzy_horizon_algorithm(float *v_ref,float *w_ref,fuzzy_horizon_result *result)
{
	/*Index variable*/
	int i,sign;
	/*Arrays initialization*/
	for(i=0; i<HORIZON_ELEMENTS; i++) {
		attractive_horizon[i]=0;
		not_attractive_horizon[i]=0;
		repulsive_horizon[i]=0;
		//kinematic_horizon[i]=0;
		not_kinematic_horizon[i]=0;
		union_horizon[i]=0;
	}
	/*--------------*/

	/*Index of the minimum element*/
	uint8_ min_index;
	/*Minimum element*/
	uint8_ min_element;
	/*Boolean value to understand if the robot is blocked*/	
	uint8_ stuck=1;
	//Module init
	//init_probstavoid_module();
	//Selecting the sensor
	#ifdef USE_URG_LASER
        create_repulsive_horizon_laser();
	#endif
	#ifdef USE_IR
	create_repulsive_horizon_ir();
	#endif
	//--------------------

	//Create attractive horizon
	create_attractive_horizon(*w_ref);
	//Resulting horizon
	compute_resulting_horizon();
	//Inclusion Horizon	
	compute_inclusion();
	//Computing the maximum
	
	//Inizializing variables for max computation
	min_index=(HORIZON_ELEMENTS>>1);
	sign=1;
	min_element=inclusion_horizon[min_index];
	//------------------------------------------
	
	for(i=1;(i<HORIZON_ELEMENTS>>1);i++)
	{	
				if(inclusion_horizon[(HORIZON_ELEMENTS>>1)-i]<inclusion_horizon[min_index])
				{
					min_index=(HORIZON_ELEMENTS>>1)-i;
					min_element=inclusion_horizon[(HORIZON_ELEMENTS>>1)-i];
					sign=sign_int(min_index-(HORIZON_ELEMENTS>>1));	
				}
				if(inclusion_horizon[(HORIZON_ELEMENTS>>1)+i]<inclusion_horizon[min_index])
                                {
                                        min_index=(HORIZON_ELEMENTS>>1)+i;
                                        min_element=inclusion_horizon[(HORIZON_ELEMENTS>>1)+i];
                                        sign=sign_int(min_index-(HORIZON_ELEMENTS>>1));
                                }

		//Stuck if all the elements in repulsive horizon are > Threshold
		//printf("[%d]: %d\n",i,repulsive_horizon[i]);
	}

	/*The minimum element has to be negate in order to scale the linear speed*/	
	min_element=~min_element;
	
	/*printf("Min index %d\n",min_index);
	printf("Min element %d\n",min_element);
	printf("nMin element %d\n",(not_min_element));
	*/
	#ifdef LOG_TO_FILE
	for(i=0;(i<HORIZON_ELEMENTS-1);i++)
        {
		//printf("Attractive [%d] : %u\n",i,attractive_horizon[i]);

        
                fprintf(attractive_horizon_file,"%u ",attractive_horizon[i]);
                fprintf(not_attractive_horizon_file,"%u ",not_attractive_horizon[i]);
                fprintf(repulsive_horizon_file,"%u ",repulsive_horizon[i]);
                fprintf(kinematic_horizon_file,"%u ",kinematic_horizon[i]);
                fprintf(not_kinematic_horizon_file,"%u ",not_kinematic_horizon[i]);
                fprintf(inclusion_horizon_file,"%u ",inclusion_horizon[i]);
                fprintf(union_horizon_file,"%u ",union_horizon[i]);
	
		stuck=stuck&(repulsive_horizon[i]>DANGER_THRESHOLD);
	}
	 
        if(stuck)
                result->stuck=1;

                fprintf(attractive_horizon_file,"; ",attractive_horizon[i]);
                fprintf(not_attractive_horizon_file,"; ",not_attractive_horizon[i]);
                fprintf(repulsive_horizon_file,"; ",repulsive_horizon[i]);
                fprintf(kinematic_horizon_file,"; ",kinematic_horizon[i]);
                fprintf(not_kinematic_horizon_file,"; ",not_kinematic_horizon[i]);
                fprintf(inclusion_horizon_file,"; ",inclusion_horizon[i]);
                fprintf(union_horizon_file,"; ",union_horizon[i]);
        #endif

	/*Getting results (2.0 is used to speed up the robot turning)*/
        //result->w_ref_hor_fuz=((min_index-sign*(roundl((ROBOT_ENCUMBRANCE_RADIANS*HORIZON_ELEMENTS)/M_PI)>>1)-(HORIZON_ELEMENTS>>1))*2.0*M_PI)/HORIZON_ELEMENTS;
	result->w_ref_hor_fuz=((min_index-(HORIZON_ELEMENTS>>1))*2.0*M_PI)/HORIZON_ELEMENTS;
	
/*-0.5 is used to speed down the linear speed of the robot*/
	result->v_ref_hor_fuz=(*v_ref)*(min_element*1.0/MAX_INT_FUZZY_MEMBERSHIP);
	/*---------------*/
	//close_probstavoid_module();
}

void create_not_robot_encumbrance(uint8_ shift,uint8_ *not_robot_encumbrance,int index)
{
	int i;
	
	for(i=0;i<HORIZON_ELEMENTS;i++)
	{
		not_robot_encumbrance[i]=MAX_INT_FUZZY_MEMBERSHIP;
		if(i>=index-(shift>>1) && i<=index+(shift>>1))		
		not_robot_encumbrance[i]=0;
	}

}

void compute_inclusion()
{
	/*Index variables*/
	int i,j;
	/*Horizon representing not robot occupancy*/
	uint8_ not_robot_encumbrance[HORIZON_ELEMENTS];
	/*Temporary array*/
	uint8_ temp[HORIZON_ELEMENTS];
	/*Max variables*/
	uint8_ max_index;
	uint8_ max_element;	
	/*-------------*/
	/*Robot occupancy cells (in the horizon)*/
	uint8_ occupancy=roundl((ROBOT_ENCUMBRANCE_RADIANS*HORIZON_ELEMENTS)/M_PI);
	
	for(i=0;i<HORIZON_ELEMENTS;i++)
	{
		max_index=0;
		max_element=0;
		/*Create occupancy shifting on all the horizon*/
		create_not_robot_encumbrance(occupancy,not_robot_encumbrance,i);
		for(j=i-(occupancy>>1);j<=i+(occupancy>>1);j++)
		{
			if(j>=0 && j<HORIZON_ELEMENTS){
			/*Lukasiewicz when A=~A e B=~B*/
			//temp[j]=min(MAX_INT_FUZZY_MEMBERSHIP,MAX_INT_FUZZY_MEMBERSHIP-union_horizon[j]+not_robot_encumbrance[j]);	
			/*Taking the maximum (I'm working with the inverse horizon (as not encumbrance!))*/			
			if(union_horizon[j]>max_element)
			{
				max_element=union_horizon[j];
				max_index=j;
			}
			}
		}
		inclusion_horizon[i]=max_element;	
		//printf("inclusion[%d]: %d\n",i,inclusion_horizon[i]);
	}
	
}

void compute_resulting_horizon()
{
	/*Cfr. slides (link on .h header file)*/
	int i;
	for(i=HORIZON_ELEMENTS-1;i>=0;i--)
	{
			not_attractive_horizon[i]=~attractive_horizon[i];
			not_kinematic_horizon[i]=~kinematic_horizon[i];
			union_horizon[i]=max(not_attractive_horizon[i],not_kinematic_horizon[i]);
			union_horizon[i]=max(union_horizon[i],repulsive_horizon[i]);
			

	}
}

void compute_horizon_index(int *horizon_destination_element,float destination)
{
	/*Variable into which store the shift derived by the direction of the goal*/
	int shift=0;
	/*Conversion (I choose to use asin(sin) in order to obtain a peculiar saturation effect [-PI/2,PI/2])*/
	float converted=asin(sin(destination));
	/*Sign of the shift*/
	int sign=(*horizon_destination_element<0)?-1:1;
	/*The shifting index is given by the multiplication of the sign and the roundling to the nearest integer of the converted value of the angle*/
	shift=sign*roundl((converted*HORIZON_ELEMENTS)/M_PI);
	/*The shifting is applied taking into account the frontal direction (mid index)*/
	*horizon_destination_element=(HORIZON_ELEMENTS>>1)+shift;
}

void init_probstavoid_module() {
	/*File opening*/
	#ifdef LOG_TO_FILE
	attractive_horizon_file=fopen("../../probstavoid/probstavoid_logs/attractive_horizon_file.txt","a+");
	not_attractive_horizon_file=fopen("../../probstavoid/probstavoid_logs/not_attractive_horizon_file.txt","a+");
	repulsive_horizon_file=fopen("../../probstavoid/probstavoid_logs/repulsive_horizon_file.txt","a+");
	kinematic_horizon_file=fopen("../../probstavoid/probstavoid_logs/kinematic_horizon_file.txt","a+");
	not_kinematic_horizon_file=fopen("../../probstavoid/probstavoid_logs/not_kinematic_horizon_file.txt","a+");
	inclusion_horizon_file=fopen("../../probstavoid/probstavoid_logs/inclusion_horizon_file.txt","a+");
	union_horizon_file=fopen("../../probstavoid/probstavoid_logs/union_horizon_file.txt","a+");
	#endif
	/*------------*/
	/*Diagonal ir sensors shift in the current horizon*/	
	diagonal_ir_shift=roundl(DIAGONAL_SENSOR_SHIFT_RADIANS*HORIZON_ELEMENTS)/M_PI;
	/*Sensor's triangle base*/
	triangle_base_degrees=roundl(SENSOR_TRIANGLE_BASE_RADIANS*HORIZON_ELEMENTS)/M_PI;
	
	//Create kinematic horizon
        create_kinematic_horizon();

}

/*This functions aim to create a triangular belonging function
*ATTENTION: The triangle is not perfectly simmetric because of the number of HORIZON_ELEMENTS is preferable odd. 
*/
void create_triangle(int base,int index,uint8_ max_triangle_value,uint8_ *horizon) {
	/*Index variable*/
	int i=0;
	/*Auxiliary variable (to store the current value of the triangle's sample)*/
	uint8_ aux=0;
	/*Triangle slope*/
	int slope=roundl(max_triangle_value/(base>>1));
        /*From the last to the first triangle element*/
	for(i=(index+(base>>1)); i>(index-(base>>1)); i--) {
		if(i>=0 && i<HORIZON_ELEMENTS){
			horizon[i]=max(aux,horizon[i]);
		if(i==index) {
			horizon[i]=max(horizon[i],max_triangle_value);
			slope=-1.0*slope;
		}
		}
		aux=aux+slope;
	}
}

/*A linear function was chosen in order to represent the membership function of the reading
 * to the set "danger (repulsion)"
 * Solving the equation of a line passing between two points:
 * y=(x*(y_2-y_1)-y_2x_1-y_1x_1)/(x_2-x_1)
 * Actually, repulsion is made by ~(attraction)!!!
 * */
void create_repulsive_horizon_ir() {
  
	
	float multiplier=MAX_INT_FUZZY_MEMBERSHIP/(SENSOR_MAX_THRESHOLD-SENSOR_MIN_THRESHOLD);	
	/*Array representing the filtered measures(see function comments above)*/
	float filtered_readings[NUM_IR];
	/*Index variable*/
	int i=0;
	
	uint8_ values[NUM_IR];

	/*Saturation*/
	filtered_readings[0]=(*(ir->range)<SENSOR_MIN_THRESHOLD)?SENSOR_MIN_THRESHOLD:*(ir->range);
	filtered_readings[0]=(*(ir->range)>SENSOR_MAX_THRESHOLD)?SENSOR_MAX_THRESHOLD:*(ir->range);
	
	filtered_readings[1]=(*(ir->range+2)<SENSOR_MIN_THRESHOLD)?SENSOR_MIN_THRESHOLD:*(ir->range+2);
	filtered_readings[1]=(*(ir->range+2)>SENSOR_MAX_THRESHOLD)?SENSOR_MAX_THRESHOLD:*(ir->range+2);
	
	filtered_readings[2]=(*(ir->range+4)<SENSOR_MIN_THRESHOLD)?SENSOR_MIN_THRESHOLD:*(ir->range+4);
	filtered_readings[2]=(*(ir->range+4)>SENSOR_MAX_THRESHOLD)?SENSOR_MAX_THRESHOLD:*(ir->range+4);

	filtered_readings[3]=(*(ir->range+3)<SENSOR_MIN_THRESHOLD)?SENSOR_MIN_THRESHOLD:*(ir->range+3);
	filtered_readings[3]=(*(ir->range+3)>SENSOR_MAX_THRESHOLD)?SENSOR_MAX_THRESHOLD:*(ir->range+3);

	filtered_readings[4]=(*(ir->range+1)<SENSOR_MIN_THRESHOLD)?SENSOR_MIN_THRESHOLD:*(ir->range+1);
	filtered_readings[4]=(*(ir->range+1)>SENSOR_MAX_THRESHOLD)?SENSOR_MAX_THRESHOLD:*(ir->range+1);
		
	/*---------------------------------------------------------------------*/

	/*Proportion and negation*/
	for(i=0; i<NUM_IR; i++) {
		values[i]=(filtered_readings[i]-SENSOR_MIN_THRESHOLD)*multiplier;
		values[i]=~values[i];
	}
	
	/*Modelling sensor readings*/
	create_triangle(triangle_base_degrees,HORIZON_ELEMENTS>>1,values[2],repulsive_horizon);
	create_triangle(triangle_base_degrees,HORIZON_ELEMENTS-1,values[0],repulsive_horizon);
	create_triangle(triangle_base_degrees,0,values[4],repulsive_horizon);
	create_triangle(triangle_base_degrees,(HORIZON_ELEMENTS>>1)-diagonal_ir_shift,values[3],repulsive_horizon);
	create_triangle(triangle_base_degrees,(HORIZON_ELEMENTS>>1)+diagonal_ir_shift,values[1],repulsive_horizon);
	/*-----------------------------------*/

}

/*The same linear function of the IR sensors was used with the laser range finders*/
void create_repulsive_horizon_laser() {
	float multiplier=MAX_INT_FUZZY_MEMBERSHIP/(SENSOR_MAX_THRESHOLD-SENSOR_MIN_THRESHOLD);
	/*Data read from the laser range finder*/
	int maximum_laser_data=get_data_max();
	int watchdog=HORIZON_ELEMENTS>>1;
	/*Laser step computation. IGNORED_LASER SAMPLES is multiplied by to in order to ignore 30° both on
	 * the right side and the left side.
	 * Alert!! THE HORIZON IS INTENDED TO BE 180° WIDE.
	 * */
	laser_step=(maximum_laser_data-(IGNORED_LASER_SAMPLES<<1))/HORIZON_ELEMENTS;
	long *data=malloc(sizeof(long)*maximum_laser_data);
	int i;
	/*Creating a copy of current laser data and fixing the invalid readings.*/
	pthread_mutex_lock(&mutex_laser_read);
	for(i=0; i<maximum_laser_data; i++) {
		data[i]=data_laser[i]/10;
		/*Invalid data is converted in maximum range.*/
		if(data_laser[i]<VALID_LASER_DATA)
			data[i]=400;
	}
	pthread_mutex_unlock(&mutex_laser_read);
	/*First half. The laser is read from the middle index to the left (ascending numbers)*/
	for(i=get_frontal_index(); i<maximum_laser_data-IGNORED_LASER_SAMPLES && watchdog<HORIZON_ELEMENTS; i=i+laser_step) {
		data[i]=(data[i]<SENSOR_MIN_THRESHOLD)?SENSOR_MIN_THRESHOLD:data[i];
		data[i]=(data[i]>SENSOR_MAX_THRESHOLD)?SENSOR_MAX_THRESHOLD:data[i];
                repulsive_horizon[watchdog]=~(int) (roundl((data[i]-SENSOR_MIN_THRESHOLD)*multiplier));
                create_triangle(triangle_base_degrees,watchdog,repulsive_horizon[watchdog],repulsive_horizon);  
		watchdog++;
	}
	watchdog=(HORIZON_ELEMENTS>>1)-1;
	/*Second half. The laser is read from the middle index (shifted by one because the mid element has been already considered) to the right (descending numbers)*/
	for(i=get_frontal_index()-laser_step; i>=IGNORED_LASER_SAMPLES && watchdog>=0; i=i-laser_step) {
		data[i]=(data[i]<SENSOR_MIN_THRESHOLD)?SENSOR_MIN_THRESHOLD:data[i];
		data[i]=(data[i]>SENSOR_MAX_THRESHOLD)?SENSOR_MAX_THRESHOLD:data[i];
	        repulsive_horizon[watchdog]=~((int)roundl((data[i]-SENSOR_MIN_THRESHOLD)*multiplier));
                create_triangle(triangle_base_degrees,watchdog,repulsive_horizon[watchdog],repulsive_horizon);  
		watchdog--;
	}
	free(data);
}

void create_attractive_horizon(float destination_angle) {
	/*Index variable*/	
	int i;
	/*Converted destination relative angle to integer representing the discretized horizon index*/
	int horizon_destination_element=0;
	int triangle_base=roundl((DESTINATION_TRIANGLE_BASE_RADIANS*HORIZON_ELEMENTS)/M_PI);
	//It would be interesting to keep the gap between the float and the integer converted value.
	/*The index of the destination into the horizon. It is shifted of PI/2 in order to have "front" into the mid element of the array*/ 
	//horizon_destination_element=roundl(((destination_angle+(M_PI/2))*HORIZON_ELEMENTS));
	compute_horizon_index(&horizon_destination_element,destination_angle);
	//printf("horizon_destination %d\n",horizon_destination_element);
//printf("triangle base %d\n",triangle_base);
	create_triangle(triangle_base,horizon_destination_element,MAX_INT_FUZZY_MEMBERSHIP,attractive_horizon);
	 for(i=0;i<HORIZON_ELEMENTS;i++){
                if(attractive_horizon[i]<SIDELOBE_THRESHOLD)
                attractive_horizon[i]=SIDELOBE_THRESHOLD;
		//printf("Attractive [%d] : %u\n",i,attractive_horizon[i]);
        }
	 //printf("---------------------------------------\n");

}

void create_kinematic_horizon() {
	/*Converted destination relative angle to integer representing the discretized horizon index*/
	//int triangle_base=roundl((KINEMATIC_TRIANGLE_BASE_RADIANS*HORIZON_ELEMENTS)/M_PI);
	//It would be interesting to keep the gap between the float and the integer converted value.
	/*The index of the destination into the horizon. It is shifted of PI/2 in order to have "front" into the mid element of the array*/ 
	//horizon_destination_element=roundl(((destination_angle+(M_PI/2))*HORIZON_ELEMENTS));
	create_triangle(HORIZON_ELEMENTS,HORIZON_ELEMENTS>>1,MAX_INT_FUZZY_MEMBERSHIP,kinematic_horizon);
}
    
void close_probstavoid_module()
{
	#ifdef LOG_TO_FILE
	fclose(attractive_horizon_file);
	fclose(not_attractive_horizon_file);
	fclose(repulsive_horizon_file);
	fclose(kinematic_horizon_file);
	fclose(not_kinematic_horizon_file);
	fclose(inclusion_horizon_file);
	fclose(union_horizon_file);
	#endif
}
  /*  int main()
    {
		fuzzy_horizon_result *result=malloc(sizeof(fuzzy_horizon_result));
	
		int i=0;
		float w_ref=0.0;
		float v_ref=0.0;
		urg_t *urg=malloc(sizeof(urg_t));
    	int i=0;
    	float horizon[HORIZON_ELEMENTS];
    	init_probstavoid_module();
    	 * for(i=0;i<HORIZON_ELEMENTS;i++)
    		horizon[i]=0.0;
    	create_triangle(20,0,1,horizon);
    	for(i=0;i<HORIZON_ELEMENTS;i++)
    
	//		printf("%f\n",horizon[i]);
    init_urg_laser(&urg,ON_DEMAND);
    read_laser_data(urg);
    init_probstavoid_module();
    apply_fuzzy_horizon_algorithm(&v_ref,&w_ref,result);
	for(i=0;i<HORIZON_ELEMENTS;i++){
		printf("Repulsive Orizzonte[%d] %f\n",i,repulsive_horizon[i]);
   	//    printf("Not Attractive Orizzonte[%d] %f\n",i,not_attractive_horizon[i]);
	//	printf("Not Kinematic Orizzonte[%d] %f\n",i,not_kinematic_horizon[i]);
	//printf("Result[%d] %f\n",i,union_horizon[i]);
	}
	close_urg_laser(urg);
	return 1;
    }
    */
