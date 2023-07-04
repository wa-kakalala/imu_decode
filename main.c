#include "imu_decode.h"

int main(){
	unsigned count = 0;
	int      can_write = 0;
 	char filename[20] = "./data/imu_l.txt";
 	char outfile[20]  = "./data/imu_l_out.csv";
 	FILE * fp = fopen(filename,"r");
 	FILE * fp_out = fopen(outfile,"w");
 	unsigned char byte_data = 0;
	ERROR err = IMU_NO_ERROR;
	time_struct_raw time_structor_raw = {0};
	time_struct_decode time_structor_decode = {0};
	
	acc_struct_raw  acc_structor_raw  = {0};
	acc_struct_decode acc_structor_decode;
	
	wspeed_struct_raw  wspeed_structor_raw  = {0};
	wspeed_struct_decode wspeed_structor_decode;
	
	for(;;){
		if( err == IMU_NO_ERROR ){
			byte_data = get_one_byte(fp,&err);
			if( err == IMU_ERROR) break;
		}
		
		if(byte_data == 0x55 || err == IMU_ERROR_TO_CONTINUE ){
			byte_data = get_one_byte(fp,&err);
			if( byte_data == TYPE_TIME_STAMP){
				err = recv_time_byte_data(fp,&time_structor_raw);
				if( err == IMU_NO_ERROR ){
					can_write ++;
					#ifdef DEBUG_TYPE
					printf("\r\n");
					debug_print_time_raw(&time_structor_raw);
					#endif
					time_structor_decode = parse_time_stamp(&time_structor_raw);
					//print_time_info(&time_structor_decode);
					//fprintf(fp_out,"%d-%d-%d %d:%d:%d %dms\n", \
					//	time_structor_decode.time_structor.yy, time_structor_decode.time_structor.mm,time_structor_decode.time_structor.dd,\
					//	time_structor_decode.time_structor.hh, time_structor_decode.time_structor.mn,time_structor_decode.time_structor.ss,
					//	time_structor_decode.time_structor.ms \
					//);
					
					printf("count: %d\n",count++);
				} else if(err == IMU_ERROR){
					break;
				} else{
					can_write = 0;
				}
				
			}else if(byte_data == TYPE_ACC_SPEED){
				err = recv_acc_byte_data(fp,&acc_structor_raw);
				
				if( err == IMU_NO_ERROR ){
					can_write ++;
					#ifdef DEBUG_TYPE
					printf("\r\n");
					debug_print_acc_raw(&acc_structor_raw);
					#endif
					acc_structor_decode = parse_acc_speed(&acc_structor_raw);
					//print_acc_infp(&acc_structor_decode);
					//fprintf(fp_out,"%lf %lf %lf %lf\n", \
					//	acc_structor_decode.acc_structor.accx, acc_structor_decode.acc_structor.accy,\
					//	acc_structor_decode.acc_structor.accz,acc_structor_decode.acc_structor.t \
					//);
					
					printf("count: %d\n",count++);
				}else if(err == IMU_ERROR){
					break;
				}else{
					can_write = 0;
				}
				
			}else if(byte_data == TYPE_ANGLE_SPEED){
				err = recv_wspeed_byte_data(fp,&wspeed_structor_raw);
				if(err == IMU_NO_ERROR){
					can_write ++;
					#ifdef DEBUG_TYPE
					printf("\r\n");
					debug_print_wspeed_raw(&wspeed_structor_raw);
					#endif
					wspeed_structor_decode = parse_w_speed(&wspeed_structor_raw);
					//print_wspeed_infp(&wspeed_structor_decode);
					//fprintf(fp_out,"%lf %lf %lf %lf\n", \
			        //   wspeed_structor_decode.wspeed_structor.wspeedx, wspeed_structor_decode.wspeed_structor.wspeedy, \
					//   wspeed_structor_decode.wspeed_structor.wspeedz,wspeed_structor_decode.wspeed_structor.vol \
		            //);
		            
		            printf("count: %d\n",count++);
				}else if(err == IMU_ERROR){
					break;
				}else{
					can_write = 0;
				}
			}else{
				
			}	
		}
		if( can_write == 3){
			fprintf(fp_out,"%d,%d,",0,0); 
			fprintf(fp_out,"%lf,%lf,%lf,",
				acc_structor_decode.acc_structor.accx, acc_structor_decode.acc_structor.accy,\
				acc_structor_decode.acc_structor.accz \
			);
			fprintf(fp_out,"%lf,%lf,%lf\n",  \
				wspeed_structor_decode.wspeed_structor.wspeedx, wspeed_structor_decode.wspeed_structor.wspeedy, \
				wspeed_structor_decode.wspeed_structor.wspeedz
			);
			can_write = 0;
		}
	}
	printf("over!!!");
 	fclose(fp);
 	fclose(fp_out);
 }

