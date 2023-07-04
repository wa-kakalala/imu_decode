#ifndef __IMU_DECODE__
#define __IMU_DECODE__
#include <stdio.h>
#include <stdlib.h>

#define TIME_STAMP_LEN  9
#define ACC_SPEED_LEN   9
#define W_SPEED_LEN     9

//#define DEBUG_TYPE 1 

typedef enum ERROR{
	IMU_NO_ERROR,
	IMU_ERROR ,
    IMU_ERROR_TO_CONTINUE
}ERROR;

typedef enum instruct_type{
	TYPE_TIME_STAMP = 0x50,
	TYPE_ACC_SPEED =  0x51,
	TYPE_ANGLE_SPEED = 0x52
	
}instruct_type;


// time stamp struct
typedef struct time_struct{
	unsigned char yy;
	unsigned char mm;
	unsigned char dd;
	unsigned char hh;
	unsigned char mn;
	unsigned char ss;
	unsigned short ms;
}time_struct;

typedef struct time_struct_raw{
	unsigned char yy;
	unsigned char mm;
	unsigned char dd;
	unsigned char hh;
	unsigned char mn;
	unsigned char ss;
	unsigned char msl;
	unsigned char msh;
	unsigned char sum;
}time_struct_raw;

typedef struct time_struct_decode{
	time_struct time_structor;
	ERROR	err;
}time_struct_decode;

// acc speed struct
typedef struct acc_struct{
	double  accx;
	double  accy;
	double  accz;
	double  t;
}acc_struct;

typedef struct acc_struct_raw{
	unsigned char  axl;
	unsigned char  axh;
	unsigned char  ayl;
	unsigned char  ayh;
	unsigned char  azl;
	unsigned char  azh;
	unsigned char  tl;
	unsigned char  th;
	unsigned char  sum;
}acc_struct_raw;

typedef struct acc_struct_decode{
	acc_struct acc_structor;
	ERROR	err;
}acc_struct_decode;

// angle speed struct
typedef struct wspeed_struct{
	double  wspeedx;
	double  wspeedy;
	double  wspeedz;
	double  vol;
}wspeed_struct;

typedef struct wspeed_struct_raw{
	unsigned char  wxl;
	unsigned char  wxh;
	unsigned char  wyl;
	unsigned char  wyh;
	unsigned char  wzl;
	unsigned char  wzh;
	unsigned char  vol;
	unsigned char  voh;
	unsigned char  sum;
}wspeed_struct_raw;

typedef struct wspeed_struct_decode{
	wspeed_struct wspeed_structor;
	ERROR	err;
}wspeed_struct_decode;

unsigned char get_one_byte(FILE * fp,ERROR* err);
// time stamp
ERROR recv_time_byte_data(FILE*fp,time_struct_raw*tsr);
time_struct_decode parse_time_stamp(const time_struct_raw*tsr);
unsigned char get_one_byte(FILE * fp,ERROR* err);
void debug_print_time_raw(const time_struct_raw*tsr);
void print_time_info(const time_struct_decode*tsd);
// acc speed
ERROR recv_acc_byte_data(FILE*fp,acc_struct_raw*asr);
void debug_print_acc_raw(const acc_struct_raw*asr) ;
acc_struct_decode parse_acc_speed(const acc_struct_raw*asr);
// wspeed
void debug_print_wspeed_raw(const wspeed_struct_raw*wsr) ;
ERROR recv_wspeed_byte_data(FILE*fp,wspeed_struct_raw*wsr); 
wspeed_struct_decode parse_w_speed(const wspeed_struct_raw*wsr);
void print_wspeed_infp(const wspeed_struct_decode*wsd);




#endif
