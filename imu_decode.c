#include "imu_decode.h"

unsigned char get_one_byte(FILE * fp,ERROR* err){
	char ch[3] = {0};
	char  index ;
	for(index = 0;index <3;index++){	
		ch[index] = fgetc(fp);  
		#ifdef DEBUG_TYPE
		printf("%c",ch[index]);
		#endif
		if(ch[index] == EOF) {
			*err = IMU_ERROR;
			return 0;
		}	
	}	
	*err = IMU_NO_ERROR;

	for(index = 0;index <2 ;index++){
		if( ch[index] >= '0' && ch[index] <= '9')  ch[index] -= '0';
		else if(ch[index] >= 'a' && ch[index] <= 'f')  ch[index] = ch[index] - 'a' + 10 ;
		else if(ch[index] >= 'A' && ch[index] <= 'F')  ch[index] = ch[index] - 'A' + 10 ;
		else ;
	}

	return (ch[0]<<4) + ch[1]; 
}

////////////////// time parser ///////////////////////////

ERROR recv_time_byte_data(FILE*fp,time_struct_raw*tsr){
	unsigned char * start = (unsigned char *)tsr;
	ERROR err;
	int i;
	for(i=0;i<TIME_STAMP_LEN;i++){
		*(start+i) = get_one_byte(fp,&err); 
		if( err == IMU_ERROR) return err;
		if(*(start+i) == 0x55) return IMU_ERROR_TO_CONTINUE;
	}
	return err;
}

time_struct_decode parse_time_stamp(const time_struct_raw*tsr){
	time_struct_decode tsd;
	unsigned char      sum;
	tsd.time_structor.yy = tsr->yy;
	tsd.time_structor.mm = tsr->mm;
	tsd.time_structor.dd = tsr->dd;
	tsd.time_structor.hh = tsr->hh;
	tsd.time_structor.mn = tsr->mn;
	tsd.time_structor.ss = tsr->ss;
	tsd.time_structor.ms = (((unsigned short)tsr->msh) << 8 ) | (unsigned short)tsr->msl; 
	
	sum = (0x55 + 0x50 +  tsr->yy + tsr->mm + tsr->dd + tsr->hh + tsr->mn + tsr->ss + tsr->msh + tsr->msl) & 0xff;
	
	if( sum == tsr->sum) {
		tsd.err	= IMU_NO_ERROR;
	}else{
		tsd.err = IMU_ERROR;
	}
	return tsd;
}

////////////////// acc speed parser ///////////////////////////
ERROR recv_acc_byte_data(FILE*fp,acc_struct_raw*asr){
	unsigned char * start = (unsigned char *)asr;
	ERROR err;
	int i;
	for(i=0;i<ACC_SPEED_LEN;i++){
		*(start+i) = get_one_byte(fp,&err); 
		if( err == IMU_ERROR) return err;
		if(*(start+i) == 0x55) return IMU_ERROR_TO_CONTINUE;
	}
	return err;
}

acc_struct_decode parse_acc_speed(const acc_struct_raw*asr){
	acc_struct_decode asd;
	unsigned char     sum;
	asd.acc_structor.accx = (short)((unsigned short)((asr->axh)<<8) | (unsigned short)asr->axl) / 32768.0 * 16.0 * 9.8;
	asd.acc_structor.accy = (short)((unsigned short)((asr->ayh)<<8) | (unsigned short)asr->ayl) / 32768.0 * 16.0 * 9.8;
	asd.acc_structor.accz = (short)((unsigned short)((asr->azh)<<8) | (unsigned short)asr->azl) / 32768.0 * 16.0 * 9.8;
	asd.acc_structor.t = (short)((unsigned short)((asr->th)<<8) | (unsigned short)asr->tl) / 100.0;
	sum = (0x55 + 0x51 + asr->axh + asr->axl + asr->ayh + asr->ayl + asr->azh + asr->azl + asr->tl + asr->th) & 0xff;
	
	if( sum == asr->sum) {
		asd.err	= IMU_NO_ERROR;
	}else{
		asd.err = IMU_ERROR;
	}
	return asd;
}

////////////////// wspeed parser ///////////////////////////
ERROR recv_wspeed_byte_data(FILE*fp,wspeed_struct_raw*wsr){
	unsigned char * start = (unsigned char *)wsr;
	ERROR err;
	int i;
	for(i=0;i<W_SPEED_LEN;i++){
		*(start+i) = get_one_byte(fp,&err); 
		if( err == IMU_ERROR) return err;
		if(*(start+i) == 0x55) return IMU_ERROR_TO_CONTINUE;
	}
	return err;
}

wspeed_struct_decode parse_w_speed(const wspeed_struct_raw*wsr){
	wspeed_struct_decode wsd;
	unsigned char        sum;
	wsd.wspeed_structor.wspeedx = (short)((unsigned short)((wsr->wxh)<<8) | (unsigned short)wsr->wxl) / 32768.0 * 2000.0;
	wsd.wspeed_structor.wspeedy = (short)((unsigned short)((wsr->wyh)<<8) | (unsigned short)wsr->wyl) / 32768.0 * 2000.0;
	wsd.wspeed_structor.wspeedz = (short)((unsigned short)((wsr->wzh)<<8) | (unsigned short)wsr->wzl) / 32768.0 * 2000.0;
	wsd.wspeed_structor.vol = (short)((unsigned short)((wsr->voh)<<8) | (unsigned short)wsr->vol) / 100.0;
	sum = (0x55 + 0x52 + wsr->wxh + wsr->wxl + wsr->wyh + wsr->wyl + wsr->wzh + wsr->wzl + wsr->vol + wsr->voh) & 0xff;
	
	if( sum == wsr->sum) {
		wsd.err	= IMU_NO_ERROR;
	}else{
		wsd.err = IMU_ERROR;
	}
	return wsd;
}

///////////// print info ////////////////

void debug_print_time_raw(const time_struct_raw*tsr) {
	printf("--------------\r\n");
	printf("yy: %d\r\n",tsr->yy);
	printf("mm: %d\r\n",tsr->mm);
	printf("dd: %d\r\n",tsr->dd);
	printf("hh: %d\r\n",tsr->hh);
	printf("mn: %d\r\n",tsr->mn);
	printf("ss: %d\r\n",tsr->ss);
	printf("msl: %d\r\n",tsr->msl);
	printf("msh: %d\r\n",tsr->msh);
	printf("sum: %d\r\n",tsr->sum);
	printf("--------------\r\n");
}

void print_time_info(const time_struct_decode*tsd){
	if(tsd->err == IMU_NO_ERROR){
		printf("%d-%d-%d %d:%d:%d %dms\r\n", \
			tsd->time_structor.yy, tsd->time_structor.mm,tsd->time_structor.dd,tsd->time_structor.hh, \
			tsd->time_structor.mn,tsd->time_structor.ss,tsd->time_structor.ms \
		);
	}else{
		printf("time stamp error\r\n");
	}	
}

void debug_print_acc_raw(const acc_struct_raw*asr) {
	printf("--------------\r\n");
	printf("axl: %d\r\n",asr->axl);
	printf("axh: %d\r\n",asr->axh);
	printf("ayl: %d\r\n",asr->ayl);
	printf("ayh: %d\r\n",asr->ayh);
	printf("azl: %d\r\n",asr->azl);
	printf("azh: %d\r\n",asr->azh);
	printf("tl: %d\r\n",asr->tl);
	printf("th: %d\r\n",asr->th);
	printf("sum: %d\r\n",asr->sum);
	printf("--------------\r\n");
}

void print_acc_infp(const acc_struct_decode*asd){
	if(asd->err == IMU_NO_ERROR){
		printf("%lf %lf %lf %lf\r\n", \
			asd->acc_structor.accx, asd->acc_structor.accy,asd->acc_structor.accz,asd->acc_structor.t \
		);
	}else{
		printf("acc speed error\r\n");
	}	
}

void debug_print_wspeed_raw(const wspeed_struct_raw*wsr) {
	printf("--------------\r\n");
	printf("wxl: %d\r\n",wsr->wxl);
	printf("wxh: %d\r\n",wsr->wxh);
	printf("wyl: %d\r\n",wsr->wyl);
	printf("wyh: %d\r\n",wsr->wyh);
	printf("wzl: %d\r\n",wsr->wzl);
	printf("wzh: %d\r\n",wsr->wzh);
	printf("vol: %d\r\n",wsr->vol);
	printf("voh: %d\r\n",wsr->voh);
	printf("sum: %d\r\n",wsr->sum);
	printf("--------------\r\n");
}

void print_wspeed_infp(const wspeed_struct_decode*wsd){
	if(wsd->err == IMU_NO_ERROR){
		printf("%lf %lf %lf %lf\r\n", \
			wsd->wspeed_structor.wspeedx, wsd->wspeed_structor.wspeedy,wsd->wspeed_structor.wspeedz,wsd->wspeed_structor.vol \
		);
	}else{
		printf("w speed error\r\n");
	}	
}

