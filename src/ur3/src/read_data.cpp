#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <string.h>
float* read_data(){
	float* data = new float[8000];
    FILE *p;
    char c = 'a';
	char num_str[2];
	int count_num = 0;
	float x;
	int i = 0
;    p = fopen("prbs_inp.csv","r");
	printf("dh");
    while (!feof(p)){
		while(1){
			fscanf(p,"%c",&c);
			//printf("%c",c);
			if(c == '\n') {
				break;
			}
			num_str[count_num] = c;
			count_num ++;
			
		}
        count_num =0;
	    x =atof(num_str);
		data[i] = x;
		i++;
		num_str[0] = 0;
		num_str[1] = 0;
		num_str[2] = 0;
		//printf("%f\n",x);
	}
	fclose(p);
	return data;	
}
 
