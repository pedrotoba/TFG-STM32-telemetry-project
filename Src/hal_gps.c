#include "hal_gps.h"

int get_gps_data(UART_HandleTypeDef *huart, UART_HandleTypeDef *ftdi, gps_hal * gps){

	gps->speed = 0.0;

		char c_received[1];
		char aux_msg[20]="";
		char aux_msg_2[20]="";
		char buffer[100];
		char data[20][20];
		float latitude, longitude;
		char * valid_data = "A";
		int cont = 0, t_con = 0, cont_p = 0;
		
		HAL_UART_Receive(huart,(uint8_t*)c_received,1, 100);
		
		//Leo el serial de el gps para coger los datos
		if(c_received[0] == '$'){
			HAL_UART_Receive(huart,(uint8_t*)c_received, 1, 100);
			while(c_received[0] != '\n'){
				buffer[cont] = c_received[0];
				HAL_UART_Receive(huart,(uint8_t*)c_received, 1, 10);
				cont = cont+1;
			}
			
			//Relleno el array con los datos para pasarlos a la estructura
			if(strstr(buffer,"GPRMC")){
				char* token = strtok(buffer, ",");
				while (token) {
					HAL_UART_Transmit(ftdi,(uint8_t*)token,10,100);
					HAL_UART_Transmit(ftdi,(uint8_t*)",",1,100);
					strcpy(data[t_con],token);
					token = strtok(NULL, ",");
					t_con = t_con + 1;
				}
			}
			
			//$GPRMC,194530.000,A,3051.8007,N,10035.9989,W,1.49,111.67,310714,,,A*74
			//$GPRMC,182518.00,A,3714.91556,N,00339.15187,W,0.086,220518,A*6
			//$GPRMC,193646.00,A,3714.91291,N,00339.15744,W,4.185,349.87,050618,,,A*79
			
			if(strcmp(data[2],valid_data) == 0 ){
				
				HAL_UART_Transmit(ftdi,(uint8_t*)"OK2\n",4,100);
				format_position(data[3],data[4],&latitude);
				format_position(data[5],data[6],&longitude);
				
				gps->speed = atof(data[7]);
				gps->latitude = latitude;
				gps->longitude = longitude;
				gps->time = atoi(data[1]);
				gps->date = atoi(data[8]);
				
				gps->state = 1;
				
				
			}else{

				gps->state = 0;
			}
			return 1;
		}else{
			return 0;
		}
}

void print_data(){

}

//MOSTRAR TODOS LOS PARAMETROS DEL GPS
void print_raw_data(UART_HandleTypeDef *huart, UART_HandleTypeDef *ftdi){
		char buffer[1];
		HAL_UART_Receive(huart,(uint8_t*)buffer, 1, 10);
		HAL_UART_Transmit(ftdi,(uint8_t*)buffer,1,10);
}

int calculate_checksum(char data[], int checksum_value){
	return 1;
}

void format_position(char * l, char * pos, float *f_position){
    float l_float,minutes,l_total;
		int l_deg;
    l_float = atof(l);
    l_deg = (int)l_float / 100;
    minutes = (l_float - (l_deg*100))/60.0;
    l_total = (float)l_deg + minutes;
    if(pos[0] == 'S' || pos[0] == 'W'){
       l_total = l_total * (-1); 
    }
    *f_position = l_total;
}

void knot_2_km_h(char * knot, float * km_h){
    float f_knot;
    f_knot = atof(knot);
    *km_h = f_knot * 1.85;
}


