#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

#define MPU6050_I2C_ADDR 0x68

#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPRT_DIV 0x19
#define REG_CONFIGS 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_FIFO_EN 0x23
#define REG_USER_CTRL 0x6A
#define REG_FIFO_COUNT_L 0x72
#define REG_FIFO_COUNT_H 0x73
#define REG_FIFO 0x74
#define REG_WHO_AM_I 0x75

int file = -1;

//    Gyroscope
//#define G_OFF_X -733
//#define G_OFF_Y 433
//#define G_OFF_Z -75
//Select the appropriate settings

#define GYRO_RANGE 3

#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 0
  #define GYRO_SENS 1
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE

#define TAU 0.05 //Complementary filter percentage
#define RAD_T_DEG 57.29577951308 //rad to degrees 180/pi


// Please note, this is not the recommanded way to write data
// to i2c devices from user space.
void i2c_write(__u8 reg_address, __u8 val) {
	char buf[2];
	if(file < 0) {
		printf("Error, i2c bus is not available\n");
		exit(1);
	}

	buf[0] = reg_address;
	buf[1] = val;

	if (write(file, buf, 2) != 2) {
		printf("Error, unable to write to i2c device\n");
		exit(1);
	}

}

char i2c_read(uint8_t reg_address) {
	char buf[1];
	if(file < 0) {
		printf("Error, i2c bus is not available\n");
		exit(1);
	}

	buf[0] = reg_address;

	if (write(file, buf, 1) != 1) {
		printf("Error, unable to write to i2c device\n");
		exit(1);
	}


	if (read(file, buf, 1) != 1) {
		printf("Error, unable to read from i2c device\n");
		exit(1);
	}

	return buf[0];

}

uint16_t merge_bytes( uint8_t LSB, uint8_t MSB) {
	return  (uint16_t) ((( LSB & 0xFF) << 8) | MSB);
}

// 16 bits data on the MPU6050 are in two registers,
// encoded in two complement. So we convert those to int16_t
/*int16_t two_complement_to_int( uint8_t LSB, uint8_t MSB) {
	int16_t signed_int = 0;
	uint16_t word;

	word = merge_bytes(LSB, MSB);

	if((word & 0x8000) == 0x8000) { // negative number
		signed_int = (int16_t) -(~word);
	} else {
		signed_int = (int16_t) (word & 0x7fff);
	}

	return signed_int;
}*/

float gr_off = 0;
float gp_off = 0;
float gy_off = 0;

void getOffsets() {
	float gyro_off[3]; //Temporary storage

	float gr_off = 0, gp_off = 0, gy_off = 0; //Initialize the offsets to zero
  printf("Calculating offsets!\n");
	for (int i = 0; i < 1000; i++) { //Use loop to average offsets
		gyro_off[0] = i2c_read(0x43) << 8 | i2c_read(0x44); //Read X registers
    gyro_off[1] = i2c_read(0x45) << 8 | i2c_read(0x46); //Read Y registers
    gyro_off[2] = i2c_read(0x47) << 8 | i2c_read(0x48); //Read Z registers
		gr_off = gr_off + gyro_off[0];
    gp_off = gp_off + gyro_off[1]; 
    gy_off = gy_off + gyro_off[2]; //Add to sum

	}

	gr_off = gr_off / 1000, gp_off = gp_off / 1000, gy_off = gy_off / 1000; //Divide by number of loops (to average)
  printf("Offsets obtained!\n");
}


int16_t two_complement_to_int( uint16_t word) {
	int16_t signed_int = 0;

	if((word & 0x8000) == 0x8000) { // negative number
		signed_int = (int16_t) -(~word);
	} else {
		signed_int = (int16_t) (word & 0x7fff);
	}

	return signed_int;
}


void MPUinit()
{
  i2c_write(REG_PWR_MGMT_1, 0x01);  //confiugrated for gyro use (X)
	i2c_write(REG_ACCEL_CONFIG, 0x00);  //leave at 2g full scale range
	i2c_write(REG_SMPRT_DIV, 0x01);   //sample rate divider = gyro output rate / (1 + smplrt_div) 
	i2c_write(REG_CONFIGS, 0x00);      //usage of fsync, frame synchronization 
  i2c_write(REG_GYRO_CONFIG, 0b00011000);
	i2c_write(REG_FIFO_EN, 0x00);     //was 88 temp and accel, try 70 for gyro xyz, 8 for accel. keeps on overflowing.
	i2c_write(REG_USER_CTRL, 0x44);   //driven by i2c master, reset fifo
}


int main(int argc, char *argv[]) {
  printf("balans\n");
	char bus_filename[250];
	//char accel_x_h,accel_x_l,accel_y_h,accel_y_l,accel_z_h,accel_z_l;
	//uint16_t fifo_len = 0;
	float _accel_angle[2];
  
  
	//int16_t temp = 0;
  
  
	snprintf(bus_filename, 250, "/dev/i2c-1");
	file = open(bus_filename, O_RDWR);
	if (file < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}


	if (ioctl(file, I2C_SLAVE, MPU6050_I2C_ADDR) < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}
  
  MPUinit();
  
  printf("Entering getoffsets()\n");
  //getOffsets();
  

	/*while(fifo_len != 1024) { //fifo keeps overflowing for no reason?
		accel_x_h = i2c_read(REG_FIFO_COUNT_L);
		accel_x_l = i2c_read(REG_FIFO_COUNT_H);
		fifo_len = merge_bytes(accel_x_h,accel_x_l);

		if(fifo_len == 1024) {
			printf("fifo overflow !\n");
			i2c_write(REG_USER_CTRL, 0x44);
			continue;
		}*/
    
    while(1)
    {

			/*
      accel_x_h = i2c_read(REG_FIFO);
			accel_x_l = i2c_read(REG_FIFO);
			accel_y_h = i2c_read(REG_FIFO);
			accel_y_l = i2c_read(REG_FIFO);
			accel_z_h = i2c_read(REG_FIFO);
			accel_z_l = i2c_read(REG_FIFO);
      temp_h = i2c_read(REG_FIFO);                                                                                            
			temp_l= i2c_read(REG_FIFO);
      */
      
      //ask for the registers over i2c instead. bitshifting high to left by 8 spots, ORing lowers
      int16_t X = i2c_read(0x43) << 8 | i2c_read(0x44); //Read X registers
      int16_t Y = i2c_read(0x45) << 8 | i2c_read(0x46); //Read Y registers
      int16_t Z = i2c_read(0x47) << 8 | i2c_read(0x48); //Read Z registers
      
      int16_t AX = i2c_read(0x3B) << 8 | i2c_read(0x3C); //Read X registers
      int16_t AY = i2c_read(0x3D) << 8 | i2c_read(0x3E); //Read Y registers
      int16_t AZ = i2c_read(0x3F) << 8 | i2c_read(0x40); //Read Z registers
      
      
      
      X = (round(X - gr_off) * 1000.0 / GYRO_SENS ) / 1000.0;
      Y = (round(Y - gp_off) * 1000.0 / GYRO_SENS ) / 1000.0;
      Z = (round(Z - gy_off) * 1000.0 / GYRO_SENS ) / 1000.0;
      
      AX = two_complement_to_int(AX);
      AY = two_complement_to_int(AY);
      AZ = two_complement_to_int(AZ);
      
      _accel_angle[0] = atan2(AZ, AY) * RAD_T_DEG - 90.0;
      _accel_angle[1] = atan2(AZ, AX) * RAD_T_DEG - 90.0;
      
      /*
			x_accel= two_complement_to_int(accel_x_h,accel_x_l);
			x_accel_g = ((float) x_accel)/16384;

			y_accel= two_complement_to_int(accel_y_h,accel_y_l);
			y_accel_g = ((float) y_accel)/16384;

			z_accel= two_complement_to_int(accel_z_h,accel_z_l);
			z_accel_g = ((float) z_accel)/16384;

			temp = two_complement_to_int(temp_h, temp_l);
			temp_f = (float)temp/340 + 36.53; // calculated as described in the MPU60%) register map document

			printf("x_accel %.3fg	y_accel %.3fg	z_accel %.3fg	temp=%.1fc         \r", x_accel_g, y_accel_g, z_accel_g, temp_f);*/
      printf("curr Y = %.3f  --  ", _accel_angle[1]);
      printf("curr X = %.3f", _accel_angle[0]);
    
      printf("\r");
		
		}

	

	return 0;
}
