#include <ev3.h>

void bocht(int);
int berekengraden(int);

//gyro_offset = readSensor(IN_3);
//gyro_value = readSensor(IN_3) - gyro_offset;

int main()
{
	InitEV3();
    setAllSensorMode(COL_COLOR, NO_SEN, GYRO_ANG, US_DIST_MM);
    Off(OUT_B);
    RotateMotor(OUT_AD, 70, berekengraden(20));
	Wait(MS_5);
    bocht(90);
    RotateMotor(OUT_AD, 50, berekengraden(115));
    Wait(MS_10);
	bocht(-135);
    RotateMotor(OUT_AD, 50, berekengraden(115));
    Wait(MS_5);
	FreeEV3();
    return 0;
    /*
	RotateMotor(OUT_AD, 100, berekengraden(155));
	Wait(MS_3);
	bocht(45);
	Wait(MS_3);
	RotateMotor(OUT_AD, 100, berekengraden(18));
	Wait(MS_3);
	RotateMotor(OUT_AD, -100, berekengraden(-18));
	Wait(MS_5);
	bocht(-45);
	Wait(MS_3);
	RotateMotor(OUT_AD, -100, berekengraden(-170));
    Wait(MS_10)
    */    
}
void bocht(int ggraden)
{
	int gyro, sensornul, langzaam;
	gyro = readSensor(IN_3);
	sensornul = readSensor(IN_3);
	ggraden = sensornul+ggraden;
	if (ggraden > sensornul)
	{
		langzaam = ggraden-33;
		while (gyro < langzaam)
		{
			gyro = readSensor(IN_3);
			OnFwdReg(OUT_D, 45);
			OnRevReg(OUT_A, 45);
			Wait(MS_3);
		}
		while (gyro >= langzaam && gyro != ggraden)
		{
			gyro = readSensor(IN_3);
			OnFwdReg(OUT_D, 10);
			OnRevReg(OUT_A, 10);
			Wait(MS_3);
		}
	}
	else if (ggraden < sensornul)
	{
		langzaam = ggraden+33;
		while (gyro > langzaam)
		{
			gyro = readSensor(IN_3);
			OnFwdReg(OUT_A, 45);
			OnRevReg(OUT_D, 45);
			Wait(MS_3);
		}
		while (gyro <= langzaam && gyro != ggraden)
		{
			gyro = readSensor(IN_3);
			OnFwdReg(OUT_A, 10);
			OnRevReg(OUT_D, 10);
			Wait(MS_3);
		}
	}
	else
	{
		LcdPrintf(1, "Foutmelding");
	}

}
int berekengraden(int ggraden)
{
	ggraden = ggraden * 20;
	return ggraden;
}
