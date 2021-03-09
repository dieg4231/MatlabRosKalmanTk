#include <stdio.h>

int main()
{

	double ancho_cubo = .1;
	double map = 1;
	int cta=0;
	for(float y = ancho_cubo, x = ancho_cubo ; x < map-ancho_cubo*2 ; x +=ancho_cubo*3  )
			printf("( polygon obstacle  obs_%d %f %f %f %f %f %f %f %f )\n",cta++,x,y,x,y+ancho_cubo,x+ancho_cubo,y+ancho_cubo,x+ancho_cubo,y);

	for(float y =  map-ancho_cubo*2, x = ancho_cubo ; x < map+ancho_cubo*2 ; x +=ancho_cubo*3  )
			printf("( polygon obstacle  obs_%d %f %f %f %f %f %f %f %f )\n",cta++,x,y,x,y+ancho_cubo,x+ancho_cubo,y+ancho_cubo,x+ancho_cubo,y);

	for(float x = ancho_cubo,y = ancho_cubo*4 ; y < map-ancho_cubo*4 ; y +=ancho_cubo*3  )
			printf("( polygon obstacle  obs_%d %f %f %f %f %f %f %f %f )\n",cta++,x,y,x,y+ancho_cubo,x+ancho_cubo,y+ancho_cubo,x+ancho_cubo,y);

	for(float x = map-ancho_cubo*2 ,y = ancho_cubo *4 ; y < map-ancho_cubo*4 ; y +=ancho_cubo*3  )
			printf("( polygon obstacle  obs_%d %f %f %f %f %f %f %f %f )\n",cta++,x,y,x,y+ancho_cubo,x+ancho_cubo,y+ancho_cubo,x+ancho_cubo,y);


	printf("\n");
	return 0;
}