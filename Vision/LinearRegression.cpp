#include <stdio.h>
#include <stdlib.h>


void LinReg(double x1, double y1, double x2, double y2, double *mi, double *c){		//Code to calculate the parameters of a linear regression (y = mi*x + c )
	*mi = (y1-y2)/(x1-x2);
	*c = (-x1)**mi + y1;
}

int main(){
	int a=3;
	double mi, c;
	LinReg(a, a+1, a/3, a/3, &mi, &c);		//Receiving the parameters of the linear regression
	printf("Mi = %f\nConstant = %f", mi, c);		//Printing them
}
