#include<iostream>
#include<fstream>
#include<sstream>
#include<stdlib.h>
#include<string.h>
#include<time.h>
#include <math.h>
#include <stdio.h>
#define max 40.000
#define min -40.000
#define n 30
#define dim 3
#define c1 0.8
#define c2 0.6
#define w 0.8
//#define ITE 1000
using namespace std;
double  formula(double x,double y);
void newposition(double *xt,double *vector,double *mybest,double *globallocation,int ind);
void randomstart(double *particle,int x,int y);
void randomvector(double * vvector,int len);//隨機起始方向向量
void PPrint(double *arr,int first,int second);
