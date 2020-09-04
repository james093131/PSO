//
//  function.cpp
//  PSO
//
//  Created by TzuChieh on 2020/3/24.
//  Copyright © 2020 TzuChieh. All rights reserved.
//
#include "function.hpp"

double  formula(double x,double y)
{
    double re=-20*(exp((-0.2)*sqrt((pow(x,2)+pow(y,2))/2)))-exp((cos(2*M_PI*x)+cos(2*M_PI*y))/2)+20+exp(1);
    return re;
}
void newposition(double *xt,double *vector,double *mybest,double *globallocation,int ind)
{
    double tempglobalbest[3]={0,0,100000};//用來暫存粒子移動後的最佳解
    for(int i=0;i<n;i++)
    {
        double rho1 = (double) rand() / (RAND_MAX + 1.0);
        double rho2 = (double) rand() / (RAND_MAX + 1.0);
        double tempx= w*(*((double*)vector +2*(i)+0))+rho1*c1*(*((double*)mybest +3*(i)+0)-*((double*)xt +3*(i)+0))+rho2*c2*(globallocation[0] -*((double*)xt +3*(i)+0));
        double tempy= w*(*((double*)vector +2*(i)+1))+rho1*c1*(*((double*)mybest +3*(i)+1)-*((double*)xt +3*(i)+1))+rho2*c2*(globallocation[1] -*((double*)xt +3*(i)+1));
        *((double*)vector +2*(i)+0)=tempx;//更新速度向量
        *((double*)vector +2*(i)+1)=tempy;
        *((double*)xt +3*(i)+0)+=tempx;//更新位置
        *((double*)xt +3*(i)+1)+=tempy;
        if(*((double*)xt +3*(i)+0)>max)//處理邊界問題
            *((double*)xt +3*(i)+0)=max;
        else if(*((double*)xt +3*(i)+0)<min)
            *((double*)xt +3*(i)+0)=min;
        if(*((double*)xt +3*(i)+1)>max)//處理邊界問題
            *((double*)xt +3*(i)+1)=max;
        else if(*((double*)xt +3*(i)+1)<min)
            *((double*)xt +3*(i)+1)=min;
        *((double*)xt +3*(i)+2)= formula(*((double*)xt +3*(i)+0), *((double*)xt +3*(i)+1));//更新當前解
        if(*((double*)xt +3*(i)+2)<*((double*)mybest +3*(i)+2))//更新單一粒子移動後的最佳解
        {
            *((double*)mybest +3*(i)+0)=*((double*)xt +3*(i)+0);
            *((double*)mybest +3*(i)+1)=*((double*)xt +3*(i)+1);
            *((double*)mybest +3*(i)+2)=*((double*)xt +3*(i)+2);
        }
        if( *((double*)xt +3*(i)+2)<tempglobalbest[2])//判斷所有粒子移動後的最佳解
        {
            tempglobalbest[0]=*((double*)xt +3*(i)+0);
            tempglobalbest[1]=*((double*)xt +3*(i)+1);
            tempglobalbest[2]=*((double*)xt +3*(i)+2);
        }
    }
    if(tempglobalbest[2]<globallocation[2])
    {
        globallocation[0]=tempglobalbest[0];
        globallocation[1]=tempglobalbest[1];
        globallocation[2]=tempglobalbest[2];
    }
}
void randomstart(double *particle,int x,int y)
{
    for(int i=0;i<x;i++)
    {
        double a=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        double b=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        double c=formula(a,b);
        *((double*)particle +y*(i)+0) = a;//隨機x
        *((double*)particle +y*(i)+1) = b;//隨機ｙ
        *((double*)particle +y*(i)+2) = c;
    }
    
}
void randomvector(double * vvector,int len)//隨機起始方向向量
{
    for(int i=0;i<n;i++)
    {
        
        double a=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        double b=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        *((double*)vvector +2*(i)+0) = a;//隨機x
        *((double*)vvector +2*(i)+1) = b;//隨機ｙ
    }
}
void PPrint(double *arr,int first,int second)
{
    for(int i=0;i<first;i++)
    {
        for(int j=0;j<second;j++)
        {
            cout<<*((double*)arr +second*i+j)<<' ';
        }
        cout<<endl;
    }
    cout<<endl;
}
