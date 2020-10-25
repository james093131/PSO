#include<iostream>
#include<fstream>
#include<sstream>
#include<stdlib.h>
#include<string.h>
#include<time.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#define max 40.000
#define min -40.000
#define n 30
#define dim 3
#define c1 0.8
#define c2 0.6
#define w 0.8
using namespace std;


typedef vector<char> c1d;
typedef vector<c1d> c2d;
typedef vector<c2d> c3d;
typedef vector<c3d> c4d;
typedef vector<int> i1d;
typedef vector<i1d> i2d;
typedef vector<i2d> i3d;
typedef vector<i3d> i4d;
typedef vector<double> d1d;
typedef vector<d1d> d2d;
typedef vector<d2d> d3d;
typedef vector<d3d> d4d;





//
//  function.hpp
//  PSO
//
//  Created by TzuChieh on 2020/3/24.
//  Copyright © 2020 TzuChieh. All rights reserved.
//
double  formula(double x,double y)
{
    double re=-20*(exp((-0.2)*sqrt((pow(x,2)+pow(y,2))/2)))-exp((cos(2*M_PI*x)+cos(2*M_PI*y))/2)+20+exp(1);
    return re;
}
void newposition(d2d &xt,d2d &vector,d2d &mybest,d1d &globallocation,int ind)
{
    double tempglobalbest[3]={0,0,100000};//用來暫存粒子移動後的最佳解
    for(int i=0;i<n;i++)
    {
        double rho1 = (double) rand() / (RAND_MAX + 1.0);
        double rho2 = (double) rand() / (RAND_MAX + 1.0);
        // double tempx= w*(*((double*)vector +2*(i)+0))+rho1*c1*(*((double*)mybest +3*(i)+0)-*((double*)xt +3*(i)+0))+rho2*c2*(globallocation[0] -*((double*)xt +3*(i)+0));
        double tempx= w*(vector[i][0]+rho1*c1*(mybest[i][0]-xt[i][0])+rho2*c2*(globallocation[0] -xt[i][0]));
        double tempy= w*(vector[i][1]+rho1*c1*(mybest[i][1]-xt[i][1])+rho2*c2*(globallocation[1] -xt[i][1]));

        vector[i][0] = tempx;//更新速度向量
        vector[i][1] = tempy;

        xt[i][0] += tempx;//更新位置
        xt[i][1] += tempy;

        if(xt[i][0] > max)//處理邊界問題
            xt[i][0] = max;
        else if(xt[i][0] < min)
            xt[i][0]=min;
        if(xt[i][1] > max)//處理邊界問題
            xt[i][1] = max;
        else if(xt[i][1] < min)
            xt[i][1] = min;
        xt[i][2]= formula(xt[i][0],xt[i][1]);//更新當前解
        if(xt[i][2]<mybest[i][2])//更新單一粒子移動後的最佳解
        {
           mybest[i][0] = xt[i][0];
           mybest[i][1] = xt[i][1];
           mybest[i][2] = xt[i][2];
        }
        if( xt[i][2] < tempglobalbest[2])//判斷所有粒子移動後的最佳解
        {
            tempglobalbest[0] = xt[i][0];
            tempglobalbest[1] = xt[i][1];
            tempglobalbest[2] = xt[i][2];
        }
    }
    if(tempglobalbest[2]<globallocation[2])
    {
        globallocation[0]=tempglobalbest[0];
        globallocation[1]=tempglobalbest[1];
        globallocation[2]=tempglobalbest[2];
    }
}
void randomstart(d2d &particle,int x,int y)
{
    for(int i=0;i<x;i++)
    {
        double a=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        double b=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        double c=formula(a,b);
        particle[i][0] = a;//隨機x
        particle[i][1] = b;//隨機ｙ
        particle[i][2] = c;//計算出的fitness
    }
    
}
void randomvector(d2d &vvector,int len)//隨機起始方向向量
{
    for(int i=0;i<n;i++)
    {
        
        double a=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        double b=(max - (min)) * rand() / (RAND_MAX + 1.0) + (min);
        vvector[i][0] = a;//隨機x
        vvector[i][1] = b;//隨機ｙ
    }
}
void PPrint(d2d arr)
{
    for(int i=0;i<arr.size();i++)
    {
        for(int j=0;j<arr[i].size();j++)
        {
            cout<<arr[i][j]<<' ';
        }
        cout<<endl;
    }
    cout<<endl;
}
void FINAL_PRINT(int R,d1d globalbest,double START,double END,int whereami)
{
    cout<<"RUN : "<<R+1<<' '<<'('<<globalbest[0]<<" ,"<<globalbest[1]<<')'<<endl<<"Optima :"<<globalbest[2]<<endl;
    cout<<"RUN "<<R+1<<' '<<"Execution Time :"<<(END - START) / CLOCKS_PER_SEC<<"(s)"<<endl;
    cout<<"optima is come from the "<<whereami<<" iteration  "<<endl<<endl;
}
void OUTPUT_FILE(int ITE,int RUN,d1d RUN_ITER_BEST)
{
    fstream file;//寫檔
    file.open("PSO_Convergence.txt",ios::out);
    for(int i=0;i<ITE;i++)
    {
    file<<i+1<<' '<<RUN_ITER_BEST[i]/RUN<<endl;
    }
}

void PSO_RUN(int ITE,int RUN)
{
    d1d RUN_ITER_BEST(ITE,0);
    int R=0;
    while(R<RUN)
    {
        double START ,END;
        START=clock();
        int iteration=1;
        d2d particle(n,d1d(dim));//用來儲存n個粒子的xyz圖
        d2d mybest(n,d1d(dim));//用來儲存n個粒子目前最好的ｘｙｚ（自己的）
        for(int i=0;i<n;i++)
        {
            mybest[i][2]=100000;
        }
        d1d globalbest(3,0);//當作最佳解的xyf(x)用
        globalbest[2]=100000;
        d2d vvector(n,d1d(2));
        randomstart(particle,n,dim);//隨機起始位置
        randomvector(vvector,n);//隨機ㄨ速度向量
        int whereami=0;
        double temp=globalbest[2];
        while(iteration<ITE)
        {
            // cout<<"Iteration "<<iteration+1<<':'<<endl;
            newposition(particle,vvector,mybest,globalbest,iteration);//更新位置
            if(globalbest[2]<temp)
            {
                temp=globalbest[2];
                whereami=iteration;
            }
            
            RUN_ITER_BEST[iteration]=globalbest[2];
            iteration++;
        }
        END=clock();
        FINAL_PRINT(R,globalbest,START,END,whereami);
        R++;
    }
    OUTPUT_FILE( ITE, RUN, RUN_ITER_BEST);

}
