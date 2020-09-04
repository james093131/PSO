//
//  main.cpp
//  PSO
//
//  Created by TzuChieh on 2020/07/29
//  Copyright © 2020 TzuChieh. All rights reserved.
//

#include "function.hpp"

int main(int argc, const char * argv[]) {
    // insert code here...
    srand((unsigned int)time(NULL));
    int ITE = atoi(argv[1]);
    int iteration=1;
    double iterbest[ITE];
    memset(iterbest,0,sizeof(iterbest));
    double particle[n][dim];//用來儲存n個粒子的xyz圖
    double mybest[n][dim];//用來儲存n個粒子目前最好的ｘｙｚ（自己的）
    memset(mybest,0,sizeof(mybest));
    for(int i=0;i<n;i++)
    {
        mybest[i][2]=100000;
    }
    double globalbest[3]={0,0,100000};//當作最佳解的xyf(x)用
    double vvector[n][2];
    randomstart((double*)particle,n,dim);//隨機起始位置
    randomvector((double*)vvector,n);//隨機ㄨ速度向量
    int whereami=0;
    double temp=globalbest[2];
    while(iteration<ITE)
    {
        cout<<"Iteration "<<iteration+1<<':'<<endl;
        newposition((double*)particle,(double*)vvector,(double*)mybest,globalbest,iteration);//更新位置
        if(globalbest[2]<temp)
        {
            temp=globalbest[2];
            whereami=iteration;
        }
        cout<<'('<<globalbest[0]<<" ,"<<globalbest[1]<<')'<<endl<<"optima :"<<globalbest[2]<<endl;
        cout<<"optima is come from the "<<whereami<<" iteration  "<<endl<<endl;
        iterbest[iteration]=globalbest[2];
        iteration++;
    }
    fstream file;//寫檔
    file.open("PSO_Convergence.txt",ios::out);
    for(int i=0;i<ITE;i++)
    {
     file<<i+1<<' '<<iterbest[i]<<endl;
    }
}
