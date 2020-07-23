#include "../include/calib_odom/Odom_Calib.hpp"
#include "stdio.h"

//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{
    /* datal_len : total data set len */
    /* now len: current data set len */ 
    
    /* need make sure row never larger then data_len*3 */
    if(now_len<INT_MAX)
    {
        //TODO: 构建超定方程组
        int i;
        printf("now len:%d\r\n", now_len);
        for(i=0; i<3; i++)
        {
            A(now_len % data_len*3 + 0, i) = Odom(i);
            A(now_len % data_len*3 + 1, i+3) = Odom(i);
            A(now_len % data_len*3 + 2, i+6) = Odom(i);
            
            b(now_len % data_len*3 + i) = scan(i);
        }

        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    
    //TODO: 求解线性最小二乘
    /* refer: https://eigen.tuxfamily.org/dox/group__LeastSquares.html */
    
    printf("A size: %d %dr\n", A.rows(), A.cols());
    
    
    Eigen::VectorXd vec_9x1 = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    
    
    correct_matrix << vec_9x1(0),vec_9x1(1),vec_9x1(2),
                      vec_9x1(3),vec_9x1(4),vec_9x1(5),
                      vec_9x1(6),vec_9x1(7),vec_9x1(8);
                      
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
