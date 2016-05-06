#include <iostream>


using namespace std;
// Let us assume that z_1 = distance and z_2 represents bearing measurements
//void EKF_Update_Dist_Bearing(int* x_hat_min, int** P_min,int* z_1,int* z_2,int** R,int g_match,int g_newlm)

// For the simplicity of programming it is assumed that the first element of every 1D pointer has its first element as its size.
// Since, we know that the matrix P and R are symmentric the number of rows = no of columns is got as a parameter


void EKF_Update_Dist_Bearing(int* z_1)
{
	int n_z = sizeof(*z_1); 
	cout<<n_z<<endl;
}
int main()
{
	int* val = new int[5];
	int count = 0;
	for(int i=0;i<5;i++)
	{
		val[i] = 1;
		count = count+1;
	}
	cout<<"In the main function"<<sizeof(*val)<<"\t"<<count<<endl;
	EKF_Update_Dist_Bearing(val);	
}
