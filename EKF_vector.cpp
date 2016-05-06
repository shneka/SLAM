#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

using namespace std;
// Multiplication of matrix
vector< vector<float> > mul2(vector< vector<float> > A,vector< vector<float> > B)
{
	int n = A.size();
	int m_1 = A[0].size();
	int m_2 = B.size();
	int p = B[0].size();
	vector< vector<float> > C( n, vector<float> ( p, 0 ) );
	if(m_1 != m_2)
	{
		cout<<"Matrix dimension mismatch"<<endl;
		exit (EXIT_FAILURE);
	}
	else
	{
		//vector< vector<float> > AB;
		for (int i=0;i<n;i++)
		{
			for(int j=0;j<p;j++)
			{
				float sum = 0;
				for(int k=0;k<m_1;k++)
				{
					sum +=(A[i][k]*B[k][j]);
				}
				C[i][j] = sum;
			}
		}	
		return C;
	}
}
// Multiplication of 3 matrices
vector< vector<float> > mul3(vector< vector<float> > A,vector< vector<float> > B,vector< vector<float> > D)
{
	vector< vector<float> > AB;
	vector< vector<float> > ABC;
	AB = mul2(A,B);
	ABC = mul2(AB,D);
	return ABC;
}

// To do matrix addition of two given matrices
vector< vector<float> > add2(vector< vector<float> > A,vector< vector<float> > B)
{
	int n = A.size();
	int m_1 = A[0].size();
	int m_2 = B.size();
	int p = B[0].size();
	vector< vector<float> > C( n, vector<float> ( p, 0 ) );
	if(n != m_2 || m_1 !=p)
	{
		cout<<"Matrix dimension mismatch"<<endl;
		exit (EXIT_FAILURE);
	}
	else
	{
		//vector< vector<float> > AB;
		for (int i=0;i<n;i++)
		{
			for(int j=0;j<m_1;j++)
			{
				C[i][j] = A[i][j]+B[i][j];
			}
		}	
		return C;
	}
}
// To do matrix sub
vector< vector<float> > sub2(vector< vector<float> > A,vector< vector<float> > B)
{
	int n = A.size();
	int m_1 = A[0].size();
	int m_2 = B.size();
	int p = B[0].size();
	vector< vector<float> > C( n, vector<float> ( p, 0 ) );
	if(n != m_2 || m_1 !=p)
	{
		cout<<"Matrix dimension mismatch"<<endl;
		exit (EXIT_FAILURE);
	}
	else
	{
		//vector< vector<float> > AB;
		for (int i=0;i<n;i++)
		{
			for(int j=0;j<m_1;j++)
			{
				C[i][j] = A[i][j]-B[i][j];
			}
		}	
		return C;
	}
}

// To find the transpose of a matrix
vector< vector<float> > tran(vector< vector<float> > A)
{
	int n = A.size();
	int m = A[0].size();
	vector< vector<float> > C( m, vector<float> ( n, 0 ) );
	
	for (int i=0;i<n;i++)
	{
		for(int j=0;j<m;j++)
		{
			C[j][i] = A[i][j];
		}
	}	
		return C;
}

// To find the inverse of a matrix (S matrix which is a 2X2 matrix)
vector< vector<float> > inverse(vector< vector<float> > A)
{
	// To find the determinant of a 2X2 Matrix
	// Runs in fixed time.
	int determinant = (A[0][0]*A[1][1]) + (A[0][1]*A[1][0]);
	vector< vector<float> > A_inv ( 2, vector<float> ( 2, 0 ) );
	if (determinant == 0)
	{
		cout<<"Matrix cannot be inverted"<<endl;
		exit (EXIT_FAILURE);	
	} 
	else
	{
		A_inv[0][0] = A[1][1]/determinant;
		A_inv[0][1] = -A[0][1]/determinant;
		A_inv[1][0] = -A[0][1]/determinant;
		A_inv[1][1] = A[0][0]/determinant;
	}
	return A_inv;
}

float cond(vector<vector<float> > A)
{
	float T  = A[0][0] + A[1][1];
	float D = (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
	float eig_1 = T/2+sqrt((T*T)/4-D);
	float eig_2 = T/2-sqrt((T*T)/4-D) ;
	float val = abs(eig_1/eig_2);
	return val;
}

// This program uses the solution provided for assignment 4

//void EKF_Update_Dist_Bearing(vector<float> x_hat_min,vector<float> z_1,vector<float> z_2,vector< vector<float> > R)
void EKF_Update_Dist_Bearing(vector<float> x_hat_min,vector<vector<float> > P_min,vector<float> z_1,vector<float> z_2,vector<vector<float> >R, int g_match,int g_newlm)
{
	int n_z = z_1.size(); // no. of measured landmarks
	vector<float> z_p1(n_z);// To store the values in the x axis
	vector<float> z_p2(n_z); // To store the values in y axis
	vector<vector<vector<float> > > Rpi (2,vector<vector<float> >(2,vector <float>(n_z,0)));

	int n_m = (x_hat_min.size()-3)/2; // no of landmarks in map
	
	//cout<<n_z<<"\t"<<n_m<<endl; check the proper working of this line
	
	// Precomputations
	
	// J matrix
	vector< vector<float> > J ( 2, vector<float> ( 2, 0 ) );
	J[0][1] = -1;
	J[1][0] = 1;
	// compute relative position measurements and covariance
	
	vector< vector<float> >	 inter_R( 2, vector<float> ( 2, 0 ) ); 
	for(int i=0;i<n_z;i++)
	{
		z_p1[i]	= z_1[i]*cos(z_2[i]);
		z_p2[i] = z_1[i]*sin(z_2[i]);
		
		vector< vector<float> >	 G( 2, vector<float> ( 2, 0 ) );
		G[0][0] = cos(z_2[i]);
		G[0][1] = -z_p2[i];
		G[1][0] = sin(z_2[i]);
		G[1][1] = z_p1[i];
		
		inter_R = mul3(G,R,tran(G));
		
		for(int p=0;p<2;p++)
			for(int q=0;q<2;q++)
				Rpi[p][q][i] = inter_R[p][q];
	}

	// rotational matrix
	vector< vector<float> > C ( 2, vector<float> ( 2, 0 ) );
	C[0][0] = cos(x_hat_min[3]);
	C[0][1] = -sin(x_hat_min[3]);
	C[1][0] = sin(x_hat_min[3]);
	C[1][1] = cos(x_hat_min[3]);
		
	// C minus declaration for the working of C_min
	vector< vector<float> > C_min_t ( 2, vector<float> ( 2, 0 ) );
	C_min_t[0][0] = -cos(x_hat_min[3]);
	C_min_t[0][1] = -sin(x_hat_min[3]);
	C_min_t[1][0] = sin(x_hat_min[3]);
	C_min_t[1][1] = -cos(x_hat_min[3]);
	
	// Data association
	// cycle through all the measurements		
	 for(int j=0;j<n_z;j++)
	 {
	 	// Pre computation
	 	// rotational matrix
		
		vector< vector<float> > rij ( 2, vector<float> ( n_m, 0 ) );
		vector<vector<vector<float> > > Hj (2,vector<vector<float> >(x_hat_min.size(),vector <float>(n_m,0)));
		// Temp value to store the value of H since direct indexing is not possible
		vector<vector<float> > Hj_store(2,vector<float>(x_hat_min.size(),0));
		vector<vector<vector<float> > > Sij_inv (2,vector<vector<float> >(2,vector <float>(n_m,0)));
		vector< vector<float> > mdij ( n_m, vector<float> ( 1, 0 ) );
		// compare measurement i with landmark j already in the map
		// and compute Mahalanobis distance mdij
		
		// Required Pre calculation for Rpi
		vector <vector<float> > Rpi_inter(2,vector<float>(2,0));
		for(int p=0;p<2;p++)
			for(int q=0;q<2;q++)
				Rpi_inter[p][q] = Rpi[p][q][j];
		
		for(int k=0;k<n_m;k++)
		{
			int lmj_ind[2];
			lmj_ind[0] = 3+2*j-1;
			lmj_ind[1] = 3+2*j-1+1;
			
			// expected measurement
			vector <vector<float> > x_matrix( 2, vector<float> (1, 0 ) );
			vector <vector<float> > z_hat_j( 2, vector<float> (1, 0 ) );
			x_matrix[0][0] = x_hat_min[lmj_ind[0]] - x_hat_min[0];
			x_matrix[1][0] = x_hat_min[lmj_ind[1]] - x_hat_min[1];
			z_hat_j = mul2(tran(C),x_matrix);
			
			// Measurement Jacobian
			vector< vector<float> > H_inter ( 2, vector<float> (3, 0 ) );	
			vector< vector<float> > Hj_mat ( 2, vector<float> (3, 0 ) );
			H_inter[0][0] = 1;
			H_inter[0][1] = 0;
			H_inter[1][0] = 0;
			H_inter[1][1] = 1;
			vector <vector<float> > Hj_inter( 2, vector<float> (1, 0 ) );
			Hj_inter = mul2(J,x_matrix);
			H_inter[2][0] = Hj_inter[0][0];
			H_inter[2][1] = Hj_inter[1][0];
			
			Hj_mat = mul2(C_min_t,H_inter);
			// To copy to get the proper values of H matrix
			for(int iter_1=0;iter_1<2;iter_1++)
				for(int iter_2=0;iter_2<3;iter_2++)
				{
					Hj[iter_1][iter_2][k] = Hj_mat[iter_1][iter_2]; 				
					Hj_store[iter_1][iter_2] = Hj_mat[iter_1][iter_2];
				}
			Hj[0][lmj_ind[0]][k] = cos(x_hat_min[3]);
			Hj_store[0][lmj_ind[0]]  = cos(x_hat_min[3]);
			
			Hj[0][lmj_ind[1]][k] = sin(x_hat_min[3]);
			Hj_store[0][lmj_ind[1]] = sin(x_hat_min[3]);
			
			Hj[1][lmj_ind[0]][k] = -sin(x_hat_min[3]);
			Hj_store[1][lmj_ind[0]] = -sin(x_hat_min[3]);
			
			Hj[1][lmj_ind[1]][k] = cos(x_hat_min[3]);
			Hj_store[1][lmj_ind[1]] = cos(x_hat_min[3]);
			
			rij[0][k] = z_p1[k] - z_hat_j[0][0];
			rij[1][k] = z_p2[k] - z_hat_j[1][0];
			
			vector <vector<float> > r_inter(2,vector<float>(1,0));
			r_inter[0][0] = rij[0][k];
			r_inter[1][0] = rij[1][k];
			
			vector <vector<float> > Sij_inter(2,vector<float>(2,0));
			Sij_inter = inverse(add2(mul3(Hj_store,P_min,tran(Hj_store)),Rpi_inter));
			
			for(int S_i=0;S_i<2;S_i++)
			{
				for(int S_j=0;S_j<2;S_j++)
					Sij_inv[S_i][S_j][k] = Sij_inter[S_i][S_j];	
			}				
			vector<vector<float> > mdij_inter(1,vector<float>(1,0));
			//mdij[k][0] = mul3(tran(rij),Sij_inter,rij);
			mdij_inter = mul3(tran(rij),Sij_inter,rij);
			mdij[k][0] = mdij_inter[0][0];
		}
		int minmd = 10000;
		int minj = 0;
		// Find the minimum mahalanobis distance
		for(int min_i=0;min_i<n_m;min_i++)
		{
			if(mdij[min_i][0] < minmd)
				minmd = mdij[min_i][0];
				minj = min_i;		
		}
	 	// update or initialize based on the minimm mahalanobis distance
		int lm_size = x_hat_min.size();
		if (minmd== 10000 || minmd> g_newlm)
		{
			cout<<"Initialize new landmark"<<endl;
			// Position of new landmark
			vector<vector<float> > P_newlm(2,vector<float>(1,0));
			P_newlm[0][0] = x_hat_min[0] + (C[0][0]*z_p1[j] + C[0][1]*z_p2[j]);
			P_newlm[0][1] = x_hat_min[1] + (C[1][0]*z_p1[j] + C[1][1]*z_p2[j]);
			// Add to the state vector
			x_hat_min.resize(lm_size+2);
			x_hat_min[lm_size] = P_newlm[0][0]; // size -1 = index -- check in all places
			x_hat_min[lm_size+1] = P_newlm[0][1];
			 
			//Jacobian
			vector <vector<float> > xR_matrix( 2, vector<float> (1, 0 ) );
			
			xR_matrix[0][0] = P_newlm[0][0] - x_hat_min[0];
			xR_matrix[1][0] = P_newlm[0][1] - x_hat_min[1];
			
			vector<vector<float> > HR_inter(2,vector<float>(3,0));
			HR_inter[0][0] = 1;
			HR_inter[0][1] = 0;
			HR_inter[1][0] = 0;
			HR_inter[1][1] = 1;
			vector <vector<float> > HRj_inter( 2, vector<float> (1, 0 ) );
			HRj_inter = mul2(J,xR_matrix);
			HR_inter[2][0] = HRj_inter[0][0];
			HR_inter[2][1] = HRj_inter[1][0];
			
			vector <vector<float> >HR_mat(2,vector<float>(3,0));
			HR_mat = mul2(C_min_t,HR_inter);
			// Remember H_li is just a C_min_t
			// Altering the covariance matrix
			vector<vector<float> >P_RR(3,vector<float>(3,0));
			vector<vector<float> >P_LR(lm_size,vector<float>(3,0));
			for(int r_i=0;r_i<lm_size;r_i++)
			{
				for(int r_j=0;r_j<3;r_j++)
				{
					P_LR[r_i][r_j] = -P_min[r_i][r_j];
					if (r_i<3)
						P_RR[r_i][r_j] = P_min[r_i][r_j];
				}
			}			
			
			vector<vector<float> > P_LL(2,vector<float>(2,0));
			vector<vector<float> > P_xL(lm_size,vector<float>(2,0));
			vector<vector<float> > P_xl_t(2,vector<float>(lm_size,0));
			
			P_LL = mul3(C,add2(mul3(HR_mat,P_RR,tran(HR_mat)),Rpi_inter),tran(C));
			P_xL = mul3(P_LR,HR_mat,tran(C));
			
			P_min.resize(lm_size+2,vector<float>(lm_size+2));
			for(int P_i=0;P_i<lm_size;P_i++)
			{
				for(int P_j=0;P_j<2;P_j++)
				{
					P_min[lm_size+P_i][P_j] = P_xL[P_i][P_j]; 
					P_min[P_j][lm_size+P_i] = P_xl_t[P_j][P_i];
					if(P_i<2)
					{
						P_min[lm_size+P_i][lm_size+P_j] = P_LL[P_i][P_j];
					}
				}
			}
		}
		else if(minmd < g_match)
		{
			// Matched measurement
			// Regular Update
			vector <vector<float> > Sij(2,vector<float>(2,0));
			vector <vector<float> > Sij_fin(2,vector<float>(2,0));
			for(int S_i=0;S_i<2;S_i++)
			{
				for(int S_j=0;S_j<2;S_j++)
				{
					Sij[S_i][S_j] = Sij_inv[S_i][S_j][j];
					Sij_fin[S_i][S_j]= Sij_inv[S_i][S_j][minj];
				}
			}
			if(cond(Sij) > 80)
			{
				cout<<"Aborted due to ill-conditioned S";
				continue;
			}
			// Kalman Gain
			vector <vector<float> > Hij_fin(2,vector<float>(lm_size,0));
			for(int S_i=0;S_i<2;S_i++)
			{
				for(int S_j=0;S_j<lm_size;S_j++)
				{
					Hij_fin[S_i][S_j]= Hj[S_i][S_j][minj];
				}
			}
			vector <vector<float> > K(lm_size,vector<float>(2,0));
			K = mul3(P_min,tran(Hij_fin),Sij_fin);
			
			vector <vector<float> > r_fin(2,vector<float>(1,0));
			r_fin[0][0] = rij[0][minj];
			r_fin[1][0] = rij[1][minj];
			// State Update
			vector <vector<float> > x_inter(lm_size,vector<float>(1,0));
			x_inter = mul2(K,r_fin);
			for(int i=0;i<lm_size;i++)
			{
				x_hat_min[i] = x_hat_min[i]+x_inter[i][0];
			}
			// Covariance update (Joseph Form for Numerical Stability)
			vector <vector<float> > Iden(lm_size,vector<float>(lm_size,0));
			for(int I_i=0;I_i<lm_size;I_i++)
			{
				for(int I_j=0;I_j<lm_size;I_j++)
					Iden[I_i][I_j] = 1;
			}
			P_min = add2(mul3(sub2(Iden,mul2(K,Hij_fin)),P_min,tran(sub2(Iden,mul2(K,Hij_fin)))),mul3(K,Rpi_inter,tran(K))); 
		}
		// Residual and Sij can be entered in a seperate file for every run 	
	}
    int lmm = x_hat_min.size();
    vector<float>  x_hat_plus(lmm);
    vector <vector<float> > P_plus(lmm,vector<float>(lmm,0));
    vector <vector<float> > P_min_t(lmm,vector<float>(lmm,0));
    for(int l_i=0;l_i<lmm;l_i++)
    {
    	//x_hat_plus[l_i] = x_hat_min[l_i];
		for(int l_j=0;l_j<lmm;l_j++)
		{
			P_plus[l_i][l_j] = (P_min[l_i][l_j]+P_min_t[l_i][l_j])/2;
		}
	}
 }
int main()
{
	vector<float> value_1(5);
	vector<float> value_2(5);
	for(int i=0;i<5;i++)
	{
		value_1[i]= 1;
		value_2[i] = 0.5;
	}	
	vector< vector<float> > R ( 2, vector<float> ( 2, 0 ) );
	for(int i=0;i<2;i++)
	{
		for(int j=0;j<2;j++)
			R[i][j] = 0.05;
	}
	//EKF_Update_Dist_Bearing(value_1,value_1,value_2,R);
}
