// Setting
//#define EIGEN_USE_MKL_ALL
#define USE_OPENMP_FOR_EACH_REPEAT

// STL
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <bench/BenchTimer.h>

// MKL
#include <mkl_cblas.h>
#include <mkl_lapacke.h>

#ifdef EIGEN_USE_MKL_ALL
	#ifdef USE_OPENMP_FOR_EACH_REPEAT
		#define MODE			"Eigen with MKL and OpenMP"
		#define MV_FILE_NAME "./bench/matrix_vector_product_MKL_OpenMP.txt"
		#define MM_FILE_NAME "./bench/matrix_matrix_product_MKL_OpenMP.txt"
		#define CH_FILE_NAME "./bench/cholesky_MKL_OpenMP.txt"
		#define SV_FILE_NAME "./bench/triangular_solve_MKL_OpenMP.txt"
	#else
		#define MODE			"Eigen with MKL"
		#define MV_FILE_NAME "./bench/matrix_vector_product_MKL.txt"
		#define MM_FILE_NAME "./bench/matrix_matrix_product_MKL.txt"
		#define CH_FILE_NAME "./bench/cholesky_MKL.txt"
		#define SV_FILE_NAME "./bench/triangular_solve_MKL.txt"
	#endif
#else
	#ifdef USE_OPENMP_FOR_EACH_REPEAT
		#define MODE			"Eigen without MKL with OpenMP"
		#define MV_FILE_NAME "./bench/matrix_vector_product_OpenMP.txt"
		#define MM_FILE_NAME "./bench/matrix_matrix_product_OpenMP.txt"
		#define CH_FILE_NAME "./bench/cholesky_OpenMP.txt"
		#define SV_FILE_NAME "./bench/triangular_solve_OpenMP.txt"
	#else
		#define MODE			"Eigen without MKL"
		#define MV_FILE_NAME "./bench/matrix_vector_product.txt"
		#define MM_FILE_NAME "./bench/matrix_matrix_product.txt"
		#define CH_FILE_NAME "./bench/cholesky.txt"
		#define SV_FILE_NAME "./bench/triangular_solve.txt"
	#endif
#endif

#ifdef _FLOAT
typedef float Scalar;
#define CBLAS_GEMV	cblas_sgemv
#define CBLAS_GEMM	cblas_sgemm
#define LAPACK_POTRF	LAPACKE_spotrf
#define CBLAS_TRSV	cblas_strsv
#else
typedef double Scalar;
#define CBLAS_GEMV	cblas_dgemv
#define CBLAS_GEMM	cblas_dgemm
#define LAPACK_POTRF	LAPACKE_dpotrf
#define CBLAS_TRSV	cblas_dtrsv
#endif

// default storage order: column-major
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MyMatrix;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> MyVector;

// benchmark
template<typename Action>
class Bench : public Action
{
public:
	static void execute(const std::string filename, std::vector<int> &dynSizes, const int rep, const bool check = false)
	{
		// file
		std::ofstream fout(filename.c_str());
		//fout << "N\tavg time Eigen\tGFLOPS Eigen\tavg time MKL\tGFLPS MKL" << std::endl;

		int N;
		double EigenAvgTime,		MKLAvgTime;
		double EigenGFLOPS,	MKLGFLOPS;
		for(unsigned int i = 0; i < dynSizes.size(); ++i)
		{
			N = dynSizes[i];
			std::cout << "N = " << N << std::endl;
			Action::execute(N, rep, EigenAvgTime, EigenGFLOPS, MKLAvgTime, MKLGFLOPS, check);
			fout << N << "\t" << EigenAvgTime << "\t" << EigenGFLOPS << "\t" << MKLAvgTime << "\t" << MKLGFLOPS << std::endl;
		}
		fout.close();
	}
};

// actions
class MatrixVectorProduct
{
public:
	static void execute(const int N, const int rep, double &EigenAvgTime, double &EigenGFLOPS, double &MKLAvgTime, double &MKLGFLOPS, const bool check = false)
	{
		// FLOP
		const int M = N;
		const double FLOP = M * N * rep;

		// matrix and timer
		MyMatrix A(M, N);
		MyVector b(N), c(N), c2(N);
		const Scalar alpha = 1;
		const Scalar beta = 1;
		Eigen::BenchTimer EigenTimer, MKLTimer;
		double EigenTotalTime = 0.0;
		double MKLTotalTime = 0.0;

		// repeats
#ifdef USE_OPENMP_FOR_EACH_REPEAT
#pragma omp parallel for private(A,b,c,c2,EigenTimer,MKLTimer) reduction(+:EigenTotalTime,MKLTotalTime) schedule(dynamic,1)
#endif
		for(int i = 0; i < rep; i++)
		{
			// random initialization
			A = MyMatrix::Random(M, N);
			b = MyVector::Random(N);
			c = MyVector::Random(N);
			c2 = c;

			// Eigen
			EigenTimer.start();
			c.noalias() += A * b;
			EigenTimer.stop();

			// MKL
			MKLTimer.start();
			#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
			CBLAS_GEMV(CblasRowMajor, CblasNoTrans, M, N, alpha, A.data(), N, b.data(), 1, beta, c2.data(), 1);
			#else
			CBLAS_GEMV(CblasColMajor, CblasNoTrans, M, N, alpha, A.data(), N, b.data(), 1, beta, c2.data(), 1);
			#endif
			MKLTimer.stop();

			// check
			if(check)
			{
				if(!c.isApprox(c2, 1e-4))	std::cout << "wrong!" << std::endl;
				//else								std::cout << "correct!" << std::endl;
			}

			// time
			EigenTotalTime += EigenTimer.value();	// EigenTimer.total()
			MKLTotalTime += MKLTimer.value();		// MKLTimer.total()
		}

		// time
		EigenAvgTime	= EigenTotalTime / (double)rep;
		EigenGFLOPS		= 1e-9*FLOP/EigenTotalTime;
		MKLAvgTime		= MKLTotalTime / (double)rep;
		MKLGFLOPS		= 1e-9*FLOP/MKLTotalTime;
	}
};

class MatrixMatrixProduct
{
public:
	static void execute(const int N, const int rep, double &EigenAvgTime, double &EigenGFLOPS, double &MKLAvgTime, double &MKLGFLOPS, const bool check = false)
	{
		// FLOP
		const int M = N;
		const int K = N;
		const double FLOP = static_cast<double>(M) * static_cast<double>(N) * static_cast<double>(K) * static_cast<double>(rep); // to avoid overflow

		// matrix and timer
		MyMatrix A(M, K), B(K, N), C(M, N), C2(M, N);
		const Scalar alpha = 1;
		const Scalar beta = 1;
		Eigen::BenchTimer EigenTimer, MKLTimer;
		double EigenTotalTime = 0.0;
		double MKLTotalTime = 0.0;

		// repeats
#ifdef USE_OPENMP_FOR_EACH_REPEAT
#pragma omp parallel for private(A,B,C,C2,EigenTimer,MKLTimer) reduction(+:EigenTotalTime,MKLTotalTime) schedule(dynamic,1)
#endif
		for(int i = 0; i < rep; i++)
		{
			// random initialization
			A = MyMatrix::Random(M, K);
			B = MyMatrix::Random(K, N);
			C = MyMatrix::Random(M, N);
			C2 = C;

			// Eigen
			EigenTimer.start();
			C.noalias() += A * B;
			EigenTimer.stop();

			// MKL
			MKLTimer.start();
			#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
			CBLAS_GEMM(CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, alpha, A.data(), K, B.data(), N, beta, C2.data(), N);
			#else
			CBLAS_GEMM(CblasColMajor, CblasNoTrans, CblasNoTrans, M, N, K, alpha, A.data(), M, B.data(), K, beta, C2.data(), M);
			#endif
			MKLTimer.stop();

			// check
			if(check)
			{
				if(!C.isApprox(C2, 1e-4))	std::cout << "wrong!" << std::endl;
				//else								std::cout << "correct!" << std::endl;
			}

			// time
			EigenTotalTime += EigenTimer.value();	// EigenTimer.total()
			MKLTotalTime += MKLTimer.value();		// MKLTimer.total()
		}

		// time
		EigenAvgTime	= EigenTotalTime / (double)rep;
		EigenGFLOPS		= 1e-9*FLOP/EigenTotalTime;
		MKLAvgTime		= MKLTotalTime / (double)rep;
		MKLGFLOPS		= 1e-9*FLOP/MKLTotalTime;
	}
};

class Cholesky
{
public:
	static void execute(const int N, const int rep, double &EigenAvgTime, double &EigenGFLOPS, double &MKLAvgTime, double &MKLGFLOPS, const bool check = false)
	{
		// FLOP
		double FLOP = 0;
		for (int i = 0; i < N; ++i)
		{
			int r = std::max(N - i -1, 0);
			FLOP += 2*(r*i+r+i);
		}
		FLOP *= static_cast<double>(rep);

		// matrix and timer
		MyMatrix A(N, N), L(N, N);
		Eigen::BenchTimer EigenTimer, MKLTimer;
		double EigenTotalTime = 0.0;
		double MKLTotalTime = 0.0;

		// repeats
#ifdef USE_OPENMP_FOR_EACH_REPEAT
#pragma omp parallel for private(A,L,EigenTimer,MKLTimer) reduction(+:EigenTotalTime,MKLTotalTime) schedule(dynamic,1)
#endif
		for(int i = 0; i < rep; i++)
		{
			// random initialization
			A = MyMatrix::Random(N, N);
			A = A * A.transpose();

			// Eigen
			EigenTimer.start();
			L = A.llt().matrixL(); // equivalent to Eigen::LLT<MyMatrix> L(A); L'=L.matrixL();
			EigenTimer.stop();

			// MKL
			MKLTimer.start();
			#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
			LAPACK_POTRF(LAPACK_ROW_MAJOR, 'L', N, &(A.coeffRef(0,0)), A.outerStride());
			#else
			LAPACK_POTRF(LAPACK_COL_MAJOR, 'L', N, &(A.coeffRef(0,0)), A.outerStride());
			#endif
			MKLTimer.stop();

			// check
			if(check)
			{
				A.triangularView<Eigen::StrictlyUpper>().setZero();
				if(!L.isApprox(A, 1e-4))	std::cout << "wrong!" << std::endl;
				//else								std::cout << "correct!" << std::endl;
			}

			// time
			EigenTotalTime += EigenTimer.value();	// EigenTimer.total()
			MKLTotalTime += MKLTimer.value();		// MKLTimer.total()
		}

		// time
		EigenAvgTime	= EigenTotalTime / (double)rep;
		EigenGFLOPS		= 1e-9*FLOP/EigenTotalTime;
		MKLAvgTime		= MKLTotalTime / (double)rep;
		MKLGFLOPS		= 1e-9*FLOP/MKLTotalTime;
	}
};

class TriSolve
{
public:
	static void execute(const int N, const int rep, double &EigenAvgTime, double &EigenGFLOPS, double &MKLAvgTime, double &MKLGFLOPS, const bool check = false)
	{
		// FLOP
		double FLOP = 0;
		for (int i = 0; i < N; ++i)
		{
			FLOP += 2*i+1;
		}
		FLOP *= static_cast<double>(rep);

		// matrix and timer
		MyMatrix L(N, N);
		MyVector b(N), x(N);
		Eigen::BenchTimer EigenTimer, MKLTimer;
		double EigenTotalTime = 0.0;
		double MKLTotalTime = 0.0;

		// repeats
#ifdef USE_OPENMP_FOR_EACH_REPEAT
#pragma omp parallel for private(L,b,x,EigenTimer,MKLTimer) reduction(+:EigenTotalTime,MKLTotalTime) schedule(dynamic,1)
#endif
		for(int i = 0; i < rep; i++)
		{
			// random initialization
			L = MyMatrix::Random(N, N);
			L += static_cast<Scalar>(3) * MyMatrix(N, N).setIdentity();
			//L.triangularView<Eigen::StrictlyUpper>().setZero();
			b = MyVector::Random(N);

			// Eigen
			EigenTimer.start();
			x = L.triangularView<Eigen::Lower>().solve(b);
			EigenTimer.stop();

			// MKL
			MKLTimer.start();
			#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
			CBLAS_TRSV(CblasRowMajor, CblasLower, CblasNoTrans, CblasNonUnit, N, L.data(), L.outerStride(), &(b.coeffRef(0,0)), 1);
			#else
			CBLAS_TRSV(CblasColMajor, CblasLower, CblasNoTrans, CblasNonUnit, N, L.data(), L.outerStride(), &(b.coeffRef(0,0)), 1);
			#endif
			MKLTimer.stop();

			// check
			if(check)
			{
				if(!x.isApprox(b, 1e-4))	std::cout << "wrong!" << std::endl;
				//else								std::cout << "correct!" << std::endl;
			}

			// time
			EigenTotalTime += EigenTimer.value();	// EigenTimer.total()
			MKLTotalTime += MKLTimer.value();		// MKLTimer.total()
		}

		// time
		EigenAvgTime	= EigenTotalTime / (double)rep;
		EigenGFLOPS		= 1e-9*FLOP/EigenTotalTime;
		MKLAvgTime		= MKLTotalTime / (double)rep;
		MKLGFLOPS		= 1e-9*FLOP/MKLTotalTime;
	}
};

int main()
{
	std::cout << MODE << std::endl;

	const bool check = true;
	const int rep = 5;
	const int dynSizes_[] = {/*4,8,16,32,64,*/128,256,512,1024,2048,4096/*,8192,16384*/};
	std::vector<int> dynSizes(dynSizes_, dynSizes_ + sizeof(dynSizes_)/sizeof(int));

	std::cout << "matrix vector product (c += A * b)" << std::endl;
	Bench<MatrixVectorProduct>::execute(MV_FILE_NAME, dynSizes, rep, check);
	std::cout << "matrix matrix product (C += A * B)" << std::endl;
	Bench<MatrixMatrixProduct>::execute(MM_FILE_NAME, dynSizes, rep, check);
	std::cout << "cholesky factorization (A = LL')" << std::endl;
	Bench<Cholesky>::execute(CH_FILE_NAME, dynSizes, rep, check);
	std::cout << "triangular solver (Lx = b)" << std::endl;
	Bench<TriSolve>::execute(SV_FILE_NAME, dynSizes, rep, check);

	system("pause");
	return 0;
}


/*
#define min(x,y) (((x) < (y)) ? (x) : (y))
#include <stdio.h>
#include <stdlib.h>
#include "mkl.h"

int main()
{
	double *A, *B, *C;
	int m, n, k, i, j;
	double alpha, beta;

	printf ("\n This example computes real matrix C=alpha*A*B+beta*C using \n"
	" Intel(R) MKL function dgemm, where A, B, and C are matrices and \n"
	" alpha and beta are double precision scalars\n\n");

	// A: mxk
	// B: kxn
	// C = A*B: mxn
	m = 2000, k = 200, n = 1000;
	printf (" Initializing data for matrix multiplication C=A*B for matrix \n"
	" A(%ix%i) and matrix B(%ix%i)\n\n", m, k, k, n);

	alpha = 1.0; beta = 0.0;

	// memory allocation
	printf (" Allocating memory for matrices aligned on 64-byte boundary for better \n"
	" performance \n\n");
	A = (double *)mkl_malloc( m*k*sizeof( double ), 64 );
	B = (double *)mkl_malloc( k*n*sizeof( double ), 64 );
	C = (double *)mkl_malloc( m*n*sizeof( double ), 64 );
	if (A == NULL || B == NULL || C == NULL)
	{
		printf( "\n ERROR: Can't allocate memory for matrices. Aborting... \n\n");
		mkl_free(A);
		mkl_free(B);
		mkl_free(C);
		return 1;
	}

	// initialization
	printf (" Intializing matrix data \n\n");
	for (i = 0; i < (m*k); i++)
	{
		A[i] = (double)(i+1);
	}
	for (i = 0; i < (k*n); i++)
	{
		B[i] = (double)(-i-1);
	}
	for (i = 0; i < (m*n); i++)
	{
		C[i] = 0.0;
	}

	// matrix product
	printf (" Computing matrix product using Intel(R) MKL dgemm function via CBLAS interface \n\n");
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, m, n, k, alpha, A, k, B, n, beta, C, n);
	printf ("\n Computations completed.\n\n");

	// print
	printf (" Top left corner of matrix A: \n");
	for (i=0; i<min(m,6); i++)
	{
		for (j=0; j<min(k,6); j++)
		{
			printf ("%12.0f", A[j+i*k]);
		}
		printf ("\n");
	}

	printf ("\n Top left corner of matrix B: \n");
	for (i=0; i<min(k,6); i++)
	{
		for (j=0; j<min(n,6); j++)
		{
			printf ("%12.0f", B[j+i*n]);
		}
		printf ("\n");
	}

	printf ("\n Top left corner of matrix C: \n");
	for (i=0; i<min(m,6); i++)
	{
		for (j=0; j<min(n,6); j++)
		{
			printf ("%12.5G", C[j+i*n]);
		}
		printf ("\n");
	}

	// deallocation
	printf ("\n Deallocating memory \n\n");
	mkl_free(A);
	mkl_free(B);
	mkl_free(C);

	printf (" Example completed. \n\n");
	return 0;
}
*/