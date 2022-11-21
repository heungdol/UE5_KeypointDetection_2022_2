#include "Descriptor_HKS.h"
#include "../MyUtil/Mesh.h"
#include "../MyUtil/MeshIO.h"
#include "../MyUtil/MultiresMesh.h"

#include "../spectra/include/MatOp/SparseSymMatProd.h"
#include "../spectra/include/MatOp/SparseCholesky.h"
#include "../spectra/include/SymGEigsSolver.h"
#include "../spectra/include/SymEigsShiftSolver.h"
#include "../spectra/include/SymEigsBase.h"

#include <ThirdParty/Eigen/Eigen/Eigenvalues>

#define N 10

Descriptor_HKS::Descriptor_HKS()
{
}

Descriptor_HKS::Descriptor_HKS(Mesh *mesh0, float t, int depth):
mesh(mesh0), m_t(t), m_depth(depth)
{
    
}
void buildAdjacency(Mesh *mesh, Eigen::SparseMatrix<double>& W)
{
    std::vector<Eigen::Triplet<double>> WTriplets;

    //int index = 0;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++)
        {
         //index++;
         //if ((index-1) != mesh->overlappingVert[index-1])
         //    continue;
        
        HalfEdgeCIter he = v->he;
        double sumCoefficients = 0.0;
        do {
            double coefficient = 0.5 * (he->cotan() + he->flip->cotan());
            if (coefficient < 0.0) coefficient = 0.0;
            sumCoefficients += coefficient;
            
            WTriplets.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index, -coefficient));
            
            he = he->flip->next;
        } while (he != v->he);
        
        WTriplets.push_back(Eigen::Triplet<double>(v->index, v->index,
                                                   sumCoefficients + 1e-8));
    }
    
    W.setFromTriplets(WTriplets.begin(), WTriplets.end());
}

void buildAreaMatrix(Mesh *mesh, Eigen::SparseMatrix<double>& A, const double scale)
{
    std::vector<Eigen::Triplet<double>> ATriplets;
    
    double sum = 0.0;
    //int index = 0;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++)
        {
        // index++;
        // if ((index-1) != mesh->overlappingVert[index-1])
        //     continue;
        
        double area = v->dualArea();
        ATriplets.push_back(Eigen::Triplet<double>(v->index, v->index, area));
        sum += area;
    }
    
    A.setFromTriplets(ATriplets.begin(), ATriplets.end());
    A *= scale/sum;
}

void Descriptor_HKS::computeEig(int K)
{
    int v = (int)mesh->vertices.size();
        
    // build adjacency operator
    Eigen::SparseMatrix<double> W(v, v);
    buildAdjacency(mesh, W);
        
    // build area matrix
    Eigen::SparseMatrix<double> A(v, v);
    buildAreaMatrix(mesh, A, v);
        
    //compute eigenvectors and eigenvalues
    Spectra::SparseSymMatProd<double> opW(W);
    Spectra::SparseCholesky<double> opA(A);

    // K 재설정
    int _K = (v / 100) * 100 / 2;
    K = std::min (_K, K);
    
    /*Spectra::SymGEigsSolver<double,
    Spectra::SMALLEST_MAGN,
    Spectra::SparseSymMatProd<double>,
    Spectra::SparseCholesky<double>,
    Spectra::GEIGS_CHOLESKY> geigs(&opW, &opA, K, 2*K);
        
    geigs.init();
    geigs.compute();
        
    if (geigs.info() == Spectra::SUCCESSFUL) {
         evals = geigs.eigenvalues();
         evecs = geigs.eigenvectors();
            
    } else {
        std::cout << "Eigen computation failed" << std::endl;
    }*/

    Spectra::SymGEigsSolver<
    Spectra::SparseSymMatProd<double>,
    Spectra::SparseCholesky<double>,
    Spectra::GEigsMode::Cholesky> geigs(opW, opA, K, 2*K);
        
    geigs.init();
    geigs.compute();
        
    if (geigs.info() == Spectra::CompInfo::Successful) {
        evals = geigs.eigenvalues();
        evecs = geigs.eigenvectors();
            
    } else {
        std::cout << "Eigen computation failed" << std::endl;
    }
        
    Eigen::MatrixXd err = W*evecs - A*evecs*evals.asDiagonal();
    std::cout << "||Lx - λAx||_inf = " << err.array().abs().maxCoeff() << std::endl;
    
}

void Descriptor_HKS::computeHks()
{
    const int K = (int)evals.size();
    const double ln = 4*log(10);
    const double tmin = ln/evals(0);
    const double step = (ln/evals(K-2) - tmin) / N;
    
    for (VertexIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        v->descriptor = Eigen::VectorXd::Zero(N);
        Eigen::VectorXd C = Eigen::VectorXd::Zero(N);
        
        for (int j = K-2; j >= 0; j--) {
            double phi2 = evecs(v->index, j)*evecs(v->index, j);
            double t = tmin;
            double factor = 0.5;
            
            for (int i = 0; i < N; i++) {
                double exponent = exp(-evals(j)*t);
                v->descriptor(i) += phi2*exponent;
                C(i) += exponent;
                t += factor*step;
                
                factor += 0.1;
            }
        }
        
        // normalize
        for (int i = 0; i < N; i++) {
            v->descriptor(i) /= C(i);
        }
    }
}

void extrapolateEvals(double& xhat, double& yhat, double& m, const Eigen::VectorXd& evals)
{
    // compute averages
    const int K = (int)evals.size();
    xhat = 0.0; yhat = 0.0;
    for (int i = 0; i < K; i++) {
        xhat += i;
        yhat += evals(i);
    }
    xhat /= K; yhat /= K;
    
    // compute slope
    double den = 0.0; m = 0.0;
    for (int i = 0; i < K; i++) {
        m += (i - xhat)*(evals(i) - yhat);
        den += (i - xhat)*(i - xhat);
    }
    m /= den;
}

void computeBinomialEntries(std::vector<Eigen::SparseMatrix<double>>& binomialSeries,
                            const Eigen::SparseMatrix<double>& L)
{
    int k = 0;
    Eigen::SparseMatrix<double> Id(L.cols(), L.cols()); Id.setIdentity();
    Eigen::SparseMatrix<double> Q = Id;
    for (int m = 0; m < (int)binomialSeries.size(); m++) {
        if (k == m-1) {
            Q = Q*(L - k*Id);
            k++;
        }
        
        if (m > 0) Q /= m;
        binomialSeries[m] = Q;
    }
}

void computeExponentialRepresentation(Eigen::SparseMatrix<double>& Kt, const double t,
                                      const std::vector<Eigen::SparseMatrix<double>>& binomialSeries)
{
    Kt.setZero();
    for (int m = 0; m < (int)binomialSeries.size(); m++) {
        Kt += binomialSeries[m]*pow(exp(-t) - 1, m);
    }
}

void sparsify(Eigen::SparseMatrix<double>& Kt, double eps)
{
    for (int i = 0; i < Kt.outerSize(); i++) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Kt, i); it; ++it) {
            if (it.valueRef() < eps) it.valueRef() = 0.0;
        }
    }
    Kt.prune(0.0);
}

double computeGaussCurvature(Mesh *mesh, Eigen::VectorXd& K)
{
    double maxGauss = -INFINITY;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        K(v->index) = v->angleDefect() / v->dualArea();
        if (maxGauss < fabs(K(v->index))) maxGauss = fabs(K(v->index));
    }
    
    return maxGauss;
}

double computeMeanCurvature(Mesh *mesh, Eigen::VectorXd& H)
{
    int vsize = (int)mesh->vertices.size();
    
    // build laplace matrix
    Eigen::SparseMatrix<double> L(vsize, vsize);
    buildAdjacency(mesh, L);
    Eigen::SparseMatrix<double> A(vsize, vsize);
    buildAreaMatrix(mesh, A, 1.0);
    L = A.cwiseInverse()*L;
    
    Eigen::MatrixXd x;
    x.resize((int)vsize, 3);
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        x.row(v->index) = v->position;
    }
    x = L*x;
    
    // set absolute mean curvature
    double maxMean = -INFINITY;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        H(v->index) = 0.5 * x.row(v->index).norm();
        if (maxMean < H(v->index)) maxMean = H(v->index);
    }
    
    return maxMean;
}

void computeCurvatures(Mesh *mesh, Eigen::VectorXd& K, Eigen::VectorXd& H)
{
    double maxGauss = computeGaussCurvature(mesh, K);
    double maxMean = computeMeanCurvature(mesh, H);
    
    // normalize gauss and mean curvature
    for (VertexIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        K(v->index) /= maxGauss;
        H(v->index) /= maxMean;
    }
}

void buildSimpleAverager(Mesh *mesh, Eigen::SparseMatrix<double>& L)
{
    std::vector<Eigen::Triplet<double>> LTriplet;
    
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        HalfEdgeCIter he = v->he;
        double degree = v->degree();
        do {
            LTriplet.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index,
                                                      1.0/degree));
            
            he = he->flip->next;
        } while (he != v->he);
    }
    
    L.setFromTriplets(LTriplet.begin(), LTriplet.end());
}

void Descriptor_HKS::computeCurve()
{
	std::vector<int> smoothLevels = {0, 1, 5, 20, 50};
	 
	// compute the mean and gaussian curvature values
    int vsize = (int)mesh->vertices.size();
    Eigen::VectorXd K(vsize);
    Eigen::VectorXd H(vsize);
	computeCurvatures(mesh, K, H);

	// allocate space for the descriptors
    for (VertexIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        v->descriptor = Eigen::VectorXd::Zero(N);
	}

	// compute a shifted-laplacian matrix for taking averages
    Eigen::SparseMatrix<double> avgM(vsize, vsize);
    buildSimpleAverager(mesh, avgM);

	// for each of the smoothing levels, smooth an appropriate number of times, then save the descriptor
	int smoothingStepsCompleted = 0;
	int iLevel = 0;
	for (int smoothLevel : smoothLevels) {
		// smooth as needed
		while (smoothingStepsCompleted < smoothLevel) {
			K = avgM * K;
			H = avgM * H;

			smoothingStepsCompleted++;
		}
        
		// save
		for (VertexIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
			v->descriptor(iLevel + 0) = K(v->index);
			v->descriptor(iLevel + 1) = H(v->index);
		}
		
		iLevel += 2;
	}
}

void Descriptor_HKS::normalize()
{
    int n = (int)mesh->vertices[0].descriptor.size();
    for (int i = 0; i < n; i++) {
        // compute min and max
        double min = 0.0, max = 0.0;
        for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
            min = std::min(min, v->descriptor(i));
            max = std::max(max, v->descriptor(i));
        }
        
        // normalize
        for (VertexIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
            v->descriptor(i) = (v->descriptor(i) - min) / (max - min);
        }
    }
}
/*
* vector <int> vrts_selected;
TArray <int> vrts_postSelected;

TArray <FVector> vrtLocs_postSelected;
TArray <FVector> vrtNors_postSelected;
TArray <EVertexType> vrtTypes_postSelected;
TArray <EVertexNormalType> vrtNorTypes_postSelected;
 */
void Descriptor_HKS::InitKeypoints(std::vector<int>& vrts_selected, TArray<int>& vrts_postSelected, TArray<FVector>& vrtLocs_postSelected
    , TArray<FVector>& vrtNors_postSelected, TArray<EVertexType>& vrtTypes_postSelected, TArray<EVertexNormalType>& vrtNorTypes_postSelected)
{
    if (mesh->GetIsEnableModel() == false)
        return;
    
    compute(HKS);

    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        if (v->isFeature(m_t, m_depth))
            vrts_selected.push_back(v->index);
    }

    for (int vrts : vrts_selected)
        vrts_postSelected.Add(vrts);
}

void Descriptor_HKS::compute(int descriptor)
{
    // compute descriptor
    switch (descriptor) {
        case HKS:
            if (mesh == nullptr)
                return;

            computeEig(500);//, eigFilename);
            computeHks();
            break;
        /*case FAST_HKS:
            computeEig(50, "");
            computeFastHks();
            break;
        case WKS:
            computeEig(500, eigFilename);
            computeWks();
            break;
        case CURVE:
            computeCurve();
            break;*/
    }
    
    // normalize
    normalize();
    
    // write to file
    /*std::ofstream out(outFilename);
    if (out.is_open()) {
        MeshIO::writeDescriptor(out, *mesh);
        out.close();
        
    } else {
		std::cout << "Not writing descriptor, no valid path specified" << std::endl;
	}*/
}
