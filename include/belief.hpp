#include <Eigen/Dense>
#include <Eigen/Core>

class Belief {
public:
	void propagateBelief(Mat &P1, Point &qNear, Point &qNew, double &sigma, Mat &P2)
	{
		dx1 = qNear.x - qNew.x;
		dy1 = qNear.y - qNew.y;
		dz1 = qNear.z - qNew.z;

		r = sqrt(pow(dx1, 2) + pow(dy1, 2) + pow(dz1, 2));

		Eigen::MatrixXd H(1, DIM);

		H.row(0) << (1 / r)*dx1, (1 / r)*dy1, (1 / r)*dz1;

		z = 1 / (1 + r * r);
		R = sigma * sigma;

		P_prd = P1 + GQG;
		S = (H * P_prd * H.transpose()).value() + M * R * M;

		K = (1 / S) * (P_prd * H.transpose());
		P2 = (I - (K*H)) * P_prd;
	}

private:
	Mat I = Eigen::MatrixXd::Identity(DIM, DIM);
	Mat A = I;
	Mat B = I;
	Mat G = I;

	Mat GQG = G * Q * G.transpose();

	Mat P_prd, P2;

	float processNoise = 0.028;
	Mat Q = pow(processNoise, 2) * I;
	double M = 1;
	double R, z, S, r;
	double dx1, dy1, dz1;
	Eigen::MatrixXd H = Eigen::MatrixXd(1, DIM);
	Eigen::MatrixXd K;
};


