#pragma once

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "Eigen/Dense"
#include "rrbt_utils.hpp"

class Vision
{
public:
    Vision() 
    {
        initParams();
    }

    void readMapData()
    {
        std::ifstream in("/home/sai/sample.ply");
		std::string line;

        std::cout << "Reading map data";  

		while (std::getline(in, line))
		{
			std::stringstream ss(line);
			float x, y, z;

			ss >> x >> y >> z;

			mapPoints.push_back(cv::Point3f(x, y, z + 100));
		}
    }

    double evaluateSample(Point& position, double& yaw)
    {
        std::vector <double> state(4);
        int dimIdx = 0;
        
        double visibility = 0.0;
        double span = 0.0;
        std::cout << "Evaluating sample at (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;

        state[0] = position.x;
        state[1] = position.y;
        state[2] = position.z;
        state[3] = yaw;

        std::vector <cv::Point2f> projectedPoints;
        computeProjectionsSingleCamera(state, mapPoints, projectedPoints);

        visibility = ((double)projectedPoints.size()/(double)mapPoints.size());
        span = computeSpan();

        return 1/span;
    }

private:
    int w, h, binSize, numBins;
    float cost;
    cv::Mat image;

    Eigen::Matrix3d K;
    std::vector <cv::Point3f> mapPoints;

    void initParams()
    {
        w = 640; h = 480;
        image = cv::Mat(h, w, CV_8UC1, cv::Scalar(0));
        binSize = 32;
        K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
        numBins = (w/binSize - 1) * (h/binSize - 1);
    }

    double computeSpan()
    {      
        int *binPixCount = new int[numBins];
        double binPixAcc = 0.0;
        int binIdx = 0;
        cv::Rect roi;

        for (int y = 0; y < image.cols-binSize; y += binSize) {
            for (int x = 0; x < image.rows-binSize; x += binSize) {
                roi = cv::Rect(y, x, binSize, binSize);    
                binPixCount[binIdx] = cv::countNonZero(image(roi));
                
                binPixAcc += binPixCount[binIdx];
                binIdx++;
            }
        }

        double mean = binPixAcc / (double)numBins;
        double varAcc = 0.0;
		for (int i = 0; i < numBins; ++i)
			varAcc += std::pow(binPixCount[i] - mean, 2);

        double variance = std::sqrt(varAcc / numBins);

		variance = binPixAcc / variance;
        //free(binPixCount);
        delete[] binPixCount;

		if (binPixAcc == 0)
			variance = 1;

        std::cout << "Variance is " << variance << std::endl;

        return variance;
    }

    void computeProjectionsSingleCamera(std::vector <double>& state, std::vector <cv::Point3f> &mapPoints, std::vector <cv::Point2f>& projectedPoints)
    {
        double yaw = 0;
        double pitch = 0;
        double roll = 0;

        Eigen::Matrix3d rMat = Eigen::Matrix3d::Identity();
        Eigen::Vector3d tvec;
        Eigen::MatrixXd P(3, 4);

        tvec << -state[0], -state[1], -state[2];

        projectedPoints.clear();

        P << rMat, tvec;
        projectPoints(mapPoints, projectedPoints, K, P);

        image = cv::Scalar::all(0); 

        for (int i = 0; i < projectedPoints.size(); ++i) {
            if (projectedPoints[i].x > 0 && projectedPoints[i].x < w && projectedPoints[i].y > 0 && projectedPoints[i].y < h){
                image.at<uchar>(std::floor(projectedPoints[i].y), std::floor(projectedPoints[i].x)) = 255;
            }
        }
    }

    void projectPoints(std::vector<cv::Point3f>& mapPoints, std::vector<cv::Point2f>& imagePoints, Eigen::Matrix3d& K, Eigen::MatrixXd& P)
    {
        for (int i = 0; i < mapPoints.size(); ++i) {
            Eigen::Vector4d objectPt;
            objectPt << mapPoints[i].x, mapPoints[i].y, mapPoints[i].z , 1;

            Eigen::Vector3d projection;
            projection = K * P * objectPt;

            // std::cout << "(" << proj(0,0)/proj(2,0) << "," << proj(1,0)/proj(2,0) << "," << proj(2,0) << ")" << std::endl;

            if (isValid(projection)) {
                imagePoints.push_back(cv::Point2f(projection(0,0)/projection(2,0), projection(1,0)/projection(2,0)));
            }
        }
    }

    bool isValid(Eigen::Vector3d& projection)
    {
        if (projection(2,0) > 0) {
            if (projection(0,0)/projection(2,0) > 0 && projection(0,0)/projection(2,0) < w && projection(1,0)/projection(2,0) > 0 && projection(1,0)/projection(2,0) < h)
                return true;
        }
        else
            return false;
    }
};