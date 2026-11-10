#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

int main()
{
    const cv::Size boardSize(13, 9);

    const cv::TermCriteria criteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
        30,
        0.001
    );

    std::vector<cv::Point3f> objp;

    for (int j = 0; j < 13; ++j)
    {
        for (int i = 0; i < 9; ++i)
        {
            objp.emplace_back(
                static_cast<float>(i),
                static_cast<float>(j),
                0.0f
            );
        }
    }

    // Points 3D et points 2D pour toutes les images
    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    std::vector<cv::String> images;
    cv::glob("images/*.jpg", images);

    cv::Size imageSize; 

    for (const auto& filename : images)
    {
        cv::Mat img = cv::imread(filename);
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        imageSize = cv::Size(gray.cols,gray.rows);

        std::vector<cv::Point2f> corners;

        // Recherche des coins du damier
        bool found = cv::findChessboardCorners(
            gray,
            boardSize,
            corners
        );

        if (found)
        {
            objpoints.push_back(objp);

            // Raffinement des coins
            cv::cornerSubPix(
                gray,
                corners,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                criteria
            );

            imgpoints.push_back(corners);

            // Affichage des coins detectes
            cv::drawChessboardCorners(
                img,
                boardSize,
                corners,
                found
            );
        }
        cv::imshow("img", img);
        cv::waitKey(300);
    }

    cv::destroyAllWindows();
    cv::Mat cameraMatrix;             // mtx
    cv::Mat distCoeffs;               // dist
    std::vector<cv::Mat> rvecs;       // rvecs
    std::vector<cv::Mat> tvecs;       // tvecs
    double ret = cv::calibrateCamera(
        objpoints, 
        imgpoints, 
        imageSize, 
        cameraMatrix, 
        distCoeffs, 
        rvecs, 
        tvecs
    );
    cv::FileStorage fs("camera_calibration.yaml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();
    return 0;
}
