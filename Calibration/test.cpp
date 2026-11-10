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

    // Points 3D du damier : (0,0,0), (1,0,0), ..., (6,5,0)
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

    // Recherche des fichiers JPG dans le dossier courant
    std::vector<cv::String> images;
    cv::glob("./CalImages/*.png", images);

    for (const auto& filename : images)
    {
        cv::Mat img = cv::imread(filename);

        if (img.empty())
        {
            std::cerr << "Impossible de lire : " << filename << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

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
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    std::cout << "Images traitees : " << images.size() << std::endl;
    std::cout << "Damier detecte dans : " << objpoints.size()
              << " image(s)" << std::endl;

    return 0;
}
