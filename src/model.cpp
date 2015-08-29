#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int counter = 0;
Point2d buffer[6];
Point2d mousePos;

void mouseCallback(int event, int x, int y, int, void* )
{
    if(event == EVENT_LBUTTONDOWN)
    {
        buffer[counter] = Point2d(static_cast<double>(x), static_cast<double>(y));
        ++counter;
    }
    else if(event == EVENT_RBUTTONDOWN)
    {
        if(counter > 0)
            --counter;
    }
    else if(event == EVENT_MOUSEMOVE)
    {
        mousePos = Point2d(static_cast<double>(x), static_cast<double>(y));
    }
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "./bin/model <path-to-video> <output-file>" << std::endl;
        return -1;
    }

    VideoCapture video(argv[1]);
    if(!video.isOpened())
    {
        std::cerr << "Failed to open video " << argv[1] << std::endl;
        return -1;
    }

    std::ofstream file(argv[2], std::ofstream::trunc);
    if(!file)
    {
        std::cerr << "Failed to open file " << argv[2] << std::endl;
        return -1;
    }

    namedWindow("Model");
    setMouseCallback("Model", mouseCallback, NULL);

    Mat frame;
    video.read(frame);

    for(;;)
    {
        if(counter == 6)
        {
            counter = 0;

            Point2d vl = buffer[1] - buffer[0];
            Point2d vr = buffer[3] - buffer[2];
            Point2d v = buffer[5] - buffer[4];

            double nl = std::sqrt(vl.x*vl.x + vl.y*vl.y);
            double nr = std::sqrt(vr.x*vr.x + vr.y*vr.y);
            vl.x /= nl; vl.y /= nl;
            vr.x /= nr; vr.y /= nr;

            double theta = acos(vl.dot(vr));

            file << theta << ' ' << v.x << ' ' << v.y << '\n';

            if(!video.read(frame))
                break;
            if(!video.read(frame))
                break;
        }

        // draw
        Mat image;
        frame.copyTo(image);
        for(int i = 0; i < counter; ++i)
            circle(image, buffer[i], 2.0, Scalar(i > 3 ? 255 : 0, i >= 2 && i <= 3 ? 255 : 0, i < 2 ? 255 : 0), 3, -1);
        for(int i = 0; i < counter/2 + counter % 2; ++i)
        {
            Point2d pt1 = buffer[2*i];
            Point2d pt2 = mousePos;
            if(2*i+1 < counter)
                pt2 = buffer[2*i+1];
            line(image, pt1, pt2, Scalar(i == 2 ? 255 : 0, i == 1 ? 255 : 0, i == 0 ? 255 : 0), 1, CV_AA);
        }

        imshow("Model", image);
        int key = waitKey(16);
        if(key == 27)
            break;
    }

    return 0;
}
