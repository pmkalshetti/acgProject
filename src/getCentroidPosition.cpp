#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void getCentroidPosition(Mat & img, vector<Point2f> & centroidPositions);
void getForegroundMask(vector<Mat> & images, Mat & foregroundMask);

int main(int argc, char* argv[])
{
  vector<Mat> images;
  // input image is background subtracted
  images.push_back(imread(argv[1]));
  images.push_back(imread(argv[2]));

  // // show input image
  // namedWindow("Input Frame");
  // imshow("Input Frame", img);

  // background subtraction
  Mat foregroundMask;
  getForegroundMask(images, foregroundMask);
  // namedWindow("Background Subtracted Image");
  // imshow("Background Subtracted Image", foregroundMask);


  // get centroid positions of objects
  vector<Point2f> centroidPositions;
  getCentroidPosition(foregroundMask, centroidPositions);

  // print positions
  for (int i = 0; i < centroidPositions.size(); i++) {
    cout << centroidPositions[i] << endl;
  }

  waitKey(0);

  // exit handling
  destroyAllWindows();
  return 0;
}

void getForegroundMask(vector<Mat> & images, Mat & foregroundMask) {
    Ptr<BackgroundSubtractor> segmentor = createBackgroundSubtractorMOG2();
    segmentor->apply(images[0], foregroundMask);
    segmentor->apply(images[1], foregroundMask);
}

void getCentroidPosition(Mat & img, vector<Point2f> & centroidPositions)
{
  // define blob parameters
  SimpleBlobDetector::Params blobParameters;
  blobParameters.filterByArea = true; blobParameters.minArea = 10.0; // pixels
  blobParameters.filterByConvexity = false; blobParameters.minConvexity = 0.75;
  blobParameters.minDistBetweenBlobs = 100;

  // blob detection
  Ptr<SimpleBlobDetector> blobDetector = SimpleBlobDetector::create(blobParameters);
  vector<KeyPoint> keypoints;
  blobDetector->detect(img, keypoints);

  // draw blobs
  Mat blobImg;
  drawKeypoints(img, keypoints, blobImg, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  namedWindow("Blob Detected Image");
  imshow("Blob Detected Image", blobImg);

  // copy into array
  for(int i = 0; i < keypoints.size(); i++) {
    centroidPositions.push_back(keypoints[i].pt);
  }
}
