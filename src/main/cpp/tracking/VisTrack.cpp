#include "VisTrack.h"

const int RETRO_HSV_MIN = 35;
const int RETRO_HSV_MAX = 78;

const int RETRO_VALUE_MIN = 100;
const int RETRO_VALUE_MAX = 255;

wml::VisionTracking vision;

// Threaded Function
void SetupVisionThread(int CamPort, int FPS, int ResHeight, int ResWidth, int Exposure, std::string Name, bool RetroTrack) {
  if (RetroTrack == true){ Exposure = -100; }
  vision.cam = vision.Camera.cam.CamSetup(CamPort, FPS, ResHeight, ResWidth, Exposure, Name);

  vision.ImageSrc = vision.Camera.cam.ImageReturn(vision.cam, Name);
}

// Thread Controller
cv::Mat wml::VisionTracking::SetupVision(int CamPort, int FPS, int ResHeight, int ResWidth, int Exposure, std::string Name, bool RetroTrack) {
  //std::thread Setup_thread_object(SetupVisionThread, CamPort, FPS, ResHeight, ResWidth, Exposure, Name, RetroTrack);
  if (RetroTrack == true){ Exposure = -100; }
  cam = Camera.cam.CamSetup(CamPort, FPS, ResHeight, ResWidth, Exposure, Name);

  ImageSrc = Camera.cam.ImageReturn(cam, Name);
  return ImageSrc;
}

void RetroTrackThread(cv::Mat Img, int ErosionSize, int DialationSize) {
  if (vision.Camera.cam.sink.GrabFrame(Img) != 0) {
    cv::cvtColor(Img, vision.imgTracking, cv::COLOR_BGR2HSV); // Uses HSV Spectrum

    // Keeps Only green pixles
    cv::inRange(vision.imgTracking, cv::Scalar(RETRO_HSV_MIN, RETRO_VALUE_MIN, RETRO_VALUE_MIN), cv::Scalar(RETRO_HSV_MAX, RETRO_VALUE_MAX, RETRO_VALUE_MAX), vision.imgTracking);

    // Removes pixles at a certain size, And dilates the image to get rid of gaps
    if (ErosionSize > 0) {
      cv::erode(vision.imgTracking, vision.imgTracking, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ErosionSize, ErosionSize)));
      cv::dilate(vision.imgTracking, vision.imgTracking, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(DialationSize, DialationSize)));
    }
  } else {
    std::cout << "Error Getting Image In Vision Library" << std::endl;
  }
}

cv::Mat wml::VisionTracking::RetroTrack(cv::Mat Img, int ErosionSize, int DialationSize) {
  //std::thread RetroTrack_thread_object(RetroTrackThread, Img, ErosionSize, DialationSize);
  RetroTrackThread(Img, ErosionSize, DialationSize);
  return imgTracking;
}

cv::Mat wml::VisionTracking::CustomTrack(cv::Mat Img, int HSVColourLowRange, int HSVColourHighRange, int ValueColourLowRange, int ValueColourHighRange, int CamExposure, int ErosionSize, int DialationSize, cs::UsbCamera cam) {
  if (Camera.cam.sink.GrabFrame(Img) != 0) {
    cv::cvtColor(Img, imgTracking, cv::COLOR_BGR2HSV); // Uses HSV Spectrum

    // Keeps Only green pixles
    cv::inRange(imgTracking, cv::Scalar(HSVColourLowRange, ValueColourLowRange, ValueColourLowRange), cv::Scalar(HSVColourHighRange, ValueColourHighRange, ValueColourHighRange), imgTracking);
    
    // Removes pixles at a certain size, And dilates the image to get rid of gaps
    if (ErosionSize > 0) {
      cv::erode(imgTracking, imgTracking, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ErosionSize, ErosionSize)));
      cv::dilate(imgTracking, imgTracking, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(DialationSize, DialationSize)));
    }
  } else {
    std::cout << "Error Getting Image" << std::endl;
  }

  return imgTracking;
}