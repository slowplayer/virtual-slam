#ifndef BAG_OF_WORDS_H
#define BAG_OF_WORDS_H

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "DBoW3/DBoW3.h"
#include "DBoW3/DescManip.h"

using namespace DBoW3;
using namespace std;

class BagOfWords
{
public:
  BagOfWords(){};
  ~BagOfWords(){};
  
  void loadFeatures();
  
  void testVocCreation();
  
  void testDatabase();
private:
  vector<cv::Mat> features;
};
#endif