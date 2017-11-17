#include "BagOfWords.h"

void BagOfWords::loadFeatures()
{
  //ORB feature
  cv::Ptr<cv::Feature2D> fdetector=cv::ORB::create();
  
  for(size_t i=0;i<10;++i)
  {
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    string path="../data/img/"+to_string(i+1)+".png";
    
    //read image,grayscale
    cv::Mat image=cv::imread(path,0);
    
    //detect keypoints and compute descriptors;
    fdetector->detectAndCompute(image,cv::Mat(),keypoints,descriptors);
    
    features.push_back(descriptors);
  }
}
void BagOfWords::testVocCreation()
{
  const int k=9;
  const int L=3;
  const WeightingType weight=TF_IDF;
  const ScoringType score=L1_NORM;
  
  DBoW3::Vocabulary voc(k,L,weight,score);
  
  //create a vocabulary
  voc.create(features);
  //compare images with images
  BowVector v1,v2;
  for(size_t i=0;i<features.size();i++)
  {
    voc.transform(features[i],v1);
    for(size_t j=0;j<features.size();j++)
    {
      voc.transform(features[j],v2);
 
      double score=voc.score(v1,v2);
      cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    }
  }
  //save the vocabulary
  voc.save("../data/voc/small_voc.yml.gz");
}
void BagOfWords::testDatabase()
{
  Vocabulary voc("../data/voc/small_voc.yml.gz");
  //create a database
  Database db(voc,false,0);
  
  for(size_t i=0;i<features.size();i++)
    db.add(features[i]);
  
  //compare images with database
  QueryResults ret;
  for(size_t i=0;i<features.size();i++)
  {
    db.query(features[i],ret,4);
    cout<<"Searching for Image "<<i<<" returns "<<ret<<endl<<endl;
  }
}