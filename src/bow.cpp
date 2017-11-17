#include "BagOfWords.h"

int main()
{
  BagOfWords bow;
  
  bow.loadFeatures();
  
  bow.testVocCreation();
  
  bow.testDatabase();
  
  return 0;
}
