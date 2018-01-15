#include "AerobicVision.h"

//TODO: TWO DIMENSIONAL FILTER TO FILTER OUTSIDE OF BIN
//TODO: Z-AXIS FILTER TO FILTER FROM 1 CM ABOVE BOTTOM AND HIGHER
//TODO: CHECK EUCLIDEAN CLUSTER EXTRACTION


int main()
{
  AerobicVision* test = new AerobicVision("192.168.1.69");
  test->RegionGrowing();
  //pcl::PointXYZ data = test->Calibrate(100);
  //cout << "Calibration Result (XYZ): "<< data.x << " , " << data.y << " , " << data.z << endl;
  return 0;
}
