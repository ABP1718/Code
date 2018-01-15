#include <opencv2/opencv.hpp>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"
#include "avansvisionlib.h"

using namespace cv;
using namespace std;

class DepthDetector {
	public:
	DepthDetector();
	vector<Point2d> getPositions();
	private:
	vector<Point2d> positions;
};
