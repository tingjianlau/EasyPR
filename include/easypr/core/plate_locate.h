//////////////////////////////////////////////////////////////////////////
// Name:	    plate_locate Header
// Version:		1.2
// Date:	    2014-09-19
// MDate:		2014-09-29
// MDate:	    2015-03-13
// Author:	    liuruoze
// Copyright:   liuruoze
// Reference:	Mastering OpenCV with Practical Computer Vision Projects
// Reference:	CSDN Bloger taotao1233
// Desciption:
// Defines CPlateLocate
//////////////////////////////////////////////////////////////////////////
#ifndef EASYPR_CORE_PLATELOCATE_H_
#define EASYPR_CORE_PLATELOCATE_H_

#include "easypr/core/plate.hpp"

/*! \namespace easypr
    Namespace where all the C++ EasyPR functionality resides
*/

using namespace std;

namespace easypr {

class CPlateLocate {
 public:
  CPlateLocate();
  const string TMP_BASEPATH = "resources/image/tmp";
  int sobelFrtSearch(const Mat& src, std::vector<Rect_<float>>& outRects);
  int sobelSecSearch(Mat& bound, Point2f refpoint,
                     std::vector<RotatedRect>& outRects);
  int sobelSecSearchPart(Mat& bound, Point2f refpoint,
                         std::vector<RotatedRect>& outRects);

  int deskew(const Mat& src, const Mat& src_b,
             std::vector<RotatedRect>& inRects, std::vector<CPlate>& outPlates,
             bool useDeteleArea = true, Color color = UNKNOWN);

  bool isdeflection(const Mat& in, const double angle, double& slope);

  int sobelOper(const Mat& in, Mat& out, int blurSize, int morphW, int morphH);

  void deleteNotArea(Mat & inmat, Color color);


  bool rotation(Mat& in, Mat& out, const Size rect_size, const Point2f center,
                const double angle);

  void affine(const Mat& in, Mat& out, const double slope);

  int plateColorLocate(Mat src, std::vector<CPlate>& candPlates, int index = 0);

  int plateSobelLocate(Mat src, std::vector<CPlate>& candPlates, int index = 0);
  int sobelOperT(const Mat& in, Mat& out, int blurSize, int morphW, int morphH);

  int plateMserLocate(Mat src, std::vector<CPlate>& candPlates, int index = 0);

  int colorSearch(const Mat& src, const Color r, Mat& out,
                  std::vector<RotatedRect>& outRects);

  int mserSearch(const Mat &src, vector<Mat>& out,
    vector<vector<CPlate>>& out_plateVec, bool usePlateMser, vector<vector<RotatedRect>>& out_plateRRect,
    int img_index = 0, bool showDebug = false);

  /*
  @brief:		车牌定位: 颜色定位法，文理定位法，文字定位法
  @method:	easypr::CPlateLocate::plateLocate
  @access:    public
  @param 		src		原图
  @param 		resultVec	车牌ROI子图
  @param 		index
  */
  int plateLocate(Mat, std::vector<Mat>&, int = 0);
  int plateLocate(Mat, std::vector<CPlate>&, int = 0);

  bool verifySizes(RotatedRect mr);

  void setLifemode(bool param);

  inline void setGaussianBlurSize(int param) { m_GaussianBlurSize = param; }
  inline int getGaussianBlurSize() const { return m_GaussianBlurSize; }

  inline void setMorphSizeWidth(int param) { m_MorphSizeWidth = param; }
  inline int getMorphSizeWidth() const { return m_MorphSizeWidth; }

  inline void setMorphSizeHeight(int param) { m_MorphSizeHeight = param; }
  inline int getMorphSizeHeight() const { return m_MorphSizeHeight; }

  inline void setVerifyError(float param) { m_error = param; }
  inline float getVerifyError() const { return m_error; }
  inline void setVerifyAspect(float param) { m_aspect = param; }
  inline float getVerifyAspect() const { return m_aspect; }

  inline void setVerifyMin(int param) { m_verifyMin = param; }
  inline void setVerifyMax(int param) { m_verifyMax = param; }

  inline void setJudgeAngle(int param) { m_angle = param; }

  inline void setDebug(bool param) { m_debug = param; }
  
  inline void setImshow(bool param) { m_imshow = param; }


  inline bool getDebug() { return m_debug; }

  inline bool getImshow() { return m_imshow; }

  static const int DEFAULT_GAUSSIANBLUR_SIZE = 5;
  static const int SOBEL_SCALE = 1;
  static const int SOBEL_DELTA = 0;
  // 输出图像的深度
  static const int SOBEL_DDEPTH = CV_16S;
  // x 方向求导的阶数 
  static const int SOBEL_X_WEIGHT = 1;
  // y 方向求导的阶数 
  static const int SOBEL_Y_WEIGHT = 0;
  static const int DEFAULT_MORPH_SIZE_WIDTH = 17;  // 17
  static const int DEFAULT_MORPH_SIZE_HEIGHT = 3;  // 3


  // 车牌区域扭正之后的固定宽度，以便SVM使用
  static const int WIDTH = 136;
  // 车牌区域扭正之后的固定高度，以便SVM使用
  static const int HEIGHT = 36;
  static const int TYPE = CV_8UC3;


  static const int DEFAULT_VERIFY_MIN = 1;   // 3
  static const int DEFAULT_VERIFY_MAX = 24;  // 20

  static const int DEFAULT_ANGLE = 60;  // 30


  static const int DEFAULT_DEBUG = 1;

  // debug时，用于显示窗口图片, add by tjliu
  static const int DEFAULT_IMSHOW = 1;

 protected:

   // 高斯模糊半径
  int m_GaussianBlurSize;

  // 闭操作结构元素宽度
  int m_MorphSizeWidth;
  // 闭操作结构元素高度
  int m_MorphSizeHeight;


  // 设立一个偏差率error，根据这个偏差率计算最大和最小的宽高比rmax、rmin
  // 判断矩形的r是否满足在rmax、rmin之间
  float m_error;
  // 车牌宽高比
  float m_aspect;
  // 车牌面积的最小比值
  int m_verifyMin;
  // 车牌面积的最大比值
  int m_verifyMax;

  int m_angle;

  bool m_debug;

  bool m_imshow;
};

} /*! \namespace easypr*/

#endif  // EASYPR_CORE_PLATELOCATE_H_