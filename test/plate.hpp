#ifndef EASYPR_PLATE_HPP
#define EASYPR_PLATE_HPP

namespace easypr {

	namespace demo {

		using namespace cv;
		using namespace std;

		/* 
		@brief:		≤‚ ‘≥µ≈∆∂®Œª 
		@method:	easypr::demo::test_plate_locate
		@access:    public 
		*/
		int test_plate_locate() {
			cout << "test_plate_locate" << endl;

			//const string file = "resources/image/plate_locate.jpg";
			//const string file = "resources/image/‘¡DR9475_20170426155554373.jpg";
			//const string file = "resources/image/À’B23B77_20170425154110628.jpg";
			const string file = "resources/image/¡…BL7P50_20170425181159458.jpg";
			//const string file = "resources/image/¥®A019W2.jpg";

			cv::Mat src = imread(file);
			if (src.data == NULL)
			{
				std::cout << "read image from " << file << " failed!" << std::endl;
				getchar();
				getchar();
				exit(0);
			}

			vector<cv::Mat> resultVec;
			CPlateLocate plate;
			plate.setDebug(false);
			plate.setImshow(false);
			//plate.setLifemode(true);

			int result = plate.plateLocate(src, resultVec);
			/*
			if (result == 0) {
				size_t num = resultVec.size();
				for (size_t j = 0; j < num; j++) {
					cv::Mat resultMat = resultVec[j];
					imshow("plate_locate", resultMat);
					waitKey(0);
				}
				destroyWindow("plate_locate");
			}
			*/

			return result;
		}

		int test_plate_judge() {
			cout << "test_plate_judge" << endl;

			cv::Mat src = imread("resources/image/∏”LU9119_20170425151556285.jpg");
			//cv::Mat src = imread("resources/image/¡…BL7P50_20170425181159458.jpg");
			//cv::Mat src = imread("resources/image/‘¡DR9475_20170426155554373.jpg");
			//cv::Mat src = imread("resources/image/plate_judge.jpg");

			vector<cv::Mat> matVec;

			vector<cv::Mat> resultVec;

			CPlateLocate lo;
			lo.setDebug(0);
			lo.setLifemode(true);

			int resultLo = lo.plateLocate(src, matVec);

			if (0 != resultLo) return -1;

			cout << "plate_locate_img" << endl;
			size_t num = matVec.size();
			/*
			for (size_t j = 0; j < num; j++) {
				Mat resultMat = matVec[j];
				imshow("plate_judge", resultMat);
				waitKey(0);
			}
			destroyWindow("plate_judge");
			*/

			int resultJu = PlateJudge::instance()->plateJudge(matVec, resultVec);

			if (0 != resultJu) return -1;

			cout << "plate_judge_img" << endl;
			num = resultVec.size();
			for (size_t j = 0; j < num; j++) {
				Mat resultMat = resultVec[j];
				//imshow("plate_judge", resultMat);
				//waitKey(0);
				utils::imwrite("resources/image/tmp/0palte_after_svm" + utils::to_str(j) + ".jpg", resultMat);
			}
			cout << "After svm-judge, we got " << num << " plates" << endl;
			return resultJu;
		}

		int test_plate_detect() {
			cout << "test_plate_detect" << endl;

			cv::Mat src = imread("resources/image/plate_detect.jpg");

			vector<CPlate> resultVec;
			CPlateDetect pd;
			pd.setPDDebug(true);
			pd.setPDLifemode(true);

			int result = pd.plateDetect(src, resultVec);
			if (result == 0) {
				size_t num = resultVec.size();
				for (size_t j = 0; j < num; j++) {
					CPlate resultMat = resultVec[j];

					imshow("plate_detect", resultMat.getPlateMat());
					waitKey(0);
				}
				destroyWindow("plate_detect");
			}

			return result;
		}

		int test_plate_recognize() {
			cout << "test_plate_recognize" << endl;

			Mat src = imread("resources/image/¡…BL7P50_20170425181159458.jpg");
			//Mat src = imread("resources/image/À’B23B77_20170425154110628.jpg");
			//Mat src = imread("resources/image/test.jpg");

			CPlateRecognize pr;
			pr.setLifemode(true);
			pr.setDebug(false);
			pr.setMaxPlates(4);
			//pr.setDetectType(PR_DETECT_COLOR | PR_DETECT_SOBEL);
			pr.setDetectType(easypr::PR_DETECT_CMSER);

			//vector<string> plateVec;
			vector<CPlate> plateVec;

			int result = pr.plateRecognize(src, plateVec);
			//int result = pr.plateRecognizeAsText(src, plateVec);
			if (result == 0) {
				size_t num = plateVec.size();
				for (size_t j = 0; j < num; j++) {
					cout << "plateRecognize: " << plateVec[j].getPlateStr() << endl;
				}
			}

			if (result != 0) cout << "result:" << result << endl;

			return result;
		}
	}
}

#endif  // EASYPR_PLATE_HPP
