#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpPose.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRxyzVector.h>
void poseQr(const vpImage< unsigned char >  &I)
{
#if defined(VISP_HAVE_ZBAR)
  try {

    // Camera parameters should be adapted to your camera
    double fx= 704.8199194976651;
    double fy= 707.2512839200566;
    double cx= 366.76503328510955;
    double cy= 242.5081171868669;
    double kud=  0.044080227363205014;
    double kdu= -0.044080227363205014;

    vpCameraParameters cam;
    cam.initPersProjWithDistortion(fx,fy,cx,cy,kud,kdu);
    
    // 3D model of the QRcode: here we consider a 12cm by 12cm QRcode
    std::vector<vpPoint> point;
    point.push_back( vpPoint(-0.023, -0.023, 0) ); // QCcode point 0 3D coordinates in plane Z=0
    point.push_back( vpPoint( 0.023, -0.023, 0) ); // QCcode point 1 3D coordinates in plane Z=0
    point.push_back( vpPoint( 0.023,  0.023, 0) ); // QCcode point 2 3D coordinates in plane Z=0
    point.push_back( vpPoint(-0.023,  0.023, 0) ); // QCcode point 3 3D coordinates in plane Z=0

    vpHomogeneousMatrix cMo;
    bool init = true;

    vpDetectorQRCode detector;
    //vpDisplay::display(I);
    bool status = detector.detect(I);  //true if QRcode is detected

    if (status) { // true if at least one QRcode is detected
        for(size_t i=0; i < detector.getNbObjects(); i++) {

          std::vector<vpImagePoint> p = detector.getPolygon(i); // get the four corners location in the image
	        std::string id = detector.getMessage(i);  // get the message of the QRcode

          for(size_t j=0; j < p.size(); j++) {
            //vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            //vpDisplay::displayText(I, p[j]+vpImagePoint(15,5), number.str(), vpColor::blue);
          }

    //computePose(point, p, cam, init, cMo); // resulting pose is available in cMo var

          vpPose pose;     
          double x=0, y=0;
          for (unsigned int i=0; i < point.size(); i ++) {
            vpPixelMeterConversion::convertPoint(cam, p[i], x, y);
            point[i].set_x(x);
            point[i].set_y(y);
            pose.addPoint(point[i]);
          }

          if (init == true) {
            vpHomogeneousMatrix cMo_dem;
            vpHomogeneousMatrix cMo_lag;
            pose.computePose(vpPose::DEMENTHON, cMo_dem);
            pose.computePose(vpPose::LAGRANGE, cMo_lag);
            double residual_dem = pose.computeResidual(cMo_dem);
            double residual_lag = pose.computeResidual(cMo_lag);
            if (residual_dem < residual_lag)
                cMo = cMo_dem;
            else
                cMo = cMo_lag;
          }
          pose.computePose(vpPose::VIRTUAL_VS, cMo);


          std::cout << "ID: " << id << std::endl
		                << "Pose translation (meter): " << cMo.getTranslationVector().t() << std::endl
                    << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t() << std::endl
                    << "pose rotation (degree): "<< vpRxyzVector(cMo.getRotationMatrix()).t() *180/3.1415926 << std::endl;
                   // << "pose_self"<<vpQuaternionVector(mo).t() << std::endl;//
          //vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        }
      }else{
        std::cout<<" No QRcode is detected "<<std::endl;

      }
 }
  catch(const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
#else
  std::cout << "ViSP is not build with zbar 3rd party." << std::endl;
#endif
}
