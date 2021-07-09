//
// Created by ai on 7/1/21.
//

#ifndef CALIBRATECAMERA_TESTSPEED_H
#define CALIBRATECAMERA_TESTSPEED_H

#include <fstream>
//#include <direct.h>	// in Windows
#include <sys/stat.h>	// in Linux
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "loadParameters.h"
#include "utils.h"


class CTrkNd
{
public:
    CTrkNd();
    CTrkNd(int nFrmCnt, cv::Rect oBBox, cv::Point2f o2dFtPt, cv::Point3f o3dFtPt, float fDep);
    ~CTrkNd();


    inline int getFrmCnt(void) { return m_nFrmCnt; }
    inline void setFrmCnt(int nFrmCnt) { m_nFrmCnt = nFrmCnt; }
    //inline cv::Rect getBBox(void) { return m_oBBox; }
    inline void setBBox(cv::Rect oBBox) { m_oBBox = oBBox; }
    //inline cv::Point2f get2dFtPt(void) { return m_o2dFtPt; }
    inline void set2dFtPt(cv::Point2f o2dFtPt) { m_o2dFtPt = o2dFtPt; }
    inline cv::Point3f get3dFtPt(void) { return m_o3dFtPt; }
    inline void set3dFtPt(cv::Point3f o3dFtPt) { m_o3dFtPt = o3dFtPt; }
    inline float getDep(void) { return m_fDep; }
    inline void setDep(float fDep) { m_fDep = fDep; }
    inline float getSpd(void) { return m_fSpd; }
    inline void setSpd(float fSpd) { m_fSpd = fSpd; }

private:
    //! frame count
    int m_nFrmCnt;
    //! object bounding box
    cv::Rect m_oBBox;
    //! 2D foot point
    cv::Point2f m_o2dFtPt;
    //! 3D foot point (in meter)
    cv::Point3f m_o3dFtPt;
    //! distance to the camera (in meter)
    float m_fDep;
    //! speed (in mi/h)
    float m_fSpd;
};


class testSpeed {
    public:
        testSpeed(Settings s);

    // backprojects 2D point to 3D ground position
//    static cv::Point3f bkproj2d23d(cv::Point2f o2dPt, float afProjMat[12], int nLenUnit = 1)
//    {
//        cv::Point3f o3dPt;
//
//        cv::Mat oMatA(3, 3, CV_64F);
//        oMatA.at<double>(0, 0) = afProjMat[0];
//        oMatA.at<double>(0, 1) = afProjMat[1];
//        oMatA.at<double>(0, 2) = -o2dPt.x;
//        oMatA.at<double>(1, 0) = afProjMat[4];
//        oMatA.at<double>(1, 1) = afProjMat[5];
//        oMatA.at<double>(1, 2) = -o2dPt.y;
//        oMatA.at<double>(2, 0) = afProjMat[8];
//        oMatA.at<double>(2, 1) = afProjMat[9];
//        oMatA.at<double>(2, 2) = -1.0;
//
//        cv::Mat oMatAInv(3, 3, CV_64F);
//        cv::invert(oMatA, oMatAInv, cv::DECOMP_SVD);
//
//        cv::Mat oMatB(3, 1, CV_64F);
//        oMatB.at<double>(0, 0) = -afProjMat[3];
//        oMatB.at<double>(1, 0) = -afProjMat[7];
//        oMatB.at<double>(2, 0) = -afProjMat[11];
//
//        cv::Mat oMatM(3, 1, CV_64F);
//        oMatM = oMatAInv * oMatB;
//
//        o3dPt = cv::Point3f(oMatM.at<double>(0, 0), oMatM.at<double>(1, 0), 0.0f) / nLenUnit;
//
//        return o3dPt;
//    }

    // std::vector<float> vfSpdPropFNThld  (afSpdPropFNThld , afSpdPropFNThld  + sizeof(afSpdPropFNThld) / sizeof(float) );

    // flag to output a complete video sequence
    //bool bOutVdoFlg = false;
    // flag to output plotted images (disable will accelerate the processing speed a lot)
   // bool bOutTrk3dImgFlg = false;
    // threshold of detection score
    // frame rate
    int fFrmRt;
    // frame size
    // projection matrix
    // tracking vectors
    std::vector< CTrkNd > vvoTrkNd;
    //std::vector<std::vector<CTrkNd> > vvoTrkNdFN;
    //
    void readVehicTrack( Settings s );
    void computeSpeed( void );




};


#endif //CALIBRATECAMERA_TESTSPEED_H
