//
// Created by ai on 7/1/21.
//

#include "testSpeed.h"

CTrkNd::CTrkNd(void){};
CTrkNd::~CTrkNd(void){};


CTrkNd::CTrkNd(int nFrmCnt, cv::Rect oBBox, cv::Point2f o2dFtPt, cv::Point3f o3dFtPt, float fDep)
{
    setFrmCnt(nFrmCnt);

    setBBox(oBBox);
    set2dFtPt(o2dFtPt);
    set3dFtPt(o3dFtPt);
    setDep(fDep);
}





testSpeed::testSpeed( Settings s) {

}

//
bool cmpDep(CTrkNd oTrkNd1, CTrkNd oTrkNd2)	// used to sort tracking nodes in a list
{
    return oTrkNd1.getDep() < oTrkNd2.getDep();
}

bool cmpFrmCnt(CTrkNd oTrkNd1, CTrkNd oTrkNd2)	// used to sort tracking nodes in a list
{
    return oTrkNd1.getFrmCnt() < oTrkNd2.getFrmCnt();
}

void testSpeed::readVehicTrack( Settings s ) {
    // read projection matrix (camera parameters)
    // read 2D tracking results
    //char acInTrkBuf[256] = {0};
    float nFrmCnt; //, nId=0, nFrmCntMax = -1;
    float fDetScr = 0.0f, fDep;
    cv::Rect oBBox;
    cv::Point2f o2dFtPt;
    cv::Point3f o3dFtPt;
    //char acDetCls[32];
    CTrkNd oTrkNd;
    std::ifstream ifsInTrkTxt;
    fFrmRt = s.fps;
    vvoTrkNd.clear();

    for (int i=0; i< s.vehicle_centers.size(); i++) {
        // read from the input txt file
        oBBox.x = (s.vehicle_centers.at(i).x - s.vehicle_centers.at(i).width/2) * s.video_width;
        oBBox.y = (s.vehicle_centers.at(i).y - s.vehicle_centers.at(i).width/2) * s.video_height;
        oBBox.width = s.vehicle_centers.at(i).width * s.video_width;
        oBBox.height = s.vehicle_centers.at(i).width * s.video_height;
        //
        o2dFtPt = cv::Point2f( s.vehicle_centers.at(i).x* s.video_width, s.vehicle_centers.at(i).y  * s.video_height); //
        o3dFtPt = bkproj2d23d(o2dFtPt, s.afProjMat);

        nFrmCnt = s.vehicle_centers.at(i).height;
        fDep = cv::norm(o3dFtPt);
        oTrkNd = CTrkNd(nFrmCnt, oBBox, o2dFtPt, o3dFtPt, fDep);
        vvoTrkNd.push_back( oTrkNd );

    }

}

void testSpeed::computeSpeed( ){

    // compute speed
    int iSt, iNd, nSpdWinSzUpd=7;
    float fDist;

    for (int j = 0; j < vvoTrkNd.size(); j++)
    {
        fDist = 0.0f;
        iSt = j - ((nSpdWinSzUpd - 1) / 2);
        iSt = (0 <= iSt) ? iSt : 0;
        iNd = j + ((nSpdWinSzUpd - 1) / 2);
        iNd = (vvoTrkNd.size() > iNd) ? iNd : (vvoTrkNd.size() - 1);

        // accumulate the distance that the object travels between every two frames
        for (int k = iSt; k < iNd-1; k++)
            fDist += cv::norm(vvoTrkNd[k].get3dFtPt() - vvoTrkNd[k + 1].get3dFtPt());

        vvoTrkNd[j].setSpd((float) fDist * fFrmRt /(float) (vvoTrkNd.at(iNd).getFrmCnt() - vvoTrkNd.at(iSt).getFrmCnt()));

    }


    // fine-tune speed
    // compute the mean of speed and the mean of speed for instances that are close to the camera
    //int nTrajLenFN, iTrkNdTPDistMin, nSpdConstAvgNum = 0;
    //float fTrkNdTPDist, fTrkNdTPDistMin;
    //double fSpdConstAvg = 0.0;
    double vfSpdMean, vfSpdStd, vfSpdMeanCls;

    vfSpdMean = 0.0;
    vfSpdMeanCls = 0.0;

    std::sort(vvoTrkNd.begin(), vvoTrkNd.end(), cmpDep);

    for (int i = 0; i < vvoTrkNd.size(); i++) {
        vfSpdMean += vvoTrkNd[i].getSpd();
        //
        // the 1/3 of vehicle trajectory are considered to be close to the camera
//        if (i >=vvoTrkNd.size() / 3  && i < 2*vvoTrkNd.size() / 3)
//            vfSpdMeanCls += vvoTrkNd[i].getSpd();
    }
    vfSpdMean /= vvoTrkNd.size();
    // the 1/3 of vehicle trajectory are considered to be close to the camera
    vfSpdMeanCls /= ((vvoTrkNd.size() +1)/ 3);

    vfSpdStd = 0.0;
    for (int i = 0; i < vvoTrkNd.size(); i++) {
        vfSpdStd += (vvoTrkNd[i].getSpd() - vfSpdMean) * (vvoTrkNd[i].getSpd() - vfSpdMean);
    }
    vfSpdStd /= vvoTrkNd.size();
    vfSpdStd = std::sqrt(vfSpdStd);

    float sup =  vfSpdMean + 1.2 * vfSpdStd/vvoTrkNd.size();
    float sdown =  vfSpdMean - 1.2 * vfSpdStd/vvoTrkNd.size();


    // tuning the speed
    int numCases=0;
    for (int i = 0; i < vvoTrkNd.size(); i++) {
        if (vvoTrkNd[i].getSpd() > sdown && vvoTrkNd[i].getSpd() < sup){
            vfSpdMeanCls += vvoTrkNd[i].getSpd();
            numCases++;
        }
    }

    vfSpdMeanCls/=numCases;



    std::printf(" average speed: %.3f\n", vfSpdMean );
    std::printf(" tuned average speed: %.3f\n", vfSpdMeanCls);
    std::printf(" 95 percent uppar limit : %.3f  lower limit: %.3f\n", sup, sdown);


}

