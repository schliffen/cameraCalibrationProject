//
// Created by ai on 6/25/21.
//
/*
 the code will load the parameters and will use DAE optimization

 * */

#include "extrinsic_calibration.h"

bool compReprojErr(extrinsic_params oCamParam1, extrinsic_params oCamParam2)
{
    return (oCamParam1.getReprojErr() < oCamParam2.getReprojErr());
}

CCfg::CCfg()
{
    //m_oFrmSz = cv::Size(1920, 1080);
//    std::strcpy(m_acInFrmPth, "./data/frm.png"); // no need to upload a frame
    //std::strcpy(m_acOutCamParamPth, "../configs/camParam.txt");
    m_nRszFrmHei = -1;
    m_nLenUnit = 1000;
    m_bCalSelVanLnFlg = false;
    m_oCalVr = cv::Point(-1, -1);
    m_oCalVl = cv::Point(-1, -1);
    m_fCalCamHeiMax = 10.0f;
    m_fCalCamHeiMin = 1.0f;
    m_nCalGrdSzR = 10;
    m_nCalGrdSzL = 10;
    m_bCalEdaOptFlg = true;
    std::vector<cv::Point>().swap(m_voCalMeasLnSegNdPt);
    std::vector<float>().swap(m_vfCalMeasLnSegDist);
}

CCfg::~CCfg()
{

}
extrinsic_calibration::~extrinsic_calibration(){

}
extrinsic_calibration::extrinsic_calibration(Settings s) {
    //
    //Fx = s.camera_params.at<double>(0,0);
    //Fy = s.camera_params.at<double>(1,1);
    //Cx = s.camera_params.at<double>(0,2);
    //Cy = s.camera_params.at<double>(1,2);
    // todo: consider the distortion

    // reading the car positions as radar speed
    // todo: scale the positions here
    cv::Rect_<float> temp_rect;
    frame_width = s.video_width;
    frame_height = s.video_height;
    // setting the center point
    m_oPrinPt.x = frame_width/2.f;
    m_oPrinPt.y = frame_height/2.f;

    for ( int i=0; i < s.vehicle_centers.size(); i++){
        // scalling back the vehicle venters
        temp_rect.x = s.vehicle_centers.at(i).x * frame_width;
        temp_rect.y = s.vehicle_centers.at(i).y * frame_height;
        temp_rect.width = s.vehicle_centers.at(i).width * frame_width;
        temp_rect.height = s.vehicle_centers.at(i).height;

        target_v_postions.push_back(temp_rect);
    }

    target_speed = s.radar_speed; // speed unit: m/s
    fps = s.fps;
    m_acOutCamParamPth = s.estimated_projection_path;

}

extrinsic_params::extrinsic_params(){ }

extrinsic_params::~extrinsic_params(){

}

bool extrinsic_calibration::process()
{
    calCamEdaOpt();
    return true;
}
//
void extrinsic_calibration::compCamParam ( cv::Point2f oPrinPt, float fCamHei, extrinsic_params* poCamParam)
{
    cv::Point2f oVrC, oVlC;
    cv::Point2f oVrCRot, oVlCRot;
    float  fF, fRoll, fPitch, fYaw;

    // calculate f, roll, pitch, and yaw using vanishing points
    // parallel lalines is going to be selected at random

    oVrC.x = - oPrinPt.x;
    oVrC.y = oPrinPt.y;
    oVlC.x = - oPrinPt.x;
    oVlC.y = oPrinPt.y;

    fRoll = std::atan2((oVrC.y - oVlC.y), (oVrC.x - oVlC.x));
    fRoll = (fRoll > (CV_PI / 2.0)) ? (fRoll - CV_PI) : fRoll;
    fRoll = (fRoll < (-CV_PI / 2.0)) ? (fRoll + CV_PI) : fRoll;

    oVrCRot = rotPt(oVrC, -fRoll);
    oVlCRot = rotPt(oVlC, -fRoll);

    fF = std::sqrt(-((oVrCRot.y * oVrCRot.y) - (oVrCRot.x * oVlCRot.x)));

    if (0 == COORD_SYS_TYP)
    {
        fPitch = -std::atan2(oVrCRot.y, fF);
        fYaw = -std::atan2(fF, (oVrCRot.x * std::cos(fPitch)));
    }
    else if (1 == COORD_SYS_TYP)
    {
        fPitch = -std::atan2(oVrCRot.y, fF);
        fYaw = -std::atan2((oVrCRot.x * std::cos(fPitch)), fF);
    }

    poCamParam->setFx(fF);
    poCamParam->setFy(fF);
    poCamParam->setCx(oPrinPt.x);
    poCamParam->setCy(oPrinPt.y);
    poCamParam->setRoll(fRoll);
    poCamParam->setPitch(fPitch);
    poCamParam->setYaw(fYaw);
    poCamParam->setTx(0.0f);
    if (0 == COORD_SYS_TYP)
    {
        poCamParam->setTy(-fCamHei * m_oCfg.getLenUnit());
        poCamParam->setTz(0.0f);
    }
    else if (1 == COORD_SYS_TYP)
    {
        poCamParam->setTy(0.0f);
        poCamParam->setTz(fCamHei * m_oCfg.getLenUnit());
    }

    poCamParam->setInParamMat(fF, fF, oPrinPt.x, oPrinPt.y);
    poCamParam->setRotMat(fRoll, fPitch, fYaw);
    if (0 == COORD_SYS_TYP)
        poCamParam->setTntMat(0.0f, (-fCamHei * m_oCfg.getLenUnit()), 0.0f);
    else if (1 == COORD_SYS_TYP)
        poCamParam->setTntMat(0.0f, 0.0f, (fCamHei * m_oCfg.getLenUnit()));
    poCamParam->calcProjMat();
}

//
void extrinsic_calibration::calCamEdaOpt()
{
    int nR = EDA_INIT_POP, nN = EDA_SEL_POP, nIterNum = EDA_ITER_NUM, iIter = 0, iProc;
    bool bProc25, bProc50, bProc75;
    float fFx, fFy, fCx, fCy, fRoll, fPitch, fYaw, fTx, fTy, fTz;
    double fReprojErr, fReprojErrMean, fReprojErrMeanPrev, fReprojErrStd;
    //
    cv::Point oStGrdPt;
    std::vector<extrinsic_params>::iterator ivoCamParam;
    extrinsic_params oCamParam, oCamParamRand;
    extrinsic_params::SParamRng sParamRng;
    // set starting grid point
    compCamParam( m_oPrinPt, ((m_oCfg.getCalCamHeiMax() + m_oCfg.getCalCamHeiMin()) / 2.0f), &oCamParam);
    // initialize range of each parameter
    sParamRng = initEdaParamRng( m_oPrinPt );
    // EDA optimization
    if (nN >= nR)
        std::printf("Error: we should have nN >= nR \n");
    std::vector<extrinsic_params> voCamParam, bestCases;

    for (int iR = 0; iR < nR; iR++)
    {
        oCamParamRand.initCamMdl(sParamRng);
        voCamParam.push_back(oCamParamRand);
    }

    std::printf("Start EDA optimization for camera calibration\n");

    while (nIterNum > iIter)
    {
        printf("==== generation %d: ====\n", iIter);
        iProc = 0;
        bProc25 = false;
        bProc50 = false;
        bProc75 = false;
        fReprojErrMean = 0.0;
        fReprojErrStd = 0.0;

        for (ivoCamParam = voCamParam.begin(); ivoCamParam != voCamParam.end(); ivoCamParam++)
        {
            fFx = ivoCamParam->getFx();
            fFy = ivoCamParam->getFy();
            fCx = ivoCamParam->getCx();
            fCy = ivoCamParam->getCy();
            fRoll = ivoCamParam->getRoll();
            fPitch = ivoCamParam->getPitch();
            fYaw = ivoCamParam->getYaw();
            fTx = ivoCamParam->getTx();
            fTy = ivoCamParam->getTy();
            fTz = ivoCamParam->getTz();

            ivoCamParam->setInParamMat(fFx, fFy, fCx, fCy);
            ivoCamParam->setRotMat(fRoll, fPitch, fYaw);
            ivoCamParam->setTntMat(fTx, fTy, fTz);
            ivoCamParam->calcProjMat();
            oCamParam = *ivoCamParam;
            fReprojErr = calcReprojErr(&oCamParam);

            ivoCamParam->setReprojErr(fReprojErr);

            fReprojErrMean += fReprojErr;
            iProc++;

            if ((((float)iProc / (float)nR) > 0.25) && (!bProc25)) {
                std::cout<< " ... 25%% ... ";  bProc25 = true; }
            if ((((float)iProc / (float)nR) > 0.50) && (!bProc50)) {
                std::cout<< "  50%% ... "; bProc50 = true; }
            if ((((float)iProc / (float)nR) > 0.75) && (!bProc75)) {
                std::cout<< "  75%% ... "; bProc75 = true; }
        }

        fReprojErrMean /= nR;

        for (ivoCamParam = voCamParam.begin(); ivoCamParam != voCamParam.end(); ivoCamParam++)
        {
            double fReprojErr = ivoCamParam->getReprojErr();
            fReprojErrStd += (fReprojErr - fReprojErrMean) * (fReprojErr - fReprojErrMean);
        }

        fReprojErrStd = std::sqrt(fReprojErrStd / nR);

        std::printf("100%%!\n");
        std::printf("current error mean = %f\n", fReprojErrMean);
        std::printf("current error standard deviation = %f\n", fReprojErrStd);

        if (!fReprojErrMean)
        {
            std::printf("Camera calibration fails.\n");
            break;
        }

        // Check if generation needs to stop
        if ((0 < iIter) && ((fReprojErrMeanPrev * EDA_REPROJ_ERR_THLD) > std::abs(fReprojErrMean - fReprojErrMeanPrev)))
        {
            std::printf("Reprojection error is small enough. Stop generation.\n");
            break;
        }

        fReprojErrMeanPrev = fReprojErrMean;
        std::stable_sort(voCamParam.begin(), voCamParam.end(), compReprojErr);
        voCamParam.erase(voCamParam.begin() + nN, voCamParam.end());
        // save the best
        std::cout<< "Best case Error: " <<   voCamParam[0].getReprojErr()  <<std::endl;
        writeProjMat(&voCamParam.at(0), oStGrdPt);

        sParamRng = estEdaParamRng(&voCamParam);

        for (int iR = 0; iR < nR; iR++)
        {
            extrinsic_params oCamParamRand;
            oCamParamRand.initCamMdl(sParamRng);
            voCamParam.push_back(oCamParamRand);
        }

        iIter++;

        std::cout<<"\n";
    }

    if (nIterNum <= iIter)
        printf("Exit: Results can not converge.\n");
}

extrinsic_params::SParamRng extrinsic_calibration::initEdaParamRng( cv::Point2f oPrinPt, extrinsic_params* poCamParam)
{
    cv::Point2f oVrC, oVlC;
    cv::Point2f oVrCRot, oVlCRot;
    float fF, fRoll, fPitch, fYaw;
    extrinsic_params::SParamRng sParamRng;

    // calculate f, roll, pitch, and yaw by using vanishing points
    oVrC.x =  - oPrinPt.x;
    oVrC.y = oPrinPt.y;
    oVlC.x = - oPrinPt.x;
    oVlC.y = oPrinPt.y;

    fRoll = std::atan2((oVrC.y - oVlC.y), (oVrC.x - oVlC.x));
    fRoll = (fRoll > (CV_PI / 2.0)) ? (fRoll - CV_PI) : fRoll;
    fRoll = (fRoll < (-CV_PI / 2.0)) ? (fRoll + CV_PI) : fRoll;

    oVrCRot = rotPt(oVrC, -fRoll);
    oVlCRot = rotPt(oVlC, -fRoll);

    if (poCamParam)
    {
        float* acK = poCamParam->getInParamMat();
        fF = (acK[0] + acK[4]) / 2.0f;
    }
    else
    {
        fF = std::sqrt(-((oVrCRot.y * oVrCRot.y) - (oVrCRot.x * oVlCRot.x)));
    }

    if (0 == COORD_SYS_TYP)
    {
        fPitch = std::atan2(oVrCRot.y, fF);
        fYaw = -std::atan2(fF, (oVrCRot.x * cos(fPitch)));
    }
    else if (1 == COORD_SYS_TYP)
    {
        fPitch = -std::atan2(oVrCRot.y, fF);
        fYaw = -std::atan2((oVrCRot.x * cos(fPitch)), fF);
    }

    // construct ranges of camera parameters
    sParamRng.fFxMax = (poCamParam) ? poCamParam->getInParamMat()[0] : fF * (1.0f + EDA_RNG_F);
    sParamRng.fFxMin = (poCamParam) ? poCamParam->getInParamMat()[0] : fF * (1.0f - EDA_RNG_F);
    sParamRng.fFyMax = (poCamParam) ? poCamParam->getInParamMat()[4] : fF * (1.0f + EDA_RNG_F);
    sParamRng.fFyMin = (poCamParam) ? poCamParam->getInParamMat()[4] : fF * (1.0f - EDA_RNG_F);
    sParamRng.fCxMax = (poCamParam) ? poCamParam->getInParamMat()[2] : oPrinPt.x + EDA_RNG_PRIN_PT;
    sParamRng.fCxMin = (poCamParam) ? poCamParam->getInParamMat()[2] : oPrinPt.x - EDA_RNG_PRIN_PT;
    sParamRng.fCyMax = (poCamParam) ? poCamParam->getInParamMat()[5] : oPrinPt.y + EDA_RNG_PRIN_PT;
    sParamRng.fCyMin = (poCamParam) ? poCamParam->getInParamMat()[5] : oPrinPt.y - EDA_RNG_PRIN_PT;

    sParamRng.fRollMax  = fRoll + deg2rad(EDA_RNG_ROT_ANG);
    sParamRng.fRollMin  = fRoll - deg2rad(EDA_RNG_ROT_ANG);
    sParamRng.fPitchMax = fPitch + deg2rad(EDA_RNG_ROT_ANG);
    sParamRng.fPitchMin = fPitch - deg2rad(EDA_RNG_ROT_ANG);
    sParamRng.fYawMax   = fYaw + deg2rad(EDA_RNG_ROT_ANG);
    sParamRng.fYawMin   = fYaw - deg2rad(EDA_RNG_ROT_ANG);

    sParamRng.fTxMax = 0.0f;
    sParamRng.fTxMin = 0.0f;
    if (0 == COORD_SYS_TYP)
    {
        sParamRng.fTyMax = -m_oCfg.getCalCamHeiMin() * m_oCfg.getLenUnit();
        sParamRng.fTyMin = -m_oCfg.getCalCamHeiMax() * m_oCfg.getLenUnit();
        sParamRng.fTzMax = 0.0f;
        sParamRng.fTzMin = 0.0f;
    }
    else if (1 == COORD_SYS_TYP)
    {
        sParamRng.fTyMax = 0.0f;
        sParamRng.fTyMin = 0.0f;
        sParamRng.fTzMax = m_oCfg.getCalCamHeiMax() * m_oCfg.getLenUnit();
        sParamRng.fTzMin = m_oCfg.getCalCamHeiMin() * m_oCfg.getLenUnit();
    }
    //
    return sParamRng;
}


void extrinsic_params::initCamMdl(SParamRng sParamRng)
{
    m_fFx = (float)get_rand_num(sParamRng.fFxMin, sParamRng.fFxMax, rand());
    m_fFy = (float)get_rand_num(sParamRng.fFyMin, sParamRng.fFyMax, rand());
    m_fCx = (float)get_rand_num(sParamRng.fCxMin, sParamRng.fCxMax, rand());
    m_fCy = (float)get_rand_num(sParamRng.fCyMin, sParamRng.fCyMax, rand());
    //
    m_fRoll  = (float)get_rand_num(sParamRng.fRollMin, sParamRng.fRollMax, rand());
    m_fPitch = (float)get_rand_num(sParamRng.fPitchMin, sParamRng.fPitchMax, rand());
    m_fYaw   = (float)get_rand_num(sParamRng.fYawMin, sParamRng.fYawMax, rand());
    m_fTx = (float)get_rand_num(sParamRng.fTxMin, sParamRng.fTxMax, rand());
    m_fTy = (float)get_rand_num(sParamRng.fTyMin, sParamRng.fTyMax, rand());
    m_fTz = (float)get_rand_num(sParamRng.fTzMin, sParamRng.fTzMax, rand());
}


extrinsic_params::SParamRng extrinsic_calibration::estEdaParamRng(std::vector<extrinsic_params>* pvoCamParam)
{
    int nCamParamNum = pvoCamParam->size(), nParamNum = 10, iParam, iCamParam = 0;
    float fParamVar;
    float* afParamMean = (float*)calloc(nParamNum, sizeof(float));
    float* afParamData = (float*)calloc(nParamNum*nCamParamNum, sizeof(float));
    std::vector<extrinsic_params>::iterator ivoCamParam;
    extrinsic_params::SParamRng sParamRng;
    // calculate means of parameters
    for (ivoCamParam = pvoCamParam->begin(); ivoCamParam != pvoCamParam->end(); ivoCamParam++)
    {
        iParam = 0;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getFx();    afParamMean[0] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getFy();    afParamMean[1] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getCx();    afParamMean[2] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getCy();    afParamMean[3] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        //
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getRoll();  afParamMean[4] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getPitch(); afParamMean[5] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getYaw();   afParamMean[6] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getTx();    afParamMean[7] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getTy();    afParamMean[8] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getTz();    afParamMean[9] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
        iCamParam++;
    }

    for (iParam = 0; iParam < nParamNum; iParam++)
        afParamMean[iParam] /= nCamParamNum;

    // fx
    iParam = 0;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fFxMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fFxMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // fy
    iParam = 1;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fFyMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fFyMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // cx
    iParam = 2;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fCxMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fCxMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // cy
    iParam = 3;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fCyMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fCyMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // roll
    iParam = 4;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fRollMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fRollMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // pitch
    iParam = 5;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fPitchMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fPitchMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // yaw
    iParam = 6;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fYawMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fYawMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // tz
    iParam = 7;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fTxMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fTxMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // ty
    iParam = 8;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fTyMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fTyMin = afParamMean[iParam] - std::sqrt(fParamVar);

    // tz
    iParam = 9;
    fParamVar = 0.0f;
    for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
        fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
                     (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
    fParamVar /= nCamParamNum;
    sParamRng.fTzMax = afParamMean[iParam] + std::sqrt(fParamVar);
    sParamRng.fTzMin = afParamMean[iParam] - std::sqrt(fParamVar);
    //
    std::free(afParamMean);
    std::free(afParamData);

    return sParamRng;
}


void extrinsic_params::setInParamMat(float fFx, float fFy, float fCx, float fCy)
{
    m_afK[0] = fFx;
    m_afK[1] = 0.0f;
    m_afK[2] = fCx;
    m_afK[3] = 0.0f;
    m_afK[4] = fFy;
    m_afK[5] = fCy;
    m_afK[6] = 0.0f;
    m_afK[7] = 0.0f;
    m_afK[8] = 1.0f;
}

void extrinsic_params::setRotMat(float fRoll, float fPitch, float fYaw)
{
    if (0 == COORD_SYS_TYP)
    {
        m_afR[0] = (std::cos(fRoll) * std::cos(fYaw)) - (std::sin(fRoll) * std::sin(fPitch) * std::sin(fYaw));
        m_afR[1] = -std::sin(fRoll) * std::cos(fPitch);
        m_afR[2] = (std::cos(fRoll) * std::sin(fYaw)) + (std::sin(fRoll) * std::sin(fPitch) * std::cos(fYaw));
        m_afR[3] = (std::sin(fRoll) * std::cos(fYaw)) + (std::cos(fRoll) * std::sin(fPitch) * std::sin(fYaw));
        m_afR[4] = std::cos(fRoll) * std::cos(fPitch);
        m_afR[5] = (std::sin(fRoll) * std::sin(fYaw)) - (std::cos(fRoll) * std::sin(fPitch) * std::cos(fYaw));
        m_afR[6] = -std::cos(fPitch) * std::sin(fYaw);
        m_afR[7] = std::sin(fPitch);
        m_afR[8] = std::cos(fPitch) * std::cos(fYaw);
    }
    else if (1 == COORD_SYS_TYP)
    {
        m_afR[0] = (-std::cos(fRoll) * std::sin(fYaw)) - (std::sin(fRoll) * std::sin(fPitch) * std::cos(fYaw));
        m_afR[1] = (-std::cos(fRoll) * std::cos(fYaw)) - (std::sin(fRoll) * std::sin(fPitch) * std::cos(fYaw));
        m_afR[2] = std::sin(fRoll) * std::cos(fPitch);
        m_afR[3] = (-std::sin(fRoll) * std::sin(fYaw)) + (std::cos(fRoll) * std::sin(fPitch) * std::cos(fYaw));
        m_afR[4] = (-std::sin(fRoll) * std::cos(fYaw)) - (std::cos(fRoll) * std::sin(fPitch) * std::sin(fYaw));
        m_afR[5] = -std::cos(fRoll) * std::cos(fPitch);
        m_afR[6] = std::cos(fPitch) * std::cos(fYaw);
        m_afR[7] = -std::cos(fPitch) * std::sin(fYaw);
        m_afR[8] = std::sin(fPitch);
    }
}


void extrinsic_params::setTntMat(float fTx, float fTy, float fTz)
{
    m_afT[0] = fTx;
    m_afT[1] = fTy;
    m_afT[2] = fTz;
}

void extrinsic_params::calcProjMat()
{
    cv::Mat oMatK(3, 3, CV_32FC1, m_afK);
    cv::Mat oMatR(3, 3, CV_32FC1, m_afR);

    // T = -Rt
    cv::Mat oMatT(3, 1, CV_32FC1);
    oMatT.ptr<float>(0)[0] = -m_afT[0];
    oMatT.ptr<float>(1)[0] = -m_afT[1];
    oMatT.ptr<float>(2)[0] = -m_afT[2];
    oMatT = oMatR * oMatT;

    // P = K [R|T]
    cv::Mat oMatP(3, 4, CV_32FC1);
    oMatP.ptr<float>(0)[0] = m_afR[0];
    oMatP.ptr<float>(0)[1] = m_afR[1];
    oMatP.ptr<float>(0)[2] = m_afR[2];
    oMatP.ptr<float>(1)[0] = m_afR[3];
    oMatP.ptr<float>(1)[1] = m_afR[4];
    oMatP.ptr<float>(1)[2] = m_afR[5];
    oMatP.ptr<float>(2)[0] = m_afR[6];
    oMatP.ptr<float>(2)[1] = m_afR[7];
    oMatP.ptr<float>(2)[2] = m_afR[8];
    oMatP.ptr<float>(0)[3] = oMatT.ptr<float>(0)[0];
    oMatP.ptr<float>(1)[3] = oMatT.ptr<float>(1)[0];
    oMatP.ptr<float>(2)[3] = oMatT.ptr<float>(2)[0];
    oMatP = oMatK * oMatP;

    m_afP[0] = oMatP.ptr<float>(0)[0];
    m_afP[1] = oMatP.ptr<float>(0)[1];
    m_afP[2] = oMatP.ptr<float>(0)[2];
    m_afP[3] = oMatP.ptr<float>(0)[3];
    m_afP[4] = oMatP.ptr<float>(1)[0];
    m_afP[5] = oMatP.ptr<float>(1)[1];
    m_afP[6] = oMatP.ptr<float>(1)[2];
    m_afP[7] = oMatP.ptr<float>(1)[3];
    m_afP[8] = oMatP.ptr<float>(2)[0];
    m_afP[9] = oMatP.ptr<float>(2)[1];
    m_afP[10] = oMatP.ptr<float>(2)[2];
    m_afP[11] = oMatP.ptr<float>(2)[3];
}
// GIZLI
double extrinsic_calibration::calcReprojErr(extrinsic_params* poCamParam)
{

    int grundTruthLength = target_v_postions.size();
    //
    cv::Point2f pointA2d, pointB2d;
    cv::Point3f pointA3d, pointB3d;
    //
    int iSt, iNd;
    double fDist;
    //
    std::vector<double> estimatedSpeed;
    //int k1,k2;
    float meanESpeed=0;

    try
    {
        for (int j = 0; j < grundTruthLength; j++) // calculations for 4 regions
        {
            iSt = j - 3;
            iSt = (0 <= iSt) ? iSt : 0;
            iNd = j + 3;
            iNd = ( grundTruthLength > iNd) ? iNd : (grundTruthLength - 1);
            fDist = 0.0f;
            // measuring the distance pointwise
            for (int ind = iSt; ind < iNd-1; ind++) {
                pointA2d.x = target_v_postions.at(ind).x;
                pointA2d.y = target_v_postions.at(ind).y;
                pointB2d.x = target_v_postions.at(ind + 1).x;
                pointB2d.y = target_v_postions.at(ind + 1).y;
                pointA3d = bkproj2d23d(pointA2d, poCamParam->getProjMat(), m_oCfg.getLenUnit());
                pointB3d = bkproj2d23d(pointB2d, poCamParam->getProjMat(), m_oCfg.getLenUnit());
//                fDist += std::sqrt(pow(pointA3d.x - pointB3d.x, 2) + pow(pointA3d.y - pointB3d.y, 2) +
//                                   pow(pointA3d.z - pointB3d.z, 2));
                fDist += cv::norm( pointA3d - pointB3d);
            }
            float espeed = (float) (fDist * fps) / (float) (target_v_postions.at(iNd).height - target_v_postions.at(iSt).height);
            estimatedSpeed.push_back(pow( espeed - target_speed, 2)); // mph * SpeedInitConfig.Instance.vfSpdScl[0] to be added
            //meanESpeed += espeed; // only for debugging
        }

    }
    catch (int ex)
    {
        std::cerr<< ex << " cant calculate the speed "<< "\n";
    }
    //std::cout<< "mean estimated speed: " <<  meanESpeed/ grundTruthLength << std::endl;
    return  (double) std::accumulate(estimatedSpeed.begin(), estimatedSpeed.end(), 0.0);
}

void extrinsic_calibration::writeProjMat(extrinsic_params* poCamParam, cv::Point oStGrdPt)
{
    FILE* pfCamParam;
    pfCamParam = std::fopen(m_acOutCamParamPth.c_str(), "w");
    float *afK, *afR, *afT, *afP;
    afK = poCamParam->getInParamMat();
    afR = poCamParam->getRotMat();
    afT = poCamParam->getTntMat();
    afP = poCamParam->getProjMat();
    //
    std::fprintf(pfCamParam, "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f\n",
                 afK[0], afK[1], afK[2], afK[3], afK[4], afK[5], afK[6], afK[7], afK[8]);
    std::fprintf(pfCamParam, "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f\n",
                 afR[0], afR[1], afR[2], afR[3], afR[4], afR[5], afR[6], afR[7], afR[8]);
    std::fprintf(pfCamParam, "%.7f %.7f %.7f\n", afT[0], afT[1], afT[2]);
    std::fprintf(pfCamParam, "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f\n",
                 afP[0], afP[1], afP[2], afP[3], afP[4], afP[5], afP[6], afP[7], afP[8], afP[9], afP[10], afP[11]);
    std::fclose(pfCamParam);

}
