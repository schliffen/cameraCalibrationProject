//
// Created by ai on 6/25/21.
//

#ifndef CALIBRATECAMERA_EXTRINSIC_CALIBRATION_H
#define CALIBRATECAMERA_EXTRINSIC_CALIBRATION_H

#include <opencv2/core/core.hpp>
#include "utils.h"
#include "loadParameters.h"

template <typename T> T deg2rad(T deg) { return deg * (CV_PI / 180.0); }
template <typename T> T rad2deg(T rad) { return rad * (180.0 / CV_PI); }


//! define the type of coordinate system for camera calibration: 0: X-Z ground plane; 1: X-Y ground plane (default: 0)
#define COORD_SYS_TYP (1)
//! define the type of Vy estimation: 0: computing center of mass; 1: RANSAC (default: 1)
#define VY_EST_TYP (1)
//! define the type of Linf estimation: 0: linear regression; 1: RANSAC (default: 1)
#define LINF_EST_TYP (1)
//! define the type of principal point estimation: 0: assuming as the image center; 1: the point with minimum distance to the perpendicular line to Linf (default: 0)
#define PRIN_PT_EST_TYP (0)
//! define the number of iterations in RANSAC for Vy estimation, necessary when  VY_EST_TYP = 1 (default: 100)
#define VY_RS_ITER_NUM (100)
//! define the threshold for the distance (divided by the frame width) of RANSAC inliers to Vy, necessary when  VY_EST_TYP = 1 (default: 2.0)
#define VY_RS_DIST_THLD (2.0)
//! define the number of iterations in RANSAC for Linf estimation, necessary when  LINF_EST_TYP = 1 (default: 100)
#define LINF_RS_ITER_NUM (100)
//! define the threshold for the distance (divided by the frame height) of RANSAC inliers to Linf, necessary when  LINF_EST_TYP = 1 (default: 0.15)
#define LINF_RS_DIST_THLD (0.20)
//! define the threshold for the distance (divided by the frame height) of RANSAC inliers to Linf, necessary when  LINF_EST_TYP = 1 (default: 0.15)
#define LINF_RS_DIST_THLD (0.20)
//! define the range for focal length in ratio in EDA optimization (default: 0.2f)
#define EDA_RNG_F (2.9f)
//! define the range for principal point coordinates in pixels in EDA optimization (default: 100)
#define EDA_RNG_PRIN_PT (1000)
//! define the range for rotation angles in degrees in EDA optimization (default: 45.0)
#define EDA_RNG_ROT_ANG (180.0)
//! define the initial population of EDA (default: 20000)
#define EDA_INIT_POP (60000)
//! define the selected population of EDA (default: 20)
#define EDA_SEL_POP (30)
//! define the number of iterations of EDA (default: 100)
#define EDA_ITER_NUM (100)
//! define the threshold of ratio of reprojection errors between iterations (default: 0.10)
#define EDA_REPROJ_ERR_THLD (0.01)
//! define image expansion ratio for plotting vanishing points and horizon line (default: 2.0f)
#define IMG_EXPN_RAT (2.0f)

//
class CCfg
{
public:
    //! full constructor
    CCfg();
    //! default destructor
    ~CCfg();
    //! loads configuration file from directory
    void ldCfgFl(void);

    inline cv::Size getFrmSz(void) { return m_oFrmSz; }
    void setFrmSz(cv::Size oFrmSz) { m_oFrmSz = oFrmSz; };

    inline char* getInFrmPth(void) { return m_acInFrmPth; }
    //inline std::string getOutCamParamPth(void) { return m_acOutCamParamPth; }
    inline void setOutCamParamPth(std::string out_proj_path) { m_acOutCamParamPth = out_proj_path; }

    inline int getRszFrmHei(void) { return m_nRszFrmHei; }
    inline int getLenUnit(void) { return m_nLenUnit; }
    inline bool getCalSelVanLnFlg(void) { return m_bCalSelVanLnFlg; }
    inline cv::Point getCalVr(void) { return m_oCalVr; }
    inline cv::Point getCalVl(void) { return m_oCalVl; }
    inline float getCalCamHeiMax(void) { return m_fCalCamHeiMax; }
    inline float getCalCamHeiMin(void) { return m_fCalCamHeiMin; }
    inline int getCalGrdSzR(void) { return m_nCalGrdSzR; }
    inline int getCalGrdSzL(void) { return m_nCalGrdSzL; }
    inline bool getCalEdaOptFlg(void) { return m_bCalEdaOptFlg; }
    inline std::vector<cv::Point> getCalMeasLnSegNdPt(void) { return m_voCalMeasLnSegNdPt; }
    inline std::vector<float> getCalMeasLnSegDist(void) { return m_vfCalMeasLnSegDist; }

private:
    //! reads char array
    std::string rdCharArr(std::string strCfg, int nParamPos);
    //! reads integer number
    int rdInt(std::string strCfg, int nParamPos);
    //! reads float number
    float rdFlt(std::string strCfg, int nParamPos);
    //! reads bool value
    bool rdBool(std::string strCfg, int nParamPos);
    //! reads 2D point
    cv::Point rd2dPt(std::string strCfg, int nParamPos);
    //! reads vector of float numbers
    std::vector<float> rdVecFlt(std::string strCfg, int nParamPos);
    //! reads vector of 2D points
    std::vector<cv::Point> rdVec2dPt(std::string strCfg, int nParamPos);

    //! video frame size
    cv::Size m_oFrmSz;
    //! path of input frame image
    char m_acInFrmPth[256];
    //! path of output text file of camera parameters
    std::string m_acOutCamParamPth;
    //! resized video frame height (-1: original size)
    int m_nRszFrmHei;
    //! the length unit, 10 or 1000 (1 cm = 10 mm, 1 m = 1000 mm)
    int m_nLenUnit;
    //! flag of selecting vanishing lines on the ground plane, necessary when m_nInCalTyp == 1
    bool m_bCalSelVanLnFlg;
    //! given vanishing point Vr, necessary when m_bCalSelVanLnFlg == false
    cv::Point m_oCalVr;
    //! given vanishing point Vl, necessary when m_bCalSelVanLnFlg == false
    cv::Point m_oCalVl;
    //! the maximum height of camera in m_nLenUnit, necessary when m_nInCalTyp > 1
    float m_fCalCamHeiMax;
    //! the minimum height of camera in m_nLenUnit, necessary when m_nInCalTyp > 1
    float m_fCalCamHeiMin;
    //! size of the 3D grid (in m_nLenUnit) on ground plane along R axis, necessary when m_nInCalTyp > 0
    int m_nCalGrdSzR;
    //! size of the 3D grid (in m_nLenUnit) on ground plane along L axis, necessary when m_nInCalTyp > 0
    int m_nCalGrdSzL;
    //! flag of EDA optimization for camera calibration, necessary when m_nInCalTyp > 0
    bool m_bCalEdaOptFlg;
    //! pair(s) of end points of measuring line segments, necessary when m_nCalReprojErrTyp == 1 and total number must be even
    std::vector<cv::Point> m_voCalMeasLnSegNdPt;
    //! ground truth distance of measuring line segments , necessary when m_nCalReprojErrTyp == 1 and total number must be half of m_voCalMeasLnSegNdPt
    std::vector<float> m_vfCalMeasLnSegDist;
};



//
class extrinsic_params {
public:
    extrinsic_params();
    ~extrinsic_params();
    struct SParamRng {
        SParamRng()
        {
            fFxMax = 5000, fFxMin = 0;
            fFyMax = 5000, fFyMin = 0;
            fCxMax = 2500, fCxMin = 0;
            fCyMax = 2500, fCyMin = 0;
            fRollMax = deg2rad(180), fRollMin = deg2rad(-180);
            fPitchMax = deg2rad(180), fPitchMin = deg2rad(-180);
            fYawMax = deg2rad(180), fYawMin = deg2rad(-180);
            fTxMax = 1000, fTxMin = -1000;
            fTyMax = 1000, fTyMin = -1000;
            fTzMax = 1000, fTzMin = -1000;
        }

        float fFxMax, fFxMin;	// camera focal length
        float fFyMax, fFyMin;	// camera focal length fy = fx * a (aspect ratio, close to 1)
        float fCxMax, fCxMin;	// optical center/principal point
        float fCyMax, fCyMin;	// optical center/principal point
        float fRollMax, fRollMin;	// roll angle
        float fPitchMax, fPitchMin;	// pitch(tilt) angle
        float fYawMax, fYawMin;	// yaw(pan) angle
        float fTxMax, fTxMin;
        float fTyMax, fTyMin;
        float fTzMax, fTzMin;
    };
    //
    inline float* getInParamMat() { return m_afK; }
    inline float* getRotMat() { return m_afR; }
    inline float* getTntMat() { return m_afT; }
    inline float* getProjMat() { return m_afP; }
    inline float getFx() { return m_fFx; }
    inline void setFx(float fFx) { m_fFx = fFx; }
    inline float getFy() { return m_fFy; }
    inline void setFy(float fFy) { m_fFy = fFy; }
    inline float getCx() { return m_fCx; }
    inline void setCx(float fCx) { m_fCx = fCx; }
    inline float getCy() { return m_fCy; }
    inline void setCy(float fCy) { m_fCy = fCy; }
    inline float getRoll() { return m_fRoll; }
    inline void setRoll(float fRoll) { m_fRoll = fRoll; }
    inline float getPitch() { return m_fPitch; }
    inline void setPitch(float fPitch) { m_fPitch = fPitch; }
    inline float getYaw() { return m_fYaw; }
    inline void setYaw(float fYaw) { m_fYaw = fYaw; }
    inline float getTx() { return m_fTx; }
    inline void setTx(float fTx) { m_fTx = fTx; }
    inline float getTy() { return m_fTy; }
    inline void setTy(float fTy) { m_fTy = fTy; }
    inline float getTz() { return m_fTz; }
    inline void setTz(float fTz) { m_fTz = fTz; }
    inline float getReprojErr() { return m_fReprojErr; }
    inline void setReprojErr(float fReprojErr) { m_fReprojErr = fReprojErr; }
    //! sets the matrix of intrinsic parameters
    void setInParamMat(float fFx, float fFy, float fCx, float fCy);
    //! sets the matrix of rotation
    void setRotMat(float fRoll, float fPitch, float fYaw);
    //! sets the matrix of translation
    void setTntMat(float fTx, float fTy, float fTz);
    //! calculates the projective matrix
    void calcProjMat();
    //! reads the file of camera intrinsic parameters
    void rdCamInParamTxt(const char* acCamInParamPth);
    //! initializes camera model
    void initCamMdl(SParamRng sParamRng);

private:
    float m_afK[9];
    float m_afR[9];
    float m_afT[3];
    float m_afP[12];

    float m_fFx;
    float m_fFy;
    float m_fCx;
    float m_fCy;
    float m_fRoll;
    float m_fPitch;
    float m_fYaw;
    float m_fTx;
    float m_fTy;
    float m_fTz;

    float m_fReprojErr;
    };


class extrinsic_calibration{
public:
    extrinsic_calibration(  Settings s );
    ~extrinsic_calibration(void);

    //! initializes the self-calibrator
    void initialize(CCfg oCfg, cv::Mat oImgBg);
    //! perform self-calibration
    bool process(void);

    //inline cv::Point2f getVyCand(int i) { return m_voVyCand[i]; }
    //inline cv::Point2f getLinfCand(int i) { return m_voLinfCand[i]; }
    //inline cv::Point2f getPrinPt(void) { return m_oPrinPt; }

private:
    //! estimates vertical vanishing point Vy by computing center of mass
    void estVyByCM(void);
    //! estimates vertical vanishing point Vy by RANSAC algorithm
    void estVyByRS(void);
    //! estimates horizon line Linf by linear regression
    void estLinfByLR(void);
    //! estimates horizon line Linf by RANSAC algorithm
    void estLinfByRS(void);
    //! estimates principal point by assuming it as the image center
    void estPrinPtByAC(void);
    //! estimates principal point by minimum distance
    void estPrinPtByMD(void);
    //! estimates vanishing points Vr and Vl on the horizon line
    void estVrVl(void);
    //! calibrates camera by direct computation
    void calCamDctComp(void);
    //! calibrates camera by EDA optimization
    void calCamEdaOpt();
    //! computes camera parameters
    void compCamParam( cv::Point2f oPrinPt, float fCamHei, extrinsic_params* poCamParam);
    //! calculates reprojection error based on measurement error compared to ground truth
    double calcReprojErr(extrinsic_params* poCamParam);
    //! calculates starting grid point
    //cv::Point calcStGrdPt(extrinsic_params* poCamParam);
    //! tests starting grid point (all corner points of the grip are within the expanded frame image)
    //bool tstStGrdPt(cv::Point oStGrdPt, extrinsic_params* poCamParam);
    //! initializes the ranges of camera parameters in EDA optimization
    extrinsic_params::SParamRng initEdaParamRng( cv::Point2f oPrinPt, extrinsic_params* poCamParam = NULL);
    //! estimates the ranges of camera parameters in EDA optimization
    extrinsic_params::SParamRng estEdaParamRng(std::vector<extrinsic_params>* pvoCamParam);
    //! plots a 3D grid on the ground plane
    void writeProjMat(extrinsic_params* poCamParam, cv::Point oStGrdPt);

    //! configuration parameters
    CCfg m_oCfg;
    //! background image for plotting results
    //cv::Mat m_oImgBg ;
    //! 25% progress flag for collecting 2D tracking data
    bool m_bCol25ProgFlg;
    //! 50% progress flag for collecting 2D tracking data
    bool m_bCol50ProgFlg;
    //! 75% progress flag for collecting 2D tracking data
    bool m_bCol75ProgFlg;
    //! candidate points for vertical vanishing point (Vy) estimation
    std::vector<cv::Point2f> m_voVyCand;
    //! candidate points for horizon line (Linf) estimation
    std::vector<cv::Point2f> m_voLinfCand;
    //! estimated vertical vanishing point
    //cv::Point2f m_oVy;
    //! the slope of estimated horizon line
    //float m_fLinfSlp;
    //! the intercept of estimated horizon line with y axis
    //float m_fLinfItcp;
    //! the sample correlation coefficient of estimated horizon line
    //float m_fLinfCorr;
    //! estimated vanishing point Vr
    //cv::Point2f m_oVr;
    //! estimated vanishing point Vl
    //cv::Point2f m_oVl;
    //! (initial) principal point (optical center) of the camera
    cv::Point2f m_oPrinPt;
    // intrinsic params
    //float Fx, Fy, Cx, Cy;
    // nominal vehicle position
    std::vector<cv::Rect_<float>> target_v_postions;
    float target_speed;
    int fps = 30;
    int frame_height;
    int frame_width;
    std::string m_acOutCamParamPth;


};







#endif //CALIBRATECAMERA_EXTRINSIC_CALIBRATION_H
