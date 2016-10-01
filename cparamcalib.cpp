#include "cparamcalib.h"
#include <string>
CParamCalib::CParamCalib()
{

}

void CParamCalib::GetParam(){

    FileStorage fs;
    fs.open("param.xml",FileStorage::READ);
    fs["CameraMat1"]>>m_CameraMat1;
    fs["CameraMat2"]>>m_CameraMat2;
    fs["DistMat1"]>>m_DistMat1;
    fs["DistMat2"]>>m_DistMat2;
    fs["R"]>>m_R;
    fs["T"]>>m_T;
    fs["R1"]>>m_R1;
    fs["R2"]>>m_R2;
    fs["P1"]>>m_P1;
    fs["P2"]>>m_P2;
    fs["Q"]>>m_Q;
    fs["map1x"]>>m_map1x;
    fs["map2x"]>>m_map2x;
    fs["map1y"]>>m_map1y;
    fs["map2y"]>>m_map2y;
    fs["roi1"]>>m_roi1;
    fs["roi2"]>>m_roi2;

}

void CParamCalib::SaveParam(){
    FileStorage fs;
    fs.open("param.xml",FileStorage::WRITE);
    fs<<"CameraMat1"<<m_CameraMat1;
    fs<<"CameraMat2"<<m_CameraMat2;
    fs<<"DistMat1"<<m_DistMat1;
    fs<<"DistMat2"<<m_DistMat2;
    fs<<"R"<<m_R;
    fs<<"T"<<m_T;
    fs<<"R1"<<m_R1;
    fs<<"R2"<<m_R2;
    fs<<"P1"<<m_P1;
    fs<<"P2"<<m_P2;
    fs<<"Q"<<m_Q;
    fs<<"map1x"<<m_map1x;
    fs<<"map2x"<<m_map2x;
    fs<<"map1y"<<m_map1y;
    fs<<"map2y"<<m_map2y;
    fs<<"roi1"<<m_roi1;
    fs<<"roi2"<<m_roi2;
}
