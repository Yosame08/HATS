//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_DEFINITIONS_H
#define INC_4_SOLVE_DEFINITIONS_H

#define TraceNumber 262144
#define NodeNumber 524288
#define RoadNumber 65536
#define RoadType 7
#define GridSize 0.001// 0.001 latitude = 111.195m
#define RecoverInterval 15
#define MaxMissing 24

#define RoadVecLen 16
#define DegMinLen 15
#define SpeedLimit 40 // Speed limit in m/s
#define PreprocessSigZ 4.07
#define PreprocessBeta 2.0
#define MinProb 1e-150 // minProb^2 doesn't exceed double

#define FeatureGranularity 2
#define FeatureOutTurn "../../Intermediate/ParamTurn.params"
#define FeatureOutLenPos "../../Intermediate/ParamLenPos.params"
#define FeatureOutLenNeg "../../Intermediate/ParamLenNeg.params"

#define EdgeFile "../../Map/edgeOSM.txt"
#define WayTypeFile "../../Map/wayTypeOSM.txt"

#define DatasetPath "../../Dataset/"
#define MapPath "../../Map/"

#endif// INC_4_SOLVE_DEFINITIONS_H
