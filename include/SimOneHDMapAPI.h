﻿// ==========================================================================
// Copyright (C) 2018 - 2021 Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// , and/or its licensors.  All rights reserved.
//
// The coded instructions, statements, computer programs, and/or related 
// material (collectively the "Data") in these files contain unpublished
// information proprietary to Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// ("51WORLD") and/or its licensors,  which is protected by the People's 
// Republic of China and/or other countries copyright law and by 
// international treaties.
//
// The Data may not be disclosed or distributed to third parties or be
// copied or duplicated, in whole or in part, without the prior written
// consent of 51WORLD.
//
// The copyright notices in the Software and this entire statement,
// including the above license grant, this restriction and the following
// disclaimer, must be included in all copies of the Software, in whole
// or in part, and all derivative works of the Software, unless such copies
// or derivative works are solely in the form of machine-executable object
// code generated by a source language processor.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
// 51WORLD DOES NOT MAKE AND HEREBY DISCLAIMS ANY EXPRESS OR IMPLIED
// WARRANTIES INCLUDING, BUT NOT LIMITED TO, THE WARRANTIES OF
// NON-INFRINGEMENT, MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE,
// OR ARISING FROM A COURSE OF DEALING, USAGE, OR TRADE PRACTICE. IN NO
// EVENT WILL 51WORLD AND/OR ITS LICENSORS BE LIABLE FOR ANY LOST
// REVENUES, DATA, OR PROFITS, OR SPECIAL, DIRECT, INDIRECT, OR
// CONSEQUENTIAL DAMAGES, EVEN IF 51WORLD AND/OR ITS LICENSORS HAS
// BEEN ADVISED OF THE POSSIBILITY OR PROBABILITY OF SUCH DAMAGES.
// ==========================================================================
#pragma once
#pragma warning(disable:4819)
#pragma warning(disable:4190)
#ifndef WITHOUT_HDMAP
#ifdef BUILD_SIMONE_API
#if defined(WIN32) || defined(_WIN32)
#define SIMONE_API __declspec(dllexport)
#elif defined(__linux__) || defined(__linux)
#define SIMONE_API __attribute__((visibility("default")))
#endif
#else
#define SIMONE_API
#endif

#include <string>
#include "Service/SimOneIOStruct.h"

#include "public/common/MEnum.h"
#include "public/common/MLaneLink.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MRoadMark.h"
#include "public/common/MSignal.h"
#include "public/common/MObject.h"
#include "public/common/MParkingSpace.h"
#include "public/common/MRoutePath.h"
#include "public/MHDMap.h"
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "SSD/SimString.h"
#include "SSD/SimVector.h"

//namespace HDMapStandalone
//{
//    struct MLaneInfo;
//	struct MLaneLink;
//}

#ifdef __cplusplus
extern "C"
{
#endif


	namespace SimOneAPI {
		struct LaneSample
		{
			int laneCode;  //1, 2, ...
			bool inJunction = false;
			SSD::SimPoint3DVector leftBoundary;    //Left boundary sample data£ºlane_line_left_data
			SSD::SimPoint3DVector rightBoundary;  //Right boundary sample data£ºlane_line_right_data
		};

		struct LaneData
		{
			SSD::SimVector<LaneSample> laneSampleList;
			SSD::SimStringVector laneNameList;
			HDMapStandalone::MRoadMark leftRoadMark;
			HDMapStandalone::MRoadMark rightRoadMark;
		};

		struct LaneInfo
		{
			SSD::SimString currentLane;
			SSD::SimVector<LaneData> dataList;
		};

		struct TyrePosInfo
		{
			SSD::SimPoint3D frontLeft;
			SSD::SimPoint3D frontRight;
			SSD::SimPoint3D rearLeft;
			SSD::SimPoint3D rearRight;
		};

		enum EDirectionType_
		{
			Forward = 0,
			TurnLeft = 1,
			TurnRight = 2,
			TurnBack = 3,
			ForwardAndTurnLeft = 4,
			ForwardAndTurnRight = 5,
			ForwardAndTurnBack = 6,
			TurnLeftAndTurnBack = 7
		};

		enum ELaneLineType_
		{
			none = 0,
			whiteSolid = 1,
			whiteDotted = 2,
			yellowSolid = 3,
			yellowDotted = 4
		};

		struct LaneIndexInfo_
		{
			int currentIndex = -99;
			SSD::SimVector<int> indexList;
		};

		struct LaneSample_
		{
			int laneCode;  //1, 2, ...
			bool inJunction = false;
			SSD::SimPoint3DVector leftBoundary;    //Left boundary sample data：lane_line_left_data
			SSD::SimPoint3DVector rightBoundary;  //Right boundary sample data：lane_line_right_data
		};

		struct OverlapLaneInfo_
		{
			bool isOverlapLeftBoundary = false;
			bool isOverlapRightBoundary = false;
		};

		struct LaneLineTypeInfo_
		{
			ELaneLineType_ leftLaneLineType;
			ELaneLineType_ rightLaneLineType;
		};

		struct LaneData_
		{
			LaneIndexInfo_ laneIndexInfo;
			EDirectionType_ laneType;
			SSD::SimVector<LaneSample_> laneSampleList;
			OverlapLaneInfo_ overlapLaneInfo;
			LaneLineTypeInfo_ laneLineTypeInfo;
		};

		struct LaneInfo_
		{
			SSD::SimVector<LaneData_> dataList;
		};

		struct TyrePosInfo_
		{
			SSD::SimPoint3D frontLeft;
			SSD::SimPoint3D frontRight;
			SSD::SimPoint3D rearLeft;
			SSD::SimPoint3D rearRight;
		};

		/*!
		加载高精度地图
		\li function:
		*	LoadHDMap
		\li brief:
		*	Load hdmap which is configured by SimOne web app.
		@param[in]
		*	timeOutSeconds: Timeout setting to repeatedly check whether hdmap is ready to load. Will stop and return when time is up.
		@return
		*	Success or not
		*/
		SIMONE_API bool LoadHDMap(int timeOutSeconds);

		/*!
		获取最接近输入点的车道，所属车道优先
		\li function:
		*	GetNearMostLane
		\li brief:
		*	Get the lane which is near most to or geometry overlapping the input point. When there are more than one lane's geometry overlaps the input point, will pick the distance near most one.
		@param[in]
		*	pos: Input 3d location
		@param[out]
		*	id: Lane ID of founded lane. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	s, t: The input point's value pair in s-t coordinate system, relative to the found lane
		@param[out]
		*	s_toCenterLine, t_toCenterLine: is the input point's value pair in s-t coordinate system, relative to the found lane's owner road's center line. Values are fuzzy accurate, please use API GetRoadST for highly accurate values for [s_toCenterLine, t_toCenterLine]
		@return
		*	True when any lane is found, else returns false
		*/
		SIMONE_API bool GetNearMostLane(const SSD::SimPoint3D& pos, SSD::SimString& id, double& s, double& t, double& s_toCenterLine, double& t_toCenterLine);
		
		/*!
		获取临近车道列表
		\li function:
		*	GetNearLanes
		\li brief:
		*	Get near lanes which are close to the input point in a specified range.
		@param[in]
		*	pos: Input 3d location
		@param[in]
		*	distance: Input range distance
		@param[out]
		*	nearLanes: Lane IDs of founded lanes. Each ID with this format roadId_sectionIndex_laneId
		@return
		*	True when any lane(lanes) is(are) found, else returns false
		*/
		SIMONE_API bool GetNearLanes(const SSD::SimPoint3D& pos, const double& distance, SSD::SimStringVector& nearLanes);

		/*!
		获取视野范围内所有车道
		\li function:
		*	GetNearLanesWithAngle
		\li brief:
		*	Get near lanes which are close to the input point in a specified range and also heading to within a specified angle range in 2d-inertial system.
		@param[in]
		*	pos: Input 3d location
		@param[in]
		*	distance: Input distance range to search
		@param[in]
		*	headingAngle: A specified heading direction's angle relative to x-axis in 2d-inertial system. headingAngle is defined as radian
		@param[in]
		*	angleShift: To help define the range of angle as [headingAngle - angleShift, headingAngle + angleShift], and angleShift is defined as radian
		@param[out]
		*	nearLanes: Lane IDs of founded lanes. Each ID with this format roadId_sectionIndex_laneId
		@return
		*	True when any lane(lanes) is(are) found, else returns false
		*/
		SIMONE_API bool GetNearLanesWithAngle(const SSD::SimPoint3D& pos, const double& distance,
			const double& headingAngle, const double& angleShift, SSD::SimStringVector& nearLanes);

		/*!
		获取离最近车道左右边缘线的距离
		\li function:
		*	GetDistanceToLaneBoundary
		\li brief:
		*	Get the distance info to the near most lane's left and right boundaries
		@param[in]
		*	pos: Input 3d location
		@param[out]
		*	id: Lane ID of founded lane. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	distToLeft, distToRight: The distance to boundaries in 3d space
		*	distToLeft2D, distToRight2D: The distance to boundaries in 2d space(ignore height)
		@return
		*	True if near most lane is found, else returns false
		*/
		SIMONE_API bool GetDistanceToLaneBoundary(const SSD::SimPoint3D& pos, SSD::SimString& id, double& distToLeft, double& distToRight, double& distToLeft2D, double& distToRight2D);

		/*!
		获取车道信息(包含车道ID，左右边缘线，虚拟中心线)
		\li function:
		*	GetLaneSample
		\li brief:
		*	 Get lane sample info.
		@param[in]
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	 info: Lane information(HDMapStandalone::MLaneInfo) of specified lane
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetLaneSample(const SSD::SimString &id, HDMapStandalone::MLaneInfo& info);

		/*!
		获取车道连接信息
		\li function:
		*	GetLaneLink
		\li brief:
		*	 Get lane's link information based on lane's ID
		@param[in]
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	 laneLink: Lane link information(HDMapStandalone::MLaneLink) of specified lane
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetLaneLink(const SSD::SimString& id, HDMapStandalone::MLaneLink& laneLink);

		/*!
		获取车道类型
		\li function:
		*	GetLaneType
		\li brief:
		*	 Get lane's type
		@param[in]
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	 laneType: Lane type of specified lane
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetLaneType(const SSD::SimString& id, HDMapStandalone::MLaneType& laneType);

		/*!
		获取车道宽度
		\li function:
		*	GetLaneWidth
		\li brief:
		*	 Get lane's width in bitangent direction of specified point.
		@param[in]
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[in]
		*	 pos: Input 3d location
		@param[out]
		*	 width: lane width of specified lane
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetLaneWidth(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& width);

		/*!
		获取相对于车道虚拟中心线的ST坐标
		\li function:
		*	GetLaneST
		\li brief:
		*	 Get the [s, t] value pair in s-t coordinate system relative to the lane's center line
		@param[in]
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[in]
		*	 pos: Input 3d location
		@param[out]
		*	 s, t: The input point's value pair in s-t coordinate system, relative to specified lane
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetLaneST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t);

		/*!
		获取相对于道路参考线的ST坐标
		\li function:
		*	GetRoadST
		\li brief:
		*	 Get the [s, t] value pair in s-t coordinate system relative to the lane's owner road's reference line
		@param[in]
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param
		*	 pos: Input 3d location
		@param[out]
		*	 s, t, z: [s, t] represents the input point's value pair in s-t coordinate system, and z represents the input point's height value in localENU
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetRoadST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t, double& z);

		/*!
		根据车道ST坐标获取局部坐标
		\li function:
		*	GetInertialFromLaneST
		\li brief:
		*	 Get [x, y, z] position in localENU, based on [s, t] value pair in s-t coordinate system relative to the lane's center line
		@param[in]
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[in]
		*	 s, t: [s, t] represents the input point's value pair in s-t coordinate system
		@param[out]
		*	 inertial: The [x, y, z] position in localENU
		*	 dir: The direction in localENU on lane middle line
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir);

		/*!
		查询指定车道是否存在于地图之中
		\li function:
		*	ContainsLane
		\li brief:
		*	Check whether lane exists in current map.
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@return
		*	True if exists, else returns false
		*/
		SIMONE_API bool ContainsLane(const SSD::SimString& id);

		/*!
		获取地图中停车位列表
		\li function:
		*	GetParkingSpaceList
		\li brief:
		*	Get parkingSpace list in the map
		@param[out]
		*	ids: Parking space list
		*/
		SIMONE_API void GetParkingSpaceList(SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaceList);

		/*!
		获取路网路径规划
		\li function:
		*	GenerateRoute
		\li brief:
		*	Generate route for specified input points
		@param[in]
		*	inputPoints: Input points that to guide generated route should pass over
		@param[out]
		*	indexOfValidPoints: Pick valid ones from input points. Valid ones will be used for generting route
		@param[out]
		*	route: Generated route
		@return
		*	True if any route has been generated, else returns false
		*/
		SIMONE_API bool GenerateRoute(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimPoint3DVector& route);

		/*!
		获取规划路径所途径道路的ID列表
		\li function:
		*	Navigate
		\li brief:
		*	Provide routing path throughed road id list.
		@param[in]
		*	inputPoints: Input points that to guide generated route should pass over
		@param[out]
		*	indexOfValidPoints: Pick valid ones from input points. Valid ones will be used for generating route
		@param[out]
		*	roadIdList: road id list that are throughed by routing path
		@return
		*	True if any route has been generated, else returns false
		*/
		SIMONE_API bool Navigate(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimVector<long>& roadIdList);

		/*!
		根据指定车道id和局部坐标获取输入点左右两侧车道标线信息
		\li function:
		*	GetRoadMark
		\li brief:
		*	Get left and right roadmarks for specified input point and specified lane
		@param[in]
		*	pos: Input 3d location
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	left: Left side roadmark
		@param[out]
		*	right: Right side roadmark
		@return
		*	True if roadmark is found, else returns false
		*/
		SIMONE_API bool GetRoadMark(const SSD::SimPoint3D& pos, const SSD::SimString& id, HDMapStandalone::MRoadMark& left, HDMapStandalone::MRoadMark& right);

		/*!
		获取地图中信号灯列表
		\li function:
		*	GetTrafficLightList
		\li brief:
		*	Get traffic light list in the map.
		@param[out]
		*	list: Traffic light object list
		*/
		SIMONE_API void GetTrafficLightList(SSD::SimVector<HDMapStandalone::MSignal>& list);

		/*!
		获取地图中交通标志列表
		\li function:
		*	GetTrafficSignList
		\li brief:
		*	Get traffic sign list in the map.
		@param[out]
		*	list: Traffic sign object list
		*/
		SIMONE_API void GetTrafficSignList(SSD::SimVector<HDMapStandalone::MSignal>& list);

		/*!
		获取交通灯给定作用车道的关联停止线列表
		\li function:
		*	GetStoplineList
		\li brief:
		*	Get the list of stoplines that belongs to traffic light's validity matched to specified lane.
		@param[in]
		*	light: Traffic light object
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	stoplineList: Stoplines list that is associated
		*/
		SIMONE_API void GetStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& stoplineList);

		/*!
		获取交通灯给定作用车道的关联人行横道线列表
		\li function:
		*	GetCrosswalkList
		\li brief:
		*	Get the list of crosswalks that belongs to traffic light's validity matched to specified lane.
		@param[in]
		*	light: Traffic light object
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	stoplineList: Crosswalks list that is associated
		*/
		SIMONE_API void GetCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crosswalkList);

		/*!
		获取指定车道所在道路上的网状线列表
		\li function:
		*	GetCrossHatchList
		\li brief:
		*	Get the list of cross hatch in the specified lane's road neighborhood
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	stoplineList: Cross hatches list that is associated
		*/
		SIMONE_API void GetCrossHatchList(const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crossHatchList);

		/*!
		获取输入点投影到指定车道中心线上的点和切线方向
		\li function:
		*	GetLaneMiddlePoint
		\li brief:
		*	Get target point that the input point is reflected onto specified lane's middle line.
		@param[in]
		*	inputPt: Input 3d location
		@param[out]
		*	id: Lane ID of founded lane. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	targetPoint: The target point that is on specified lane's middle line
		@param[out]
		*	dir: The tangent direction on the target point on lane's middle line
		@return
		*	True when any lane is found, else returns false
		*/
		SIMONE_API bool GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir);

		/*!
		获取路网指定坐标点的高程列表
		\li function:
		*	GetHeights
		\li brief:
		*	Get height list of input point's radius area that covers in the map. The point may in vertical intersect with multiple roads with different heights, e.g. at highway or tunnel.
		@param[in]
		*	inputPt: Input 3d location
		@param[out]
		*	radius: Radius indicates how far away to detect in circle. It should be set larger than 3 meters in length. Setting as 3 meters is recommended
		@param[out]
		*	heights: Returns heights/one height
		@param[out]
		*	roadIds: Returns the road ids that the target height is based one
		@param[out]
		*	insideRoadStates: Returns whether inputPt is inside the target roads or not
		@return
		*	True if any height is found, else returns false
		*/
		SIMONE_API bool GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights,
			SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates);


		// %%%%%%%%%%%% V2 Add %%%%%%%%%%%%
		/*!
		获取所有车道线信息列表。
		\li function:
		*	GetLaneData
		\li brief:
		*	 Get all lane's info in the map
		@param[out]
		*	 data: All lane's MLaneInfo object as a list
		*/
		SIMONE_API void GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data);

		/*!
		获取所有Junction ID列表。
		\li function:
		*	GetJunctionList
		\li brief:
		*	 Get all junction id list in the map.
		@param
		*	None
		@return
		*	Junction Id list.
		*/
		SIMONE_API SSD::SimVector<long> GetJunctionList();

		/*!
		获取道路长度
		\li function:
		*	GetRoadLength
		\li brief:
		*	 Get road's length.
		@param
		*	 id: Input road ID
		@return
		*	Length of road.
		*/
		SIMONE_API double GetRoadLength(const long& roadId);

		/*!
		获取指定车道线所在Section的所有车道线ID列表
		\li function:
		*	GetSectionLaneList
		\li brief:
		*	Get lane id list in the same section for specified lane id. Note that roadId_sectionIndex_laneId's laneId should not be set as 0, as it does not make sense to use 0
		@param[in]
		*	laneId: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	sectionLaneList: Lane id list in the same section for specified lane
		@return
		*	True when any lane(lanes) is(are) found, else returns false
		*/
		SIMONE_API bool GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList);

		/*!
		判断指定道路是否是双向道路
		\li function:
		*	IsTwoSideRoad
		\li brief:
		*	Check whether specified road is two-side road or not
		@param
		*	roadId: Input road ID
		@return
		*	True if is two-side road, else returns false
		*/
		SIMONE_API bool IsTwoSideRoad(const long& roadId);

		/*!
		获取车道长度
		\li function:
		*	GetLaneLength
		\li brief:
		*	 Get lane's length.
		@param
		*	 id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@return
		*	Length of lane.
		*/
		SIMONE_API double GetLaneLength(const SSD::SimString& id);

		/*!
		判断车道是否为机动车道
		\li function:
		*	IsDriving
		\li brief:
		*	Check whether current lane is driving
		@param
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@return
		*	True if specified lane is driving type, else returns false
		*/
		SIMONE_API bool IsDriving(const SSD::SimString& id);

		/*!
		判断车道是否在交叉路口内
		\li function:
		*	IsInJunction
		\li brief:
		*	Check whether current lane belongs to a junction.
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	juncId: owner junction id
		@return
		*	True if specified lane is in a junction, else returns false.
		*/
		SIMONE_API bool IsInJunction(const SSD::SimString& id, long& juncId);

		/*!
		判断坐标点是否在车道内
		\li function:
		*	IsInsideLane
		\li brief:
		*	Check whether current lane belongs to a junction.
		@param[in]
		*	inputPt: Input 3d location
		@param[in]
		*	laneName: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	sideState: MSideState that specifies whether it is outside as left side or right side, or is inside
		@return
		*	True if specified point is inside the specified lane, else returns false.
		*/
		SIMONE_API bool IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState);


		// %%%%%%%%%%%% V3 Add %%%%%%%%%%%%
		/*!
		获取主车位置所在车道信息(包含车道ID，左右边缘线，虚拟中心线)
		\li function:
		*	GetLaneSampleByLocation
		\li brief:
		*	 Get all lane data in the loaded map
		@param[in]
		*	 pos: Input 3d location
		@param[out]
		*	 info: Lane information(HDMapStandalone::MLaneInfo) of specified lane
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info);

		//%%%%%%%%%%%%%% V4 Add %%%%%%%%%%%%%%%

		/*!
		获取所有车道线信息列表。
		\li function:
		*	GetLaneData
		\li brief:
		*	 Get all lane's info in the map
		@param[out]
		*	 data: All lane's MLaneLineInfo object as a list
		*/
		SIMONE_API void GetLaneLineInfo(SSD::SimVector<HDMapStandalone::MLaneLineInfo> & data);

		/*!
		获取给定车道的关联人行横道线列表
		\li function:
		*	GetSpecifiedLaneCrosswalkList
		\li brief:
		*	Get the list of crosswalks that belongs to matched specified lane.
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	stoplineList: Crosswalks list that is associated
		*/
		SIMONE_API void GetSpecifiedLaneCrosswalkList(const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crosswalkList);

		/*!
		获取给定车道的关联停止线列表
		\li function:
		*	GetSpecifiedLaneStoplineList
		\li brief:
		*	Get the list of stoplines that belongs to specified lane.
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	stoplineList: Stoplines list that is associated
		*/
		SIMONE_API void GetSpecifiedLaneStoplineList(const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& stoplineList);

		/*!
		获取地图中指定车道关联交通标志列表
		\li function:
		*	GetSpecifiedLaneTrafficSignalList
		\li brief:
		*	Get traffic sign list that belongs to specified lane.
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	list: Traffic sign object list
		*/
		SIMONE_API void GetSpecifiedLaneTrafficSignalList(const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MSignal>& list);

		/*!
		获取指定车道绑定的信号的列表
		\li function:
		*	GetspecifiedLaneTrafficLightList
		\li brief:
		*	Get traffic light list that belongs to specified lane.
		@param[in]
		*	id: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[out]
		*	list: Traffic light object list
		*/
		SIMONE_API void GetSpecifiedLaneTrafficLightList(const SSD::SimString & id, SSD::SimVector<HDMapStandalone::MSignal>& list);

		/*!
		获取地图中指定点前方一定距离停车位列表
		\li function:
		*	GetParkingSpaceIds
		\li brief:
		*	Get parkingSpace list in range of distance
		@param[in]
		*	inputPt: Input 3d location
		@param[in]
		*	distance: Input range distance
		@param[out]
		*	ids: Parking space list
		@return
		*	True if exists, else returns false
		*/
		SIMONE_API bool GetParkingSpaceIds(const SSD::SimPoint3D& inputPt, double distance, SSD::SimStringVector& ids);

		/*!
		获取前方一定距离内的所有车道信息(包含车道ID，左右边缘线，虚拟中心线)
		\li function:
		*	GetForwardLaneSample
		\li brief:
		*	 Get forward lane sample info .
		@param[in]
		*	 inputPt: Input current pos, pos is a 3d point.
		@param[in]
		*	 id: Lane ID of founded lane. ID with this format roadId_sectionIndex_laneId
		@param[in]
		*	 forward: The forward distance from inputPt
		@param[out]
		*	 laneInfoList: Lane information(HDMapStandalone::MLaneInfo) of specified forward range
		@return
		*	True if specified lane exists in the map, else returns false
		*/
		SIMONE_API bool GetForwardLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, const double& forward,
			SSD::SimVector<HDMapStandalone::MLaneInfo>& laneInfoList);

		/*!
		通过信号类型获取指定道路的车道指示信息，地面标识（type="Graphics"）
		\li function:
		*	GetSignalListOnLaneByType
		\li brief:
		*	 Get signal list of Specified type on Specified Lanename.
		@param[in]
		*	 laneName: Input lane ID. ID with this format roadId_sectionIndex_laneId
		@param[in]
		*	 type: Signal type string like "Graphics"
		@param[out]
		* signalInfo：signalInfo list as MSignal
		*/
		SIMONE_API void GetSignalListOnLaneByType(const SSD::SimString& laneName, const SSD::SimString& type, SSD::SimVector<HDMapStandalone::MSignal> & signalInfo);

		/*!
		获取指定道路id内所有的车道名称
		\li function:
		*	GetLaneList
		\li brief:
		*	 roadId: Get Lane list in road id
		@param[out]
		*	 laneList: Lane name list in roadId
		*/
		SIMONE_API void GetLaneList(const long& roadId, SSD::SimStringVector &laneList);

		/*!
		获取指定十字路口的详细信息
		\li function:
		*	GetJunction
		\li brief:
		*	 Get junction detail info of Specified junctionId
		@param[out]
		*	 junction: junction detail info as MJunction
		*/
		SIMONE_API void GetJunction(const long& junctionId, HDMapStandalone::MJunction &junction);

		/*!
		获取路网路径规划详细信息
		\li function:
		*	GenerateRoute_V2
		\li brief:
		*	Generate route detail info for specified input points
		@param[in]
		*	inputPoints: Input points that to guide generated route should pass over
		@param[out]
		*	indexOfValidPoints: Pick valid ones from input points. Valid ones will be used for generting route
		@param[out]
		*	path: waypoints and segment info of generated route
		@param[out]
		*	routePtList: lane detail info of generated route
		@return
		*	True if any route has been generated, else returns false
		*/
		SIMONE_API bool GenerateRoute_V2(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints,
			HDMapStandalone::MRoutePath &path, SSD::SimVector<HDMapStandalone::MRoutePoint>& routePtList);
	}
#ifdef __cplusplus
}
#endif
#endif
