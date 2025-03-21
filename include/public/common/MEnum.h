// ==========================================================================
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

#include "public/libexport.h"

namespace HDMapStandalone {

	//@brief When loading xodr gets failed, LoadData() will report error code.
	//eNotLocalENU - the system only accepts xodr based on localENU.
	//eInvalidVersion - xodr is not a valid version.
	//eEmptyRoad - xodr does not have any road.
	//eInvalidData - xodr contains invalid data that may cause runtime exception.
	enum class MLoadErrorCode
	{
		eNotLocalENU,
		eInvalidVersion,
		eEmptyRoad,
		eInvalidData
	};

	//@brief When loading xodr, need to specify xodr's data type.
	//eXodrFile - xodr file, as file name, and geographic info is LocalENU only.
	//eXodrContent - xodr content, and geographic info is localENU only.
	//eDBContent - xodr data is from database.
	enum class MXodrSourceType
	{
		eXodrFile,
		eXodrContent,
		eDBContent
	};

	//@brief enum type definition of lane type. Its value set perfectly matches with opendrive format.
	enum class MLaneType
	{
		none,
		driving,
		stop,
		shoulder,
		biking,
		sidewalk,
		border,
		restricted,
		parking,
		bidirectional,
		median,
		special1,
		special2,
		special3,
		roadWorks,
		tram,
		rail,
		entry,
		exit,
		offRamp,
		onRamp,
		mwyEntry,
		mwyExit
	};

	//@brief enum type definition of road type. Its value set perfectly matches with opendrive format.
	enum class MRoadType
	{
		unknown,
		rural,
		motorway,
		town,
		lowSpeed,
		pedestrian,
		bicycle,
		rail
	};

	enum class MSideState
	{
		eLeftSide,
		eInner,
		eRightSide
	};

	enum class MInsideType
	{
		eInsideLane,
		eInsideParkingSpace,
		eNA
	};
}