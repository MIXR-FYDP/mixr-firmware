/*******************************************************************************
*
********************************************************************************
*     ModuleList.h  Created on 10-Mar-2020 16:36:40
********************************************************************************
*
*     Description:  Module list for the target
*
*     Copyright: (c) 2019 DSP Concepts, Inc. All rights reserved.
*                         3235 Kifer Road
*                         Santa Clara, CA 95054-1527
*
*******************************************************************************/

#define TOTALNUMBEROFCLASSES 2

extern const UINT32 awe_modSecondOrderFilterSmoothedClass;
extern const UINT32 awe_modTypeConversionClass;


#define LISTOFCLASSOBJECTS \
&awe_modSecondOrderFilterSmoothedClass, \
&awe_modTypeConversionClass
