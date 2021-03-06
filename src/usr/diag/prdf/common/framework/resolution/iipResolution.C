/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/diag/prdf/common/framework/resolution/iipResolution.C $ */
/*                                                                        */
/* OpenPOWER HostBoot Project                                             */
/*                                                                        */
/* COPYRIGHT International Business Machines Corp. 1996,2014              */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/*                                                                        */
/* IBM_PROLOG_END_TAG                                                     */

// Module Description **************************************************
//
// Description: PRD resolution definition
//
// End Module Description **********************************************

//----------------------------------------------------------------------
//  Includes
//----------------------------------------------------------------------
#define iipResolution_C

#include <iipconst.h>
#include <CcAutoDeletePointer.h>
#include <iipSystem.h>
#include <prdfGlobal.H>
//#include <iipCalloutMap.h>
#include <iipCalloutResolution.h>
#include <iipstep.h>
#include <iipCaptureData.h>
#include <iipServiceDataCollector.h>
#include <iipErrorRegister.h>
#include <iipEregResolution.h>
#include <iipsdbug.h>
#include <iipResolutionList.h>
#include <iipCallAttnResolution.h>
#include <iipTerminateResolution.h>
#include <iipAnalyzeChipResolution.h>
#include <xspprdTryResolution.h>
#include <iipchip.h>
#include <prdfCalloutConnected.H>
#include <prdfAnalyzeConnected.H>
#include <prdfPlatServices.H>
#undef iipResolution_C

namespace PRDF
{

using namespace PlatServices;

//----------------------------------------------------------------------
//  User Types
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//  Constants
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//  Macros
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//  Internal Function Prototypes
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//  Global Variables
//----------------------------------------------------------------------

//---------------------------------------------------------------------
// Resolution Member Function Specifications
//---------------------------------------------------------------------
Resolution::~Resolution() {}

//---------------------------------------------------------------------
// EregResolution Member Function Specifications
//---------------------------------------------------------------------

int32_t EregResolution::Resolve( STEP_CODE_DATA_STRUCT & io_data )
{
    int32_t rc = PRD_INTERNAL_CODE_ERROR;
    if( errorRegister != NULL )
    {
        rc = errorRegister->Analyze( io_data );
    }
    return rc;
}


//---------------------------------------------------------------------
// CalloutResolution Member Function Specifications
// using MruValues (xspiiCallout.h)
//---------------------------------------------------------------------

int32_t CalloutResolution::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    /*
    This resolution is only needed when we callout self.So during RuleChip
    creation,we create CalloutResolution passing NULL as target value.In Resolve
    function when we need to update SDC with target to be called out we get it
    from Service Data Collector.It is because target currently under analysis is
    the target that needs to be called out here.By instantiating Callout
    resolution with just priority info and NULL target , we are able to reduce
    CalloutResolution objects to one per priority instead of one per target per
    priority.So,this reduction in number of resolution objects shall eventually
    reduce memory utilization.
    */

    if ( PRDcalloutData::TYPE_TARGET == xMruCallout.getType() )
    {
        PRDcallout l_targetCallout( ServiceDataCollector::getTargetAnalyzed() );
        io_serviceData.service_data->SetCallout( l_targetCallout, xPriority );
    }
    else
    {
        io_serviceData.service_data->SetCallout( xMruCallout,xPriority );
    }

    return(SUCCESS);
}

//--------------------------------------------------------------------
// ResolutionList Member Functions
//--------------------------------------------------------------------

int32_t ResolutionList::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    int32_t rc = SUCCESS;
    for(std::vector<void *>::iterator iter = resolutionList.begin();
        iter != resolutionList.end();
        ++iter)
    {
        Resolution * r = (Resolution *) *iter;
        rc = r->Resolve( io_serviceData );
        if( rc != SUCCESS ) break;
    }
    return(rc);
}

//--------------------------------------------------------------------
// ThresholdResolution Member Functions
//--------------------------------------------------------------------

//int32_t ThresholdResolution::Resolve(STEP_CODE_DATA_STRUCT & error)
//{
//  ++count;
//  error.service_data->SetHits((uint16_t)count);
//  error.service_data->SetThreshold((uint16_t)threshold);
//  if((count >= threshold) || (error.service_data->IsFlooding()))
//  {
//    error.service_data->SetThresholdMaskId(maskId);  // threshold, degraded YES
//  }
//  int32_t rc = SUCCESS;
//  if(xRes != NULL) rc = xRes->Resolve(error);
//  return rc;
//}

//--------------------------------------------------------------------
// Call all chips raising attention as reported by sp sysdebug area
//--------------------------------------------------------------------
int32_t CallAttnResolution::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    int32_t rc = NO_DOMAINS_AT_ATTENTION;
    SYSTEM_DEBUG_CLASS systemDebug;

    ErrorSignature * signature =
                        io_serviceData.service_data->GetErrorSignature();
    signature->clear();
    signature->setChipId(0xffffffff);

    systemDebug.CalloutThoseAtAttention( io_serviceData );

    signature->setErrCode((uint16_t)NO_PRD_ANALYSIS);

    return(rc);
}

// ********************************************************************

int32_t TerminateResolution::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    io_serviceData.service_data->SetTerminate();
    return(SUCCESS);
}

// ********************************************************************

int32_t AnalyzeChipResolution::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    // mk442956 a
    return xChip.Analyze( io_serviceData,
                          io_serviceData.service_data->GetCauseAttentionType() );
}

// ********************************************************************

int32_t TryResolution::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    // Save the current error signature
    ErrorSignature * es = io_serviceData.service_data->GetErrorSignature();
    ErrorSignature temp = *es;
    // Try the tryResolution
    int32_t rc = xTryResolution->Resolve( io_serviceData  );
    if ( (SUCCESS != rc) && (PRD_NO_CLEAR_FIR_BITS != rc) ) // if it didn't work
    {
        // Restore signature
        *es = temp;
        // Call the default signature
        rc = xDefaultResolution->Resolve( io_serviceData );
    }
    return rc;
}

int32_t CalloutConnected::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    using namespace TARGETING;

    TargetHandle_t sourceTrgt  = ServiceDataCollector::getTargetAnalyzed();
    TargetHandle_t connTrgt    = NULL;
    TargetHandle_t srcEndPoint = NULL;

    if(TYPE_NA == iv_peerConnType)
    {
        TargetHandleList list = getConnected( sourceTrgt, iv_targetType );

        if ( 0xffffffff == iv_idx )
        {
            if ( 0 < list.size() )
                connTrgt = list[0];
        }
        else
        {
            for (TargetHandleList::iterator i = list.begin();
                 i != list.end();
                 i++)
            {
                if ( iv_idx == getTargetPosition(*i) )
                {
                    connTrgt = *i;
                    break;
                }
            }
        }
    }
    else
    {
        srcEndPoint = getConnectedChild( sourceTrgt, iv_peerConnType, iv_idx );

        if ( NULL != srcEndPoint )
            connTrgt = getConnectedPeerTarget( srcEndPoint );
    }

    if ( NULL != connTrgt )
        io_serviceData.service_data->SetCallout( connTrgt, iv_priority );
    else
    {
        if ( NULL != iv_altResolution )
            iv_altResolution->Resolve( io_serviceData );
        else
        {
            PRDF_ERR( "[CalloutConnected::Resolve] No connected chip found:"
                      " sourceTrgt=0x%08x, iv_peerConnType=0x%x",
                        getHuid(sourceTrgt), iv_peerConnType);

            io_serviceData.service_data->SetCallout( sourceTrgt );
        }
    }

    return SUCCESS;
}

//--------------------------------------------------------------------
// AnalyzeConnected Member Functions
//--------------------------------------------------------------------
int32_t AnalyzeConnected::Resolve( STEP_CODE_DATA_STRUCT & io_serviceData )
{
    using namespace TARGETING;

    TargetHandle_t sourceTrgt = ServiceDataCollector::getTargetAnalyzed();
    TargetHandle_t connTrgt   = NULL;

    TargetHandleList list = getConnected( sourceTrgt, iv_targetType );

    if ( 0xffffffff == iv_idx )
    {
        if ( 0 < list.size() )
            connTrgt = list[0];
    }
    else
    {
        for (TargetHandleList::iterator i = list.begin(); i != list.end(); i++)
        {
            if ( iv_idx == getTargetPosition(*i) )
            {
                connTrgt = *i;
                break;
            }
        }
    }

    // If valid chip found, look up in global system container.
    CHIP_CLASS * connChip = NULL;
    if ( NULL != connTrgt )
    {
        connChip = systemPtr->GetChip( connTrgt );
    }

    // Analyze chip.
    if ( NULL != connChip )
        return connChip->Analyze( io_serviceData,
                        io_serviceData.service_data->GetCauseAttentionType() );
    else
        return PRD_UNRESOLVED_CHIP_CONNECTION;
}

} // end namespace PRDF

