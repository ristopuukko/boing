/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>

Modified by Roman Ponomarev <rponom@gmail.com>
01/22/2010 : Constraints reworked
01/27/2010 : Replaced COLLADA export with Bullet binary export

Modified by Francisco Gochez <fjgochez@gmail.com>
Nov 2011 - Dec 2011 : Added logic for soft bodies
*/

#include "BulletSoftBody/btSoftBody.h"

//pluginMain.cpp
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MDGMessage.h>
#include <maya/MSceneMessage.h>
#include <maya/MPxTransformationMatrix.h>

#include "mayaUtils.h"
#include "boingRBNode.h"
#include "constraint/nailConstraintNode.h"
#include "constraint/hingeConstraintNode.h"
#include "constraint/sliderConstraintNode.h"
#include "constraint/sixdofConstraintNode.h"
#include "bSolverNode.h"
#include "bSolverCmd.h"
#include "boingRBCmds.h"
#include "bCallBackNode.h"
#include "constraint/dNailConstraintCmd.h"
#include "constraint/dHingeConstraintCmd.h" 
#include "constraint/dSliderConstraintCmd.h" 
#include "constraint/dsixdofConstraintCmd.h" 
#include "mvl/util.h"
#include "bulletExport.h"
#include "colladaExport.h"
#include "softBodyNode.h"
#include "dSoftBodyCmd.h"


const char *const bulletOptionScript = "bulletExportOptions";
const char *const bulletDefaultOptions =    "groups=1;"    "ptgroups=1;"    "materials=1;"    "smoothing=1;"    "normals=1;"    ;

const char *const colladaOptionScript = "bulletExportOptions";
const char *const colladaDefaultOptions =    "groups=1;"    "ptgroups=1;"    "materials=1;"    "smoothing=1;"    "normals=1;"    ;


///we need to register a scene exit callback, otherwise the btRigidBody destructor might assert in debug mode
///if some constraints are still attached to it
MCallbackId fMayaExitingCB=0;
void releaseCallback(void* clientData)
{
	solver_t::destroyWorld();
    solver_t::cleanup();
}



MStatus initializePlugin( MObject obj )
{
    MStatus   status;
	char version[1024];
	sprintf(version,"Bullet %d, build %s",BT_BULLET_VERSION,__DATE__);
#ifdef BT_DEBUG
	 sprintf(version,"Bullet %d, debug build %s",BT_BULLET_VERSION,__DATE__);
#else
	 sprintf(version,"Bullet %d, release build %s",BT_BULLET_VERSION,__DATE__);
#endif
	  MFnPlugin plugin( obj, "Risto Puukko Enterprises", version, "Any");

    solver_t::initialize();
    


// Register the translator with the system
   status =  plugin.registerFileTranslator( "Bullet Physics export", "none",
                                          BulletTranslator::creator,
                                          (char *)bulletOptionScript,
                                          (char *)bulletDefaultOptions,
                                           false,
                                           MFnPlugin::kDefaultDataLocation);

	MCHECKSTATUS(status,"registerFileTranslator Bullet Physics export");

#ifdef BT_USE_COLLADA
   status =  plugin.registerFileTranslator( "COLLADA Physics export", "none",
                                          ColladaTranslator::creator,
                                          (char *)colladaOptionScript,
                                          (char *)colladaDefaultOptions,
                                           false,
                                           MFnPlugin::kDefaultDataLocation);

	MCHECKSTATUS(status,"registerFileTranslator COLLADA Physics export");

#endif
    
    //
    /*
    status = plugin.registerNode( rigidBodyNode::typeName, rigidBodyNode::typeId,
                                  rigidBodyNode::creator,
                                  rigidBodyNode::initialize,
                                  MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering rigidBodyNode")
    MDGMessage::addNodeRemovedCallback(rigidBodyNode::nodeRemoved, rigidBodyNode::typeName);
*/
    status = plugin.registerNode( boingRBNode::typeName, boingRBNode::typeId,
                                 boingRBNode::creator,
                                 boingRBNode::initialize,
                                 MPxNode::kDependNode );
    MCHECKSTATUS(status, "registering boingRBNode")
    MDGMessage::addNodeRemovedCallback(boingRBNode::nodeRemoved, boingRBNode::typeName);

    /*
    status = plugin.registerNode( rigidBodyArrayNode::typeName, rigidBodyArrayNode::typeId,
                                  rigidBodyArrayNode::creator,
                                  rigidBodyArrayNode::initialize,
                                  MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering rigidBodyArrayNode")
    MDGMessage::addNodeRemovedCallback(rigidBodyArrayNode::nodeRemoved, rigidBodyArrayNode::typeName);
/*

    //
    /*
    status = plugin.registerNode( collisionShapeNode::typeName, collisionShapeNode::typeId,
                                  collisionShapeNode::creator,
                                  collisionShapeNode::initialize,
                                  MPxNode::kDependNode );
    MCHECKSTATUS(status, "registering collisionShapeNode")
*/
    //
    status = plugin.registerNode( nailConstraintNode::typeName, nailConstraintNode::typeId,
                                  nailConstraintNode::creator,
                                  nailConstraintNode::initialize,
                                  MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering nailConstraintNode")
    MDGMessage::addNodeRemovedCallback(nailConstraintNode::nodeRemoved, nailConstraintNode::typeName);

    //
    status = plugin.registerNode( hingeConstraintNode::typeName, hingeConstraintNode::typeId,
                                  hingeConstraintNode::creator,
                                  hingeConstraintNode::initialize,
                                  MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering hingeConstraintNode")
    MDGMessage::addNodeRemovedCallback(hingeConstraintNode::nodeRemoved, hingeConstraintNode::typeName);

	//
    status = plugin.registerNode( sliderConstraintNode::typeName, sliderConstraintNode::typeId,
                                  sliderConstraintNode::creator,
                                  sliderConstraintNode::initialize,
                                  MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering sliderConstraintNode")
    MDGMessage::addNodeRemovedCallback(sliderConstraintNode::nodeRemoved, sliderConstraintNode::typeName);

	//
    status = plugin.registerNode( sixdofConstraintNode::typeName, sixdofConstraintNode::typeId,
                                  sixdofConstraintNode::creator,
                                  sixdofConstraintNode::initialize,
                                  MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering sixdofConstraintNode")
    MDGMessage::addNodeRemovedCallback(sixdofConstraintNode::nodeRemoved, sixdofConstraintNode::typeName);

	//
    /*
    status = plugin.registerNode( dSolverNode::typeName, dSolverNode::typeId,
                                  dSolverNode::creator, 
                                  dSolverNode::initialize,
//                                  MPxNode::kDependNode );
                                  MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering dSolverNode")

    status = plugin.registerCommand( dSolverCmd::typeName,
                                     dSolverCmd::creator,
                                     dSolverCmd::syntax);
    MCHECKSTATUS(status, "registering dSolverCmd")

    status = plugin.registerCommand( dRigidBodyCmd::typeName,
                                     dRigidBodyCmd::creator,
                                     dRigidBodyCmd::syntax);
    MCHECKSTATUS(status, "registering dRigidBodyCmd")*/
    
     
//<rp 2014>
    status = plugin.registerNode( bSolverNode::typeName, bSolverNode::typeId,
                                 bSolverNode::creator,
                                 bSolverNode::initialize,
                                 //MPxNode::kDependNode );
                                 MPxNode::kLocatorNode );
    MCHECKSTATUS(status, "registering bSolverNode")
    
    status = plugin.registerCommand( bSolverCmd::typeName,
                                    bSolverCmd::creator,
                                    bSolverCmd::syntax);
    MCHECKSTATUS(status, "registering bSolverCmd")
    
    status = plugin.registerCommand( createBoingRBCmd::typeName,
                                    createBoingRBCmd::creator,
                                    createBoingRBCmd::syntax);
    MCHECKSTATUS(status, "registering createBoingRBCmd")

    status = plugin.registerCommand( boingRbCmd::typeName,
                                    boingRbCmd::creator,
                                    boingRbCmd::cmdSyntax);
    MCHECKSTATUS(status, "registering boingRbCmd")
    
    const MString classification = "drawdb/geometry/transform";
    status = plugin.registerTransform( bCallBackNode::typeName,
                                 bCallBackNode::typeId,
                                 bCallBackNode::creator,
                                 bCallBackNode::initialize,
                                      MPxTransformationMatrix::creator,
                                      MPxTransformationMatrix::baseTransformationMatrixId,
                                 &classification);
    //                             MPxNode::kLocatorNode );

    MCHECKSTATUS(status, "registering bCallBackNode")
    
//</rp 2014>
    /*
    status = plugin.registerCommand( dRigidBodyArrayCmd::typeName,
                                     dRigidBodyArrayCmd::creator,
                                     dRigidBodyArrayCmd::syntax);
    MCHECKSTATUS(status, "registering dRigidBodyArrayCmd")
*/
    status = plugin.registerCommand( dNailConstraintCmd::typeName,
                                     dNailConstraintCmd::creator,
                                     dNailConstraintCmd::syntax);
    MCHECKSTATUS(status, "registering dNailConstraintCmd")

    status = plugin.registerCommand( dHingeConstraintCmd::typeName,
                                     dHingeConstraintCmd::creator,
                                     dHingeConstraintCmd::syntax);
    MCHECKSTATUS(status, "registering dHingeConstraintCmd")

	status = plugin.registerCommand( dSliderConstraintCmd::typeName,
                                     dSliderConstraintCmd::creator,
                                     dSliderConstraintCmd::syntax);
    MCHECKSTATUS(status, "registering dSliderConstraintCmd")

	status = plugin.registerCommand( dSixdofConstraintCmd::typeName,
                                     dSixdofConstraintCmd::creator,
                                     dSixdofConstraintCmd::syntax);
    MCHECKSTATUS(status, "registering dSixdofConstraintCmd")

	status = plugin.registerCommand( dSoftBodyCmd::typeName,
		dSoftBodyCmd::creator, dSoftBodyCmd::syntax);
	MCHECKSTATUS(status, "registering dSoftBodyCmd")

	status = plugin.registerNode( SoftBodyNode::typeName, SoftBodyNode::typeId,
                                  SoftBodyNode::creator,
                                  SoftBodyNode::initialize,
                                  MPxNode::kLocatorNode );
	MDGMessage::addNodeRemovedCallback(SoftBodyNode::nodeRemoved, SoftBodyNode::typeName);
    MCHECKSTATUS(status, "registering SoftBodyNode")

	MGlobal::executeCommand( "source dynamicaUI.mel" );
    MGlobal::executeCommand( "dynamicaUI_initialize" );
    

	fMayaExitingCB				= MSceneMessage::addCallback( MSceneMessage::kMayaExiting,				releaseCallback, 0);

    return status;
}



MStatus uninitializePlugin( MObject obj )
{
    MStatus   status;
    MFnPlugin plugin( obj );

    status = plugin.deregisterCommand(dNailConstraintCmd::typeName);
    MCHECKSTATUS(status, "deregistering dNailConstraintCmd")

    status = plugin.deregisterCommand(dHingeConstraintCmd::typeName);
    MCHECKSTATUS(status, "deregistering dHingeConstraintCmd")

    status = plugin.deregisterCommand(dSliderConstraintCmd::typeName);
    MCHECKSTATUS(status, "deregistering dSliderConstraintCmd")

    status = plugin.deregisterCommand(dSixdofConstraintCmd::typeName);
    MCHECKSTATUS(status, "deregistering dSixdofConstraintCmd")

/*
	status = plugin.deregisterCommand(dRigidBodyArrayCmd::typeName);
    MCHECKSTATUS(status, "deregistering dRigidBodyArrayCmd")
 */
/*
    status = plugin.deregisterCommand(dRigidBodyCmd::typeName);
    MCHECKSTATUS(status, "deregistering dRigidBodyCmd")
*/
    //<rp 2014>
    status = plugin.deregisterCommand(createBoingRBCmd::typeName);
    MCHECKSTATUS(status, "deregistering createBoingRBCmd")

    status = plugin.deregisterCommand( boingRbCmd::typeName );
    MCHECKSTATUS(status, "deregistering boingRbCmd")

    status = plugin.deregisterNode(bCallBackNode::typeId);
    MCHECKSTATUS(status, "deregistering bCallBackNode")

    status = plugin.deregisterCommand(bSolverCmd::typeName);
    MCHECKSTATUS(status, "deregistering bSolverCmd")
    
    status = plugin.deregisterNode(boingRBNode::typeId);
    MCHECKSTATUS(status, "deregistering boingRBNode")

	status = plugin.deregisterNode(bSolverNode::typeId);
    MCHECKSTATUS(status, "deregistering bSolverNode")

    //</rp 2014>

    /*status = plugin.deregisterCommand(dSolverCmd::typeName);
    MCHECKSTATUS(status, "deregistering dSolverCmd")
*/
	status = plugin.deregisterCommand(dSoftBodyCmd::typeName);
	MCHECKSTATUS(status, "deregistering dSoftBodyCmd")


    status = plugin.deregisterNode(nailConstraintNode::typeId);
    MCHECKSTATUS(status, "deregistering nailConstraintNode")

	status = plugin.deregisterNode(hingeConstraintNode::typeId);
    MCHECKSTATUS(status, "deregistering hingeConstraintNode")

	status = plugin.deregisterNode(sliderConstraintNode::typeId);
    MCHECKSTATUS(status, "deregistering sliderConstraintNode")

	status = plugin.deregisterNode(sixdofConstraintNode::typeId);
    MCHECKSTATUS(status, "deregistering sixdofConstraintNode")
/*
	status = plugin.deregisterNode(collisionShapeNode::typeId);
    MCHECKSTATUS(status, "deregistering collisionShapeNode")
*/
    /*
    status = plugin.deregisterNode(rigidBodyArrayNode::typeId);
    MCHECKSTATUS(status, "deregistering rigidBodyArrayNode")
     */
/*
    status = plugin.deregisterNode(rigidBodyNode::typeId);
    MCHECKSTATUS(status, "deregistering rigidBodyNode")
*/
    /*
	status = plugin.deregisterNode(dSolverNode::typeId);
    MCHECKSTATUS(status, "deregistering dSolverNode")
*/
	status = plugin.deregisterNode(SoftBodyNode::typeId);
	MCHECKSTATUS(status, "deregistering SoftBodyNode")

    status =  plugin.deregisterFileTranslator( "Bullet Physics export" );
    MCHECKSTATUS(status,"deregistering Bullet Physics export" )


#ifdef BT_USE_COLLADA
	status =  plugin.deregisterFileTranslator( "COLLADA Physics export" );
    MCHECKSTATUS(status,"deregistering COLLADA Physics export" )
#endif //BT_USE_COLLADA

	solver_t::destroyWorld();

    solver_t::cleanup();

    return status;
}


