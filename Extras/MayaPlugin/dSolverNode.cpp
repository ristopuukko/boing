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

Modified by Francisco Gochez <fjgochez@gmail.com>
Nov 2011 - Dec 2011 : Added logic for soft bodies

Modified by Dongsoo Han <dongsoo.han@amd.com>
04/11/2012 : Added contactData attribute and contact information code to enable/disable updating contactCount, contactName and 
			 contactPosition attributes in rigidBodyNode.
*/

//dSolverNode.cpp

#include "BulletSoftBody/btSoftBody.h"

#include <maya/MFnNumericAttribute.h>           
#include <maya/MFnEnumAttribute.h>           
#include <maya/MFnTypedAttribute.h>           
#include <maya/MFnUnitAttribute.h>           
#include <maya/MFnMessageAttribute.h>           
#include <maya/MFnStringData.h>           
#include <maya/MFnStringArrayData.h>
#include <maya/MPlugArray.h>
#include <maya/MFnDagNode.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>
#include <maya/MFnTransform.h>
#include <maya/MFnMesh.h>
#include <maya/MVector.h>
#include <maya/MGlobal.h>
#include <maya/MFnField.h>
#include <maya/MVectorArray.h>
#include <maya/MItDag.h>
#include <maya/MConditionMessage.h>
#include <maya/MDGModifier.h>
#include <maya/MDGMessage.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MSceneMessage.h>
#include <maya/MFloatPointArray.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MFnDependencyNode.h>


#include <fstream>
#include <sstream>
#include <iostream>
#include <iterator>

#include "mayaUtils.h"
#include "dSolverNode.h"
#include "solver.h"
#include "rigidBodyNode.h"
#include "rigidBodyArrayNode.h"
#include "constraint/nailConstraintNode.h"
#include "constraint/hingeConstraintNode.h"
#include "constraint/sliderConstraintNode.h"
#include "constraint/sixdofConstraintNode.h"
#include "LinearMath/btGeometryUtil.h"
#include "pdbIO.h"
#include "collisionShapeNode.h"
#include "softBodyNode.h"
#include "soft_body_t.h"
#include "bt_rigid_body.h"
#include "bt_solver.h"

#include <omp.h>

MTypeId dSolverNode::typeId(0x100331);
MString dSolverNode::typeName("dSolver");

MObject dSolverNode::ia_time;
MObject dSolverNode::ia_startTime;
MObject dSolverNode::ia_gravity;
MObject dSolverNode::ia_enabled;
MObject dSolverNode::ia_collisionMargin; //mb
MObject dSolverNode::ia_disableCollisionsBetweenLinkedBodies;
MObject dSolverNode::ia_fixedPhysicsRate;
MObject dSolverNode::ia_splitImpulse;
MObject dSolverNode::ia_substeps;
MObject dSolverNode::oa_rigidBodies;
MObject dSolverNode::oa_softBodies;
MObject dSolverNode::ia_contactData;
//MObject dSolverNode::ssSolverType;
bool	dSolverNode::isStartTime;


MObject     dSolverNode::ia_DBG_DrawWireframe;
MObject     dSolverNode::ia_DBG_DrawAabb;
MObject     dSolverNode::ia_DBG_DrawFeaturesText;
MObject     dSolverNode::ia_DBG_DrawContactPoints;
MObject     dSolverNode::ia_DBG_NoDeactivation;
MObject     dSolverNode::ia_DBG_NoHelpText;
MObject     dSolverNode::ia_DBG_DrawText;
MObject     dSolverNode::ia_DBG_ProfileTimings;
MObject     dSolverNode::ia_DBG_EnableSatComparison;
MObject     dSolverNode::ia_DBG_DisableBulletLCP;
MObject     dSolverNode::ia_DBG_EnableCCD;
MObject     dSolverNode::ia_DBG_DrawConstraints;
MObject     dSolverNode::ia_DBG_DrawConstraintLimits;
MObject     dSolverNode::ia_DBG_FastWireframe;

float dSolverNode::collisionMarginOffset; //mb

#define ATTR_POSITION "position"
//#define ATTR_POSITION_TYPE VECTOR_ATTR
#define ATTR_VELOCITY "velocity"
//#define ATTR_VELOCITY_TYPE VECTOR_ATTR

#define ATTR_IN_RAXIS  "in_rotvec"               /* vector */
//#define ATTR_IN_RAXIS_TYPE VECTOR_ATTR
#define ATTR_IN_RANGLE "in_rotang"               /* float  */
//#define ATTR_IN_RANGLE_TYPE FLOAT_ATTR
#define ATTR_IN_ROTVEL "in_rotvel"               /* float  */
//#define ATTR_IN_ROTVEL_TYPE FLOAT_ATTR
#define ATTR_IN_RAXISNEXT  "in_rotvec_next"      /* vector */
//#define ATTR_IN_RAXISNEXT_TYPE VECTOR_ATTR
#define ATTR_IN_RANGLENEXT "in_rotang_next"      /* float  */
//#define ATTR_IN_RANGLENEXT_TYPE FLOAT_ATTR




static int getDbgDrawVal(const MObject& thisObj, const MObject& attr, int flag)
{
	MPlug plug(thisObj, attr);
	bool retVal = false;
	plug.getValue(retVal);
	return retVal ? (1 << flag) : 0;
}

void dSolverNode::getRigidBodies(MStringArray& rbds) {

	MStatus stat;
    //MStringArray result;
    //	MItDag dagIterator( MItDag::kBreadthFirst, MFn::kInvalid, &stat);
	MItDag dagIterator( MItDag::kDepthFirst, MFn::kInvalid, &stat);
   	if (stat != MS::kSuccess)
	{
        std::cout << "Failure in DAG iterator setup" << std::endl;
   	    return;
   	}
	for ( ;!dagIterator.isDone(); dagIterator.next())
	{
   	    MDagPath dagPath;
   	    stat = dagIterator.getPath( dagPath );
   		if(stat != MS::kSuccess)
		{
			std::cout << "Failure in getting DAG path" << std::endl;
   			return;
		}
		// skip over intermediate objects
		MFnDagNode dagNode( dagPath, &stat );
		if (dagNode.isIntermediateObject())
		{
			continue;
		}
		if(!dagPath.hasFn(MFn::kDependencyNode))
		{
			continue;
		}
		MObject mObj = dagNode.object(&stat);
		if(stat != MS::kSuccess)
		{
			std::cout << "Failure in getting MObject" << std::endl;
   			return;
		}
        MFnDependencyNode fnNode(mObj, &stat);
		if(stat != MS::kSuccess)
		{
			std::cout << "Failure in getting dependency node" << std::endl;
   			return;
		}
        
        if(fnNode.typeId() == rigidBodyNode::typeId)
		{
			MPlug plgCollisionShape(mObj, rigidBodyNode::ia_collisionShape);
			MObject update;
			//force evaluation of the shape
			plgCollisionShape.getValue(update);
			cout<<"plgCollisionShape : "<<dagNode.userNode()->name().asChar()<<endl;
			if(plgCollisionShape.isConnected())
			{
                rbds.append(dagNode.userNode()->name());
				//rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(dagNode.userNode());
				//rbNode->update();
			}
		}
        
        /*
        
		if(fnNode.typeId() == SoftBodyNode::typeId)
		{
			MPlug plgMeshShape(mObj, SoftBodyNode::inputMesh);
			MObject update;
			plgMeshShape.getValue(update);
			if(plgMeshShape.isConnected())
			{
				SoftBodyNode *sbNode = static_cast<SoftBodyNode*>(dagNode.userNode());
				sbNode->update();
			}
		}
        if(fnNode.typeId() == nailConstraintNode::typeId)
		{
			MPlug plgRbA(mObj, nailConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, nailConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				nailConstraintNode *ncNode = static_cast<nailConstraintNode*>(dagNode.userNode());
				ncNode->update();
			}
		}
        if(fnNode.typeId() == hingeConstraintNode::typeId)
		{
			MPlug plgRbA(mObj, hingeConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, hingeConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				hingeConstraintNode *hcNode = static_cast<hingeConstraintNode*>(dagNode.userNode());
				hcNode->update();
			}
		}
        if(fnNode.typeId() == sliderConstraintNode::typeId)
		{
			MPlug plgRbA(mObj, hingeConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, hingeConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				sliderConstraintNode *scNode = static_cast<sliderConstraintNode*>(dagNode.userNode());
				scNode->update();
			}
		}
        if(fnNode.typeId() == sixdofConstraintNode::typeId)
		{
			MPlug plgRbA(mObj, hingeConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, hingeConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				sixdofConstraintNode *sdNode = static_cast<sixdofConstraintNode*>(dagNode.userNode());
				sdNode->update();
			}
		}*/
	}
}

void dSolverNode::runCallBacks(MObjectArray callBackNodes) {

    for(int i=0; i<callBackNodes.length(); ++i) {
        MFnDependencyNode fNode(callBackNodes[i]);
        MObject attribute = fNode.attribute("callbackscript");
        MPlug plug(callBackNodes[i], attribute);
        MString callBackString;
        plug.getValue(callBackString);
        //cout<<"Node "<<fNode.name()<<" has callback string: "<<callBackString<<endl;
        MGlobal::executeCommand(callBackString);
    }
}


void dSolverNode::traverseCallBacks() {
    MItDependencyNodes it(MFn::kInvalid);

    fScallBackNodes.clear();
    fEcallBackNodes.clear();
    sScallBackNodes.clear();
    sEcallBackNodes.clear();
    
    // keep looping until done
    while(!it.isDone())
    {
        
        // get a handle to this node
        MObject obj = it.item();
        
        // write the node type found
        //cout<<obj.apiTypeStr()<<endl;
        if ( (obj.apiType() == MFn::kPluginTransformNode )) {
            MFnDependencyNode fNode(obj);
            if (fNode.typeName() == "dCallBack") {
                MPlug eplug(obj, fNode.attribute("Enable") );
                bool enable;
                eplug.getValue(enable);
                MPlug tplug(obj, fNode.attribute("callbacktype") );
                int callbacktype;
                tplug.getValue(callbacktype);
                //cout<<"callbacktype : "<<callbacktype<<endl;
                if (enable) {
                    switch (callbacktype) {
                        case dCallBackNode::FRAME_START:
                            fScallBackNodes.append(obj);
                            break;
                        case dCallBackNode::FRAME_END:
                            fEcallBackNodes.append(obj);
                            break;
                        case dCallBackNode::SUBSTEP_START:
                            sScallBackNodes.append(obj);
                            break;
                        case dCallBackNode::SUBSTEP_END:
                            sEcallBackNodes.append(obj);
                            break;
                        default:
                            break;
                    }
                }
            }
            
        }
        // move on to next node
        it.next();
    }
    /*
    cout<<"fScallBackNodes.length() : "<<fScallBackNodes.length()<<endl;
    cout<<"fEcallBackNodes.length() : "<<fEcallBackNodes.length()<<endl;
    cout<<"sScallBackNodes.length() : "<<sScallBackNodes.length()<<endl;
    cout<<"sEcallBackNodes.length() : "<<sEcallBackNodes.length()<<endl;
    */
    
}

void	dSolverNode::draw(	M3dView & view, const MDagPath & path,
							M3dView::DisplayStyle style,
							M3dView::DisplayStatus status )
{
    view.beginGL();
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    glDisable(GL_LIGHTING);

    if( !(status == M3dView::kActive ||
        status == M3dView::kLead ||
        status == M3dView::kHilite ||
        ( style != M3dView::kGouraudShaded && style != M3dView::kFlatShaded )) ) {
        glColor3f(1.0, 1.0, 0.0); 
    }
/*
    glBegin(GL_LINES);

    glColor3f(1.0, 0.5, 0.5);
    glVertex3f(0.0, 0.0, 0.0);
    glColor3f(0.5, 0.5, 1.0);
    glVertex3f(1.f, 1.f, 1.f);

	glEnd();
*/
	MObject thisObject = thisMObject();
    MFnDagNode fnDagNode(thisObject);
    MFnTransform fnParentTransform(fnDagNode.parent(0));
	fnParentTransform.setTranslation(MVector(0.f, 0.f, 0.f), MSpace::kObject); // lock translation
	fnParentTransform.setRotation(MEulerRotation(0., 0., 0.)); // lock rotation
	double fixScale[3] = { 1., 1., 1. };  // lock scale
	fnParentTransform.setScale(fixScale);


	int dbgMode = 0;
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DrawWireframe, 0); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DrawAabb, 1); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DrawFeaturesText, 2); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DrawContactPoints, 3); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_NoDeactivation, 4); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_NoHelpText, 5); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DrawText, 6); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_ProfileTimings, 7); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_EnableSatComparison, 8); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DisableBulletLCP, 9); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_EnableCCD, 10); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DrawConstraints, 11); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_DrawConstraintLimits, 12); 
	dbgMode |= getDbgDrawVal(thisObject,ia_DBG_FastWireframe, 13); 
	solver_t::debug_draw(dbgMode);

    glPopAttrib();
    view.endGL();

}


static void sceneLoadedCB(void* clientData)
{
	dSolverNode::updateAllRigidBodies();
}

#if 0
static void connCB( MPlug & srcPlug, MPlug & destPlug, bool made, void* clientData)
{
	static int numConn = 0;
	MObject objSrc = srcPlug.node();
	MObject objDst = destPlug.node();
	MFnDependencyNode fnNodeSrc(objSrc);
	MFnDependencyNode fnNodeDst(objDst);
	if(fnNodeSrc.typeId() == collisionShapeNode::typeId)
	{
		if(fnNodeDst.typeId() == rigidBodyNode::typeId)
		{
			numConn++;
		}
	}
	if(fnNodeDst.typeId() == collisionShapeNode::typeId)
	{
		if(fnNodeSrc.typeId() == rigidBodyNode::typeId)
		{
			numConn++;
		}
	}
}
#endif

static void updateSceneCB(MTime & time, void* clientData)
{
	dSolverNode::updateAllRigidBodies();
}


void dSolverNode::updateAllRigidBodies()
{
	MStatus stat;
//	MItDag dagIterator( MItDag::kBreadthFirst, MFn::kInvalid, &stat);
	MItDag dagIterator( MItDag::kDepthFirst, MFn::kInvalid, &stat);
   	if (stat != MS::kSuccess)
	{
        std::cout << "Failure in DAG iterator setup" << std::endl;
   	    return;
   	}
	for ( ;!dagIterator.isDone(); dagIterator.next()) 
	{
   	    MDagPath dagPath;
   	    stat = dagIterator.getPath( dagPath );
   		if(stat != MS::kSuccess)
		{
			std::cout << "Failure in getting DAG path" << std::endl;
   			return;
		}
		// skip over intermediate objects
		MFnDagNode dagNode( dagPath, &stat );
		if (dagNode.isIntermediateObject()) 
		{
			continue;
		}
		if(!dagPath.hasFn(MFn::kDependencyNode))
		{
			continue;
		}
		MObject mObj = dagNode.object(&stat);
		if(stat != MS::kSuccess)
		{
			std::cout << "Failure in getting MObject" << std::endl;
   			return;
		}
        MFnDependencyNode fnNode(mObj, &stat);
		if(stat != MS::kSuccess)
		{
			std::cout << "Failure in getting dependency node" << std::endl;
   			return;
		}

        if(fnNode.typeId() == rigidBodyNode::typeId) 
		{
			MPlug plgCollisionShape(mObj, rigidBodyNode::ia_collisionShape);
			MObject update;
			//force evaluation of the shape
			plgCollisionShape.getValue(update);
			if(plgCollisionShape.isConnected()) 
			{
				rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(dagNode.userNode()); 
				rbNode->update();
			}
		}

		if(fnNode.typeId() == SoftBodyNode::typeId)
		{
			MPlug plgMeshShape(mObj, SoftBodyNode::inputMesh);
			MObject update;
			plgMeshShape.getValue(update);
			if(plgMeshShape.isConnected())
			{
				SoftBodyNode *sbNode = static_cast<SoftBodyNode*>(dagNode.userNode());
				sbNode->update();
			}
		}
        if(fnNode.typeId() == nailConstraintNode::typeId) 
		{
			MPlug plgRbA(mObj, nailConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, nailConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				nailConstraintNode *ncNode = static_cast<nailConstraintNode*>(dagNode.userNode()); 
				ncNode->update();
			}
		}
        if(fnNode.typeId() == hingeConstraintNode::typeId) 
		{
			MPlug plgRbA(mObj, hingeConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, hingeConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				hingeConstraintNode *hcNode = static_cast<hingeConstraintNode*>(dagNode.userNode()); 
				hcNode->update();
			}
		}
        if(fnNode.typeId() == sliderConstraintNode::typeId) 
		{
			MPlug plgRbA(mObj, hingeConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, hingeConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				sliderConstraintNode *scNode = static_cast<sliderConstraintNode*>(dagNode.userNode()); 
				scNode->update();
			}
		}
        if(fnNode.typeId() == sixdofConstraintNode::typeId) 
		{
			MPlug plgRbA(mObj, hingeConstraintNode::ia_rigidBodyA);
			MPlug plgRbB(mObj, hingeConstraintNode::ia_rigidBodyB);
			MObject update;
			//force evaluation
			plgRbA.getValue(update);
			plgRbB.getValue(update);
			bool connA = plgRbA.isConnected();
			bool connB = plgRbB.isConnected();
			if(connA || connB)
			{
				sixdofConstraintNode *sdNode = static_cast<sixdofConstraintNode*>(dagNode.userNode()); 
				sdNode->update();
			}
		}
	}
}



MStatus dSolverNode::initialize()
{ 
    MStatus                 status;
    MFnEnumAttribute        fnEnumAttr;
    MFnMessageAttribute     fnMsgAttr;
    MFnUnitAttribute        fnUnitAttr;
    MFnNumericAttribute     fnNumericAttr;

	MCallbackId updateSceneCBId = MDGMessage::addForceUpdateCallback(updateSceneCB, NULL, NULL ); 
	MCallbackId sceneLoadedCBId = MSceneMessage::addCallback(MSceneMessage::kAfterOpen, sceneLoadedCB, NULL, NULL);
//	MCallbackId connCBId = MDGMessage::addConnectionCallback(connCB, NULL, NULL ); 

    //
    //ssSolverType = fnEnumAttr.create( "ssSolverType", "ssst", 0, &status );
    //MCHECKSTATUS(status, "creating ssSolverType attribute")
    //fnEnumAttr.addField( "Bullet Physics", 0 );
    //fnEnumAttr.addField( "Ageia PhysX", 1 );
    //fnEnumAttr.addField( "Stanford PhysBAM", 2 );
    //status = addAttribute(ssSolverType);
    //MCHECKSTATUS(status, "adding ssSolverType attribute")

    //
    ia_time = fnUnitAttr.create( "inTime", "it", MFnUnitAttribute::kTime, 0.0, &status );
    MCHECKSTATUS(status, "creating ia_time attribute")
    fnUnitAttr.setHidden(true);
    status = addAttribute(ia_time);
    MCHECKSTATUS(status, "adding ia_time attribute")

    ia_startTime = fnUnitAttr.create( "startTime", "stm", MFnUnitAttribute::kTime, 1.0, &status );
    MCHECKSTATUS(status, "creating ia_startTime attribute")
    status = addAttribute(ia_startTime);
    MCHECKSTATUS(status, "adding ia_startTime attribute")

    oa_rigidBodies = fnMsgAttr.create("rigidBodies", "rbds", &status);
    MCHECKSTATUS(status, "creating oa_rigidBodies attribute")
    status = addAttribute(oa_rigidBodies);
    MCHECKSTATUS(status, "adding oa_rigidBodies attribute")

	oa_softBodies = fnMsgAttr.create("softBodies", "sbds", &status);
    MCHECKSTATUS(status, "creating oa_softBodies attribute")
    status = addAttribute(oa_softBodies);
    MCHECKSTATUS(status, "adding oa_softBodies attribute")

    ia_gravity = fnNumericAttr.createPoint("gravity", "grvt", &status);
    MCHECKSTATUS(status, "creating gravity attribute")
    fnNumericAttr.setDefault(0.0, -9.81, 0.0);
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_gravity);
    MCHECKSTATUS(status, "adding ia_gravity attribute")

    ia_substeps = fnNumericAttr.create("substeps", "sbs", MFnNumericData::kInt, 1, &status);
    MCHECKSTATUS(status, "creating substeps attribute")
    fnNumericAttr.setKeyable(true);
	fnNumericAttr.setMin(0);
	fnNumericAttr.setMax(100);
    status = addAttribute(ia_substeps);
    MCHECKSTATUS(status, "adding ia_substeps attribute")

	ia_fixedPhysicsRate = fnNumericAttr.create("physicsrate", "fps", MFnNumericData::kInt, 240, &status); //MB
    MCHECKSTATUS(status, "creating physicsrate attribute")
    fnNumericAttr.setKeyable(true);
	fnNumericAttr.setMin(60);
	fnNumericAttr.setMax(6000);
    status = addAttribute(ia_fixedPhysicsRate);
    MCHECKSTATUS(status, "adding physicsrate attribute")

	ia_disableCollisionsBetweenLinkedBodies = fnNumericAttr.create("disableCollisionsBetweenLinkedBodies", "dCol", MFnNumericData::kBoolean, true, &status);
    MCHECKSTATUS(status, "creating disableCollisionsBetweenLinkedBodies attribute")
    status = addAttribute(ia_disableCollisionsBetweenLinkedBodies);
    MCHECKSTATUS(status, "adding disableCollisionsBetweenLinkedBodies attribute")

    ia_enabled = fnNumericAttr.create("enabled", "enbl", MFnNumericData::kBoolean, true, &status);
    MCHECKSTATUS(status, "creating enabled attribute")
    status = addAttribute(ia_enabled);
    MCHECKSTATUS(status, "adding ia_enabled attribute")

	ia_collisionMargin = fnNumericAttr.create("collisionMargin", "colm", MFnNumericData::kFloat, 0.04, &status);
    MCHECKSTATUS(status, "creating collisionMargin attribute")
	//fnNumericAttr.setHidden(true);
	fnNumericAttr.setMin(0);
	fnNumericAttr.setMax(10);
    status = addAttribute(ia_collisionMargin);
    MCHECKSTATUS(status, "adding ia_collisionMargin attribute")


    ia_splitImpulse = fnNumericAttr.create("splitImpulse", "spli", MFnNumericData::kBoolean,true, &status);
    MCHECKSTATUS(status, "creating splitImpulse attribute")
    status = addAttribute(ia_splitImpulse);
    MCHECKSTATUS(status, "adding ia_splitImpulse attribute")

	ia_DBG_DrawWireframe = fnNumericAttr.create("drawWireframe", "dwfr", MFnNumericData::kBoolean, true, &status);
    MCHECKSTATUS(status, "creating ia_DBG_DrawWireframe attribute")
    status = addAttribute(ia_DBG_DrawWireframe);
    MCHECKSTATUS(status, "adding ia_DBG_DrawWireframe attribute")
	ia_DBG_DrawAabb = fnNumericAttr.create("drawAabb", "daabb", MFnNumericData::kBoolean, false, &status);
	MCHECKSTATUS(status, "creating ia_DBG_DrawAabb attribute")
	status = addAttribute(ia_DBG_DrawAabb);
    MCHECKSTATUS(status, "adding ia_DBG_DrawAabb attribute")
//	ia_DBG_DrawFeaturesText = fnNumericAttr.create("drawFeaturesText", "dft", MFnNumericData::kBoolean, false, &status);
//	MCHECKSTATUS(status, "creating ia_DBG_DrawFeaturesText attribute")
//	status = addAttribute(ia_DBG_DrawFeaturesText);
//    MCHECKSTATUS(status, "adding ia_DBG_DrawFeaturesText attribute")
	ia_DBG_DrawContactPoints = fnNumericAttr.create("drawContactPoints", "dcp", MFnNumericData::kBoolean, false, &status);
	MCHECKSTATUS(status, "creating ia_DBG_DrawContactPoints attribute")
	status = addAttribute(ia_DBG_DrawContactPoints);
    MCHECKSTATUS(status, "adding ia_DBG_DrawContactPoints attribute")
	ia_DBG_NoDeactivation = fnNumericAttr.create("noDeactivation", "dnda", MFnNumericData::kBoolean, false, &status);
	MCHECKSTATUS(status, "creating ia_DBG_NoDeactivation attribute")
	status = addAttribute(ia_DBG_NoDeactivation);
    MCHECKSTATUS(status, "adding ia_DBG_NoDeactivation attribute")
//	ia_DBG_NoHelpText = fnNumericAttr.create("noHelpText", "dnht", MFnNumericData::kBoolean, false, &status);
//	MCHECKSTATUS(status, "creating ia_DBG_NoHelpText attribute")
//	status = addAttribute(ia_DBG_NoHelpText);
 //   MCHECKSTATUS(status, "adding ia_DBG_NoHelpText attribute")
//	ia_DBG_DrawText = fnNumericAttr.create("drawText", "dtxt", MFnNumericData::kBoolean, false, &status);
//	MCHECKSTATUS(status, "creating ia_DBG_DrawText attribute")
//	status = addAttribute(ia_DBG_DrawText);
 //   MCHECKSTATUS(status, "adding ia_DBG_DrawText attribute")
	ia_DBG_ProfileTimings = fnNumericAttr.create("pProfileTimings", "dptm", MFnNumericData::kBoolean, false, &status);
	MCHECKSTATUS(status, "creating ia_DBG_ProfileTimings attribute")
	status = addAttribute(ia_DBG_ProfileTimings);
    MCHECKSTATUS(status, "adding ia_DBG_ProfileTimings attribute")
//	ia_DBG_EnableSatComparison = fnNumericAttr.create("enableSatComparison", "desc", MFnNumericData::kBoolean, false, &status);
//	MCHECKSTATUS(status, "creating ia_DBG_EnableSatComparison attribute")
//	status = addAttribute(ia_DBG_EnableSatComparison);
//    MCHECKSTATUS(status, "adding ia_DBG_EnableSatComparison attribute")
//	ia_DBG_DisableBulletLCP = fnNumericAttr.create("disableBulletLCP", "dblcp", MFnNumericData::kBoolean, false, &status);
	//MCHECKSTATUS(status, "creating ia_DBG_DisableBulletLCP attribute")
//	status = addAttribute(ia_DBG_DisableBulletLCP);
 //   MCHECKSTATUS(status, "adding ia_DBG_DisableBulletLCP attribute")
//	ia_DBG_EnableCCD = fnNumericAttr.create("enableCCD", "deccd", MFnNumericData::kBoolean, false, &status);
//	MCHECKSTATUS(status, "creating ia_DBG_EnableCCD attribute")
//	status = addAttribute(ia_DBG_EnableCCD);
 //   MCHECKSTATUS(status, "adding ia_DBG_EnableCCD attribute")
	ia_DBG_DrawConstraints = fnNumericAttr.create("drawConstraints", "dcnst", MFnNumericData::kBoolean, false, &status);
	MCHECKSTATUS(status, "creating ia_DBG_DrawConstraints attribute")
	status = addAttribute(ia_DBG_DrawConstraints);
    MCHECKSTATUS(status, "adding ia_DBG_DrawConstraints attribute")
	ia_DBG_DrawConstraintLimits = fnNumericAttr.create("drawConstraintLimits", "dcsl", MFnNumericData::kBoolean, false, &status);
	MCHECKSTATUS(status, "creating ia_DBG_DrawConstraintLimits attribute")
	status = addAttribute(ia_DBG_DrawConstraintLimits);
    MCHECKSTATUS(status, "adding ia_DBG_DrawConstraintLimits attribute")
//	ia_DBG_FastWireframe = fnNumericAttr.create("fastWireframe", "dfwf", MFnNumericData::kBoolean, false, &status);
//	MCHECKSTATUS(status, "creating ia_DBG_FastWireframe attribute")
//	status = addAttribute(ia_DBG_FastWireframe);
 //   MCHECKSTATUS(status, "adding ia_DBG_FastWireframe attribute")

    status = attributeAffects(ia_time, oa_rigidBodies);
    MCHECKSTATUS(status, "adding attributeAffects(ia_time, oa_rigidBodies)")
	
	status = attributeAffects(ia_time, oa_softBodies);
    MCHECKSTATUS(status, "adding attributeAffects(ia_time, oa_softBodies)")
    status = attributeAffects(ia_enabled, oa_rigidBodies);
    MCHECKSTATUS(status, "adding attributeAffects(ia_enabled, oa_rigidBodies)")

	ia_contactData = fnNumericAttr.create("contactData", "contactData", MFnNumericData::kBoolean, false, &status);
    MCHECKSTATUS(status, "creating contactData attribute")
    status = addAttribute(ia_contactData);
    MCHECKSTATUS(status, "adding ia_contactData attribute")

    return MS::kSuccess;
}

//only ever create a single dSolverNode at one time

dSolverNode::dSolverNode()
{
		m_reInitialize = false;
}



dSolverNode::~dSolverNode()
{
}

void dSolverNode::postConstructor()
{
    //prevent deletion if all the rigidbodies are deleted
    setExistWithoutOutConnections(true);
}


void* dSolverNode::creator()
{

	return new dSolverNode();
}

bool dSolverNode::setInternalValueInContext( const  MPlug & plug, const  MDataHandle & dataHandle,  MDGContext & ctx )
{
    return false;
}


MStatus dSolverNode::compute(const MPlug& plug, MDataBlock& data)
{
//  std::cout << "Calling dSolverNode::compute \n";
//	std::cout << "Plug: " << plug.name().asChar() << std::endl;

	if(plug == oa_rigidBodies) {
         computeRigidBodies(plug, data);
    } else {
        return MStatus::kUnknownParameter;
    } 

    return MStatus::kSuccess;
}

void destroySoftBody(const MPlug& plug, MObject& node, MDataBlock& data)
{
	MFnDagNode fnDagNode(node);
	SoftBodyNode *sbNode = static_cast<SoftBodyNode *>(fnDagNode.userNode()); 
  	sbNode->destroySoftBody();
}

void destroyRigidBody(const MPlug& plug, MObject& node, MDataBlock& data)
{
	MFnDagNode fnDagNode(node);
	rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
  	rbNode->destroyRigidBody();
}

void dSolverNode::initRigidBody(const MPlug& plug, MObject& node, MDataBlock& data)
{
    //dSolverNode::collisionMarginOffset = data.inputValue(dSolverNode::ia_collisionMargin).asFloat();
	
	MFnDagNode fnDagNode(node);

    rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
    
	
	if (m_reInitialize)
		rbNode->computeRigidBody(plug,data);
	
	
	rigid_body_t::pointer rb = rbNode->rigid_body();


    if(fnDagNode.parentCount() == 0) {
        std::cout << "No transform found!" << std::endl;
        return;
    }

    MFnTransform fnTransform(fnDagNode.parent(0));

    MPlug plgMass(node, rigidBodyNode::ia_mass);
    float mass = 0.f;
    plgMass.getValue(mass);
    if(mass > 0.f) 
	{
        //active rigid body, set the world transform from the initial* attributes
        MObject obj;

        vec3f pos;
        MPlug plgInitialValue(node, rigidBodyNode::ia_initialPosition);
        plgInitialValue.child(0).getValue(pos[0]);
        plgInitialValue.child(1).getValue(pos[1]);
        plgInitialValue.child(2).getValue(pos[2]);

        vec3f rot;
        plgInitialValue.setAttribute(rigidBodyNode::ia_initialRotation);
        plgInitialValue.child(0).getValue(rot[0]);
        plgInitialValue.child(1).getValue(rot[1]);
        plgInitialValue.child(2).getValue(rot[2]);

        vec3f vel;
        plgInitialValue.setAttribute(rigidBodyNode::ia_initialVelocity);
        plgInitialValue.child(0).getValue(vel[0]);
        plgInitialValue.child(1).getValue(vel[1]);
        plgInitialValue.child(2).getValue(vel[2]);
        
        plgInitialValue.setAttribute(rigidBodyNode::ia_velocity);
        plgInitialValue.child(0).setValue(vel[0]);
        plgInitialValue.child(1).setValue(vel[1]);
        plgInitialValue.child(2).setValue(vel[2]);

        vec3f spin;
        plgInitialValue.setAttribute(rigidBodyNode::ia_initialSpin);
        plgInitialValue.child(0).getValue(spin[0]);
        plgInitialValue.child(1).getValue(spin[1]);
        plgInitialValue.child(2).getValue(spin[2]);

        MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
        MQuaternion mquat = meuler.asQuaternion();
        rb->set_transform(pos, quatf((float)mquat.w, (float)mquat.x, (float)mquat.y, (float)mquat.z)); 
        rb->set_linear_velocity(vel);
        rb->set_angular_velocity(spin);
        rb->set_kinematic(false);

		//rbNode->updateShape(plug, data, dSolverNode::collisionMarginOffset); //mb

        fnTransform.setRotation(meuler);
        fnTransform.setTranslation(MVector(pos[0], pos[1], pos[2]), MSpace::kTransform);
    } else {
        //passive rigid body, get the world trasform from from the shape node
        MQuaternion mquat;
        fnTransform.getRotation(mquat);
        MVector mpos(fnTransform.getTranslation(MSpace::kTransform));
        rb->set_transform(vec3f((float)mpos.x, (float)mpos.y, (float)mpos.z), quatf((float)mquat.w, (float)mquat.x, (float)mquat.y, (float)mquat.z));
        rb->set_interpolation_transform(vec3f((float)mpos.x, (float)mpos.y, (float)mpos.z), quatf((float)mquat.w, (float)mquat.x, (float)mquat.y, (float)mquat.z));
        rb->set_kinematic(true);
    }
}


void dSolverNode::initSoftBody(const MPlug& plug, MObject& node, MDataBlock& data)
{
	// std::cout << "(dSolverNode::initSoftBody) | " << "plug: " << plug.name().asChar() << std::endl;
	
	MFnDagNode fnDagNode(node);

    SoftBodyNode *sbNode = static_cast<SoftBodyNode*>(fnDagNode.userNode());
	MObject thisObject(thisMObject());

	if (m_reInitialize)
		sbNode->computeSoftBody(plug,data);

	/*
	MObject temp;
	
	MPlug plgSoft(node, SoftBodyNode::ca_softBody);
	plgSoft.setValue(false);
	plgSoft.getValue(temp);
	*/
	MPlug plgInputMesh(node, SoftBodyNode::inputMesh);
    MObject upd;
    //force evaluation of the shape
    plgInputMesh.getValue(upd);
	
	btAssert(plgInputMesh.isConnected());
	MPlugArray connections;
	plgInputMesh.connectedTo(connections, true, false);
	
	// MFnDependencyNode fnNode(connections[0].node());
	btAssert( connections.length() != 0);

	// std::cout << "(SoftBodyNode::computeSoftBody) Dependency node fn name: | " << fnNode.name() << std::endl;
	
	MFnMesh meshFn(connections[0].node());
	
	soft_body_t::pointer sb = sbNode->soft_body();
	if(sb)
	{
		solver_t::remove_soft_body(sb);
	}
	std::vector<int> triVertIndices;
	std::vector<float> triVertCoords;
	
	SoftBodyNode::createHelperMesh(meshFn, triVertIndices, triVertCoords, MSpace::kTransform);
	// this->m_soft_body = solver_t::create_soft_body(triVertCoords, triVertIndices);	
	sb = solver_t::create_soft_body(triVertCoords, triVertIndices);	

	solver_t::add_soft_body(sb, this->name().asChar());
	sbNode->set_soft_body(sb);

	// get transforms and apply
	MFnTransform fnTransform(fnDagNode.parent(0));
	MVector mtranslation = fnTransform.getTranslation(MSpace::kTransform);
    MQuaternion mrotation;
    fnTransform.getRotation(mrotation, MSpace::kObject);
	
	//sb->set_transform(vec3f((float)mtranslation.x, (float)mtranslation.y, (float)mtranslation.z),
	//							quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));


	MPlug plgOutputMesh(node, SoftBodyNode::outputMesh);
	MObject update;
		
	//force evaluation of the shape
	plgOutputMesh.getValue(update);
	
	btAssert(plgOutputMesh.isConnected());
	
	plgOutputMesh.connectedTo(connections, false, true);
							
	MFnMesh meshFnOut(connections[0].node());
	MFloatPointArray newPoints;
			
	for(int j = 0;j < triVertCoords.size() / 3; j ++)
	{
		newPoints.append(triVertCoords[j*3], triVertCoords[j*3 + 1], triVertCoords[j*3 + 2], 1.0f);
	}

	MStatus setPtStat = meshFnOut.setPoints(newPoints, MSpace::kTransform);
	btAssert(setPtStat == MStatus::kSuccess);

}

void destroyConstraint(const MPlug& plug, MObject& bodyNode, MDataBlock& data) 
{
	MFnDagNode fnDagNode(bodyNode);
    rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
	MPlug plgMessages(bodyNode, rbNode->message);
	MPlugArray rbMsgConnections;
	plgMessages.connectedTo(rbMsgConnections, false, true);
	for(size_t j = 0; j < rbMsgConnections.length(); j++) 
	{
		MObject msgNode = rbMsgConnections[j].node();
		MFnDagNode msgDagNode(msgNode);
		if(msgDagNode.typeId() == nailConstraintNode::typeId) 
		{
			nailConstraintNode* nailNode = static_cast<nailConstraintNode*>(msgDagNode.userNode());
			nailNode->destroyConstraint();
		}
		
		if(msgDagNode.typeId() == hingeConstraintNode::typeId) 
		{
			hingeConstraintNode* hingeNode = static_cast<hingeConstraintNode*>(msgDagNode.userNode());
			hingeNode->destroyConstraint();
		}

		if(msgDagNode.typeId() == sliderConstraintNode::typeId) 
		{
			sliderConstraintNode* sliderNode = static_cast<sliderConstraintNode*>(msgDagNode.userNode());
			sliderNode->destroyConstraint();
		}

		if(msgDagNode.typeId() == sixdofConstraintNode::typeId) 
		{
			sixdofConstraintNode* sixdofNode = static_cast<sixdofConstraintNode*>(msgDagNode.userNode());
			sixdofNode->destroyConstraint();
		}
		

	}
}

void dSolverNode::initConstraint(const MPlug& plug, MObject& bodyNode, MDataBlock& data) 
{
    MFnDagNode fnDagNode(bodyNode);
    rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
	MPlug plgMessages(bodyNode, rbNode->message);
	MPlugArray rbMsgConnections;
	plgMessages.connectedTo(rbMsgConnections, false, true);
	for(size_t j = 0; j < rbMsgConnections.length(); j++) 
	{
		MObject msgNode = rbMsgConnections[j].node();
		MFnDagNode msgDagNode(msgNode);
		if(msgDagNode.typeId() == nailConstraintNode::typeId) 
		{
			nailConstraintNode* nailNode = static_cast<nailConstraintNode*>(msgDagNode.userNode());
	
			if (m_reInitialize)
				nailNode->reComputeConstraint(plug,data);
			

			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for nail constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			nail_constraint_t::pointer nail = nailNode->constraint();
			if (nail)
			{
				vec3f constrPos;
				nail->get_world(constrPos);
				nail->set_enabled(true); //re-enables constraint if broken dynamically
				msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
				msgTransform.setRotation(MEulerRotation(0., 0., 0.));
			}
		}
		if(msgDagNode.typeId() == hingeConstraintNode::typeId) 
		{
			hingeConstraintNode* hingeNode = static_cast<hingeConstraintNode*>(msgDagNode.userNode());
			if (m_reInitialize)
				hingeNode->reComputeConstraint(plug,data);

			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for hinge constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			hinge_constraint_t::pointer hinge = hingeNode->constraint();
			vec3f constrPos;
			quatf constrRot;
			hinge->get_world(constrPos, constrRot);
			hinge->set_enabled(true); //re-enables constraint if broken dynamically
			msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
            msgTransform.setRotation(MQuaternion(constrRot[1], constrRot[2], constrRot[3], constrRot[0]));
		}
		if(msgDagNode.typeId() == sliderConstraintNode::typeId) 
		{
			sliderConstraintNode* sliderNode = static_cast<sliderConstraintNode*>(msgDagNode.userNode());
					
			if (m_reInitialize)
				sliderNode->reComputeConstraint();

			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for slider constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			slider_constraint_t::pointer slider = sliderNode->constraint();
			vec3f constrPos;
			quatf constrRot;
			slider->get_world(constrPos, constrRot);
			slider->set_enabled(true); //re-enables constraint if broken dynamically
			msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
            msgTransform.setRotation(MQuaternion(constrRot[1], constrRot[2], constrRot[3], constrRot[0]));

		}
		if(msgDagNode.typeId() == sixdofConstraintNode::typeId) 
		{
			sixdofConstraintNode* sixdofNode = static_cast<sixdofConstraintNode*>(msgDagNode.userNode());
					
			if (m_reInitialize)
				sixdofNode->reComputeConstraint(plug,data);

			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for sixdof constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			sixdof_constraint_t::pointer sixdof = sixdofNode->constraint();
			vec3f constrPos;
			quatf constrRot;
			sixdof->get_world(constrPos, constrRot);
			sixdof->set_enabled(true); //re-enables constraint if broken dynamically
			msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
            msgTransform.setRotation(MQuaternion(constrRot[1], constrRot[2], constrRot[3], constrRot[0]));
		}
	}
}

void destroyRigidBodyArray(const MPlug& plug, MObject &node, MDataBlock& data)
{
	 MFnDagNode fnDagNode(node);

    rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
    std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();
	rbNode->destroyRigidBodies();
}

void dSolverNode::initRigidBodyArray(const MPlug& plug, MObject &node, MDataBlock& data)
{
    MFnDagNode fnDagNode(node);

    rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
    std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();
	 
	if (m_reInitialize)
		rbNode->reComputeRigidBodies(plug,data);
	


    if(fnDagNode.parentCount() == 0) {
        std::cout << "No transform found!" << std::endl;
        return;
    }

    MFnTransform fnTransform(fnDagNode.parent(0));

    MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
    float mass = 0.f;
    plgMass.getValue(mass);
	bool active = (mass>0.f);

    if(active) {
        //active rigid body, set the world transform from the initial* attributes
        MObject obj;

        MPlug plgInitialPosition(node, rigidBodyArrayNode::ia_initialPosition);
        MPlug plgInitialRotation(node, rigidBodyArrayNode::ia_initialRotation);
        MPlug plgInitialVelocity(node, rigidBodyArrayNode::ia_initialVelocity);
        MPlug plgInitialSpin(node, rigidBodyArrayNode::ia_initialSpin);

        MPlug plgPosition(node, rigidBodyArrayNode::io_position);
        MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

        MPlug plgElement;
        for(size_t j = 0; j < rbs.size(); ++j) {
            vec3f pos;
            plgElement = plgInitialPosition.elementByLogicalIndex(j);
            plgElement.child(0).getValue(pos[0]);
            plgElement.child(1).getValue(pos[1]);
            plgElement.child(2).getValue(pos[2]);

            vec3f rot;
            plgElement = plgInitialRotation.elementByLogicalIndex(j);
            plgElement.child(0).getValue(rot[0]);
            plgElement.child(1).getValue(rot[1]);
            plgElement.child(2).getValue(rot[2]);

            vec3f vel;
            plgElement = plgInitialVelocity.elementByLogicalIndex(j);
            plgElement.child(0).getValue(vel[0]);
            plgElement.child(1).getValue(vel[1]);
            plgElement.child(2).getValue(vel[2]);

            vec3f spin;
            plgElement = plgInitialSpin.elementByLogicalIndex(j);
            plgElement.child(0).getValue(spin[0]);
            plgElement.child(1).getValue(spin[1]);
            plgElement.child(2).getValue(spin[2]);

            MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
            MQuaternion mquat = meuler.asQuaternion();

            rbs[j]->set_transform(pos, quatf((float)mquat.w, (float)mquat.x, (float)mquat.y, (float)mquat.z)); 
            rbs[j]->set_linear_velocity(vel);
            rbs[j]->set_angular_velocity(spin);
            rbs[j]->set_kinematic(false);

            plgElement = plgPosition.elementByLogicalIndex(j);
            plgElement.child(0).setValue(pos[0]);
            plgElement.child(1).setValue(pos[1]);
            plgElement.child(2).setValue(pos[2]);

            plgElement = plgRotation.elementByLogicalIndex(j);
            plgElement.child(0).setValue(rot[0]);
            plgElement.child(1).setValue(rot[1]);
            plgElement.child(2).setValue(rot[2]);
        }

    } else {
        //passive rigid body, get the world trasform from from the position/rotation attributes
        MPlug plgPosition(node, rigidBodyArrayNode::io_position);
        MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

        MPlug plgElement;
        for(size_t j = 0; j < rbs.size(); ++j) {
            vec3f pos;
            plgElement = plgPosition.elementByLogicalIndex(j);
            plgElement.child(0).getValue(pos[0]);
            plgElement.child(1).getValue(pos[1]);
            plgElement.child(2).getValue(pos[2]);

            vec3f rot;
            plgElement = plgRotation.elementByLogicalIndex(j);
            plgElement.child(0).getValue(rot[0]);
            plgElement.child(1).getValue(rot[1]);
            plgElement.child(2).getValue(rot[2]);

            MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
            MQuaternion mquat = meuler.asQuaternion();
            rbs[j]->set_transform(pos, quatf((float)mquat.w, (float)mquat.x, (float)mquat.y, (float)mquat.z)); 
            rbs[j]->set_kinematic(false);
        }
    }
}

void dSolverNode::deleteRigidBodies(const MPlug& plug, MPlugArray &rbConnections, MDataBlock& data)
{
	//std::cout << "Deleting rigid bodies" << std::endl;

    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDependencyNode fnNode(node);

        if(fnNode.typeId() == rigidBodyNode::typeId) {
			destroyConstraint(plug,node,data);
			destroyRigidBody(plug, node, data);
        } else if(fnNode.typeId() == rigidBodyArrayNode::typeId) {
            destroyRigidBodyArray(plug,node,data);
        }
		else if(fnNode.typeId() == SoftBodyNode::typeId)
		{
			destroySoftBody(plug, node, data);
		}
    }


}


//init the rigid bodies to it's first frame configuration
void dSolverNode::initRigidBodies(const MPlug& plug, MPlugArray &rbConnections, MDataBlock& data)
{
	//std::cout << "Initializing rigid bodies" << std::endl;
	dSolverNode::collisionMarginOffset = data.inputValue(dSolverNode::ia_collisionMargin).asFloat(); //mb

    for(size_t i = 0; i < rbConnections.length(); ++i) 
	{
        MObject node = rbConnections[i].node();
        MFnDependencyNode fnNode(node);
        if(fnNode.typeId() == rigidBodyNode::typeId) {
            initRigidBody(plug, node, data);
        } else if(fnNode.typeId() == rigidBodyArrayNode::typeId) {
            initRigidBodyArray(plug,node,data);
        }
		else if(fnNode.typeId() == SoftBodyNode::typeId)
		{
			initSoftBody(plug, node, data);
		}
    }

	 for(size_t i = 0; i < rbConnections.length(); ++i) 
	 {
        MObject node = rbConnections[i].node();
        MFnDependencyNode fnNode(node);
		if(fnNode.typeId() == rigidBodyNode::typeId) 
		 {
		 	initConstraint(plug,node,data);
		 }
    }
    
    //<rp 2014>
    traverseCallBacks();
    //</rp 2014>
    
}

void dSolverNode::initSoftBodies(const MPlug& plug, MPlugArray &sbConnections, MDataBlock& data)
{
	// std::cout << " (dSolverNode::initSoftBodies)| Initializing soft bodies" << std::endl;
	
    for(size_t i = 0; i < sbConnections.length(); ++i) {
        MObject node = sbConnections[i].node();
        MFnDependencyNode fnNode(node);

       if(fnNode.typeId() == SoftBodyNode::typeId)
		{
			initSoftBody(plug, node, data);
		}
    }

}

//gather previous and current frame transformations for substep interpolation
void dSolverNode::gatherPassiveTransforms(MPlugArray &rbConnections, std::vector<xforms_t> &xforms)
{
    xforms.resize(0);
    xforms_t xform;


    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                continue;
            }

            MFnTransform fnTransform(fnDagNode.parent(0));

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
                MQuaternion mquat;
                fnTransform.getRotation(mquat);
                MVector mpos(fnTransform.getTranslation(MSpace::kTransform));
                rb->get_transform(xform.m_x0, xform.m_q0); 
                
                xform.m_x1 = vec3f((float)mpos.x, (float)mpos.y, (float)mpos.z);
                xform.m_q1 = quatf((float)mquat.w, (float)mquat.x, (float)mquat.y, (float)mquat.z); 
                xforms.push_back(xform);
            }
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();
        
            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                return;
            }

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
                MPlug plgPosition(node, rigidBodyArrayNode::io_position);
                MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

                MPlug plgElement;
                for(size_t j = 0; j < rbs.size(); ++j) {
                    rbs[j]->get_transform(xform.m_x0, xform.m_q0); 

                    plgElement = plgPosition.elementByLogicalIndex(j);
                    plgElement.child(0).getValue(xform.m_x1[0]);
                    plgElement.child(1).getValue(xform.m_x1[1]);
                    plgElement.child(2).getValue(xform.m_x1[2]);

                    vec3f rot;
                    plgElement = plgRotation.elementByLogicalIndex(j);
                    plgElement.child(0).getValue(rot[0]);
                    plgElement.child(1).getValue(rot[1]);
                    plgElement.child(2).getValue(rot[2]);

                    MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
                    MQuaternion mquat = meuler.asQuaternion();
                    xform.m_q1 = quatf((float)mquat.w, (float)mquat.x, (float)mquat.y, (float)mquat.z); 
                    xforms.push_back(xform);
                }
            }
        }
    }
}

//update the passive rigid bodies by interpolation of the previous and current frame
void dSolverNode::updatePassiveRigidBodies(MPlugArray &rbConnections, std::vector<xforms_t> &xforms, float t)
{
    size_t pb = 0;
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                continue;
            }

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
				/* Why do we need that?
				Static objects are animated in Maya
				So just set transform as is
                //do linear interpolation for now
                rb->set_transform(xforms[pb].m_x0 + t * (xforms[pb].m_x1 - xforms[pb].m_x0),
                                  normalize(xforms[pb].m_q0 + t * (xforms[pb].m_q1 - xforms[pb].m_q0))); 
				*/
                rb->set_transform(xforms[pb].m_x1, xforms[pb].m_q1); 
                ++pb;
            }
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();
        
            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                return;
            }

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
                for(size_t j = 0; j < rbs.size(); ++j) {
                    rbs[j]->set_transform(xforms[pb].m_x0 + t * (xforms[pb].m_x1 - xforms[pb].m_x0),
                                      normalize(xforms[pb].m_q0 + t * (xforms[pb].m_q1 - xforms[pb].m_q0))); 
                    ++pb;
                }
            }
        }
    }
}

void dSolverNode::updateConstraint(MObject& bodyNode) 
{
    MFnDagNode fnDagNode(bodyNode);
    rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
	MPlug plgMessages(bodyNode, rbNode->message);
	MPlugArray rbMsgConnections;
	plgMessages.connectedTo(rbMsgConnections, false, true);
	for(size_t j = 0; j < rbMsgConnections.length(); j++) 
	{
		MObject msgNode = rbMsgConnections[j].node();
		MFnDagNode msgDagNode(msgNode);
		if(msgDagNode.typeId() == nailConstraintNode::typeId) 
		{
			nailConstraintNode* nailNode = static_cast<nailConstraintNode*>(msgDagNode.userNode());
			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for nail constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			nail_constraint_t::pointer nail = nailNode->constraint();
			vec3f constrPos;
			nail->get_world(constrPos);
			msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
			msgTransform.setRotation(MEulerRotation(0., 0., 0.));
		}
		if(msgDagNode.typeId() == hingeConstraintNode::typeId) 
		{
			hingeConstraintNode* hingeNode = static_cast<hingeConstraintNode*>(msgDagNode.userNode());
			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for hinge constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			hinge_constraint_t::pointer hinge = hingeNode->constraint();
			vec3f constrPos;
			quatf constrRot;
			hinge->get_world(constrPos, constrRot);
			msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
            msgTransform.setRotation(MQuaternion(constrRot[1], constrRot[2], constrRot[3], constrRot[0]));
		}
		if(msgDagNode.typeId() == sliderConstraintNode::typeId) 
		{
			sliderConstraintNode* sliderNode = static_cast<sliderConstraintNode*>(msgDagNode.userNode());
			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for slider constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			slider_constraint_t::pointer slider = sliderNode->constraint();
			vec3f constrPos;
			quatf constrRot;
			slider->get_world(constrPos, constrRot);
			msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
            msgTransform.setRotation(MQuaternion(constrRot[1], constrRot[2], constrRot[3], constrRot[0]));
		}
		if(msgDagNode.typeId() == sixdofConstraintNode::typeId) 
		{
			sixdofConstraintNode* sixdofNode = static_cast<sixdofConstraintNode*>(msgDagNode.userNode());
			if(msgDagNode.parentCount() == 0) 
			{
				std::cout << "No transform for sixdof constraint found!" << std::endl;
				continue;
			}
			MFnTransform msgTransform(msgDagNode.parent(0));
			sixdof_constraint_t::pointer sixdof = sixdofNode->constraint();
			vec3f constrPos;
			quatf constrRot;
			sixdof->get_world(constrPos, constrRot);
			msgTransform.setTranslation(MVector(constrPos[0], constrPos[1], constrPos[2]), MSpace::kTransform);
            msgTransform.setRotation(MQuaternion(constrRot[1], constrRot[2], constrRot[3], constrRot[0]));
		}
	}
}


void dSolverNode::updateActiveSoftBodies(MPlugArray &sbConnections)
{	
	for(size_t i = 0; i < sbConnections.length(); i++) 
	{    
		MObject node = sbConnections[i].node();
        MFnDagNode fnDagNode(node);	
		if(fnDagNode.typeId() == SoftBodyNode::typeId)
		{
			SoftBodyNode *sbNode = static_cast<SoftBodyNode*>(fnDagNode.userNode());             
			soft_body_t::pointer sb = sbNode->soft_body();

			std::vector<int> triIndices;
			std::vector<vec3f> triCoords;			
			sb->get_mesh(triIndices, triCoords);
			
			MPlug plgOutputMesh(sbNode->thisMObject(), SoftBodyNode::outputMesh);
			MObject update;
		
			//force evaluation of the shape
			plgOutputMesh.getValue(update);
	
			// assert(plgOutputMesh.isConnected());
			MPlugArray connections;
			plgOutputMesh.connectedTo(connections, false, true);
				
			// assert( connections.length() != 0);
	
			MFnMesh meshFn(connections[0].node());
			MFloatPointArray newPoints;

//			std::cout << "(void dSolverNode::updateActiveSoftBodies) | mesh: ";
			
			for(int j = 0;j < triCoords.size(); j ++)
			{ 
				newPoints.append(triCoords[j][0], triCoords[j][1], triCoords[j][2], 1.0f);
			}
	
			MStatus setPtStat = meshFn.setPoints(newPoints, MSpace::kTransform);
			// assert(setPtStat == MStatus::kSuccess);
		}		
	}
}



//update the scene after a simulation step

void dSolverNode::updateActiveRigidBodies(MPlugArray &rbConnections)
{
   //update the active rigid bodies to the new configuration
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        
		MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        //std::cout << "(dSolverNode::updateActiveRigidBodies) | current connection: " <<  fnDagNode.name().asChar() << std::endl;
		if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();
			if (rb)
			{
				if(fnDagNode.parentCount() == 0) {
					std::cout << "No transform found!" << std::endl;
					continue;
				}

				MFnTransform fnTransform(fnDagNode.parent(0));

				MPlug plgMass(node, rigidBodyNode::ia_mass);
				float mass = 0.f;
				plgMass.getValue(mass);
				bool active = (mass>0.f);
				if(active) {
					quatf rot;
					vec3f pos;
                    vec3f vel, av;
                    MPlug plgVelocity(node, rigidBodyNode::ia_velocity);
                    rb->get_linear_velocity(vel);
                    plgVelocity.child(0).setValue((double) vel[0]);
                    plgVelocity.child(1).setValue((double) vel[1]);
                    plgVelocity.child(2).setValue((double) vel[2]);
                    MPlug plgAngularVelocity(node, rigidBodyNode::ia_angularvelocity);
                    rb->get_angular_velocity(av);
                    plgAngularVelocity.child(0).setValue((double) av[0]);
                    plgAngularVelocity.child(1).setValue((double) av[1]);
                    plgAngularVelocity.child(2).setValue((double) av[2]);
					rb->get_transform(pos, rot);
					fnTransform.setRotation(MQuaternion(rot[1], rot[2], rot[3], rot[0]));
					fnTransform.setTranslation(MVector(pos[0], pos[1], pos[2]), MSpace::kTransform);
				}
			}
			updateConstraint(node);
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            //write the position and rotations 
            if(active) {
                MPlug plgPosition(node, rigidBodyArrayNode::io_position);
                MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

                MPlug plgElement;
                for(size_t j = 0; j < rbs.size(); ++j) {
                    vec3f pos;
                    quatf rot;
                    rbs[j]->get_transform(pos, rot); 

                    MEulerRotation meuler(MQuaternion(rot[1], rot[2], rot[3], rot[0]).asEulerRotation());

                    plgElement = plgPosition.elementByLogicalIndex(j);
                    plgElement.child(0).setValue(pos[0]);
                    plgElement.child(1).setValue(pos[1]);
                    plgElement.child(2).setValue(pos[2]);

                    plgElement = plgRotation.elementByLogicalIndex(j);
                    plgElement.child(0).setValue(rad2deg(meuler.x));
                    plgElement.child(1).setValue(rad2deg(meuler.y));
                    plgElement.child(2).setValue(rad2deg(meuler.z));

                }
            }

            //check if we have to output the rigid bodies to a file
            MPlug plgFileIO(node, rigidBodyArrayNode::ia_fileIO);
            bool doIO;
            plgFileIO.getValue(doIO);
            if(doIO) {
                dumpRigidBodyArray(node);
            }
        }
    }
}
//apply fields in the scene from the rigid body
void dSolverNode::applyFields(MPlugArray &rbConnections, float dt)
{
    MVectorArray position;
    MVectorArray velocity;
    MDoubleArray mass;

    std::vector<rigid_body_t*> rigid_bodies;
	std::vector<soft_body_t::pointer> soft_bodies;
    //gather active rigid bodies
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(active) {
                rigid_bodies.push_back(rb.get());
            }
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
			plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(active) {
                for(size_t j = 0; j < rbs.size(); ++j) {
                    rigid_bodies.push_back(rbs[j].get());
                }
            }
		} else if(fnDagNode.typeId() == SoftBodyNode::typeId) {
			// soft body node
			SoftBodyNode *sbNode = static_cast<SoftBodyNode*>(fnDagNode.userNode());
			soft_bodies.push_back(sbNode->soft_body());	

		}
    }

    //clear forces and get the properties needed for field computation    
    for(size_t i = 0; i < rigid_bodies.size(); ++i) {
        if (rigid_bodies[i])
		{	
			rigid_bodies[i]->clear_forces();
			vec3f pos, vel;
			quatf rot;
			rigid_bodies[i]->get_transform(pos, rot); 
			rigid_bodies[i]->get_linear_velocity(vel);
			position.append(MVector(pos[0], pos[1], pos[2]));
			velocity.append(MVector(vel[0], vel[1], vel[2]));
			//TODO
			mass.append(1.0);
		}
        //
    }		   
	
	// arrays storing soft-body information for applying forces
	MVectorArray softPosition;
    MVectorArray softVelocity;
    MDoubleArray softMass;
	// now apply forces to soft bodies
	for(size_t i = 0; i < soft_bodies.size(); i++)
	{
		
		// get the central point of the soft body.  Note that
		// at the moment, forces are applied uniformly to all nodes
		// based on the value of the force at the centre, rather than
		// node by node
		if (soft_bodies[i])
		{
			vec3f pos = soft_bodies[i]->get_center();
			softPosition.append(MVector(pos[0], pos[1], pos[2]));		

			vec3f vel = soft_bodies[i]->get_average_velocity();
			softVelocity.append(MVector(vel[0], vel[1], vel[2]));
			softMass.append(soft_bodies[i]->get_total_mass());
		}
	}
	
	//apply the fields to the rigid and soft bodies
	MVectorArray force;
	MVectorArray softForce;
    for(MItDag it(MItDag::kDepthFirst, MFn::kField); !it.isDone(); it.next()) {
        MFnField fnField(it.item());
        fnField.getForceAtPoint(position, velocity, mass, force, dt);
        for(size_t i = 0; i < rigid_bodies.size(); ++i) {
			if ( rigid_bodies[i])
			{
				rigid_bodies[i]->apply_central_force(vec3f((float)force[i].x, (float)force[i].y, (float)force[i].z));
			}
        }
		fnField.getForceAtPoint(softPosition, softVelocity, softMass, softForce, dt);
		for(size_t i = 0; i < soft_bodies.size(); ++i) {
			if (soft_bodies[i])
			{
				soft_bodies[i]->apply_central_force(vec3f((float)softForce[i].x, (float)softForce[i].y, (float)softForce[i].z));
			}
        }
    }

	//apply external forces and external torques
    for(size_t i = 0; i < rbConnections.length(); ++i) 
	{
	
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) 
		{
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(active) 
			{
				{
					MPlug plgExternalForce(node, rigidBodyNode::ia_externalForce);
					float externalForce[3];
					plgExternalForce.child(0).getValue(externalForce[0]);
					plgExternalForce.child(1).getValue(externalForce[1]);
					plgExternalForce.child(2).getValue(externalForce[2]);
					rb->apply_central_force(vec3f(externalForce[0], externalForce[1], externalForce[2]));
				}
				{
					MPlug plgExternalTorque(node, rigidBodyNode::ia_externalTorque);
					float externalTorque[3];
					plgExternalTorque.child(0).getValue(externalTorque[0]);
					plgExternalTorque.child(1).getValue(externalTorque[1]);
					plgExternalTorque.child(2).getValue(externalTorque[2]);
					rb->apply_torque(vec3f(externalTorque[0], externalTorque[1], externalTorque[2]));
				}
				
            }
        }
	} 
}

rigidBodyNode* dSolverNode::getRigidBodyNode(btCollisionObject* btColObj)
{
	rigidBodyNode** nodePtr = m_hashColObjectToRBNode.find((const void*)btColObj);
	return *nodePtr;
}

/*
	Note : This is the big update function which steps the solver
*/


void dSolverNode::computeRigidBodies(const MPlug& plug, MDataBlock& data)
{
    bool enabled = data.inputValue(ia_enabled).asBool();
    if(!enabled) {
        data.outputValue(oa_rigidBodies).set(true);
        data.setClean(plug);
        return;
    }

    MTime time = data.inputValue(ia_time).asTime();
    MTime startTime = data.inputValue(ia_startTime).asTime();
    int subSteps = data.inputValue(ia_substeps).asInt();
	int fixedPhysicsFrameHertz = data.inputValue(ia_fixedPhysicsRate).asInt();
    MObject thisObject = thisMObject();
    MPlug plgRigidBodies(thisObject, oa_rigidBodies); 
	MPlugArray rbConnections;
    plgRigidBodies.connectedTo(rbConnections, true, true);
	

    MPlug plgSplitImpulse(thisObject, ia_splitImpulse);
    bool splitImpulseEnabled;
    plgSplitImpulse.getValue(splitImpulseEnabled);

    if(time == startTime) {
        //first frame, init the simulation	

		//need to re-construct the world and re-insert all objects...
		isStartTime = true;

		if (m_reInitialize)
		{
			deleteRigidBodies(plug, rbConnections, data);
			solver_t::destroyWorld();
			solver_t::createWorld();
		}
		
		initRigidBodies(plug, rbConnections, data);

		// update m_hashColObjectToRBNode
		m_hashColObjectToRBNode.clear();
        #pragma omp for
		for(size_t i = 0; i < rbConnections.length(); ++i) 
		{
			MObject node = rbConnections[i].node();
			MFnDagNode fnDagNode(node);
			
			if(fnDagNode.typeId() == rigidBodyNode::typeId) 
			{
				rigidBodyNode* rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
				rigid_body_t::pointer rb = rbNode->rigid_body();

				if (rb)
				{
					if(fnDagNode.parentCount() == 0) {
						//std::cout << "No transform found!" << std::endl;
						continue;
					}
				
					const void* rigidBody = ((bt_rigid_body_t*)(rb->impl()))->body();

					if ( !rigidBody )
						continue;

					m_hashColObjectToRBNode.insert(rigidBody, rbNode);
				}
			}
		}

		clearContactRelatedAttributes(rbConnections);

		m_reInitialize = true;
        solver_t::set_split_impulse(splitImpulseEnabled);
        m_prevTime = time;
    } else {
		// std::cout  << time.value() << std::endl;
		isStartTime = false;
        double delta_frames = (time - m_prevTime).value();
        bool playback = MConditionMessage::getConditionState("playingBack");
        if(time > m_prevTime && 
           ((delta_frames <= 1.0) || (playback && delta_frames < 100))) {
            if (fScallBackNodes.length())
                runCallBacks(fScallBackNodes);
            //step the simulation forward,
            //don't update if we are jumping more than one frame
    
            float dt = (float)(time - m_prevTime).as(MTime::kSeconds);
    
            //gather start and end transform for passive rigid bodies, used for interpolation
            std::vector<xforms_t> passiveXForms;
            gatherPassiveTransforms(rbConnections, passiveXForms);
    
    
            //set the gravity in the solver
            MPlug plgGravity(thisObject, ia_gravity);
            vec3f gravity;
            plgGravity.child(0).getValue(gravity[0]);
            plgGravity.child(1).getValue(gravity[1]);
            plgGravity.child(2).getValue(gravity[2]);
    
            solver_t::set_gravity(gravity);
    
            solver_t::set_split_impulse(splitImpulseEnabled);
            
            for(int i = 1; i <= subSteps; ++i) {
                if (sScallBackNodes.length())
                    runCallBacks(sScallBackNodes);
                //first update the passive rigid bodies
                updatePassiveRigidBodies(rbConnections, passiveXForms, i * dt / subSteps);
    
                //fields
                applyFields(rbConnections, dt / subSteps);
    
                //do callback
                
                
                //step the simulation
				MPlug plgContactData(thisObject, ia_contactData);
				bool bContactData;
				plgContactData.getValue(bContactData);

                solver_t::step_simulation(dt / subSteps, 1.f/(float)fixedPhysicsFrameHertz);
				
				// update contactCount, contactName and contactPosition attributes 
				if ( bContactData )
				{
					clearContactRelatedAttributes(rbConnections);

					shared_ptr<solver_impl_t> solv = solver_t::get_solver();
					btSoftRigidDynamicsWorld* dynamicsWorld = ((bt_solver_t*)solv.get())->dynamicsWorld();

					btCollisionWorld* pCollisionWorld = dynamicsWorld->getCollisionWorld();
					int numManifolds = pCollisionWorld->getDispatcher()->getNumManifolds();
                    #pragma omp for
					for ( int i=0;i<numManifolds;i++)
					{
						btPersistentManifold* contactManifold = pCollisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
						btCollisionObject* obA = (btCollisionObject*)(contactManifold->getBody0());
						btCollisionObject* obB = (btCollisionObject*)(contactManifold->getBody1());

						int numContacts = contactManifold->getNumContacts();

						rigidBodyNode* rbNodeA = getRigidBodyNode(obA);
						rigidBodyNode* rbNodeB = getRigidBodyNode(obB);
												
						for (int j=0;j<numContacts;j++)
						{
							btManifoldPoint& pt = contactManifold->getContactPoint(j);

							btVector3 ptA = pt.getPositionWorldOnA();
							btVector3 ptB = pt.getPositionWorldOnB();

							if ( rbNodeA && rbNodeB )
							{
								rbNodeA->addContactInfo(rbNodeB->name(), MVector(ptA.getX(), ptA.getY(), ptA.getZ()));
								rbNodeB->addContactInfo(rbNodeA->name(), MVector(ptB.getX(), ptB.getY(), ptB.getZ()));
							}
						}
					}
				}
                if (sEcallBackNodes.length())
                    runCallBacks(sEcallBackNodes);
            }
    
            m_prevTime = time;

            updateActiveRigidBodies(rbConnections);
			updateActiveSoftBodies(rbConnections);
            if (fEcallBackNodes.length())
                runCallBacks(fEcallBackNodes);
        }
    }

    data.outputValue(oa_rigidBodies).set(true);
    data.setClean(plug);
}

void tokenize(const std::string& str,
              std::vector<std::string>& tokens,
              const std::string& delimiters = " ")
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

void replace(std::string &str, std::string const& what, std::string const& with_what)
{
    size_t pos;
    while((pos = str.find(what)) != std::string::npos) {
        str.replace(pos, pos + what.size(), with_what);
    }
}

bool dSolverNode::expandFileExpression(std::string const& expr, std::string &base_name, std::string &extension)
{
    std::vector<std::string> tokens;

    //check for extension
    tokenize(expr, tokens, ".");
    if(tokens.size() < 2) return false;
    if(tokens.back().size() != 3) return false;
    extension = tokens.back();

    std::copy(expr.begin(), expr.end() - 4, std::back_inserter(base_name));

    int time = (int) m_prevTime.value();

    std::stringstream ss;
    ss << time;
    std::string str_time(ss.str());
    //replace ${frame}
    replace(base_name, "${frame}", str_time);

    //replace ${waveFrame}
    while(str_time.size() < 4) str_time = "0" + str_time;
    replace(base_name, "${waveFrame}", str_time);

    return true;
}

void dSolverNode::dumpRigidBodyArray(MObject &node)
{ 
   // std::cout << "dSolverNode::dumpRigidBodyArray" << std::endl;

    MFnDagNode fnDagNode(node);
    rigidBodyArrayNode *rbaNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode());

    MPlug plgFiles(node, rigidBodyArrayNode::ia_fioFiles);
  //  MPlug plgPositionAttribute(node, rigidBodyArrayNode::ia_fioPositionAttribute);
   // MPlug plgRotationAttribute(node, rigidBodyArrayNode::ia_fioRotationAttribute);

    MString mstr;
    plgFiles.getValue(mstr);
    std::string expr(mstr.asChar());
    std::string base_path, extension;
    if(!expandFileExpression(expr, base_path, extension)) {
        std::cout << "dSolverNode::dumpRigidBodyArray: syntax error in file expression: " << std::endl <<
            expr << std::endl;
        return;
    }
    if(extension != "pdb") {
        std::cout << "dSolverNode::dumpRigidBodyArray: only pdb files are supported" << std::endl;
        return;
    }
    std::string file_name = base_path + "." + extension;

    std::vector<rigid_body_t::pointer>& rbs = rbaNode->rigid_bodies();
    pdb_io_t pdb_io;
    pdb_io.m_num_particles = rbs.size();
    pdb_io.m_time = (float)m_prevTime.value();
    pdb_io.m_attributes.resize(3);
    //position
    pdb_io.m_attributes[0].m_num_elements = 1;
    pdb_io.m_attributes[0].m_name = ATTR_POSITION;
    pdb_io.m_attributes[0].m_type = pdb_io_t::kVector;
    pdb_io.m_attributes[0].m_vector_data.resize(rbs.size());

    //rotation angle (in degrees)
    pdb_io.m_attributes[1].m_num_elements = 1;
    pdb_io.m_attributes[1].m_name = ATTR_IN_RANGLE;
    pdb_io.m_attributes[1].m_type = pdb_io_t::kReal;
    pdb_io.m_attributes[1].m_real_data.resize(rbs.size());

    //rotation vector
    pdb_io.m_attributes[2].m_num_elements = 1;
    pdb_io.m_attributes[2].m_name = ATTR_IN_RAXIS;
    pdb_io.m_attributes[2].m_type = pdb_io_t::kVector;
    pdb_io.m_attributes[2].m_vector_data.resize(rbs.size());

    for(size_t i = 0; i < rbs.size(); ++i) {
        vec3f pos;
        quatf rot;
        rbs[i]->get_transform(pos, rot);
        pdb_io.m_attributes[0].m_vector_data[i].x = pos[0];
        pdb_io.m_attributes[0].m_vector_data[i].y = pos[1];
        pdb_io.m_attributes[0].m_vector_data[i].z = pos[2];

        //
        vec3f axis;
        float angle;
        q_to_axis_angle(rot, axis, angle);
        pdb_io.m_attributes[1].m_real_data[i] = rad2deg(angle);

        pdb_io.m_attributes[2].m_vector_data[i].x = axis[0];
        pdb_io.m_attributes[2].m_vector_data[i].y = axis[1];
        pdb_io.m_attributes[2].m_vector_data[i].z = axis[2];
    }

    std::ofstream out(file_name.c_str());
    pdb_io.write(out);
}

void dSolverNode::clearContactRelatedAttributes(MPlugArray &rbConnections)
{
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        
		MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);

		if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

			if (rbNode)
				rbNode->clearContactInfo();		
		}
	}
}
