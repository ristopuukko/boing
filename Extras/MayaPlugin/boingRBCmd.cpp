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
*/

//boingRBCmd.cpp
#include "BulletSoftBody/btSoftBody.h"
#include <maya/MGlobal.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MSyntax.h>

#include <iostream>

#include "boingRBNode.h"
#include "boingRBCmd.h"


MString boingRBCmd::typeName("boingRb");

boingRBCmd::boingRBCmd()
  : m_argDatabase(0),
    m_dagModifier(0)
{
}


boingRBCmd::~boingRBCmd()
{
  if (m_argDatabase) {
    delete m_argDatabase;
  }

  if (m_dagModifier) {
    delete m_dagModifier;
  }
}


void *
boingRBCmd::creator()
{
  return new boingRBCmd;
}


MSyntax
boingRBCmd::syntax()
{
    MSyntax syntax;
    syntax.enableQuery(false);
    syntax.enableEdit(false);

    syntax.addFlag("-n", "-name", MSyntax::kString);
  //  syntax.addFlag("-fn", "-filename", MSyntax::kString);
   // syntax.addFlag("-col", "-color", MSyntax::kString);
   // syntax.addFlag("-dia", "-diameter", MSyntax::kDouble);

    return syntax;
}


MStatus
boingRBCmd::doIt(const MArgList &args)
{
    MStatus stat;
    m_argDatabase = new MArgDatabase(syntax(), args, &stat);
    if (stat == MS::kFailure) {
	return stat;
    }
    return redoIt();
}


MStatus
boingRBCmd::undoIt()
{
  MGlobal::setActiveSelectionList(m_undoSelectionList);

  if (m_dagModifier) {
      m_dagModifier->undoIt();
      delete m_dagModifier;
      m_dagModifier = 0;
  }

  return MS::kSuccess;
}


MStatus
boingRBCmd::redoIt()
{
    MGlobal::getActiveSelectionList(m_undoSelectionList);

    MString name;
    if (m_argDatabase->isFlagSet("-name")) {
	m_argDatabase->getFlagArgument("-name", 0, name);
    }
    if (!name.length()) {
	name = "boingRb";
    }

    m_dagModifier = new MDagModifier;

    
    MStringArray commandResult;
    MStatus status;
    MGlobal::executeCommand("polyCube", commandResult, &status);
    cout<<"commandResult : "<<commandResult<<endl;
    
    
    
    // create maya cube !!JUST FOR THE TESTING!!!
    // transform for the mesh
    /*MObject parentObj = m_dagModifier->createNode("transform", MObject::kNullObj);
    m_dagModifier->renameNode(parentObj, "pCube#");
    m_dagModifier->doIt();*/
    // mesh
    
    
    /*MObject meshObj = m_dagModifier->createNode(MString("mesh"), parentObj);
    std::string meshObjName = MFnDependencyNode(parentObj).name().asChar();
    std::string::size_type meshPos = meshObjName.find_last_not_of("0123456789");
    meshObjName.insert(meshPos + 1, "Shape");
    m_dagModifier->renameNode(meshObj, meshObjName.c_str());
    cout<<"meshObjName : "<<meshObjName<<endl;
    m_dagModifier->doIt();*/
    

    //creator
    
    /*MObject createObj = m_dagModifier->createNode(MString("polyCube"), MObject::kNullObj);
    std::string createObjName = MFnDependencyNode(parentObj).name().asChar();
    //std::string::size_type createPos = createObjName.find_last_not_of("0123456789");
    //createObjName.insert(createPos + 1, );
    //m_dagModifier->renameNode(createObj, createObjName.c_str());
    cout<<"createObjName : "<<createObjName<<endl;
    m_dagModifier->doIt();*/
    
    
    // done with the mesh
    
    //connect the polyCube-node to mesh
    // worldMesh -> collisionShape
    /*MPlug plgPolyCubeOutput = MFnDependencyNode(createObj).findPlug("output", false);
    cout<<plgPolyCubeOutput.name().asChar()<<endl;
    MPlug plgMeshInShape = MFnDependencyNode(meshObj).findPlug("inMesh", false);
    cout<<plgMeshInShape.name().asChar()<<endl;
    m_dagModifier->connect(plgPolyCubeOutput, plgMeshInShape);
    m_dagModifier->doIt();
    */
    // done connecting polyCube

    
    MObject boingRbObj = m_dagModifier->createNode(boingRBNode::typeId);
    std::string boingRbName = name.asChar();
    //std::string::size_type pos = boingRbName.find_last_not_of("0123456789");
    //cout<<"pos : "<<pos<<endl;
    //boingRbName.insert(pos + 1);
    m_dagModifier->renameNode(boingRbObj, "boingRb#" ) ;//boingRbName.c_str());
    cout<<"boingRbName : "<<boingRbName<<endl;
    m_dagModifier->doIt();

    
    //connect the mesh to the boingRb and vice versa
    // worldMesh -> collisionShape
    MPlug plgCollisionShape(boingRbObj, boingRBNode::ia_shape);
    cout<<"plgCollisionShape.name() :"<<plgCollisionShape.name().asChar()<<endl;
    //MPlug plgMeshShape = MFnDependencyNode(meshObj).findPlug("worldMesh", false);
    //MPlug plgMeshShape = MFnDependencyNode(nameToNode(commandResult[1])).findPlug("worldMesh", false);
    MPlug plgMeshShape = nameToNodePlug("worldMesh", nameToNode(commandResult[1]));
    cout<<"plgMeshShape.name() : "<<plgMeshShape.name().asChar()<<endl;
    m_dagModifier->connect(plgMeshShape, plgCollisionShape);
    m_dagModifier->doIt();

    // translate -> ia_initialPosition
/*
    MPlug plgInitPos(boingRbObj, boingRBNode::ia_initialPosition);
    cout<<"plgInitPos.name() :"<<plgInitPos.name().asChar()<<endl;
    //MPlug plgTranslation = MFnDependencyNode(parentObj).findPlug("translate", false);
    MPlug plgTranslation = MFnDependencyNode(nameToNode(commandResult[0])).findPlug("translate", false);
    cout<<"plgTranslation.name() : "<<plgTranslation.name().asChar()<<endl;
    m_dagModifier->connect(plgTranslation, plgInitPos);
    m_dagModifier->doIt();

    
    // rotation -> ia_initialRotation
    
    MPlug plgInitRot(boingRbObj, boingRBNode::ia_initialRotation);
    cout<<"plgInitRot.name() :"<<plgInitRot.name().asChar()<<endl;
    //MPlug plgRotation = MFnDependencyNode(parentObj).findPlug("rotate", false);
    MPlug plgRotation = MFnDependencyNode(nameToNode(commandResult[0])).findPlug("rotate", false);
    cout<<"plgRotation.name() :"<<plgRotation.name().asChar()<<endl;
    m_dagModifier->connect(plgRotation, plgInitRot);
    m_dagModifier->doIt();
*/
    // ia_position -> translate
    
    MPlug plgPos(boingRbObj, boingRBNode::ia_position);
    cout<<"plgPos.name() :"<<plgPos.name().asChar()<<endl;
    MPlug plgTranslation = MFnDependencyNode(nameToNode(commandResult[0])).findPlug("translate", false);
    m_dagModifier->connect(plgPos, plgTranslation);
    m_dagModifier->doIt();
    
    
    // ia_rotation -> rotation
    
    MPlug plgRot(boingRbObj, boingRBNode::ia_rotation);
    cout<<"plgRot.name() :"<<plgRot.name().asChar()<<endl;
    MPlug plgRotation = MFnDependencyNode(nameToNode(commandResult[0])).findPlug("rotate", false);
    m_dagModifier->connect(plgRot, plgRotation);
    m_dagModifier->doIt();
    
    // connect the solver attribute
    MPlug plgSolver(boingRbObj, boingRBNode::ia_solver);
    MSelectionList slist;
    slist.add("bSolver");
    MObject solverObj;
    if(slist.length() != 0) {
        slist.getDependNode(0, solverObj);
		MPlug boingRbs = MFnDependencyNode(solverObj).findPlug("rigidBodies", false);		
		m_dagModifier->connect(boingRbs, plgSolver);
		m_dagModifier->doIt();
    } else {
        cerr<<"Cannot find bSolver"<<endl;
    }

  //  MGlobal::select(parentObj, MGlobal::kReplaceList);

    setResult(MFnDependencyNode(boingRbObj).name());

    return MS::kSuccess;
}


MObject boingRBCmd::nameToNode( MString name ) {
    MSelectionList selList;
    selList.add( name );
    MObject node;
    selList.getDependNode( 0, node );
    return node;
}

MPlug boingRBCmd::nameToNodePlug( MString attrName, MObject nodeObject ) {
    MFnDependencyNode depNodeFn( nodeObject );
    MObject attrObject = depNodeFn.attribute( attrName );
    MPlug plug( nodeObject, attrObject );
    return plug;
}
