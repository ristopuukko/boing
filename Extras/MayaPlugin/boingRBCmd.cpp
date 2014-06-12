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
#include "boingRBCmd.h"
#include "boingRBNode.h"
#include "BulletSoftBody/btSoftBody.h"
#include <maya/MGlobal.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MSyntax.h>
#include <maya/MFnDependencyNode.h>
#include <iostream>



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

    //MStringArray commandResult;
    MStatus status;
    
    MObject boingRbObj = m_dagModifier->createNode(boingRBNode::typeId, MObject::kNullObj, &status);
    if (status !=MS::kSuccess) {
        cerr<<"error creating boingRb!"<<endl;
        if (status == MS::kInvalidParameter) cerr<<"MS::kInvalidParameter"<<endl;
    }
    //m_dagModifier->renameNode(boingRbObj, name);
    m_dagModifier->doIt();

    setResult(MFnDependencyNode(boingRbObj).name());

    return MS::kSuccess;
}


/*

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
*/