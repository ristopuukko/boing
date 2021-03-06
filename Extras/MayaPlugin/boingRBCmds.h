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

//boingRBCmds.h

#ifndef DYN_BOINGRBCMD_H
#define DYN_BOINGRBCMD_H

#include <maya/MArgDatabase.h>
#include <maya/MDagModifier.h>
#include <maya/MDGModifier.h>
#include <maya/MDagModifier.h>
#include <maya/MSelectionList.h>
#include <maya/MPxCommand.h>
#include <maya/MFnMesh.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>
#include <maya/MArgList.h>

#include "bSolverNode.h"
#include "collision_shape.h"
//#include "boing.h"

class createBoingRBCmd : public MPxCommand
{
public:
    createBoingRBCmd();
    virtual ~createBoingRBCmd();

    static void *creator();
    static MSyntax syntax();

    static MString typeName;

    MStatus doIt(const MArgList &i_mArgList);
    MStatus redoIt();
    MStatus undoIt();
    bool isUndoable() const { return true; }
    bool hasSyntax() const { return true; }

protected:
    MArgDatabase *m_argDatabase;
    MDGModifier *m_dgModifier;
    //MDagModifier *m_dagModifier;
    MSelectionList m_undoSelectionList;
};



class boingRbCmd : public MPxCommand
{
public:
    boingRbCmd();
    virtual ~boingRbCmd();
    
    static void *creator();
    static MSyntax cmdSyntax();
    static MString typeName;
    
    MStatus doIt(const MArgList &i_mArgList);
    MStatus redoIt();
    //MStatus undoIt();
    bool isUndoable() const { return false; }
    bool hasSyntax() const { return true; }
    MObject nameToNode( MString name ) ;
    MPlug nameToNodePlug( MString attrName, MObject nodeObject );
    MString checkAttribute(MString &attr);
    MString checkCustomAttribute(MString &name, MString &attr);
    MStringArray parseArguments(MString arg, MString token);
    MStatus setBulletVectorAttribute(MString &name, MString &attr, MVector &vec);
    MDoubleArray getBulletVectorAttribute(MString &name, MString &attr);

private:
    
//protected:
    static rigid_body_t::pointer getPointerFromName(MString &name);
    bool    isSetAttr;
    bool    isGetAttr;
    bool    isAddAttr;
    bool    isCreate;
    bool    isDelete;
    bool    isValue;
    bool    isType;
    bool    isExists;
    bool    isAttributeExist;
    MArgDatabase *argParser;
    MArgList *argsList;
    

};

#endif