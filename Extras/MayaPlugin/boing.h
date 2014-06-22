//
//  boing.h
//  BulletMayaPlugin
//
//  Created by Risto Puukko on 19/06/14.
//
//

#ifndef __BulletMayaPlugin__boing__
#define __BulletMayaPlugin__boing__
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MVector.h>
#include <vector>
#include <set>
#include <iostream>
#include <string>

#include "bSolverNode.h"

class boing
{
public:
    
    boing(MString value);
    virtual ~boing();

    virtual MString get_data(MString &name);
    virtual void add_data(MString &attr, MString &data);
    virtual void set_data(MString &attr, MString &data);
    
    static MString name;

private:
    MStringArray attrArray;
    MStringArray dataArray;
    collision_shape_t::pointer boing::m_collision_shape;

protected :
    friend class bSolverNode;
    
public:
    MStatus deleteRigidBody(MString &name);
    MStatus setBulletVectorAttribute(MString &name, MString &attr, MVector &vec);
    MVector getBulletVectorAttribute(MString &name, MString &attr);
    MString checkAttribute(MString &attr);
    MString checkCustomAttribute(MString &name, MString &attr);
    MStringArray parseArguments(MString arg, MString token);
    collision_shape_t::pointer createCollisionShape(const MObject& node);
    MStatus createRigidBody(collision_shape_t::pointer  &collision_shape,
                            MObject &node,
                            MString &name,
                            MVector &vel,
                            MVector &pos,
                            MVector &rot,
                            MVector &av);
    
    MStatus deleteRigidBody();
    
    
    static MString typeName;
    
//private:
    
protected:
    
    bool    isSetAttr;
    bool    isGetAttr;
    bool    isAddAttr;
    bool    isCreate;
    bool    isDelete;
    bool    isValue;
    bool    isType;
    MArgParser *argParser;
    static rigid_body_t::pointer getPointerFromName(MString &name);
};

#endif /* defined(__BulletMayaPlugin__boing__) */

