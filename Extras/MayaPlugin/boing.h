//
//  boing.h
//  BulletMayaPlugin
//
//  Created by Risto Puukko on 19/06/14.
//
//

#ifndef __BulletMayaPlugin__boing__
#define __BulletMayaPlugin__boing__
#include <maya/MItDependencyNodes.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnDagNode.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MObject.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MVector.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFnMesh.h>
#include <maya/MSelectionList.h>
#include <vector>
#include <set>
#include <iostream>
#include <string>
#include "bt_solver.h"
#include "shared_ptr.h"
#include "bt_rigid_body.h"
#include "solver.h"
#include "bSolverNode.h"
//#include "LinearMath/btSerializer.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "bSolverNode.h"
#include "collision_shape.h"

class boing
{
public:
    
    boing(MObject &node, MString &name, MVector &vel, MVector &pos, MVector &rot, MVector &av);
    virtual ~boing();

    static MString get_data(MString &name);
    static void add_data(MString &attr, MString &data);
    static void set_data(MString &attr, MString &data);
    
    static MStatus deleteRigidBody(MString &name);
    collision_shape_t::pointer createCollisionShape(const MObject& node);
    void createRigidBody();
    
    MStatus deleteRigidBody();
    MObject nameToNode( MString name ) ;
  
    //MStatus set_bullet_attribute();
    static MString name;
    friend class bSolverNode;
    friend class boingRbCmd;
    
private:
    static MObject node;
    static MVector initial_velocity;
    static MVector initial_position;
    static MVector initial_rotation;
    static MVector initial_angularvelocity;
    static MStringArray attrArray;
    static MStringArray dataArray;
    static collision_shape_t::pointer m_collision_shape;
    static rigid_body_t::pointer m_rigid_body;
    
//protected:
    
};

#endif /* defined(__BulletMayaPlugin__boing__) */

