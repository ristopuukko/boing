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

//boingRBCmds.cpp
#include "boingRBCmds.h"
#include "boingRBNode.h"
#include "BulletSoftBody/btSoftBody.h"
#include <maya/MGlobal.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MSyntax.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnDagNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MStringArray.h>

#include <iostream>
#include "bt_solver.h"
#include "shared_ptr.h"
#include "bt_rigid_body.h"
#include "solver.h"
#include "bSolverNode.h"
#include "LinearMath/btSerializer.h"

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"


MString createBoingRBCmd::typeName("createBoingRb");

createBoingRBCmd::createBoingRBCmd()
  : m_argDatabase(0),
    m_dgModifier(0),
    m_dagModifier(0)

{
}


createBoingRBCmd::~createBoingRBCmd()
{
  if (m_argDatabase) {
    delete m_argDatabase;
  }

  if (m_dgModifier) {
    delete m_dgModifier;
  }
  if (m_dagModifier) {
    delete m_dagModifier;
  }
}


void *
createBoingRBCmd::creator()
{
  return new createBoingRBCmd;
}


MSyntax
createBoingRBCmd::syntax()
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
createBoingRBCmd::doIt(const MArgList &args)
{
    MStatus stat;
    m_argDatabase = new MArgDatabase(syntax(), args, &stat);
    if (stat == MS::kFailure) {
	return stat;
    }
    return redoIt();
}


MStatus
createBoingRBCmd::undoIt()
{
  MGlobal::setActiveSelectionList(m_undoSelectionList);

  if (m_dgModifier) {
      m_dgModifier->undoIt();
      delete m_dgModifier;
      m_dgModifier = 0;
  }
  if (m_dagModifier) {
      m_dagModifier->undoIt();
      delete m_dagModifier;
      m_dagModifier = 0;
  }

  return MS::kSuccess;
}


MStatus
createBoingRBCmd::redoIt()
{
    MGlobal::getActiveSelectionList(m_undoSelectionList);

    MString name;
    if ( m_argDatabase->isFlagSet("-name") ) {
        m_argDatabase->getFlagArgument("-name", 0, name);
    } else if ( m_argDatabase->isFlagSet("-n") ) {
        m_argDatabase->getFlagArgument("-n", 0, name);
    }
    
    if (!name.length()) {
        name = "boingRb1";
    }

    m_dgModifier = new MDagModifier;

    //MStringArray commandResult;
    MStatus status;
    
    MObject boingRbObj = m_dgModifier->createNode(boingRBNode::typeId, &status);
    if (status !=MS::kSuccess) {
        cerr<<"error creating boingRb!"<<endl;
        if (status == MS::kInvalidParameter) cerr<<"MS::kInvalidParameter"<<endl;
    }
    m_dgModifier->renameNode(boingRbObj, name);
    m_dgModifier->doIt();

    setResult(MFnDependencyNode(boingRbObj).name());

    return MS::kSuccess;
}

MString boingRbCmd::typeName("boingRb");

boingRbCmd::boingRbCmd()
: argParser(0)
{}


boingRbCmd::~boingRbCmd()
{
    if (argParser) {
        delete argParser;
    }
    
    //if (m_dgModifier) {
    //  delete m_dgModifier;
    //}
}


void *
boingRbCmd::creator()
{
    return new boingRbCmd;
}

MSyntax
boingRbCmd::cmdSyntax()
{
    MSyntax syntax;
    syntax.enableQuery(false);
    syntax.enableEdit(false);
    
    syntax.addFlag("-set", "-setAttr", MSyntax::kString);
    syntax.addFlag("-get", "-getAttr", MSyntax::kString);
    syntax.addFlag("-cr", "-create", MSyntax::kString);
    syntax.addFlag("-del", "-delete", MSyntax::kString);
    syntax.addFlag("-val", "-value", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    
    return syntax;
}


MStatus
boingRbCmd::doIt(const MArgList &args)
{
    MStatus stat;
    
    argParser = new MArgParser(syntax(), args, &stat);
    
    if (stat == MS::kFailure) {
        cerr<<"failed to read argData"<<endl;
        return stat;
    }
    return redoIt();
}


/*MStatus
 boingRbCmd::undoIt()
 {
 //MGlobal::setActiveSelectionList(m_undoSelectionList);
 
 if (m_dgModifier) {
 m_dgModifier->undoIt();
 delete m_dgModifier;
 m_dgModifier = 0;
 
 return MS::kSuccess;
 }*/


MStatus boingRbCmd::redoIt()
{
    //MGlobal::getActiveSelectionList(m_undoSelectionList)
    
    /*
     if (argData->isFlagSet("help"))
     {
     MGlobal::displayInfo(MString("Tässä olisi helppi-teksti"));
     return MS::kSuccess;
     }*/
    isSetAttr = argParser->isFlagSet("-setAttr");
    isGetAttr = argParser->isFlagSet("-getAttr");
    isCreate = argParser->isFlagSet("-create");
    isDelete = argParser->isFlagSet("-delete");
    isValue = argParser->isFlagSet("-value");
    
    if (isSetAttr && isValue) {
        MString sAttr;
        //MArgList argList;
        argParser->getFlagArgument("setAttr", 0, sAttr);
        //cout<<sAttr<<endl;
        
        MStringArray jobArgsArray = parseArguments(sAttr, ".");
        MVector value;
        argParser->getFlagArgument("-value", 0, value.x);
        argParser->getFlagArgument("-value", 1, value.y);
        argParser->getFlagArgument("-value", 2, value.z);
        //cout<<"jobArgsArray[1] : "<<jobArgsArray[1]<<endl;
        MString attr = checkAttribute(jobArgsArray[1]);
        setBulletVectorAttribute(jobArgsArray[0], attr, value);
        
        //cout<<value<<endl;
        setResult(MS::kSuccess);
        
    } else if ( isGetAttr) {
        MString gAttr;
        argParser->getFlagArgument("getAttr", 0, gAttr);
        
        //cout<<gAttr<<endl;
        MStringArray jobArgsArray = parseArguments(gAttr, ".");
        
        MString attr = checkAttribute(jobArgsArray[1]);
        //cout<<"attr = "<<attr<<endl;
        if (attr!="name") {
            MVector result = getBulletVectorAttribute(jobArgsArray[0], attr);
            MDoubleArray dResult;
            dResult.append(result.x);
            dResult.append(result.y);
            dResult.append(result.z);
            setResult(dResult);
        } else {
            MStringArray result;
            shared_ptr<solver_impl_t> solv = solver_t::get_solver();
            std::set<rigid_body_t::pointer> rbds = solver_t::get_rigid_bodies();
            std::set<rigid_body_t::pointer>::iterator rit;
            for(rit=rbds.begin(); rit!=rbds.end(); ++rit) {
                rigid_body_t::pointer rb = (*rit);
                void *namePtr = rb->collision_shape()->getBulletCollisionShape()->getUserPointer();
                char *name = static_cast<char*>(namePtr);
                result.append(name);
            }
            setResult(result);
        }
        
    } else if ( isCreate  ) {
        MString aArgument;
        argParser->getFlagArgument("-create", 0, aArgument);
        
        MStringArray createArgs = parseArguments(aArgument,";");
        int size = createArgs.length();
        MString inputShape;
        MString rbname = "dummyRb";
        MVector vel;
        MVector pos;
        MVector rot;
        
        for (int i=0; i<size; ++i) {
            MStringArray singleArg = parseArguments(createArgs[i],"=");
            if (singleArg[0] == "name") {
                rbname = singleArg[1];
            } else if (singleArg[0] == "geo") {
                //geo
                inputShape = singleArg[1];
            } else if (singleArg[0] == "vel") {
                //initialvelocity
                MStringArray velArray = parseArguments(singleArg[1], ",");
                vel = MVector(velArray[0].asDouble(),
                              velArray[1].asDouble(),
                              velArray[2].asDouble()
                              );
                //cout<<"velocity = "<<vel<<endl;
                
            } else if (singleArg[0] == "pos") {
                //initialposition
                MStringArray posArray = parseArguments(singleArg[1], ",");
                pos = MVector(posArray[0].asDouble(),
                              posArray[1].asDouble(),
                              posArray[2].asDouble()
                              );
                //cout<<"position = "<<pos<<endl;
                
            } else if (singleArg[0] == "rot") {
                //initialrotation
                MStringArray rotArray = parseArguments(singleArg[1], ",");
                rot = MVector(rotArray[0].asDouble(),
                              rotArray[1].asDouble(),
                              rotArray[2].asDouble()
                              );
                //cout<<"rotation = "<<rot<<endl;
                
            } else {
                cout<<"Unrecognized parameter : "<<singleArg[0]<<endl;
                
            }
        }
        // create the collisionShape-node
        MObject node = nameToNode(inputShape);
        collision_shape_t::pointer collShape = createCollisionShape(node);
        createRigidBody(collShape, node, rbname, vel, pos, rot);
        
    } else if ( isDelete  ) {
        MString aArgument;
        argParser->getFlagArgument("-delete", 0, aArgument);
        if (aArgument != "") {
            deleteRigidBody(aArgument);
        }
    }
    
    return MS::kSuccess;
}

MStringArray boingRbCmd::parseArguments(MString arg, MString token) {
    
    MStringArray jobArgsArray;
    MString stringBuffer;
    for (unsigned int charIdx = 0; charIdx < arg.numChars(); charIdx++) {
        MString ch = arg.substringW(charIdx, charIdx);
        //cout<<"ch = "<<ch<<endl;
        if (ch == token ) {
            if (stringBuffer.length() > 0) {
                jobArgsArray.append(stringBuffer);
                //cout<<"jobArgsArray = "<<jobArgsArray<<endl;
                stringBuffer.clear();
            }
        } else {
            stringBuffer += ch;
            //cout<<"stringBuffer = "<<stringBuffer<<endl;
        }
    }
    jobArgsArray.append(stringBuffer);
    
    return jobArgsArray;
}

MStatus boingRbCmd::createRigidBody(collision_shape_t::pointer  &collision_shape,
                                    MObject node,
                                    MString name,
                                    MVector vel,
                                    MVector pos,
                                    MVector rot)
{
    //MGlobal::getActiveSelectionList(m_undoSelectionList);
    
    
    if (!name.length()) {
        name = "boingRb1";
    }
    
    double mscale[3] = {1,1,1};
    MQuaternion mrotation = MEulerRotation(rot).asQuaternion();
    
    //collision_shape_t::pointer  collision_shape;
    if(!collision_shape) {
        //not connected to a collision shape, put a default one
        collision_shape = solver_t::create_box_shape();
    } else {
        if ( !node.isNull() ) {
            MFnDagNode fnDagNode(node);
            //cout<<"node : "<<fnDagNode.partialPathName()<<endl;
            MFnTransform fnTransform(fnDagNode.parent(0));
            //cout<<"MFnTransform node : "<<fnTransform.partialPathName()<<endl;
            if ( pos == MVector::zero ) {
                pos = fnTransform.getTranslation(MSpace::kTransform);
            }
            if ( rot == MVector::zero ) {
                fnTransform.getRotation(mrotation, MSpace::kTransform);
            }
            fnTransform.getScale(mscale);
        }
    }
    
    shared_ptr<solver_impl_t> solv = solver_t::get_solver();
    
    rigid_body_t::pointer m_rigid_body = solver_t::create_rigid_body(collision_shape);
    
    char *rName = (char *)name.asChar();
    bSolverNode::procRbArray.append(name);
    void *namePtr = rName;
    collision_shape->getBulletCollisionShape()->setUserPointer(namePtr);
    
    m_rigid_body->set_transform(vec3f((float)pos.x, (float)pos.y, (float)pos.z),
                                quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));
    
    m_rigid_body->set_linear_velocity( vec3f((float)vel.x,(float)vel.y,(float)vel.z) );
    
    m_rigid_body->collision_shape()->set_scale(vec3f((float)mscale[0], (float)mscale[1], (float)mscale[2]));
    
    
    float mass = 1.f;
    m_rigid_body->set_mass(mass);
    m_rigid_body->set_inertia((float)mass * m_rigid_body->collision_shape()->local_inertia());
    
    
    //initialize those default values too
    float restitution = 0.3f;
    //MPlug(thisObject, rigidBodyNode::ia_restitution).getValue(restitution);
    m_rigid_body->set_restitution(restitution);
    float friction = 0.5f;
    //MPlug(thisObject, rigidBodyNode::ia_friction).getValue(friction);
    m_rigid_body->set_friction(friction);
    float linDamp = 0.f;
    //MPlug(thisObject, rigidBodyNode::ia_linearDamping).getValue(linDamp);
    m_rigid_body->set_linear_damping(linDamp);
    float angDamp = 0.2f;
    //MPlug(thisObject, rigidBodyNode::ia_angularDamping).getValue(angDamp);
    m_rigid_body->set_angular_damping(angDamp);

    solver_t::add_rigid_body(m_rigid_body, name.asChar());

    return MS::kSuccess;
}


collision_shape_t::pointer boingRbCmd::createCollisionShape(const MObject& node)
{
    
    collision_shape_t::pointer collision_shape = 0;
    
    int type=0;
    
    switch(type) {
        case 0:
        {
            //convex hull
            {
                //cout<<"going on creating a convex hull collision shape"<<endl;
                if(node.hasFn(MFn::kMesh)) {
                    MDagPath dagPath;
                    MDagPath::getAPathTo(node, dagPath);
                    MFnMesh fnMesh(dagPath);
                    //cout<<"going on creating a convex hull from : "<<fnMesh.name()<<endl;
                    MFloatPointArray mpoints;
                    MFloatVectorArray mnormals;
                    MIntArray mtrianglecounts;
                    MIntArray mtrianglevertices;
                    fnMesh.getPoints(mpoints, MSpace::kObject);
                    fnMesh.getNormals(mnormals, MSpace::kObject);
                    fnMesh.getTriangles(mtrianglecounts, mtrianglevertices);
                    
                    std::vector<vec3f> vertices(mpoints.length());
                    std::vector<vec3f> normals(mpoints.length());
                    std::vector<unsigned int> indices(mtrianglevertices.length());
                    
                    btAlignedObjectArray<btVector3> btVerts; //mb
                    
                    for(size_t i = 0; i < vertices.size(); ++i) {
                        vertices[i] = vec3f(mpoints[i].x, mpoints[i].y, mpoints[i].z);
                        normals[i] = vec3f(mnormals[i].x, mnormals[i].y, mnormals[i].z);
                        
#if UPDATE_SHAPE //future collision margin adjust
                        btVerts.push_back(btVector3(mpoints[i].x, mpoints[i].y, mpoints[i].z)); //mb
#endif
                    }
                    for(size_t i = 0; i < indices.size(); ++i) {
                        indices[i] = mtrianglevertices[i];
                    }
                    
#if UPDATE_SHAPE //future collision margin adjust
                    btAlignedObjectArray<btVector3> planeEquations;
                    btGeometryUtil::getPlaneEquationsFromVertices(btVerts, planeEquations);
                    
                    btAlignedObjectArray<btVector3> shiftedPlaneEquations;
                    for (int p=0;p<planeEquations.size();p++)
                    {
                        btVector3 plane = planeEquations[p];
                        plane[3] += collisionShapeNode::collisionMarginOffset;
                        shiftedPlaneEquations.push_back(plane);
                    }
                    
                    btAlignedObjectArray<btVector3> shiftedVertices;
                    btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations, shiftedVertices);
                    
                    std::vector<vec3f> shiftedVerticesVec3f(shiftedVertices.size());
                    for(size_t i = 0; i < shiftedVertices.size(); ++i)
                    {
                        shiftedVerticesVec3f[i] = vec3f(shiftedVertices[i].getX(), shiftedVertices[i].getY(), shiftedVertices[i].getZ());
                        //std::cout << "orig verts: " << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << std::endl;
                        //std::cout << "shft verts: " << shiftedVertices[i].getX() << " " << shiftedVertices[i].getY() << " " << shiftedVertices[i].getZ() << std::endl;
                        //std::cout << std::endl;
                    }
                    
                    collision_shape = solver_t::create_convex_hull_shape(&(shiftedVerticesVec3f[0]), shiftedVerticesVec3f.size(), &(normals[0]), &(indices[0]), indices.size());
                    
#endif
                    
#if UPDATE_SHAPE == 0
                    collision_shape = solver_t::create_convex_hull_shape(&(vertices[0]), vertices.size(), &(normals[0]), &(indices[0]), indices.size()); //original
#endif
                }
            }
        }
            
            break;
        case 1:
        {
            //mesh
            {
                if(node.hasFn(MFn::kMesh)) {
                    MDagPath dagPath;
                    MDagPath::getAPathTo(node, dagPath);
                    MFnMesh fnMesh(dagPath);
                    MFloatPointArray mpoints;
                    MFloatVectorArray mnormals;
                    MIntArray mtrianglecounts;
                    MIntArray mtrianglevertices;
                    fnMesh.getPoints(mpoints, MSpace::kObject);
                    fnMesh.getNormals(mnormals, MSpace::kObject);
                    fnMesh.getTriangles(mtrianglecounts, mtrianglevertices);
                    
                    std::vector<vec3f> vertices(mpoints.length());
                    std::vector<vec3f> normals(mpoints.length());
                    std::vector<unsigned int> indices(mtrianglevertices.length());
                    
                    for(size_t i = 0; i < vertices.size(); ++i) {
                        vertices[i] = vec3f(mpoints[i].x, mpoints[i].y, mpoints[i].z);
                        normals[i] = vec3f(mnormals[i].x, mnormals[i].y, mnormals[i].z);
                    }
                    for(size_t i = 0; i < indices.size(); ++i) {
                        indices[i] = mtrianglevertices[i];
                    }
                    bool dynamicMesh = true;
                    collision_shape = solver_t::create_mesh_shape(&(vertices[0]), vertices.size(), &(normals[0]),
                                                                  &(indices[0]), indices.size(),dynamicMesh);
                }
            }
        }
            break;
        case 2:
            //cylinder
            break;
        case 3:
            //capsule
            break;
            
        case 7:
            //btBvhTriangleMeshShape
        {
            
            if(node.hasFn(MFn::kMesh)) {
                MDagPath dagPath;
                MDagPath::getAPathTo(node, dagPath);
                MFnMesh fnMesh(dagPath);
                MFloatPointArray mpoints;
                MFloatVectorArray mnormals;
                MIntArray mtrianglecounts;
                MIntArray mtrianglevertices;
                fnMesh.getPoints(mpoints, MSpace::kObject);
                fnMesh.getNormals(mnormals, MSpace::kObject);
                fnMesh.getTriangles(mtrianglecounts, mtrianglevertices);
                
                std::vector<vec3f> vertices(mpoints.length());
                std::vector<vec3f> normals(mpoints.length());
                std::vector<unsigned int> indices(mtrianglevertices.length());
                
                for(size_t i = 0; i < vertices.size(); ++i) {
                    vertices[i] = vec3f(mpoints[i].x, mpoints[i].y, mpoints[i].z);
                    normals[i] = vec3f(mnormals[i].x, mnormals[i].y, mnormals[i].z);
                }
                for(size_t i = 0; i < indices.size(); ++i) {
                    indices[i] = mtrianglevertices[i];
                }
                bool dynamicMesh = false;
                collision_shape = solver_t::create_mesh_shape(&(vertices[0]), vertices.size(), &(normals[0]),
                                                              &(indices[0]), indices.size(),dynamicMesh);
            }
        }
            break;
        case 8:
            //hacd convex decomposition
        {
            
            {
                if(node.hasFn(MFn::kMesh)) {
                    MDagPath dagPath;
                    MDagPath::getAPathTo(node, dagPath);
                    MFnMesh fnMesh(dagPath);
                    MFloatPointArray mpoints;
                    MFloatVectorArray mnormals;
                    MIntArray mtrianglecounts;
                    MIntArray mtrianglevertices;
                    fnMesh.getPoints(mpoints, MSpace::kObject);
                    fnMesh.getNormals(mnormals, MSpace::kObject);
                    fnMesh.getTriangles(mtrianglecounts, mtrianglevertices);
                    
                    std::vector<vec3f> vertices(mpoints.length());
                    std::vector<vec3f> normals(mpoints.length());
                    std::vector<unsigned int> indices(mtrianglevertices.length());
                    
                    for(size_t i = 0; i < vertices.size(); ++i) {
                        vertices[i] = vec3f(mpoints[i].x, mpoints[i].y, mpoints[i].z);
                        normals[i] = vec3f(mnormals[i].x, mnormals[i].y, mnormals[i].z);
                    }
                    for(size_t i = 0; i < indices.size(); ++i) {
                        indices[i] = mtrianglevertices[i];
                    }
                    bool dynamicMesh = false;
                    collision_shape = solver_t::create_hacd_shape(&(vertices[0]), vertices.size(), &(normals[0]),
                                                                  &(indices[0]), indices.size(),dynamicMesh);
                }
            }
        }
            break;
        default:
        {
        }
    }
    
    return collision_shape;
}


MStatus boingRbCmd::deleteRigidBody(MString &name) {
    
    MStatus result = MS::kFailure;
    
    rigid_body_t::pointer rb = getPointerFromName(name);
    
    solver_t::remove_rigid_body(rb);
    for(int i=0; i<bSolverNode::procRbArray.length(); i++){
        if (bSolverNode::procRbArray[i] == name) {
            result = bSolverNode::procRbArray.remove(i);
            break;
        }
        
    }
    return result;
}


MString boingRbCmd::checkAttribute(MString attr) {
    MString result;
    
    if (attr=="vel" || attr=="velocity") {
        result = "velocity";
    } else if (attr=="pos" || attr=="position") {
        result = "position";
    } else if ( attr=="av" || attr=="angularVelocity") {
        result = "angularVelocity";
    } else if ( attr=="n" || attr=="name") {
        result = "name";
    }
    //cout<<"checkAttribute : "<<result<<endl;
    return result;
}

MStatus boingRbCmd::setBulletVectorAttribute(MString &name, MString attr, MVector vec) {
    
    cout<<"setting attribute : "<<attr<<" for rb "<<name<<endl;
    
    rigid_body_t::pointer rb = getPointerFromName(name);
    if (rb != 0)
    {
        float mass = rb->get_mass();
        bool active = (mass>0.f);
        if(active) {
            if (attr=="velocity") {
                vec3f vel;
                vel = vec3f((float)vec.x,(float)vec.y,(float)vec.z);
                rb->set_linear_velocity(vel);
            } else if (attr=="position") {
                vec3f pos;
                quatf rot;
                rb->get_transform(pos, rot);
                pos = vec3f((float)vec.x,(float)vec.y,(float)vec.z);
                rb->set_transform(pos, rot);
            } else if (attr=="angularVelocity") {
                vec3f av;
                av = vec3f((float)vec.x,(float)vec.y,(float)vec.z);
                rb->set_angular_velocity(av);
            }
        }
    }
    
    return MS::kSuccess;
    
}

MVector boingRbCmd::getBulletVectorAttribute(MString &name, MString attr) {
    
    MVector vec;
    
    rigid_body_t::pointer rb = getPointerFromName(name);
    
    if (rb != 0) {
        float mass = rb->get_mass();
        bool active = (mass>0.f);
        if(active) {
            if (attr=="velocity") {
                vec3f vel;
                rb->get_linear_velocity(vel);
                vec = MVector((double)vel[0], (double)vel[1], (double)vel[2]);
            } else if (attr=="position") {
                vec3f pos;
                quatf rot;
                rb->get_transform(pos, rot);
                vec = MVector((double)pos[0], (double)pos[1], (double)pos[2]);
            } else if (attr=="angularVelocity") {
                vec3f av;
                rb->get_angular_velocity(av);
                vec = MVector((double)av[0], (double)av[1], (double)av[2]);
            }
        }
    }
    
    return vec;
    
}

rigid_body_t::pointer boingRbCmd::getPointerFromName(MString &name)
{
    
    rigid_body_t::pointer rb = 0;
    
    shared_ptr<solver_impl_t> solv = solver_t::get_solver();
    std::set<rigid_body_t::pointer> rbds = solver_t::get_rigid_bodies();
    std::set<rigid_body_t::pointer>::iterator rit;
    for(rit=rbds.begin(); rit!=rbds.end(); ++rit) {
        rigid_body_t::pointer temprb = (*rit);
        void *namePtr = temprb->collision_shape()->getBulletCollisionShape()->getUserPointer();
        MString n = MString(static_cast<char*>(namePtr));
        if (name == n) {
            rb = temprb;
            //cout<<"getPointerFromName -> rb : "<<rb<<endl;
            break;
        }
    }
    
    return rb;
}

MObject boingRbCmd::nameToNode( MString name ) {
    MSelectionList selList;
    selList.add( name );
    MObject node;
    selList.getDependNode( 0, node );
    return node;
}

MPlug boingRbCmd::nameToNodePlug( MString attrName, MObject nodeObject ) {
    MFnDependencyNode depNodeFn( nodeObject );
    MObject attrObject = depNodeFn.attribute( attrName );
    MPlug plug( nodeObject, attrObject );
    return plug;
}