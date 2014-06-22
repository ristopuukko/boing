//
//  boing.cpp
//  BulletMayaPlugin
//
//  Created by Risto Puukko on 19/06/14.
//
//

#include "boing.h"


MString boing::name;
collision_shape_t::pointer boing::m_collision_shape;


boing::boing(MString value) {
    name = value;
    attrArray = MStringArray();
    dataArray = MStringArray();
    std::cout<<"creating a new boing node : "<<name<<endl;
    
    createRigidBody(<#collision_shape_t::pointer &collision_shape#>, <#MObject &node#>, <#MString &name#>, <#MVector &vel#>, <#MVector &pos#>, <#MVector &rot#>, <#MVector &av#>)
}

boing::~boing() {}

MString boing::get_data(MString &name) {
    for ( int i=0; i<attrArray.length();++i) {
        if (attrArray[i] == name)
            return dataArray[i];
    }
    return MString("");
}

void boing::add_data(MString &attr, MString &data) {
    attrArray.append(attr);
    dataArray.append(data);
}

void boing::set_data(MString &attr, MString &data) {
    for ( int i=0; i<name.length();++i) {
        if (attrArray[i] == attr) {
            dataArray[i] == data;
            break;
        }
    }
}


MStatus boingRbCmd::setBulletVectorAttribute(MString &name, MString &attr, MVector &vec) {
    
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
            } else { // set a custom attribute
                boing *b = static_cast<boing*>( rb->impl()->body()->getUserPointer() );
                MString vecStr = "";
                vecStr.set( vec.x );
                
                vecStr += MString(",");
                vecStr += vec.y;
                vecStr += MString(",");
                vecStr += vec.z;
                b->set_data(attr, vecStr);
            }
        }
        
    }
    
    return MS::kSuccess;
    
}

MVector boingRbCmd::getBulletVectorAttribute(MString &name, MString &attr) {
    
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
            } else {
                boing *b = static_cast<boing*>( rb->impl()->body()->getUserPointer() );
                MString vecStr = b->get_data(attr);
                MStringArray vecArray = parseArguments(vecStr, ",");
                vec = MVector(vecArray[0].asDouble(), vecArray[1].asDouble(), vecArray[2].asDouble());
            }
        }
    }
    
    return vec;
    
}

MString boingRbCmd::checkCustomAttribute(MString &name, MString &attr) {
    MString result;
    
    rigid_body_t::pointer rb = getPointerFromName(name);
    boing *b = static_cast<boing*>( rb->impl()->body()->getUserPointer() );
    result = b->get_data(attr);
    return result;
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
                                    MObject &node,
                                    MString &name,
                                    MVector &vel,
                                    MVector &pos,
                                    MVector &rot,
                                    MVector &av)
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
    
    
    
    //char *rName = (char *)name.asChar();
    //bSolverNode::procRbArray.append(name);
    //void *namePtr = rName;
    
    
    
    //std::vector<boing> nodes = bSolverNode::get_nodes();
    //std::cout<<"bSolverNode::bSolverNode::myRbNodes.size() : "<<nodes.size()<<endl;
    //std::cout<<"b pointer  : "<<&b<<endl;
    shared_ptr<bSolverNode> bSolv = bSolverNode::get_bsolver_node();
    boing* b_ptr = bSolv->createNode(name);
    std::cout<<"b_ptr->name  : "<<b_ptr->name<<endl;
    m_rigid_body->collision_shape()->getBulletCollisionShape()->setUserPointer(b_ptr);
    
    m_rigid_body->set_transform(vec3f((float)pos.x, (float)pos.y, (float)pos.z),
                                quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));
    
    m_rigid_body->set_linear_velocity( vec3f((float)vel.x,(float)vel.y,(float)vel.z) );
    m_rigid_body->set_angular_velocity( vec3f((float)av.x,(float)av.y,(float)av.z) );
    
    
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
    
    rigid_body_t::pointer rb = getPointerFromName(name);
    //boing *b = static_cast<boing *>( rb->impl()->body()->getUserPointer() );
    
    //bSolverNode::erase_node(*b);
    
    solver_t::remove_rigid_body(rb);
    
    return MS::kSuccess;
}


MString boingRbCmd::checkAttribute(MString &attr) {
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


rigid_body_t::pointer boingRbCmd::getPointerFromName(MString &name)
{
    
    rigid_body_t::pointer rb = 0;
    
    shared_ptr<solver_impl_t> solv = solver_t::get_solver();
    std::set<rigid_body_t::pointer> rbds = solver_t::get_rigid_bodies();
    std::set<rigid_body_t::pointer>::iterator rit;
    for(rit=rbds.begin(); rit!=rbds.end(); ++rit) {
        rigid_body_t::pointer temprb = (*rit);
        boing *myBoingRb = static_cast<boing *>(temprb->collision_shape()->getBulletCollisionShape()->getUserPointer());
        //MString n = MString(static_cast<char*>(namePtr));
        if (name == myBoingRb->name) {
            rb = temprb;
            //cout<<"getPointerFromName -> rb : "<<rb<<endl;
            break;
        }
    }
    
    return rb;
}