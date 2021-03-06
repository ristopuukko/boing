//
//  boing.cpp
//  BulletMayaPlugin
//
//  Created by Risto Puukko on 19/06/14.
//
//

#include "boing.h"
#include "collision_shape.h"
#include "boingRBNode.h"


collision_shape_t::pointer boing::m_collision_shape;
MString boing::name;
MString boing::typeName;
MObject boing::node;
MVector boing::m_initial_velocity;
MVector boing::m_initial_position;
MVector boing::m_initial_rotation;
MVector boing::m_initial_angularvelocity;
MVector boing::m_vel;
MVector boing::m_pos;
MVector boing::m_rot;
MVector boing::m_av;
float boing::m_mass;
MStringArray boing::attrArray;
MStringArray boing::dataArray;
int boing::count;

rigid_body_t::pointer boing::m_rigid_body;


boing::boing(MObject inputShape,MString inname, MString inTypeName, MVector pos, MVector vel, MVector rot, MVector av, float mass)
{
    node = inputShape;
    name = inname;
    typeName = inTypeName;
    m_initial_velocity = vel;
    m_initial_position = pos;
    m_initial_rotation = rot;
    m_initial_angularvelocity = av;
    m_mass = mass;
    attrArray = MStringArray();
    dataArray = MStringArray();
    m_collision_shape = createCollisionShape(node);
    /*cout<<"m_collision_shape : "<<m_collision_shape<<endl;
    std::cout<<"creating a new (boing::boing) boing node : "<<inname<<" type "<<inTypeName<<"from node "<<MFnDependencyNode(inputShape).name()<<" in pos "<<m_initial_position<<" vel "<<m_initial_velocity<<" rotation "<<m_initial_rotation<<" and av "<<m_initial_angularvelocity<<endl;*/
    count++;
    std::cout<<"count = "<<count<<std::endl;
    createRigidBody();

}




boing::~boing() {
    m_collision_shape=0;
    m_rigid_body=0;
}

void * boing::getPointer() {
    return (void *) this;
}

MObject boing::nameToNode( MString name ) {
    MSelectionList selList;
    selList.add( name );
    MObject node;
    selList.getDependNode( 0, node );
    return node;
}

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


void boing::createRigidBody()
{
    //MGlobal::getActiveSelectionList(m_undoSelectionList);
    
    if (!name.length()) {
        name = "procBoingRb1";
    }
    std::cout<<"name in  boing::createRigidBody() "<<name<<endl;
    
    double mscale[3] = {1,1,1};
    MQuaternion mrotation = MEulerRotation(m_initial_rotation).asQuaternion();

    //collision_shape_t::pointer  collision_shape;
    if(!m_collision_shape) {
        //not connected to a collision shape, put a default one
        m_collision_shape = solver_t::create_box_shape();
    } else {
        if ( !node.isNull() ) {
            MFnDagNode fnDagNode(node);
            //cout<<"node : "<<fnDagNode.partialPathName()<<endl;
            MFnTransform fnTransform(fnDagNode.parent(0));
            //cout<<"MFnTransform node : "<<fnTransform.partialPathName()<<endl;
            if ( m_initial_position == MVector::zero ) {
                m_initial_position = fnTransform.getTranslation(MSpace::kTransform);
            }
            if ( m_initial_rotation == MVector::zero ) {
                fnTransform.getRotation(mrotation, MSpace::kTransform);
            }
            fnTransform.getScale(mscale);
        }
    }

    shared_ptr<solver_impl_t> solv = solver_t::get_solver();
    
    m_rigid_body = solver_t::create_rigid_body(m_collision_shape);
    
    m_rigid_body->set_transform(vec3f((float)m_initial_position.x, (float)m_initial_position.y, (float)m_initial_position.z),
                                quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));
    
    m_rigid_body->set_linear_velocity( vec3f((float)m_initial_velocity.x,(float)m_initial_velocity.y,(float)m_initial_velocity.z) );
    m_rigid_body->set_angular_velocity( vec3f((float)m_initial_angularvelocity.x,(float)m_initial_angularvelocity.y,(float)m_initial_angularvelocity.z) );
    
    
    m_rigid_body->collision_shape()->set_scale(vec3f((float)mscale[0], (float)mscale[1], (float)mscale[2]));
    
    /*
    float mass = 1.f;
    m_rigid_body->set_mass(mass);
    m_rigid_body->set_inertia((float)mass * m_rigid_body->collision_shape()->local_inertia());
    */
    
	//float mass = 0.f;
    if (typeName == boingRBNode::typeName) {
        MPlug(node, boingRBNode::ia_mass).getValue(m_mass);
    }
    
	float curMass = m_rigid_body->get_mass();
	bool changedMassStatus= false;
	if ((curMass > 0.f) != (m_mass > 0.f))
	{
		changedMassStatus = true;
	}
	if (changedMassStatus)
		solver_t::remove_rigid_body(m_rigid_body);
	
	m_rigid_body->set_mass(m_mass);
	m_rigid_body->set_inertia((float)m_mass * m_rigid_body->collision_shape()->local_inertia());
    
    
	if (changedMassStatus)
		solver_t::add_rigid_body(m_rigid_body, name.asChar());
    
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

    m_rigid_body->impl()->body()->setUserPointer((void*) this);
    
    
}


collision_shape_t::pointer boing::createCollisionShape(const MObject& node)
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


rigid_body_t::pointer boing::getPointerFromName(MString &name)
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


MStatus boing::deleteRigidBody(MString &name) {
    
    rigid_body_t::pointer rb = getPointerFromName(name);
    //boing *b = static_cast<boing *>( rb->impl()->body()->getUserPointer() );
    
    //bSolverNode::erase_node(*b);
    
    solver_t::remove_rigid_body(rb);
    
    return MS::kSuccess;
}


