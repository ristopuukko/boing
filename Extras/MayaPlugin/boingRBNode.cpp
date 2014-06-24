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
 
 
Modified by Risto Puukko <risto.puukko@gmail.com>
04/06/2014 : changed to dg node
 
*/

//boingRBNode.cpp

#include "boingRBNode.h"

#include "BulletSoftBody/btSoftBody.h"
#include <maya/MFnDependencyNode.h>
#include <maya/MPlugArray.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnTransform.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MVector.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnStringArrayData.h>
#include <maya/MStringArray.h>
#include <maya/MFnVectorArrayData.h>
#include <maya/MVectorArray.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MDagPath.h>
#include <maya/MIntArray.h>
#include <maya/MFnMesh.h>
#include "mayaUtils.h"

#include "solver.h"
#include "bSolverNode.h"

#include <iterator>
#include <vector>
#include <assert.h>
#include <shared_ptr.h>
#include "LinearMath/btGeometryUtil.h"

#define UPDATE_SHAPE 0 //future collision margin adjust

MTypeId     boingRBNode::typeId(0x10032f);
MString     boingRBNode::typeName("boingRb");

MObject     boingRBNode::ia_collisionShape;
MObject     boingRBNode::ia_solver;
MObject     boingRBNode::ia_mass;
MObject     boingRBNode::ia_restitution;
MObject     boingRBNode::ia_friction;
MObject     boingRBNode::ia_linearDamping;
MObject     boingRBNode::ia_angularDamping;
MObject     boingRBNode::ia_initialPosition;
MObject     boingRBNode::ia_initialRotation;
MObject     boingRBNode::ia_initialVelocity;
MObject     boingRBNode::ia_initialSpin;

MObject     boingRBNode::ia_position;
MObject     boingRBNode::ia_rotation;
MObject     boingRBNode::ia_rotationX;
MObject     boingRBNode::ia_rotationY;
MObject     boingRBNode::ia_rotationZ;

MObject     boingRBNode::ia_velocity;
MObject     boingRBNode::ia_angularvelocity;

MObject     boingRBNode::ia_externalForce;
MObject     boingRBNode::ia_externalTorque;

MObject     boingRBNode::worldMatrix;

MObject     boingRBNode::ca_rigidBody;
MObject     boingRBNode::ca_rigidBodyParam;
MObject     boingRBNode::ca_solver;

MObject     boingRBNode::oa_contactCount;
MObject		boingRBNode::oa_contactName;
MObject		boingRBNode::oa_contactPosition;

//collisionShapeNode
MObject     boingRBNode::ia_draw;
MObject     boingRBNode::ia_shape;
MObject     boingRBNode::ia_type;
MObject     boingRBNode::ia_scale;
MObject     boingRBNode::ca_collisionShape;
MObject     boingRBNode::ca_collisionShapeParam;
MObject     boingRBNode::oa_collisionShape;
//-----

MStatus boingRBNode::initialize()
{
    MStatus status;
    MFnMessageAttribute fnMsgAttr;
    MFnNumericAttribute fnNumericAttr;
    MFnMatrixAttribute fnMatrixAttr;
	MFnTypedAttribute typedAttr;
	MFnEnumAttribute fnEnumAttr;
    MFnUnitAttribute fnUnitAttr;
    
    
    ia_draw = fnNumericAttr.create("drawBulletShape", "dbs", MFnNumericData::kBoolean, 1.0, &status);
    MCHECKSTATUS(status, "creating drawBulletShape attribute")
    status = addAttribute(ia_draw);

    ia_collisionShape = fnMsgAttr.create("inCollisionShape", "incs", &status);
    MCHECKSTATUS(status, "creating inCollisionShape attribute")
    status = addAttribute(ia_collisionShape);
    MCHECKSTATUS(status, "adding inCollisionShape attribute")
    worldMatrix = fnMatrixAttr.create( "worldMatrix", "wm", MFnMatrixAttribute::kDouble, &status  );
    MCHECKSTATUS(status, "creating worldMatrix attribute")
    fnMatrixAttr.setStorable(false);
    fnMatrixAttr.setConnectable(true);
    status = addAttribute(worldMatrix);
    MCHECKSTATUS(status, "adding worldMatrix attribute")
    
    
    ia_solver = fnMsgAttr.create("solver", "solv", &status);
    MCHECKSTATUS(status, "creating solver attribute")
    status = addAttribute(ia_solver);
    MCHECKSTATUS(status, "adding solver attribute")

    ia_mass = fnNumericAttr.create("mass", "ma", MFnNumericData::kDouble, 1.0, &status);
    MCHECKSTATUS(status, "creating mass attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_mass);
    MCHECKSTATUS(status, "adding mass attribute")

    ia_restitution = fnNumericAttr.create("restitution", "rst", MFnNumericData::kDouble, 0.1, &status);
    MCHECKSTATUS(status, "creating restitution attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_restitution);
    MCHECKSTATUS(status, "adding restitution attribute")

    ia_friction = fnNumericAttr.create("friction", "fc", MFnNumericData::kDouble, 0.5, &status);
    MCHECKSTATUS(status, "creating friction attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_friction);
    MCHECKSTATUS(status, "adding friction attribute")

    ia_linearDamping = fnNumericAttr.create("linearDamping", "ld", MFnNumericData::kDouble, 0.3, &status);
    MCHECKSTATUS(status, "creating linearDamping attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_linearDamping);
    MCHECKSTATUS(status, "adding linearDamping attribute")

    ia_angularDamping = fnNumericAttr.create("angularDamping", "ad", MFnNumericData::kDouble, 0.3, &status);
    MCHECKSTATUS(status, "creating angularDamping attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_angularDamping);
    MCHECKSTATUS(status, "adding angularDamping attribute")

    ia_initialPosition = fnNumericAttr.createPoint("initialPosition", "inpo", &status);
    MCHECKSTATUS(status, "creating initialPosition attribute")
    status = addAttribute(ia_initialPosition);
    MCHECKSTATUS(status, "adding initialPosition attribute")

    ia_initialRotation = fnNumericAttr.createPoint("initialRotation", "inro", &status);
    MCHECKSTATUS(status, "creating initialRotation attribute")
    status = addAttribute(ia_initialRotation);
    MCHECKSTATUS(status, "adding initialRotation attribute")

    ia_initialVelocity = fnNumericAttr.createPoint("initialVelocity", "inve", &status);
    MCHECKSTATUS(status, "creating initialVelocity attribute")
    status = addAttribute(ia_initialVelocity);
    MCHECKSTATUS(status, "adding initialVelocity attribute")

    ia_initialSpin = fnNumericAttr.createPoint("initialSpin", "insp", &status);
    MCHECKSTATUS(status, "creating initialSpin attribute")
    status = addAttribute(ia_initialSpin);
    MCHECKSTATUS(status, "adding initialSpin attribute")
    
    ia_position = fnNumericAttr.createPoint("position", "pos", &status);
    MCHECKSTATUS(status, "creating position attribute")
    status = addAttribute(ia_position);
    MCHECKSTATUS(status, "adding position attribute")
    
    
    ia_rotationX = fnUnitAttr.create("rotateX", "rotx", MFnUnitAttribute::kAngle, 0.0, &status);
    ia_rotationY = fnUnitAttr.create("rotateY", "roty", MFnUnitAttribute::kAngle, 0.0, &status);
    ia_rotationZ = fnUnitAttr.create("rotateZ", "rotz", MFnUnitAttribute::kAngle, 0.0, &status);
    ia_rotation = fnNumericAttr.create("rotate", "rot", ia_rotationX, ia_rotationY, ia_rotationZ, &status);

    MCHECKSTATUS(status, "creating rotation attribute")
    status = addAttribute(ia_rotation);
    MCHECKSTATUS(status, "adding rotation attribute")
    
    
    
    ia_velocity = fnNumericAttr.createPoint("velocity", "vel", &status);
    MCHECKSTATUS(status, "creating velocity attribute")
    status = addAttribute(ia_velocity);
    MCHECKSTATUS(status, "adding velocity attribute")
    
    ia_angularvelocity = fnNumericAttr.createPoint("angularVelocity", "av", &status);
    MCHECKSTATUS(status, "creating angularVelocity attribute")
    status = addAttribute(ia_angularvelocity);
    MCHECKSTATUS(status, "adding angularVelocity attribute")

    
    

	ia_externalForce = fnNumericAttr.createPoint("externalForce", "exfo", &status);
    MCHECKSTATUS(status, "creating externalForce attribute")
    status = addAttribute(ia_externalForce);
    MCHECKSTATUS(status, "adding externalForce attribute")

	ia_externalTorque = fnNumericAttr.createPoint("externalTorque", "exto", &status);
    MCHECKSTATUS(status, "creating externalTorque attribute")
    status = addAttribute(ia_externalTorque);
    MCHECKSTATUS(status, "adding externalTorque attribute")

    ca_rigidBody = fnNumericAttr.create("ca_rigidBody", "carb", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_rigidBody attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_rigidBody);
    MCHECKSTATUS(status, "adding ca_rigidBody attribute")

    ca_rigidBodyParam = fnNumericAttr.create("ca_rigidBodyParam", "carbp", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_rigidBodyParam attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding ca_rigidBodyParam attribute")

    ca_solver = fnNumericAttr.create("ca_solver", "caso", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_solver attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_solver);
    MCHECKSTATUS(status, "adding ca_solver attribute")

	oa_contactCount = fnNumericAttr.create("contactCount", "contactCount", MFnNumericData::kInt, 0, &status);
    MCHECKSTATUS(status, "creating oa_contactCount attribute")
    fnNumericAttr.setConnectable(true);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(oa_contactCount);
    MCHECKSTATUS(status, "adding oa_contactCount attribute");

	MFnStringArrayData stringArrayData;
	oa_contactName = typedAttr.create("contactName", "contactName", MFnData::kStringArray, stringArrayData.create(), &status);
	MCHECKSTATUS(status, "creating oa_contactName attribute")
	typedAttr.setHidden(true);
	status = addAttribute(oa_contactName);
	MCHECKSTATUS(status, "adding oa_contactName attribute");

	oa_contactPosition = typedAttr.create("contactPosition", "contactPosition", MFnVectorArrayData::kVectorArray, &status);	 
	MCHECKSTATUS(status, "creating oa_contactPosition attribute")
	typedAttr.setHidden(true);
    status = addAttribute(oa_contactPosition);
	MCHECKSTATUS(status, "adding oa_contactPosition attribute");

//collisionShapeNode
    ia_type = fnEnumAttr.create("type", "tp", 7, &status);
    MCHECKSTATUS(status, "creating type attribute")
        fnEnumAttr.addField("Convex Hull", 0);
    fnEnumAttr.addField("Mesh", 1);
    fnEnumAttr.addField("Cylinder", 2);
    fnEnumAttr.addField("Capsule", 3);
    fnEnumAttr.addField("Box", 4);
    fnEnumAttr.addField("Sphere", 5);
    fnEnumAttr.addField("Plane", 6);
    fnEnumAttr.addField("BvhMesh", 7);
    fnEnumAttr.addField("HACD", 8);
    fnEnumAttr.setKeyable(true);
    fnEnumAttr.setDefault(4);
    
    status = addAttribute(ia_type);
    MCHECKSTATUS(status, "adding type attribute")

    ia_scale = fnNumericAttr.createPoint("scale", "sc", &status);
    MCHECKSTATUS(status, "creating ia_scale attribute")
    fnNumericAttr.setDefault(1.0, 1.0, 1.0);
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_scale);
    MCHECKSTATUS(status, "adding ia_scale attribute")

    oa_collisionShape = fnMsgAttr.create("outCollisionShape", "oucs", &status);
    MCHECKSTATUS(status, "creating outCollisionShape attribute")
    status = addAttribute(oa_collisionShape);
    MCHECKSTATUS(status, "adding outCollisionShape attribute")

    ia_shape = fnMsgAttr.create("inShape", "insh", &status);
    MCHECKSTATUS(status, "creating inShape attribute")
    status = addAttribute(ia_shape);
    MCHECKSTATUS(status, "adding inShape attribute")

    ca_collisionShape = fnNumericAttr.create("ca_collisionShape", "ccs", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_collisionShape attribute")
    fnNumericAttr.setWorldSpace(true);
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_collisionShape);
    MCHECKSTATUS(status, "adding ca_collisionShape attribute")

    ca_collisionShapeParam = fnNumericAttr.create("collisionShapeParam", "cspm", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_collisionShapeParam attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_collisionShapeParam);
    MCHECKSTATUS(status, "adding ca_collisionShapeParam attribute")

        //

    status = attributeAffects(ia_shape, oa_collisionShape);
    MCHECKSTATUS(status, "adding attributeAffects(ia_shape, oa_collisionShape)")

    status = attributeAffects(ia_type, oa_collisionShape);
    MCHECKSTATUS(status, "adding attributeAffects(ia_type, oa_collisionShape)")

    status = attributeAffects(ia_scale, oa_collisionShape);
    MCHECKSTATUS(status, "adding attributeAffects(ia_scale, oa_collisionShape)")

        //
    status = attributeAffects(ia_shape, ca_collisionShape);
    MCHECKSTATUS(status, "adding attributeAffects(ia_shape, ca_collisionShape)")

    status = attributeAffects(ia_type, ca_collisionShape);
    MCHECKSTATUS(status, "adding attributeAffects(ia_shape, ca_collisionShape)")

        //
    status = attributeAffects(ia_shape, ca_collisionShapeParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_shape, ca_collisionShapeParam)")

    status = attributeAffects(ia_scale, ca_collisionShapeParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_scale, ca_collisionShapeParam)")

    status = attributeAffects(ia_type, ca_collisionShapeParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_type, ca_collisionShapeParam)")

//-----



	status = attributeAffects(ia_mass, ca_rigidBody);
    MCHECKSTATUS(status, "adding attributeAffects(ia_mass, ca_rigidBodyParam)")

    status = attributeAffects(ia_collisionShape, ca_rigidBody);
    MCHECKSTATUS(status, "adding attributeAffects(ia_collisionShape, ca_rigidBody)")


    status = attributeAffects(ia_collisionShape, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_collisionShape, ca_rigidBodyParam)")

    status = attributeAffects(ia_mass, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_mass, ca_rigidBodyParam)")

    status = attributeAffects(ia_restitution, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_restitution, ca_rigidBodyParam)")

    status = attributeAffects(ia_friction, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_friction, ca_rigidBodyParam)")

    status = attributeAffects(ia_linearDamping, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_linearDamping, ca_rigidBodyParam)")

    status = attributeAffects(ia_angularDamping, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_angularDamping, ca_rigidBodyParam)")

    status = attributeAffects(ia_initialPosition, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_initialPosition, ca_rigidBodyParam)")


    status = attributeAffects(ia_velocity, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_velocity, ca_rigidBodyParam)")

    status = attributeAffects(ia_angularvelocity, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_angularvelocity, ca_rigidBodyParam)")

    
    status = attributeAffects(ia_solver, ca_solver);
    MCHECKSTATUS(status, "adding attributeAffects(ia_solver, ca_solver)")


    return MS::kSuccess;
}

boingRBNode::boingRBNode() : m_contactCount(0)
{
    // std::cout << "boingRBNode::boingRBNode" << std::endl;
}

boingRBNode::~boingRBNode()
{
    // std::cout << "boingRBNode::~boingRBNode" << std::endl;
}

void boingRBNode::nodeRemoved(MObject& node, void *clientData)
{
    std::cout << "boingRBNode::nodeRemoved" << std::endl;
    MFnDependencyNode fnNode(node);
    solver_t::remove_rigid_body(static_cast<boingRBNode*>(fnNode.userNode())->m_rigid_body);
}

void* boingRBNode::creator()
{
    return new boingRBNode();
}


bool boingRBNode::setInternalValueInContext ( const  MPlug & plug,
                                                   const  MDataHandle & dataHandle,
                                                   MDGContext & ctx)
{
   /* if ((plug == pdbFiles) || (plug == ia_scale) || (plug == ia_percent)) {
        m_framesDirty = true;
    } else if(plug == textureFiles) {
        gpufx::m_renderer.setColorTextureDirty();
    }*/
    return false; //setInternalValueInContext(plug,dataHandle,ctx);
}

static bool isCompound(const MPlug& plgInShape)
{
    MPlugArray plgaConnectedTo;
    plgInShape.connectedTo(plgaConnectedTo, true, true);
    int numSelectedShapes = plgaConnectedTo.length();

    if(numSelectedShapes > 0) {

        MObject node = plgaConnectedTo[0].node();

        MDagPath dagPath;
        MDagPath::getAPathTo(node, dagPath);
        int numChildren = dagPath.childCount();

        if (node.hasFn(MFn::kTransform))
        {
            MFnTransform trNode(dagPath);
            const char* name = trNode.name().asChar();
            printf("name = %s\n", name);

            for (int i=0;i<numChildren;i++)
            {
                MObject child = dagPath.child(i);
                if(child.hasFn(MFn::kMesh))
                {
                    return false;
                }

                if(child.hasFn(MFn::kTransform))
                {
                    MDagPath dagPath;
                    MDagPath::getAPathTo(child, dagPath);
                    MFnTransform trNode(dagPath);
                    const char* name = trNode.name().asChar();
                    printf("name = %s\n", name);

                    int numGrandChildren = dagPath.childCount();
                    {
                        for (int i=0;i<numGrandChildren;i++)
                        {
                            MObject grandchild = dagPath.child(i);
                            if(grandchild.hasFn(MFn::kMesh))
                            {
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}


collision_shape_t::pointer boingRBNode::createCollisionShape(const MObject& node)
{
    
	collision_shape_t::pointer collision_shape = 0;
    
	MObject thisObject(thisMObject());
	MPlug plgType(thisObject, ia_type);
	int type;
	plgType.getValue(type);
    
	switch(type) {
        case 0:
		{
			//convex hull
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

void boingRBNode::computeCollisionShape(const MPlug& plug, MDataBlock& data)
{
	//std::cout << collisionShapeNode::collisionMarginOffset << std::endl;
    
	MObject thisObject(thisMObject());
    
	MPlug plgInShape(thisObject, ia_shape);
    
    
	if (isCompound(plgInShape))
	{
		m_collision_shape = createCompositeShape(plgInShape);
	} else
	{
        
		MObject thisObject(thisMObject());
		MPlug plgType(thisObject, ia_type);
		int type;
		plgType.getValue(type);
        
		switch(type)
		{
            case 4:
                //box
                //cout<<"boingRBNode::computeCollisionShape creates a box!"<<endl;
                m_collision_shape = solver_t::create_box_shape();
                break;
            case 5:
                //sphere
                //cout<<"boingRBNode::computeCollisionShape creates a sphere!"<<endl;
                m_collision_shape = solver_t::create_sphere_shape();
                break;
            case 6:
                //plane
                //cout<<"boingRBNode::computeCollisionShape creates a plane!"<<endl;
                m_collision_shape = solver_t::create_plane_shape();
                break;
            
            default:
			{
                //cout<<"boingRBNode::computeCollisionShape creates a mesh!"<<endl;
				MPlugArray plgaConnectedTo;
				plgInShape.connectedTo(plgaConnectedTo, true, true);
				int numSelectedShapes = plgaConnectedTo.length();
                
				if(numSelectedShapes > 0) 
				{
                    MFnDependencyNode fnNode(plgaConnectedTo[0].node());
                    //cout<<"boingRBNode::computeCollisionShape fnNode.name : "<<fnNode.name().asChar()<<endl;
					MObject node = plgaConnectedTo[0].node();
					m_collision_shape = createCollisionShape(node);
				}
			}
		}
	}
    
    
    
	//btAssert(m_collision_shape);
    
	data.setClean(plug);
}


/*
MStatus boingRBNode::setDependentsDirty( const  MPlug & plug,  MPlugArray & plugArray)
{
    std::cout << "boingRBNode::setDependentsDirty: " << plug.name().asChar() << std::endl;
    MObject thisNode = thisMObject();
    if(plug == ia_solver) {
        MPlug plgAffected(thisNode, ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ia_collisionShape) {
        //ia_collisionShape -> ca_rigidBody
        MPlug plgAffected(thisNode, ca_rigidBody);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);

        //ia_collisionShape -> ca_rigidBodyParam
        plgAffected.setAttribute(ca_rigidBodyParam);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ca_rigidBody) {
        //ca_rigidBody -> ca_update
        MPlug plgAffected(thisNode, ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);

        //ca_rigidBody -> ca_rigidBodyParam
        plgAffected.setAttribute(ca_rigidBodyParam);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ia_mass) {
        //ia_mass -> ca_rigidBodyParam
        MPlug plgAffected(thisNode, ca_rigidBodyParam);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);

        plgAffected.setAttribute(ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ca_rigidBodyParam) {
        //ca_rigidBodyParam -> ca_update
        MPlug plgAffected(thisNode, ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    }
    return MS::kSuccess;
}*/


MStatus boingRBNode::compute(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "boingRBNode::compute | plug " << plug.name() << std::endl;
    //MTime time = data.inputValue( boingRBNode::inTime ).asTime();
    if(plug == ca_rigidBody) {
		computeOutputShape(plug, data);
        computeRigidBody(plug, data);
    } else if(plug == ca_rigidBodyParam) {
		computeCollisionShapeParam(plug, data);
        computeRigidBodyParam(plug, data);
    } else if(plug == ca_solver) {
		data.inputValue(ia_solver).asBool();
    } else if(plug.isElement()) {
        if(plug.array() == worldMatrix && plug.logicalIndex() == 0) {
            computeWorldMatrix(plug, data);
        } else {
            return MStatus::kUnknownParameter;
        }
    } else if(plug == oa_collisionShape) {
		computeOutputShape(plug, data);
	} else if(plug == ca_collisionShape) {
		computeCollisionShape(plug, data);
	} else  if(plug == ca_collisionShapeParam) {
		computeCollisionShapeParam(plug, data);
	} else {
        return MStatus::kUnknownParameter;
    }
    return MStatus::kSuccess;
}


void boingRBNode::computeOutputShape(const MPlug& plug, MDataBlock& data)
{
	//std::cout << "collisionShapeNode::computeOutputShape" << std::endl;
    
	MObject thisObject(thisMObject());
    MFnDependencyNode fnNode(thisObject);
    //cout<< fnNode.name().asChar()<<endl;
	MObject update;
	MPlug(thisObject, ca_collisionShape).getValue(update);
	MPlug(thisObject, ca_collisionShapeParam).getValue(update);
    
	data.setClean(plug);
}

void boingRBNode::computeCollisionShapeParam(const MPlug& plug, MDataBlock& data)
{
	//std::cout << "collisionShapeNode::computeCollisionShapeParam" << std::endl;
    
	MObject thisObject(thisMObject());
    
	MObject update;
	MPlug(thisObject, ia_shape).getValue(update);
	MPlug(thisObject, ia_type).getValue(update);
    
	float3& scale = data.inputValue(ia_scale).asFloat3();
    //cout<<" scale :"<<scale[0]<<" "<<scale[1]<<" "<<scale[2]<<endl;
	if (m_collision_shape)
		m_collision_shape->set_scale(vec3f(scale[0], scale[1], scale[2]));

    //MFnDependencyNode fnNode(thisObject);
    //cout<< fnNode.name().asChar()<<" -> m_collision_shape : "<<m_collision_shape<<endl;
    
	data.setClean(plug);
}

collision_shape_t::pointer boingRBNode::createCompositeShape(const MPlug& plgInShape)
{
    
	std::vector<collision_shape_t::pointer> childCollisionShapes;
	std::vector< vec3f> childPositions;
	std::vector< quatf> childOrientations;
    
	MPlugArray plgaConnectedTo;
	plgInShape.connectedTo(plgaConnectedTo, true, true);
	int numSelectedShapes = plgaConnectedTo.length();
    
	if(numSelectedShapes > 0) {
        
		MObject node = plgaConnectedTo[0].node();
        
		MDagPath dagPath;
		MDagPath::getAPathTo(node, dagPath);
		int numChildren = dagPath.childCount();
        
		if (node.hasFn(MFn::kTransform))
		{
			MFnTransform trNode(dagPath);
			MVector mtranslation = trNode.getTranslation(MSpace::kTransform);
			MObject thisObject(thisMObject());
            
			MFnDagNode fnDagNode(thisObject);
			MStatus status;
            
			MFnTransform fnParentTransform(fnDagNode.parent(0, &status));
            
			mtranslation = trNode.getTranslation(MSpace::kPostTransform);
			mtranslation = fnParentTransform.getTranslation(MSpace::kTransform);
            
			const char* name = trNode.name().asChar();
			printf("name = %s\n", name);
            
			for (int i=0;i<numChildren;i++)
			{
				MObject child = dagPath.child(i);
				if(child.hasFn(MFn::kMesh))
				{
					return false;
				}
                
				if(child.hasFn(MFn::kTransform))
				{
					MDagPath dagPath;
					MDagPath::getAPathTo(child, dagPath);
					MFnTransform trNode(dagPath);
                    
					MVector mtranslation = trNode.getTranslation(MSpace::kTransform);
					mtranslation = trNode.getTranslation(MSpace::kPostTransform);
                    
					MQuaternion mrotation;
					trNode.getRotation(mrotation, MSpace::kTransform);
					double mscale[3];
					trNode.getScale(mscale);
                    
					vec3f childPos((float)mtranslation.x, (float)mtranslation.y, (float)mtranslation.z);
					quatf childOrn((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z);
                    
					const char* name = trNode.name().asChar();
					printf("name = %s\n", name);
                    
					int numGrandChildren = dagPath.childCount();
					{
						for (int i=0;i<numGrandChildren;i++)
						{
							MObject grandchild = dagPath.child(i);
							if(grandchild.hasFn(MFn::kMesh))
							{
                                
								collision_shape_t::pointer childShape = createCollisionShape(grandchild);
								if (childShape)
								{
									childCollisionShapes.push_back(childShape);
									childPositions.push_back(childPos);
									childOrientations.push_back(childOrn);
								}
							}
						}
					}
				}
			}
		}
	}
    
	if (childCollisionShapes.size()>0)
	{
        
		composite_shape_t::pointer composite = solver_t::create_composite_shape(
                                                                                &childCollisionShapes[0],
                                                                                &childPositions[0],
                                                                                &childOrientations[0],
                                                                                childCollisionShapes.size()
                                                                                );
		return composite;
	}
	return 0;
}


/*
void boingRBNode::draw( M3dView & view, const MDagPath &path,
                             M3dView::DisplayStyle style,
                             M3dView::DisplayStatus status )
{
	
  //  std::cout << "boingRBNode::draw" << std::endl;
	MObject thisObject(thisMObject());
    update();
	
    view.beginGL();
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    if(m_rigid_body) {
        //remove the scale, since it's already included in the node transform 
        vec3f scale;
        m_rigid_body->collision_shape()->get_scale(scale);

        glPushMatrix();
        glScalef(1/scale[0], 1/scale[1], 1/scale[2]); 
    
        if(style == M3dView::kFlatShaded || style == M3dView::kGouraudShaded) {
            glEnable(GL_LIGHTING);
			MPlug plug(thisObject, boingRBNode::ia_mass);
			float mass;
			plug.getValue(mass);
			if (mass) {
				float material[] = { 0.2f, 1.0f, 0.2f, 1.0f };
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material);
			} else {
				float material[] = { 1.0f, 0.3f, 0.1f, 1.0f };
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material);
			}
            //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material);
            m_rigid_body->collision_shape()->gl_draw(collision_shape_t::kDSSolid);
        } 
    
    
        if( status == M3dView::kActive ||
            status == M3dView::kLead ||
            status == M3dView::kHilite ||
            ( style != M3dView::kGouraudShaded && style != M3dView::kFlatShaded ) ) {
            
            glDisable(GL_LIGHTING);
            m_rigid_body->collision_shape()->gl_draw(collision_shape_t::kDSWireframe);
    
        }
        glPopMatrix();
    }
    glPopAttrib();
    view.endGL();
}
 */
 
 /*
bool boingRBNode::isBounded() const
{
    //return true;
    return false;
}

  */
  
  /*
MBoundingBox boingRBNode::boundingBox() const
{
    // std::cout << "boingRBNode::boundingBox()" << std::endl;
    //load the pdbs
    MObject node = thisMObject();

    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

   */
   
void boingRBNode::destroyRigidBody()
{
	if (m_rigid_body)
	{
        //boing *b_ptr = static_cast<boing *>( m_rigid_body->impl()->body()->getUserPointer() );
        //b->destroy();
        //bSolverNode::erase_node(b);
		solver_t::remove_rigid_body(m_rigid_body);
//		where to remove constraints?
		m_rigid_body->remove_all_constraints();
		m_rigid_body = 0;
	}
}

//standard attributes
/*
Rigid bodies are added to the solver here
*/
void boingRBNode::computeRigidBody(const MPlug& plug, MDataBlock& data)
{

    MObject thisObject(thisMObject());
    MPlug plgCollisionShape(thisObject, ia_collisionShape);
    MObject update;
    //force evaluation of the shape
    plgCollisionShape.getValue(update);
	
	// The following two variables are not actually used!
    vec3f prevCenter(0, 0, 0);
    quatf prevRotation(qidentity<float>());
    if(m_rigid_body) {
        prevCenter = m_rigid_body->collision_shape()->center();
        prevRotation = m_rigid_body->collision_shape()->rotation();
    }

    computeCollisionShape(plug, data);
    
    //collision_shape_t::pointer  collision_shape ;
	
    /*
    if(plgCollisionShape.isConnected()) {
        MPlugArray connections;
        plgCollisionShape.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == collisionShapeNode::typeId) {
                collisionShapeNode *pCollisionShapeNode = static_cast<collisionShapeNode*>(fnNode.userNode());
                collision_shape = pCollisionShapeNode->collisionShape();
            } else {
                std::cout << "boingRBNode connected to a non-collision shape node!" << std::endl;
            }
        }
    }*/

    //cout << "m_collision_shape : "<<m_collision_shape<<endl;
    
    if(!m_collision_shape) {
        //not connected to a collision shape, put a default one
        m_collision_shape = solver_t::create_sphere_shape();
    }
	
    //cout<<"removing m_rigid_body"<<endl;
	solver_t::remove_rigid_body(m_rigid_body);
    m_rigid_body = solver_t::create_rigid_body(m_collision_shape);
    solver_t::add_rigid_body(m_rigid_body,name().asChar());
    MString rbname = name();
    
    // once at creation/load time : get transform from Maya transform node
    
    MPlugArray conn;
    
    MPlug(thisObject, ia_shape).connectedTo(conn, true, false);
    MObject connObj(conn[0].node());
    
	MFnDagNode fnDagNode(connObj);
    
    //MFnDependencyNode fnNode(thisObject);
    
    MFnTransform fnTransform(fnDagNode.parent(0));
    MVector mtranslation = fnTransform.getTranslation(MSpace::kTransform);
    MString name = MFnDependencyNode(thisObject).name();
    
    MQuaternion mrotation;
    MEulerRotation eurotation;
    fnTransform.getRotation(mrotation, MSpace::kTransform);
    fnTransform.getRotation(eurotation);//, MSpace::kTransform);
    MVector rot = eurotation.asVector();
	double mscale[3];
    fnTransform.getScale(mscale);
    
    
    MPlug ilv(thisObject, boingRBNode::ia_initialVelocity);
	MDataHandle hInitLinVel= ilv.asMDataHandle();
    MVector initLinVel = hInitLinVel.asVector();
    MPlug iav(thisObject, boingRBNode::ia_initialSpin);
	MDataHandle hInitAngVel= iav.asMDataHandle();
    MVector initAngVel = hInitAngVel.asVector();
	//float3 &initLinVel= hInitLinVel.asFloat3();
	//vec3f lv(initLinVel[0],initLinVel[1],initLinVel[2]);
	//m_rigid_body->set_linear_velocity(lv);

    
	m_rigid_body->set_transform(vec3f((float)mtranslation.x, (float)mtranslation.y, (float)mtranslation.z),
								quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));
    m_rigid_body->collision_shape()->set_scale(vec3f((float)mscale[0], (float)mscale[1], (float)mscale[2]));

    m_rigid_body->set_linear_velocity(vec3f((float)initLinVel.x,(float)initLinVel.y,(float)initLinVel.z));
    m_rigid_body->set_angular_velocity(vec3f((float)initAngVel.x,(float)initAngVel.y,(float)initAngVel.z));
    
	float mass = 0.f;
	MPlug(thisObject, boingRBNode::ia_mass).getValue(mass);

	float curMass = m_rigid_body->get_mass();
	bool changedMassStatus= false;
	if ((curMass > 0.f) != (mass > 0.f))
	{
		changedMassStatus = true;
	}
	if (changedMassStatus)
		solver_t::remove_rigid_body(m_rigid_body);
	
	m_rigid_body->set_mass(mass);
	m_rigid_body->set_inertia((float)mass * m_rigid_body->collision_shape()->local_inertia());


	if (changedMassStatus)
		solver_t::add_rigid_body(m_rigid_body, (const char*)rbname.asChar());

	//initialize those default values too
	float restitution = 0.f;
	MPlug(thisObject, boingRBNode::ia_restitution).getValue(restitution);
	m_rigid_body->set_restitution(restitution);
	float friction = 0.5f;
	MPlug(thisObject, boingRBNode::ia_friction).getValue(friction);
    m_rigid_body->set_friction(friction);
    float linDamp = 0.f;
	MPlug(thisObject, boingRBNode::ia_linearDamping).getValue(linDamp);
	m_rigid_body->set_linear_damping(linDamp);
    float angDamp = 0.f;
	MPlug(thisObject, boingRBNode::ia_angularDamping).getValue(angDamp);
	m_rigid_body->set_angular_damping(angDamp);

    bSolverNode::m_custom_data *d = new bSolverNode::m_custom_data;
    
    d->node = thisObject;
    d->name = rbname;
    d->typeName = typeName;
    d->m_initial_velocity = initLinVel;
    d->m_initial_position = mtranslation;
    d->m_initial_rotation = rot;
    d->m_initial_angularvelocity = initAngVel;
    d->m_mass = mass;
    d->attrArray = MStringArray();
    d->dataArray = MStringArray();
    d->m_collision_shape = m_collision_shape;
    d->m_rigid_body = m_rigid_body;
    d->m_rigid_body->impl()->body()->setUserPointer((void*) d);
    shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
    b_solv.get()->insertData(rbname, d);

	data.outputValue(ca_rigidBody).set(true);
    
    data.setClean(plug);
}


void boingRBNode::computeWorldMatrix(const MPlug& plug, MDataBlock& data)
{
	if (!m_rigid_body)
		return;

    std::cout << "boingRBNode::computeWorldMatrix" << std::endl;
    MObject thisObject(thisMObject());
    //MFnDagNode fnDagNode(thisObject);
    MFnDependencyNode fnNode(thisObject);

    MObject update;
    MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    
    vec3f pos;
    quatf rot;

    MStatus status;

    //MFnTransform fnParentTransform(fnDagNode.parent(0, &status));

	float3& mscale = data.inputValue(ia_scale).asFloat3();
    MVector scale;
    //fnParentTransform.getScale(mscale);
	m_rigid_body->get_transform(pos, rot);
	
	if(bSolverNode::isStartTime)
	{ // allow to edit ptranslation and rotation
		//MVector mtranslation = fnParentTransform.getTranslation(MSpace::kTransform, &status);
		//MQuaternion mrotation;
		//fnParentTransform.getRotation(mrotation, MSpace::kTransform);
		MVector mtranslation = data.inputValue(ia_position).asVector();

        MVector rotation = data.inputValue(ia_rotation).asVector();
        MEulerRotation meuler = MEulerRotation(rotation, MEulerRotation::kXYZ);
		MQuaternion mrotation = meuler.asQuaternion();

		float deltaPX = (float)mtranslation.x - pos[0];
		float deltaPY = (float)mtranslation.y - pos[1];
		float deltaPZ = (float)mtranslation.z - pos[2];
		float deltaRX = (float)mrotation.x - rot[1];
		float deltaRY = (float)mrotation.y - rot[2];
		float deltaRZ = (float)mrotation.z - rot[3];
		float deltaRW = (float)mrotation.w - rot[0];
		float deltaSq = deltaPX * deltaPX + deltaPY * deltaPY  + deltaPZ * deltaPZ 
						+ deltaRX * deltaRX + deltaRY * deltaRY + deltaRZ * deltaRZ + deltaRW * deltaRW;
		if(deltaSq > FLT_EPSILON)
		{
			m_rigid_body->set_transform(vec3f((float)mtranslation.x, (float)mtranslation.y, (float)mtranslation.z),
										quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));
			m_rigid_body->set_interpolation_transform(vec3f((float)mtranslation.x, (float)mtranslation.y, (float)mtranslation.z),
														quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));
			m_rigid_body->update_constraint();
				MDataHandle hInitPos = data.outputValue(ia_initialPosition);
				float3 &ihpos = hInitPos.asFloat3();
				ihpos[0] = (float)mtranslation.x;
				ihpos[1] = (float)mtranslation.y;
				ihpos[2] = (float)mtranslation.z;
				MDataHandle hInitRot = data.outputValue(ia_initialRotation);
				float3 &ihrot = hInitRot.asFloat3();
				MEulerRotation newrot(mrotation.asEulerRotation());
				ihrot[0] = rad2deg((float)newrot.x);
				ihrot[1] = rad2deg((float)newrot.y);
				ihrot[2] = rad2deg((float)newrot.z);
		}
	}
	else
	{ // if not start time, lock position and rotation for active rigid bodies
        float mass = 0.f;
		MPlug(thisObject, boingRBNode::ia_mass).getValue(mass);
        if(mass > 0.f) 
		{
            MPlug plgPos(thisObject, ia_position);
            MPlug plgRot(thisObject, ia_rotation);
			//fnParentTransform.setTranslation(MVector(pos[0], pos[1], pos[2]), MSpace::kTransform);
			//fnParentTransform.setRotation(MQuaternion(rot[1], rot[2], rot[3], rot[0]));
            plgPos.child(0).setValue(pos[0]);
            plgPos.child(1).setValue(pos[1]);
            plgPos.child(2).setValue(pos[2]);
            
            MVector r = (MQuaternion(rot[1], rot[2], rot[3], rot[0]).asEulerRotation()).asVector();
            
            plgRot.child(0).setValue(r[0]);
            plgRot.child(1).setValue(r[1]);
            plgRot.child(2).setValue(r[2]);
		}
	}

	float mass = 0.f;
	MPlug(thisObject, boingRBNode::ia_mass).getValue(mass);

	float curMass = m_rigid_body->get_mass();
	bool changedMassStatus= false;
	if ((curMass > 0.f) != (mass > 0.f))
	{
		changedMassStatus = true;
	}
	if (changedMassStatus)
		solver_t::remove_rigid_body(m_rigid_body);
	
	m_rigid_body->set_mass(mass);
	m_rigid_body->set_inertia((float)mass * m_rigid_body->collision_shape()->local_inertia());

	if (changedMassStatus)
		solver_t::remove_rigid_body(m_rigid_body);

	float restitution = 0.f;
	 MPlug(thisObject, boingRBNode::ia_restitution).getValue(restitution);
	 m_rigid_body->set_restitution(restitution);
	 float friction = 0.5f;
	  MPlug(thisObject, boingRBNode::ia_friction).getValue(friction);
    m_rigid_body->set_friction(friction);
    float linDamp = 0.f;
	  MPlug(thisObject, boingRBNode::ia_linearDamping).getValue(linDamp);
	m_rigid_body->set_linear_damping(linDamp);
    float angDamp = 0.f;
	  MPlug(thisObject, boingRBNode::ia_angularDamping).getValue(angDamp);
	m_rigid_body->set_angular_damping(angDamp);

    data.setClean(plug);
    //set the scale to the collision shape
    m_rigid_body->collision_shape()->set_scale(vec3f((float)mscale[0], (float)mscale[1], (float)mscale[2]));
}


void boingRBNode::computeRigidBodyParam(const MPlug& plug, MDataBlock& data)
{
	//std::cout << "(boingRBNode::computeRigidBodyParam) | " << std::endl;
    //cout << "m_rigid_body : "<<m_rigid_body<<endl;

	if (!m_rigid_body)
		return;

    MObject thisObject(thisMObject());
    MObject update;
    
    MPlug(thisObject, ca_rigidBody).getValue(update);
    double mass = data.inputValue(ia_mass).asDouble();

	bool changedMassStatus= false;
    
	float curMass = m_rigid_body->get_mass();
	if ((curMass > 0.f) != (mass > 0.f))
	{
		changedMassStatus = true;
	}
	if (changedMassStatus)
		solver_t::remove_rigid_body(m_rigid_body);

	m_rigid_body->set_mass((float)mass);
	m_rigid_body->set_inertia((float)mass * m_rigid_body->collision_shape()->local_inertia());

	if (changedMassStatus)
		solver_t::add_rigid_body(m_rigid_body,name().asChar());

	m_rigid_body->set_restitution((float)data.inputValue(ia_restitution).asDouble());
    m_rigid_body->set_friction((float)data.inputValue(ia_friction).asDouble());
    m_rigid_body->set_linear_damping((float)data.inputValue(ia_linearDamping).asDouble());
    m_rigid_body->set_angular_damping((float)data.inputValue(ia_angularDamping).asDouble());


    //MVector vel = data.inputValue(ia_velocity).asVector();
    //m_rigid_body->set_linear_velocity(vec3f(vel.x,vel.y,vel.z) );
    //MVector av = data.inputValue(ia_angularvelocity).asVector();
    //m_rigid_body->set_angular_velocity(vec3f(av.x,av.y,av.z) );

    data.outputValue(ca_rigidBodyParam).set(true);
    data.setClean(plug);
}

#if UPDATE_SHAPE //future collision margin adjust
void boingRBNode::updateShape(const MPlug& plug, MDataBlock& data, float& collisionMarginOffset)
{
	MObject thisObject(thisMObject());
    MPlug plgCollisionShape(thisObject, ia_collisionShape);
    MObject update;
    //force evaluation of the shape
    plgCollisionShape.getValue(update);

   /* vec3f prevCenter(0, 0, 0);
    quatf prevRotation(qidentity<float>());
    if(m_rigid_body) {
        prevCenter = m_rigid_body->collision_shape()->center();
        prevRotation = m_rigid_body->collision_shape()->rotation();
    }*/

    collision_shape_t::pointer  collision_shape;
    
	
    if(plgCollisionShape.isConnected()) {
        MPlugArray connections;
        plgCollisionShape.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == collisionShapeNode::typeId) {
				collisionShapeNode *pCollisionShapeNode = static_cast<collisionShapeNode*>(fnNode.userNode());
				pCollisionShapeNode->setCollisionMarginOffset(collisionMarginOffset); //mb
                collision_shape = pCollisionShapeNode->collisionShape();
                char * name;
                m_collision_shape->get_user_pointer(name);
                cout<<"collision_shape->get_user_pointer() : "<<name<<endl;
            } else {
                std::cout << "boingRBNode connected to a non-collision shape node!" << std::endl;
            }
        }
    }

	data.outputValue(ca_rigidBody).set(true);
    data.setClean(plug);
}
#endif

rigid_body_t::pointer boingRBNode::rigid_body()
{

    MObject thisObject(thisMObject());
    MObject update;
    //std::cout<<"going to boingRBNode::computeRigidBody."<<std::endl;
    MPlug(thisObject, ca_rigidBody).getValue(update);
    //std::cout<<"going to boingRBNode::computeRigidBodyParam."<<std::endl;
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    
    return m_rigid_body;
    
} 

void boingRBNode::update()
{
    MObject thisObject(thisMObject());

    MObject update;

	MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    MPlug(thisObject, ca_solver).getValue(update);
    MPlug(thisObject, worldMatrix).elementByLogicalIndex(0).getValue(update);
}

void boingRBNode::clearContactInfo()
{
	MObject thisObject(thisMObject());

	// contactCount
	m_contactCount = 0;
	MPlug plugContactCount(thisObject, boingRBNode::oa_contactCount);
	plugContactCount.setValue(m_contactCount);
	
	// contactName
	MStringArray stringArray;
	stringArray.clear();
	MFnStringArrayData stringArrayData;
	MObject strArrObject = stringArrayData.create(stringArray);

	MPlug plugContactName(thisObject, boingRBNode::oa_contactName);

	if ( !plugContactName.isNull() )
		plugContactName.setValue(strArrObject);

	// contactPosition
	MPlug plugContactPosition(thisObject, boingRBNode::oa_contactPosition);
	//bool isArray = plugContactPosition.isArray();
				
	MVectorArray vectorArray;
	vectorArray.clear();

	MFnVectorArrayData vectorArrayData;
	MObject arrObject = vectorArrayData.create(vectorArray);		

	if ( !plugContactPosition.isNull() )
		plugContactPosition.setValue(arrObject);
}

void boingRBNode::addContactInfo(const MString& contactObjectName, const MVector& point)
{
	MObject thisObject(thisMObject());

	// contactCount
	m_contactCount++;
	MPlug plugContactCount(thisObject, boingRBNode::oa_contactCount);
	plugContactCount.setValue(m_contactCount);
	
	// contactName
	MPlug plugContactName(thisObject, boingRBNode::oa_contactName);

	if ( !plugContactName.isNull() )
	{
		MObject strArrObject;
		plugContactName.getValue(strArrObject);

		MFnStringArrayData stringArrayData(strArrObject);
		MStringArray stringArray = stringArrayData.array();

		stringArray.append(contactObjectName);

		MFnStringArrayData newStringArrayData;
		MObject newStrArrObject = newStringArrayData.create(stringArray);
	
		plugContactName.setValue(newStrArrObject);
	}

	// contactPosition
	MPlug plugContactPosition(thisObject, boingRBNode::oa_contactPosition);

	if ( !plugContactPosition.isNull() )
	{
		MObject arrObject;
		plugContactPosition.getValue(arrObject);	

		MFnVectorArrayData vectorArrayData(arrObject);
		MVectorArray vectorArray = vectorArrayData.array();

		vectorArray.append(point);
	
		MFnVectorArrayData newVectorArrayData;
		MObject newArrObject = newVectorArrayData.create(vectorArray);	
	
		plugContactPosition.setValue(newArrObject);
	}
}

