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
*/

//rigidBodyNode.cpp

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
#include <maya/MFnStringArrayData.h>
#include <maya/MStringArray.h>
#include <maya/MFnVectorArrayData.h>
#include <maya/MVectorArray.h>

#include "rigidBodyNode.h"
#include "collisionShapeNode.h"
#include "mayaUtils.h"

#include "solver.h"
#include "dSolverNode.h"

#define UPDATE_SHAPE 0 //future collision margin adjust

MTypeId     rigidBodyNode::typeId(0x10032f);
MString     rigidBodyNode::typeName("dRigidBody");

MObject     rigidBodyNode::ia_collisionShape;
MObject     rigidBodyNode::ia_solver;
MObject     rigidBodyNode::ia_mass;
MObject     rigidBodyNode::ia_restitution;
MObject     rigidBodyNode::ia_friction;
MObject     rigidBodyNode::ia_linearDamping;
MObject     rigidBodyNode::ia_angularDamping;
MObject     rigidBodyNode::ia_initialPosition;
MObject     rigidBodyNode::ia_initialRotation;
MObject     rigidBodyNode::ia_initialVelocity;
MObject     rigidBodyNode::ia_initialSpin;

MObject     rigidBodyNode::ia_velocity;
MObject     rigidBodyNode::ia_angularvelocity;

MObject     rigidBodyNode::ia_externalForce;
MObject     rigidBodyNode::ia_externalTorque;

MObject     rigidBodyNode::ca_rigidBody;
MObject     rigidBodyNode::ca_rigidBodyParam;
MObject     rigidBodyNode::ca_solver;

MObject     rigidBodyNode::oa_contactCount;
MObject		rigidBodyNode::oa_contactName;
MObject		rigidBodyNode::oa_contactPosition;

MStatus rigidBodyNode::initialize()
{
    MStatus status;
    MFnMessageAttribute fnMsgAttr;
    MFnNumericAttribute fnNumericAttr;
    MFnMatrixAttribute fnMatrixAttr;
	MFnTypedAttribute typedAttr;

    ia_collisionShape = fnMsgAttr.create("inCollisionShape", "incs", &status);
    MCHECKSTATUS(status, "creating inCollisionShape attribute")
    status = addAttribute(ia_collisionShape);
    MCHECKSTATUS(status, "adding inCollisionShape attribute")

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

rigidBodyNode::rigidBodyNode() : m_contactCount(0)
{
    // std::cout << "rigidBodyNode::rigidBodyNode" << std::endl;
}

rigidBodyNode::~rigidBodyNode()
{
    // std::cout << "rigidBodyNode::~rigidBodyNode" << std::endl;
}

void rigidBodyNode::nodeRemoved(MObject& node, void *clientData)
{
   // std::cout << "rigidBodyNode::nodeRemoved" << std::endl;
    MFnDependencyNode fnNode(node);
    solver_t::remove_rigid_body(static_cast<rigidBodyNode*>(fnNode.userNode())->m_rigid_body);
}

void* rigidBodyNode::creator()
{
    return new rigidBodyNode();
}


bool rigidBodyNode::setInternalValueInContext ( const  MPlug & plug,
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

/*
MStatus rigidBodyNode::setDependentsDirty( const  MPlug & plug,  MPlugArray & plugArray)
{
    std::cout << "rigidBodyNode::setDependentsDirty: " << plug.name().asChar() << std::endl;
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


MStatus rigidBodyNode::compute(const MPlug& plug, MDataBlock& data)
{
    // std::cout << "rigidBodyNode::compute | plug " << plug.name() << std::endl;
    //MTime time = data.inputValue( rigidBodyNode::inTime ).asTime();
    if(plug == ca_rigidBody) {
        computeRigidBody(plug, data);
    } else if(plug == ca_rigidBodyParam) {
        computeRigidBodyParam(plug, data);
    } else if(plug == ca_solver) {
		data.inputValue(ia_solver).asBool();
    } else if(plug.isElement()) {
        if(plug.array() == worldMatrix && plug.logicalIndex() == 0) {
            computeWorldMatrix(plug, data);
        } else {
            return MStatus::kUnknownParameter;
        }
    } else {
        return MStatus::kUnknownParameter;
    }
    return MStatus::kSuccess;
}

void rigidBodyNode::draw( M3dView & view, const MDagPath &path,
                             M3dView::DisplayStyle style,
                             M3dView::DisplayStatus status )
{
	
  //  std::cout << "rigidBodyNode::draw" << std::endl;
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
			MPlug plug(thisObject, rigidBodyNode::ia_mass);
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

bool rigidBodyNode::isBounded() const
{
    //return true;
    return false;
}

MBoundingBox rigidBodyNode::boundingBox() const
{
    // std::cout << "rigidBodyNode::boundingBox()" << std::endl;
    //load the pdbs
    MObject node = thisMObject();

    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

void rigidBodyNode::destroyRigidBody()
{
	if (m_rigid_body)
	{
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
void rigidBodyNode::computeRigidBody(const MPlug& plug, MDataBlock& data)
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

    collision_shape_t::pointer  collision_shape;
	
    if(plgCollisionShape.isConnected()) {
        MPlugArray connections;
        plgCollisionShape.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == collisionShapeNode::typeId) {
                collisionShapeNode *pCollisionShapeNode = static_cast<collisionShapeNode*>(fnNode.userNode());
                collision_shape = pCollisionShapeNode->collisionShape();
            } else {
                std::cout << "rigidBodyNode connected to a non-collision shape node!" << std::endl;
            }
        }
    }

    if(!collision_shape) {
        //not connected to a collision shape, put a default one
        collision_shape = solver_t::create_sphere_shape();
    }	
    cout<<"removing m_rigid_body"<<endl;
	solver_t::remove_rigid_body(m_rigid_body);
    m_rigid_body = solver_t::create_rigid_body(collision_shape);
    solver_t::add_rigid_body(m_rigid_body,name().asChar());
// once at creation/load time : get transform from Maya transform node
    
	MFnDagNode fnDagNode(thisObject);
    MFnTransform fnTransform(fnDagNode.parent(0));
    MVector mtranslation = fnTransform.getTranslation(MSpace::kTransform);

    MQuaternion mrotation;
    fnTransform.getRotation(mrotation, MSpace::kTransform);
	double mscale[3];
    fnTransform.getScale(mscale);
	
	m_rigid_body->set_transform(vec3f((float)mtranslation.x, (float)mtranslation.y, (float)mtranslation.z),
								quatf((float)mrotation.w, (float)mrotation.x, (float)mrotation.y, (float)mrotation.z));
    m_rigid_body->collision_shape()->set_scale(vec3f((float)mscale[0], (float)mscale[1], (float)mscale[2]));


	float mass = 0.f;
	MPlug(thisObject, rigidBodyNode::ia_mass).getValue(mass);

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
		solver_t::add_rigid_body(m_rigid_body, name().asChar());

	//initialize those default values too
	float restitution = 0.f;
	MPlug(thisObject, rigidBodyNode::ia_restitution).getValue(restitution);
	m_rigid_body->set_restitution(restitution);
	float friction = 0.5f;
	MPlug(thisObject, rigidBodyNode::ia_friction).getValue(friction);
    m_rigid_body->set_friction(friction);
    float linDamp = 0.f;
	MPlug(thisObject, rigidBodyNode::ia_linearDamping).getValue(linDamp);
	m_rigid_body->set_linear_damping(linDamp);
    float angDamp = 0.f;
	MPlug(thisObject, rigidBodyNode::ia_angularDamping).getValue(angDamp);
	m_rigid_body->set_angular_damping(angDamp);

	/*
	//this is not necessary, initialize linear/angular velocity (spin) is already set at initRigidBodyArray in dSolverNode.cpp
	MPlug ilv(thisObject, rigidBodyNode::ia_initialVelocity);
	MDataHandle hInitLinVel= ilv.asMDataHandle();
	float3 &initLinVel= hInitLinVel.asFloat3();
	vec3f lv(initLinVel[0],initLinVel[1],initLinVel[2]);
	m_rigid_body->set_linear_velocity(lv);
	*/

	//data.outputValue(ca_rigidBody).set(true);
    data.setClean(plug);
}


void rigidBodyNode::computeWorldMatrix(const MPlug& plug, MDataBlock& data)
{
	if (!m_rigid_body)
		return;

  //  std::cout << "rigidBodyNode::computeWorldMatrix" << std::endl;
    MObject thisObject(thisMObject());
    MFnDagNode fnDagNode(thisObject);

    MObject update;
    MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    
    vec3f pos;
    quatf rot;

    MStatus status;

    MFnTransform fnParentTransform(fnDagNode.parent(0, &status));

	double mscale[3];
    fnParentTransform.getScale(mscale);
	m_rigid_body->get_transform(pos, rot);
	
	
	if(dSolverNode::isStartTime)
	{ // allow to edit ptranslation and rotation
		MVector mtranslation = fnParentTransform.getTranslation(MSpace::kTransform, &status);
		MQuaternion mrotation;
		fnParentTransform.getRotation(mrotation, MSpace::kTransform);

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
		MPlug(thisObject, rigidBodyNode::ia_mass).getValue(mass);
        if(mass > 0.f) 
		{
			fnParentTransform.setTranslation(MVector(pos[0], pos[1], pos[2]), MSpace::kTransform);
			fnParentTransform.setRotation(MQuaternion(rot[1], rot[2], rot[3], rot[0])); 
		}
	}

	float mass = 0.f;
	MPlug(thisObject, rigidBodyNode::ia_mass).getValue(mass);

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
	 MPlug(thisObject, rigidBodyNode::ia_restitution).getValue(restitution);
	 m_rigid_body->set_restitution(restitution);
	 float friction = 0.5f;
	  MPlug(thisObject, rigidBodyNode::ia_friction).getValue(friction);
    m_rigid_body->set_friction(friction);
    float linDamp = 0.f;
	  MPlug(thisObject, rigidBodyNode::ia_linearDamping).getValue(linDamp);
	m_rigid_body->set_linear_damping(linDamp);
    float angDamp = 0.f;
	  MPlug(thisObject, rigidBodyNode::ia_angularDamping).getValue(angDamp);
	m_rigid_body->set_angular_damping(angDamp);

    data.setClean(plug);
    //set the scale to the collision shape
    m_rigid_body->collision_shape()->set_scale(vec3f((float)mscale[0], (float)mscale[1], (float)mscale[2]));
}


void rigidBodyNode::computeRigidBodyParam(const MPlug& plug, MDataBlock& data)
{
	// std::cout << "(rigidBodyNode::computeRigidBodyParam) | " << std::endl;
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
void rigidBodyNode::updateShape(const MPlug& plug, MDataBlock& data, float& collisionMarginOffset)
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
            } else {
                std::cout << "rigidBodyNode connected to a non-collision shape node!" << std::endl;
            }
        }
    }

	data.outputValue(ca_rigidBody).set(true);
    data.setClean(plug);
}
#endif

rigid_body_t::pointer rigidBodyNode::rigid_body()
{
 //   std::cout << "rigidBodyNode::rigid_body" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);

    return m_rigid_body;
} 

void rigidBodyNode::update()
{
    MObject thisObject(thisMObject());

    MObject update;

	MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    MPlug(thisObject, ca_solver).getValue(update);
    MPlug(thisObject, worldMatrix).elementByLogicalIndex(0).getValue(update);
}

void rigidBodyNode::clearContactInfo()
{
	MObject thisObject(thisMObject());

	// contactCount
	m_contactCount = 0;
	MPlug plugContactCount(thisObject, rigidBodyNode::oa_contactCount);
	plugContactCount.setValue(m_contactCount);
	
	// contactName
	MStringArray stringArray;
	stringArray.clear();
	MFnStringArrayData stringArrayData;
	MObject strArrObject = stringArrayData.create(stringArray);

	MPlug plugContactName(thisObject, rigidBodyNode::oa_contactName);

	if ( !plugContactName.isNull() )
		plugContactName.setValue(strArrObject);

	// contactPosition
	MPlug plugContactPosition(thisObject, rigidBodyNode::oa_contactPosition);
	bool isArray = plugContactPosition.isArray();
				
	MVectorArray vectorArray;
	vectorArray.clear();

	MFnVectorArrayData vectorArrayData;
	MObject arrObject = vectorArrayData.create(vectorArray);		

	if ( !plugContactPosition.isNull() )
		plugContactPosition.setValue(arrObject);
}

void rigidBodyNode::addContactInfo(const MString& contactObjectName, const MVector& point)
{
	MObject thisObject(thisMObject());

	// contactCount
	m_contactCount++;
	MPlug plugContactCount(thisObject, rigidBodyNode::oa_contactCount);
	plugContactCount.setValue(m_contactCount);
	
	// contactName
	MPlug plugContactName(thisObject, rigidBodyNode::oa_contactName);

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
	MPlug plugContactPosition(thisObject, rigidBodyNode::oa_contactPosition);

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

