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

Modified by Dongsoo Han <dongsoo.han@amd.com>
04/11/2012 : Added contactCount, contactName and contactPosition attributes and void clearContactInfo(), void addContactInfo(..) 
			 in order to provide contact information through MEL script.
 
Modified by Risto Puukko <risto.puukko@gmail.com>
04/06/2014 : changed to dg node
 
 
*/

//boingRBNode.h

#ifndef DYN_BOING_RB_NODE_H
#define DYN_BOING_RB_NODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
//#include <maya/MPxLocatorNode.h>
#include <maya/MPxNode.h>
#include <maya/MFnEnumAttribute.h>

#include "collision_shape.h"

#include "solver.h"

class boingRBNode: public MPxNode
{
public:
    boingRBNode();
    virtual ~boingRBNode();

    virtual bool        setInternalValueInContext ( const  MPlug & plug,
                                                    const  MDataHandle & dataHandle,
                                                    MDGContext & ctx);

  //  virtual MStatus  	setDependentsDirty ( const  MPlug & plug,  MPlugArray & plugArray);

    virtual MStatus     compute( const MPlug& plug, MDataBlock& data );

  /*  virtual void        draw( M3dView & view, const MDagPath & path,
                              M3dView::DisplayStyle style,
                              M3dView::DisplayStatus status );
*/

/*    virtual bool            isBounded() const ;*/
/*    virtual MBoundingBox    boundingBox() const;*/

/*    virtual bool        excludeAsLocator() const { return false; }*/
/*    virtual bool        isTransparent() const { return false; }*/

    static  void *      creator();
    static  MStatus     initialize();

public:

    rigid_body_t::pointer rigid_body();
    void update();
    
public:

    //Attributes
    static  MObject     ia_collisionShape;
    static  MObject     ia_solver;
    static  MObject     ia_mass;
    static  MObject     ia_restitution;
    static  MObject     ia_friction;
    static  MObject     ia_linearDamping;
    static  MObject     ia_angularDamping;

    static  MObject     ia_initialPosition;
    static  MObject     ia_initialRotation;
    static  MObject     ia_initialVelocity;
    static  MObject     ia_initialSpin;

	static  MObject     ia_externalForce;
	static  MObject     ia_externalTorque;
    
    static  MObject     ia_velocity;
    static  MObject     ia_angularvelocity;
    
    static  MObject     ia_position;
    static  MObject     ia_rotation;
    static  MObject     ia_rotationX;
    static  MObject     ia_rotationY;
    static  MObject     ia_rotationZ;

    static  MObject     ca_rigidBody;
    static  MObject     ca_rigidBodyParam;
    static  MObject     ca_solver;

	static  MObject		oa_contactCount;
	static  MObject		oa_contactName; 
	static  MObject		oa_contactPosition;
    static  MObject     worldMatrix;
    //Attributes
    static  MObject     ia_shape;
    static  MObject     ia_type;
    static  MObject     ia_scale;
    static  MObject     ca_collisionShape;
    static  MObject     ca_collisionShapeParam;
    static  MObject     oa_collisionShape;
    

public:
    static  MTypeId     typeId;
    static  MString     typeName;

	void destroyRigidBody();
    void computeRigidBody(const MPlug& plug, MDataBlock& data);
    void computeWorldMatrix(const MPlug& plug, MDataBlock& data);
    void computeRigidBodyParam(const MPlug& plug, MDataBlock& data);
	void clearContactInfo();
	void addContactInfo(const MString& contactObjectName, const MVector& point);

public:
    static void nodeRemoved(MObject& node, void *clientData);
	void updateShape(const MPlug& plug, MDataBlock& data, float& collisionMarginOffset); //future collision margin adjust

private:
    rigid_body_t::pointer       m_rigid_body;
	int m_contactCount;
///
protected:
    void computeCollisionShape(const MPlug& plug, MDataBlock& data);
    void computeCollisionShapeParam(const MPlug& plug, MDataBlock& data);
    void computeOutputShape(const MPlug& plug, MDataBlock& data);
	collision_shape_t::pointer createCollisionShape(const MObject& node);
	collision_shape_t::pointer createCompositeShape(const MPlug& plgInShape);
    
protected:
	float collisionMarginOffset;
    
private:
    collision_shape_t::pointer m_collision_shape;
    
    
////
};



#endif
