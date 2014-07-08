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

Modified by Francisco Gochez <fjgochez@gmail.com>
Nov 2011 - Dec 2011 : Added soft body logic

Modified by Dongsoo Han <dongsoo.han@amd.com>
04/11/2012 : Added contactData attribute and contact information code to enable/disable updating contactCount, contactName and 
			 contactPosition attributes in boingRBNode.
*/

//bSolverNode.h

#ifndef DYN_BSOLVERNODE_H
#define DYN_BSOLVERNODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MStatus.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MTime.h>
#include <maya/MObjectArray.h>
#include <maya/MStringArray.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MIntArray.h>
#include <maya/MVector.h>
#include <vector>
#include "mathUtils.h"
#include "shared_ptr.h"
#include "boingRBNode.h"
//#include "boingRbCmd.h"
#include "bCallBackNode.h"
#include "LinearMath/btHashMap.h"

//#include "boing.h"


//class boing;
//using namespace std;

class bSolverNode : public MPxLocatorNode
{
public:
    bSolverNode();
    virtual ~bSolverNode();
    virtual void postConstructor(); 
    virtual void	draw(	M3dView & view, const MDagPath & path,
							M3dView::DisplayStyle style,
							M3dView::DisplayStatus status );


	virtual bool            isBounded() const { 
		return false;
	}
    virtual MBoundingBox boundingBox() const
    {
        MObject node = thisMObject();
        MPoint corner1(-1, -1, -1);
        MPoint corner2(1, 1, 1);
        return MBoundingBox(corner1, corner2);
    }

    virtual bool        excludeAsLocator() const { 
		return false; 
	}
    virtual bool        isTransparent() const { 
		return false; 
	}

    static  void *          creator();
    static  MStatus         initialize();

    virtual bool setInternalValueInContext ( const  MPlug & plug, const  MDataHandle & dataHandle,  MDGContext & ctx );

	void initConstraint(const MPlug& plug, MObject& bodyNode, MDataBlock& data); 
	//void initRigidBodyArray(const MPlug& plug, MObject &node, MDataBlock& data);
	void initRigidBody(const MPlug& plug, MObject& node, MDataBlock& data);
    
    //<rp 2014>
    void drawBoingRb( M3dView & view, const MDagPath &path,
                                  M3dView::DisplayStyle style,
                                  M3dView::DisplayStatus status);
    void traverseCallBacks();
    void runCallBacks(MObjectArray);
    void getConnectedTransform(MObject& node, MObject &retNode);
    //</rp 2014>

    virtual MStatus     compute( const MPlug& plug, MDataBlock& data );

    static  MObject     ia_time;
    static  MObject     ia_startTime;
    static  MObject     ia_gravity;
	static  MObject		ia_collisionMargin; //mb
	static  MObject     ia_disableCollisionsBetweenLinkedBodies; //mb
    static  MObject     ia_enabled;
    static  MObject     ia_splitImpulse;
    static  MObject     ia_substeps;
	static  MObject		ia_fixedPhysicsRate;
    static  MObject     oa_rigidBodies;
	static  MObject		oa_softBodies;
	static  MObject     ia_contactData;

    //Solver Settings
    //static  MObject     ssSolverType;
	static float collisionMarginOffset; //mb
//

	static  MObject     ia_DBG_DrawWireframe;
	static  MObject     ia_DBG_DrawAabb;
	static  MObject     ia_DBG_DrawFeaturesText;
	static  MObject     ia_DBG_DrawContactPoints;
	static  MObject     ia_DBG_NoDeactivation;
	static  MObject     ia_DBG_NoHelpText;
	static  MObject     ia_DBG_DrawText;
	static  MObject     ia_DBG_ProfileTimings;
	static  MObject     ia_DBG_EnableSatComparison;
	static  MObject     ia_DBG_DisableBulletLCP;
	static  MObject     ia_DBG_EnableCCD;
	static  MObject     ia_DBG_DrawConstraints;
	static  MObject     ia_DBG_DrawConstraintLimits;
	static  MObject     ia_DBG_FastWireframe;

    
    //

public: 
    static  MTypeId	typeId;
    static  MString     typeName;
	static	bool	isStartTime;
	bool	m_reInitialize;
	

	static void updateAllRigidBodies();
    //<rp 2014>
    static void getRigidBodies(MObject &node, MStringArray& rbds, std::set<boingRBNode*>&nodes);
    
    MStatus createNode(MObject, MString, MString, MVector, MVector, MVector, MVector, float);
    collision_shape_t::pointer createCollisionShape(const MObject& node);
    //void destroyNode(boing *b);
    //void erase_node(boing *b);
    //boing*  get_node(MString &name);
    //std::vector<boing*> get_all_nodes();
    //static void delete_all_names();
    //static void set_name(char *new_name);
    
    static shared_ptr<bSolverNode> get_bsolver_node();
    //</rp 2014>

    
private:
    //static MStringArray node_name_ptr;
    void updateRigidBodies();

protected:
    //<rp 2014>
    MObjectArray fScallBackNodes;
    MObjectArray sScallBackNodes;
    MObjectArray sEcallBackNodes;
    MObjectArray fEcallBackNodes;


    //static MStringArray procRbArray;
    //friend  class boing;
    
    //</rp 2014>
    
    
    struct xforms_t {
        vec3f m_x0;
        vec3f m_x1;
        quatf m_q0;
        quatf m_q1;
    };
    
	void deleteRigidBodies(const MPlug& plug, MPlugArray &rbConnections, MDataBlock& data);
    void computeRigidBodies(const MPlug& plug, MDataBlock& data);
	void computeSoftBodies(const MPlug& plug, MDataBlock& data);
	void initSoftBody(const MPlug& plug, MObject& node, MDataBlock& data);
    //void dumpRigidBodyArray(MObject &node);
    bool expandFileExpression(std::string const& expr, std::string &base_name, std::string &extension);

    void initRigidBodies(const MPlug& plug, MPlugArray &rbConnections, MDataBlock& data);
	void initSoftBodies(const MPlug& plug, MPlugArray &sbConnections, MDataBlock& data);
    void gatherPassiveTransforms(MPlugArray &rbConnections, std::vector<xforms_t> &xforms);
    void updatePassiveRigidBodies(MPlugArray &rbConnections, std::vector<xforms_t> &xforms, float t);
    void updateActiveRigidBodies(MPlugArray &rbConnections);
	void updateActiveSoftBodies(MPlugArray &sbConnections);
    void applyFields(MPlugArray &rbConnections, float dt);
	void updateConstraint(MObject& bodyNode);
	void clearContactRelatedAttributes();//MPlugArray &rbConnections);

	boingRBNode* getboingRBNode(btCollisionObject* btColObj);

public:
    struct m_custom_data {
        MString name;
        MString typeName;
        int count;
        MObject node;
        MVector m_initial_velocity;
        MVector m_initial_position;
        MVector m_initial_rotation;
        MVector m_initial_angularvelocity;
        MVector m_vel;
        MVector m_pos;
        MVector m_rot;
        MVector m_av;
        float m_mass;
        int m_contact_count;
        MStringArray m_contact_objects;
        MPointArray m_contact_positions;
        //custom attr name array
        MStringArray m_attr_data;
        //custom attribute type array
        MStringArray m_attr_type;
        
        //custom int attr array
        MIntArray m_int_data;
        //custom double attr array
        MDoubleArray m_double_data;
        //custom vector attr array
        MVectorArray m_vector_data;
        //custom string attr array
        MStringArray m_string_data;
        
        collision_shape_t::pointer m_collision_shape ;
        rigid_body_t::pointer m_rigid_body ;
    };
    int getdatalength();
    void addContactInfo(m_custom_data*  contactDataContainer, const MString& contactObjectName, const MVector& point);
    m_custom_data *getdata(MString name);
    void insertData(MString n, m_custom_data *data);
    void deletedata(MString name);
    void deleteAllData();
    MStringArray get_all_keys();
    void add_key(MString &key);
    MStatus delete_key(MString &key, int index);
    void delete_all_keys();
    // functions to handle custom attributes ( int / float / vector / string )
    //set
    void set_custom_data(MString &attr, void * value);
    //void set_custom_float(MString &attr, float &value);
    //void set_custom_vector(MString &attr, MVector &value);
    //void set_custom_string(MString &attr, MString &value);
    // get
    void * get_custom_data(MString &attr);
    
    void saveAttrType(MString &attr, MString &type);
    //delete
    void delete_all_custom_data();
    bool attribute_exists(MString &attr);
    MString getAttrType(MString &attr);
    //float get_custom_float(MString &attr, float &value);
    //MVector get_custom_vector(MString &attr, MVector &value);
    //MString get_custom_string(MString &attr, MString &value);
    // delete
    
    friend  class boingRBNode;

protected:
    MStringArray keyArray;
    MTime m_prevTime;
	btHashMap<btHashPtr, boingRBNode*> m_hashColObjectToRBNode;
    
	btHashMap<btHashString, m_custom_data*> m_hashNameToData;
	btHashMap<btHashString, void*> m_hashNameToAttrData;
	btHashMap<btHashString, MString> m_hashNameToAttrType;
    
	//btHashMap<btHashString, MString> m_hashNameToStringData;
	//btHashMap<btHashString, float> m_hashNameToFloatData;
	//btHashMap<btHashString, MVector> m_hashNameToVectorData;
	//btHashMap<btHashString, int> m_hashNameToIntData;
    
    static shared_ptr<bSolverNode> m_bsolvernode;
    
//friend:
//    bCallBack;
    
};


#endif
