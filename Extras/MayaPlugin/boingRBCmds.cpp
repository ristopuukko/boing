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
#include <maya/MPointArray.h>

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
    
    syntax.addFlag("-h", "-help");
    syntax.addFlag("-set", "-setAttr", MSyntax::kString);
    syntax.addFlag("-get", "-getAttr", MSyntax::kString);
    syntax.addFlag("-add", "-addAttr", MSyntax::kString);
    syntax.addFlag("-cr", "-create", MSyntax::kString);
    syntax.addFlag("-del", "-delete", MSyntax::kString);
    syntax.addFlag("-typ", "-type", MSyntax::kString);
    syntax.addFlag("-ex", "-exists", MSyntax::kString);
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
    
    
    if (argParser->isFlagSet("help") || argParser->isFlagSet("h"))
    {
        MString helpMsg = "boingRB - command : Boing - bullet plugin by Risto Puukko\n";
        helpMsg += "---------------------------------------------------------\n";
        helpMsg += "boingRB [flag] [args] \n";
        helpMsg += "\n";
        helpMsg += "flags :\n";
        helpMsg += "  -getAttr [name.attr]\n";
        helpMsg += "     example : boingRb -getAttr (\"boingRb1.velocity\");\n" ;
        helpMsg += "\n";
        helpMsg += "  -getAttr [*.attr]\n";
        helpMsg += "     example : boingRb -getAttr (\"*.name\");\n" ;
        helpMsg += "\n";
        helpMsg += "  -setAttr [name.attr] -value [float float float ]\n";
        helpMsg += "     example : boingRb -setAttr (\"boingRb1.velocity\") -value 0 4 3 ;\n" ;
        helpMsg += "\n";
        helpMsg += "  -addAttr [name.attr] -type [int/float/string/vector]\n";
        helpMsg += "     example : boingRb -addAttr \"sampleRb.IntAttribute\" -type \"int\";\n" ;
        helpMsg += "     example : boingRb -addAttr \"sampleRb.VectorAttribute\" -type \"vector\";\n" ;
        helpMsg += "\n";
        helpMsg += "  -create [ ( \"name=string;geo=string;velocity/vel=float,float,float;position/pos=float,float,float\" )]\n";
        helpMsg += "     example : boingRb -create (\"name=sampleRb;geo=pCubeShape1;pos=0,4,0\");\n";
        helpMsg += "\n";
        helpMsg += "  -exists [name]\n";
        helpMsg += "     example : boingRb -exists \"sampleRb\";\n" ;
        helpMsg += "\n";
        helpMsg += "  -delete [name]\n";
        helpMsg += "     example : boingRb -delete \"sampleRb\";\n";
        helpMsg += "\n";
        helpMsg += "---------------------------------------------------------\n";

        
        MGlobal::displayInfo(helpMsg);
        return MS::kSuccess;
    }
    
    isSetAttr = argParser->isFlagSet("-setAttr");
    isGetAttr = argParser->isFlagSet("-getAttr");
    isAddAttr = argParser->isFlagSet("-addAttr");
    isType = argParser->isFlagSet("-type");
    isExists = argParser->isFlagSet("-exists");
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
        if (attr.length()<1) {
            attr = checkCustomAttribute(jobArgsArray[0], jobArgsArray[1]);
        }
        setBulletVectorAttribute(jobArgsArray[0], attr, value);
        
        //cout<<value<<endl;
        setResult(MS::kSuccess);
        
    } else if ( isAddAttr && isType ) {
        MString aAttr;
        argParser->getFlagArgument("addAttr", 0, aAttr);
        MStringArray jobArgsArray = parseArguments(aAttr, ".");
        cout<<jobArgsArray<<endl;
        MString rbname = jobArgsArray[0];
        MString attrAdded = jobArgsArray[1];
        MString attrtype;
        argParser->getFlagArgument("-type", 0, attrtype);
        cout<<attrtype<<endl;
        //shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
        //bSolverNode::m_custom_data *data = b_solv->getdata(rbname);
        //data->m_attr_array.append(MString(attrtype+","+attrAdded));
        
        //rigid_body_t::pointer rb = getPointerFromName(jobArgsArray[0]);
        //boing *b = static_cast<boing*>( rb->impl()->body()->getUserPointer() );
        //b->add_data(jobArgsArray[0], jobArgsArray[1]);
        //b->boing::customAttributes.insert(boing::customAttributes.end(),atr);
        
    } else if ( isGetAttr) {
        MString gAttr;
        shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
        argParser->getFlagArgument("getAttr", 0, gAttr);
        
        //cout<<gAttr<<endl;
        MStringArray jobArgsArray = parseArguments(gAttr, ".");
        MString rbname = jobArgsArray[0];
        //std::cout<<"name : "<<rbname<<std::endl;
        if ( rbname == "" ) {
            MString errorMsg = "ERROR ! boing -getAttr must provide a rigid body name!";
            displayWarning(errorMsg, true);
            return MS::kFailure;
        }
        MString attr = checkAttribute(jobArgsArray[1]);
        //cout<<"attr = "<<attr<<endl;
        if ( attr=="velocity" || attr=="position" || attr=="angularVelocity" ) {
            //MVector result =
            MDoubleArray dResult = getBulletVectorAttribute(rbname, attr);
            setResult(dResult);
        } else if (attr == "contactPositions") {
            bSolverNode::m_custom_data *data = b_solv->getdata(rbname);
            MDoubleArray d_points = MDoubleArray();
            if ( data->m_contact_count > 0 ) {
                MPointArray points = data->m_contact_positions;
                for (int i=0; i<points.length();i++) {
                    d_points.append(points[i].x);
                    d_points.append(points[i].y);
                    d_points.append(points[i].z);
                }
            }
            setResult(d_points);
        } else if (attr == "contactGeos") {
            MStringArray contact_objects = MStringArray();
            bSolverNode::m_custom_data *data = b_solv->getdata(rbname);
            if ( data->m_contact_count > 0 ) {
                MStringArray contact_objects = data->m_contact_objects;
            }
            setResult(contact_objects);
        } else if (attr == "contactCount") {
            bSolverNode::m_custom_data *data = b_solv->getdata(rbname);
            int contact_count = data->m_contact_count;
            setResult(contact_count);
        } else if (attr == "name") {
            MStringArray result;
            MStringArray names = b_solv->get_all_keys();
            //std::cout<<"names : "<<names<<std::endl;
            //std::cout<<"b_solv->getdatalength() : "<<b_solv->getdatalength()<<std::endl;
            for(int i=0; i < names.length(); i++) {
                bSolverNode::m_custom_data *data = b_solv->getdata(names[i]);
                if ( NULL != data) {
                    //std::cout<<"data->name : "<<data->name<<std::endl;
                    //std::cout<<"data->m_initial_position: "<<data->m_initial_position<<std::endl;
                    result.append(data->name);
                }
            }
            setResult(result);
        } else if ( attr == "" ) {
            MString errorMsg = "ERROR ! boing -getAttr must provide an attribute name to query!";
            displayWarning(errorMsg, true);
            return MS::kFailure;
        }
        
    } else if ( isCreate  ) {
        MString aArgument;
        argParser->getFlagArgument("-create", 0, aArgument);
        
        MStringArray createArgs = parseArguments(aArgument,";");
        int size = createArgs.length();
        MString inputShape;
        MString rbname = "dummyRb";
        MVector av;
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
                
            } else if (singleArg[0] == "av") {
                //initialAngularVelocity
                MStringArray avArray = parseArguments(singleArg[1], ",");
                av = MVector(avArray[0].asDouble(),
                             avArray[1].asDouble(),
                             avArray[2].asDouble()
                             );
                //cout<<"initialAngularVelocity = "<<av<<endl;
                
            } else {
                cout<<"Unrecognized parameter : "<<singleArg[0]<<endl;
            }
        }
        // create boing node
        shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
        MObject node = nameToNode(inputShape);
        float mass = 1.0f;
        MString tname = "boing";
        b_solv->createNode(node, rbname, tname, pos, vel, rot, av, mass);
        
    } else if ( isDelete  ) {
        MString aArgument;
        argParser->getFlagArgument("-delete", 0, aArgument);
        //std::cout<<"delete aArgument "<<aArgument<<std::endl;
        if (aArgument != "") {
            shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
            b_solv->deletedata(aArgument);
            MStatus stat = b_solv->delete_key(aArgument, -1);
            if (stat != MS::kSuccess) {
                std::cerr<<"error occurred deleting "<<aArgument<<" ."<<std::endl;
                setResult(1);
            }
        }
    } else if ( isExists ) {
        MString exArg;
        argParser->getFlagArgument("-exists", 0, exArg);
        if (exArg != "") {
            shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
            bSolverNode::m_custom_data *data = b_solv->getdata(exArg);
            if ( NULL != data )
                setResult ( data->name );
        }
    }
    
    return MS::kSuccess;
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
            } /*else { // set a custom attribute
                boing *b = static_cast<boing*>( rb->impl()->body()->getUserPointer() );
                MString vecStr = "";
                vecStr.set( vec.x );
                
                vecStr += MString(",");
                vecStr += vec.y;
                vecStr += MString(",");
                vecStr += vec.z;
                b->set_data(attr, vecStr);
            }*/
        }
        
    }
    
    return MS::kSuccess;
    
}

MDoubleArray boingRbCmd::getBulletVectorAttribute(MString &name, MString &attr) {
    
    MVector vec;
    MDoubleArray result;
    
    shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
    MStringArray nodes;
    if (name == "*") {
        nodes = b_solv->get_all_keys();
    } else {
        nodes.append(name);
    }
    //std::cout<<"nodes : "<<nodes<<std::endl;
    //std::cout<<"nodes.length() : "<<nodes.length()<<std::endl;
    
    for (int i=0; i<nodes.length(); i++) {
        
        //std::cout<<"trying to get rb...."<<std::endl;
        rigid_body_t::pointer rb = b_solv->getdata(nodes[i])->m_rigid_body;
        //std::cout<<"got rb : "<<rb<<std::endl;
        
        //rigid_body_t::pointer rb = getPointerFromName(name);
        
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
                } /*else {
                    boing *b = static_cast<boing*>( rb->impl()->body()->getUserPointer() );
                    MString vecStr = b->get_data(attr);
                    MStringArray vecArray = parseArguments(vecStr, ",");
                    vec = MVector(vecArray[0].asDouble(), vecArray[1].asDouble(), vecArray[2].asDouble());
                }*/
            }
        }
        for (int j=0; j<3; j++) {
            result.append(vec[i]);
        }
    }
    
    return result;
    
}

rigid_body_t::pointer boingRbCmd::getPointerFromName(MString &name)
{
    
    rigid_body_t::pointer rb = 0;
    
    shared_ptr<solver_impl_t> solv = solver_t::get_solver();
    std::set<rigid_body_t::pointer> rbds = solver_t::get_rigid_bodies();
    shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
    MStringArray names = b_solv->get_all_keys();
    for( int i=0; i<names.length(); i++) {
        bSolverNode::m_custom_data * data = b_solv->getdata(names[i]);
        if (NULL != data) {
            if (name == data->name) {
                rb = data->m_rigid_body;
                break;
            }
        } 
    }
    
    return rb;
}


MString boingRbCmd::checkCustomAttribute(MString &name, MString &attr)
{
    MString result = "";
    /*
    
    shared_ptr<solver_impl_t> solv = solver_t::get_solver();
    std::set<rigid_body_t::pointer> rbds = solver_t::get_rigid_bodies();
    shared_ptr<bSolverNode> b_solv = bSolverNode::get_bsolver_node();
    MStringArray names = b_solv->get_all_keys();
    for( int i=0; i<names.length(); i++) {
        bSolverNode::m_custom_data * data = b_solv->getdata(names[i]);
        if (NULL != data) {
            if (name == data->name) {
                for (int j=0; j<data->m_attr_data.length(); i++) {
                    if ( data->m_attr_array[i] == attr ) {
                        result = data->m_data_array[i];
                        return result;
                    }
                }
            }
        }
    }
     */
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
    } else if ( attr=="ctcps" || attr=="contactPositions") {
        result = "contactPositions";
    } else if ( attr=="ctcgs" || attr=="contactGeos") {
        result = "contactGeos";
    } else if ( attr=="ctccnt" || attr=="contactCount") {
        result = "contactCount";
    }
    //cout<<"checkAttribute : "<<result<<endl;
    return result;
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