/*
 
Written by: Risto Puukko <risto.puukko@gmail.com>

*/

//bCallBackNode.cpp


#include "bCallBackNode.h"
#include <maya/MString.h>
#include <maya/MObject.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnStringData.h>
#include "mayaUtils.h"


MTypeId bCallBackNode::typeId(0x100329);
MString bCallBackNode::typeName("bCallBack");

MObject bCallBackNode::ia_enabled;
MObject bCallBackNode::ia_script;
MObject bCallBackNode::ia_callbacktype;

MStatus bCallBackNode::initialize()
{
    MStatus                 status;
    MFnEnumAttribute        fnEnumAttr;
    //MFnMessageAttribute     fnMsgAttr;
    //MFnUnitAttribute        fnUnitAttr;
    MFnNumericAttribute     fnNumericAttr;
    MFnTypedAttribute       fnTypedAttr;
    
    
    ia_enabled = fnNumericAttr.create("Enable", "en", MFnNumericData::kBoolean, true);
    fnNumericAttr.setStorable(true);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ia_enabled);
    MCHECKSTATUS(status, "adding ia_enabled attribute");
    
    
    
    // Declare the function set and a MObject for the attribute data.
    MFnStringData                         fnStringData;
    MObject                               defaultString;
    
    // Create the default string.
    defaultString = fnStringData.create( "" );
    
    // Create the attribute and specify its default.
    ia_script = fnTypedAttr.create( "callbackscript", "cbs", MFnData::kString, defaultString );
    fnTypedAttr.setStorable(true);
    fnTypedAttr.setKeyable(false);
    
    status = addAttribute(ia_script);
    MCHECKSTATUS(status, "adding ia_script attribute");

//	MCallbackId connCBId = MDGMessage::addConnectionCallback(connCB, NULL, NULL );

    
    ia_callbacktype = fnEnumAttr.create( "callbacktype", "cbt", 0, &status );
    MCHECKSTATUS(status, "creating ia_callbacktype attribute")
    fnEnumAttr.addField( "Frame start", FRAME_START );
    fnEnumAttr.addField( "Substep start", SUBSTEP_START );
    fnEnumAttr.addField( "Substep end", SUBSTEP_END );
    fnEnumAttr.addField( "Frame end", FRAME_END );
    status = addAttribute(ia_callbacktype);
    MCHECKSTATUS(status, "adding ia_callbacktype attribute")

    //


    return MS::kSuccess;
}

bCallBackNode::bCallBackNode() {}
bCallBackNode::~bCallBackNode() {}


void* bCallBackNode::creator() { return new bCallBackNode(); }


/*
MStatus bCallBackNode::compute(const MPlug& plug, MDataBlock& data)
{
//  std::cout << "Calling bSolverNode::compute \n";
//	std::cout << "Plug: " << plug.name().asChar() << std::
    
    return MStatus::kUnknownParameter;
}

*/
