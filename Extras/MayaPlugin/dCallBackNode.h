/*
 
 Written by: Risto Puukko <risto.puukko@gmail.com>
 
 */

//dCallBackNode.h

#ifndef DYN_DCALLBACKNODE_H
#define DYN_DCALLBACKNODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
//#include <maya/MPxNode.h>
//#include <maya/MPxLocatorNode.h>
#include <maya/MPxTransform.h>


class dCallBackNode : public MPxTransform
{
public:
    dCallBackNode();
    dCallBackNode(MPxTransformationMatrix *);
    virtual ~dCallBackNode();
    static  void *      creator();
    static  MStatus     initialize();
	//virtual void postConstructor();

    //virtual MStatus validateAndSetValue(const MPlug& plug,
    //                                    const MDataHandle& handle, const MDGContext& context);
    
    //virtual void  resetTransformation (MPxTransformationMatrix *);
    //virtual void  resetTransformation (const MMatrix &);
    
    //virtual bool        isBounded() const {
	//	return false;
	//}
    
    //virtual MStatus     compute( const MPlug& plug, MDataBlock& data );

	static  MObject     ia_enabled;
    static  MObject     ia_script;
    static  MObject     ia_callbacktype;
    static  MString     typeName;
    static  MTypeId     typeId;
    /// callback node types
    static const int FRAME_START   = 0;
    static const int SUBSTEP_START = 1;
    static const int SUBSTEP_END   = 2;
    static const int FRAME_END     = 3;
    ///
    
protected:
    typedef MPxTransform ParentClass;

    
};


#endif
