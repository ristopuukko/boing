//
//  boing.h
//  BulletMayaPlugin
//
//  Created by Risto Puukko on 19/06/14.
//
//

#ifndef __BulletMayaPlugin__boing__
#define __BulletMayaPlugin__boing__
#include <maya/MString.h>
#include <maya/MVector.h>
#include <vector>
#include <set>
#include <iostream>

using namespace std;

class boing
{
public:
    boing();
    virtual ~boing();

    void destroy() { delete this; }
    
    virtual void set_name(MString name) {
        rbname = name;
    }
    virtual MString get_name() {
        return rbname;
    }

    virtual void set_float(MString &attr, float &value);
    friend  class boingRbCmd;
    
    struct attrs {
        static MString name;
        static int ivalue;
        static float fvalue;
        static double dvalue;
        static MVector vvalue;
        static MString svalue;
        static void* value;
    };
    MString rbname;
    
    static set<attrs> customAttributes;

//private:
    
    
};

#endif /* defined(__BulletMayaPlugin__boing__) */
