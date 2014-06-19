//
//  boing.cpp
//  BulletMayaPlugin
//
//  Created by Risto Puukko on 19/06/14.
//
//

#include "boing.h"

set<boing::attrs> boing::customAttributes;

boing::boing() {}

boing::~boing() {}

void boing::set_float(MString &attr,float &value)  {
    set<boing::attrs>::iterator it;
    
    for(it=customAttributes.begin(); it!=customAttributes.end();++it) {
        if ((*it).name == attr ) {
            (*it).fvalue = value;
        }
    }
}