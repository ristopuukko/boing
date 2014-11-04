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
Nov 2011 - Dec 2011 : Added logic for soft bodies
*/

//solver.h

//basic class for all solvers

#ifndef DYN_SOLVER_H
#define DYN_SOLVER_H

#include <set>
#include <vector>

#include "mathUtils.h"
#include "shared_ptr.h"

#include "rigid_body.h"
#include "sphere_shape.h"
#include "plane_shape.h"
#include "box_shape.h"
#include "convex_hull_shape.h"
#include "mesh_shape.h"
#include "solver_impl.h"
#include "soft_body_t.h"
#include "composite_shape_t.h"

#include "constraint/nail_constraint.h"
#include "constraint/hinge_constraint.h"
#include "constraint/slider_constraint.h"
#include "constraint/sixdof_constraint.h"

class solver_t
{
public:
    static void initialize();
    static void cleanup();

    //creation methods
    static sphere_shape_t::pointer create_sphere_shape(float radius = 1.0); 

    static plane_shape_t::pointer create_plane_shape(vec3f const& normal = vec3f(0, 1, 0), float d = 0); 

    static box_shape_t::pointer create_box_shape(vec3f const& halfExtents = vec3f(0.5f, 0.5f, 0.5f)); 

    static convex_hull_shape_t::pointer create_convex_hull_shape(vec3f const* vertices, size_t num_vertices,
                                                                 vec3f const* normals,
                                                                 unsigned int const *indices, size_t num_indices); 

    static mesh_shape_t::pointer create_mesh_shape(vec3f const* vertices, size_t num_vertices,
                                                   vec3f const* normals,
                                                   unsigned int const *indices, size_t num_indices, bool dynamicMesh, bool usepivot, vec3f center);
	static hacd_shape_t::pointer create_hacd_shape(vec3f const* vertices, size_t num_vertices,
                                                   vec3f const* normals,
                                                   unsigned int const *indices, size_t num_indices, bool dynamicMesh); 
	static composite_shape_t::pointer create_composite_shape(
				collision_shape_t::pointer* childShapes, 
				vec3f* childPositions,
				quatf* childOrientations,
				int numChildren);

    static rigid_body_t::pointer create_rigid_body(collision_shape_t::pointer& cs);
	
	static soft_body_t::pointer create_soft_body(const std::vector<float> &triVertexCoords, const std::vector<int> &triVertexIndices  );

    static nail_constraint_t::pointer create_nail_constraint(rigid_body_t::pointer& rb, vec3f const& pivot);
    static nail_constraint_t::pointer create_nail_constraint(rigid_body_t::pointer& rbA, rigid_body_t::pointer& rbB, vec3f const& pivotInA, vec3f const& pivotInB);
    static hinge_constraint_t::pointer create_hinge_constraint(rigid_body_t::pointer& rb, vec3f const& pivot, quatf const& rot);
    static hinge_constraint_t::pointer create_hinge_constraint(rigid_body_t::pointer& rbA, vec3f const& pivotA, quatf const& rotA, rigid_body_t::pointer& rbB, vec3f const& pivotB, quatf const& rotB);
    static slider_constraint_t::pointer create_slider_constraint(rigid_body_t::pointer& rb, vec3f const& pivot, quatf const& rot);
    static slider_constraint_t::pointer create_slider_constraint(rigid_body_t::pointer& rbA, vec3f const& pivotA, quatf const& rotA, rigid_body_t::pointer& rbB, vec3f const& pivotB, quatf const& rotB);
    static sixdof_constraint_t::pointer create_sixdof_constraint(rigid_body_t::pointer& rb, vec3f const& pivot, quatf const& rot);
    static sixdof_constraint_t::pointer create_sixdof_constraint(rigid_body_t::pointer& rbA, vec3f const& pivotA, quatf const& rotA, rigid_body_t::pointer& rbB, vec3f const& pivotB, quatf const& rotB);

    //add/remove from world
	static void add_rigid_body(rigid_body_t::pointer& rb,const char* name);
	static void add_soft_body(soft_body_t::pointer& sb, const char* name);
    static void remove_rigid_body(rigid_body_t::pointer& rb);
	static void remove_soft_body(soft_body_t::pointer &sb);
    static void remove_all_rigid_bodies();
	static void remove_all_soft_bodies();

    //add/remove from world
    static void add_constraint(constraint_t::pointer& c, bool disableCollide = false);
    static void remove_constraint(constraint_t::pointer& c);
    static void remove_all_constraints();

    //
    static void set_gravity(vec3f const& g);

    //
    static void set_split_impulse(bool enabled); 

    //
	static void set_collision_margin(float cm); //mb

    static void step_simulation(float dt,  float fixedPhysicsFrameRate);

	static void createWorld();

	static void destroyWorld();

	static void debug_draw(int dbgMode);

    static shared_ptr<solver_impl_t> get_solver();
    
    static std::set<rigid_body_t::pointer> get_rigid_bodies();

private:
    static shared_ptr<solver_impl_t> m_impl;
    static std::set<rigid_body_t::pointer> m_rigid_bodies;
	static std::set<soft_body_t::pointer> m_soft_bodies;
    static std::set<constraint_t::pointer> m_constraints;
};



#endif

