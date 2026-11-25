/**************************************************************************/
/*  test_jolt_motion_queries.h                                            */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "../jolt_physics_server_3d.h"

#include "tests/test_macros.h"

namespace TestJoltMotionQueries {

// The test runner only creates a physics server for `[SceneTree]` and `[Editor]` tests, so these tests
// own a Jolt server of their own for their whole duration.
class JoltTestWorld {
	LocalVector<RID> bodies;
	LocalVector<RID> shapes;

public:
	JoltPhysicsServer3D *server = nullptr;
	RID space;

	JoltTestWorld() {
		REQUIRE_MESSAGE(PhysicsServer3D::get_singleton() == nullptr, "These tests need to own the physics server.");
		server = memnew(JoltPhysicsServer3D(false));
		server->init();
		space = server->space_create();
		server->space_set_active(space, true);
	}

	~JoltTestWorld() {
		for (const RID &body : bodies) {
			server->free_rid(body);
		}
		for (const RID &shape : shapes) {
			server->free_rid(shape);
		}
		server->free_rid(space);
		server->finish();
		memdelete(server);
	}

	RID sphere(real_t p_radius) {
		const RID shape = server->sphere_shape_create();
		server->shape_set_data(shape, p_radius);
		shapes.push_back(shape);
		return shape;
	}

	RID box(const Vector3 &p_half_extents) {
		const RID shape = server->box_shape_create();
		server->shape_set_data(shape, p_half_extents);
		shapes.push_back(shape);
		return shape;
	}

	RID capsule(real_t p_radius, real_t p_height) {
		Dictionary data;
		data["radius"] = p_radius;
		data["height"] = p_height;
		const RID shape = server->capsule_shape_create();
		server->shape_set_data(shape, data);
		shapes.push_back(shape);
		return shape;
	}

	// An 8x8 m quad in the YZ plane, facing +X and -X.
	RID quad() {
		Dictionary data;
		data["faces"] = PackedVector3Array({ Vector3(0, -4, -4), Vector3(0, 4, -4), Vector3(0, 4, 4),
				Vector3(0, -4, -4), Vector3(0, 4, 4), Vector3(0, -4, 4) });
		data["backface_collision"] = true;
		const RID shape = server->concave_polygon_shape_create();
		server->shape_set_data(shape, data);
		shapes.push_back(shape);
		return shape;
	}

	// A flat 8x8 sample height field (7x7 m) at height 0. Large enough to be built as a Jolt height field.
	RID height_map() {
		constexpr int size = 8;
		Vector<real_t> heights;
		heights.resize(size * size);
		heights.fill(0.0);
		Dictionary data;
		data["width"] = size;
		data["depth"] = size;
		data["heights"] = heights;
		const RID shape = server->heightmap_shape_create();
		server->shape_set_data(shape, data);
		shapes.push_back(shape);
		return shape;
	}

	RID add_body(PhysicsServer3D::BodyMode p_mode, RID p_shape, const Transform3D &p_transform, const Transform3D &p_shape_transform = Transform3D()) {
		const RID body = server->body_create();
		server->body_set_mode(body, p_mode);
		server->body_add_shape(body, p_shape, p_shape_transform, false);
		server->body_set_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM, p_transform);
		server->body_set_space(body, space);
		bodies.push_back(body);
		return body;
	}

	// Returns the distance the shape can travel along `p_motion` before it hits something.
	real_t cast_motion(RID p_shape, const Transform3D &p_from, const Vector3 &p_motion) {
		PhysicsDirectSpaceState3D::ShapeParameters parameters;
		parameters.shape_rid = p_shape;
		parameters.transform = p_from;
		parameters.motion = p_motion;

		real_t safe = 1.0;
		real_t unsafe = 1.0;
		PhysicsDirectSpaceState3D *state = server->space_get_direct_state(space);
		REQUIRE(state != nullptr);
		CHECK(state->cast_motion(parameters, safe, unsafe, nullptr));
		return safe * p_motion.length();
	}

	// Returns the distance the body can travel along `p_motion`, as `move_and_collide` would see it.
	real_t test_motion(RID p_body, const Transform3D &p_from, const Vector3 &p_motion, bool &r_collided) {
		PhysicsServer3D::MotionParameters parameters(p_from, p_motion);
		PhysicsServer3D::MotionResult result;
		r_collided = server->body_test_motion(p_body, parameters, &result);
		return result.travel.length();
	}
};

constexpr real_t MOTION_LENGTH = 2.0;

// The bisection in `cast_motion` resolves a 2 m sweep to about 2 mm.
constexpr real_t TOLERANCE = 0.005;

struct Target {
	const char *name;
	RID shape;
	Transform3D transform;
	Vector3 start;
	Vector3 motion;
};

// Each target is a static body in its own lane, 1 m ahead of the start point along the motion.
LocalVector<Target> add_targets(JoltTestWorld &p_world) {
	const Basis rotated_45 = Basis(Vector3(0, 1, 0), Math::deg_to_rad(45.0));
	const Vector3 forward = Vector3(MOTION_LENGTH, 0, 0);

	LocalVector<Target> targets;
	targets.push_back({ "box facing", p_world.box(Vector3(0.1, 4, 4)), Transform3D(Basis(), Vector3(1.1, 0, 0)), Vector3(0, 0, 0), forward });
	targets.push_back({ "box at 45 degrees", p_world.box(Vector3(0.1, 4, 4)), Transform3D(rotated_45, Vector3(1.5, 0, 20)), Vector3(0, 0, 20), forward });
	targets.push_back({ "mesh facing", p_world.quad(), Transform3D(Basis(), Vector3(1, 0, 40)), Vector3(0, 0, 40), forward });
	targets.push_back({ "mesh at 45 degrees", p_world.quad(), Transform3D(rotated_45, Vector3(1.5, 0, 60)), Vector3(0, 0, 60), forward });
	targets.push_back({ "height field below", p_world.height_map(), Transform3D(Basis(), Vector3(0, -1, 80)), Vector3(0, 0, 80), Vector3(0, -MOTION_LENGTH, 0) });

	for (const Target &target : targets) {
		p_world.add_body(PhysicsServer3D::BODY_MODE_STATIC, target.shape, target.transform);
	}

	return targets;
}

TEST_CASE("[JoltPhysics] cast_motion with a scaled shape matches the equivalent unscaled shape") {
	JoltTestWorld world;
	const LocalVector<Target> targets = add_targets(world);

	const real_t radius = 0.2;
	const Vector3 half_extents = Vector3(0.2, 0.2, 0.2);
	const Basis rotation = Basis(Vector3(0, 1, 0), Math::deg_to_rad(30.0)) * Basis(Vector3(1, 0, 0), Math::deg_to_rad(20.0));

	for (const real_t scale : { real_t(1.0 / 6.0), real_t(0.5), real_t(1.0), real_t(2.0) }) {
		const RID sphere = world.sphere(radius);
		const RID sphere_resized = world.sphere(radius * scale);
		const RID box = world.box(half_extents);
		const RID box_resized = world.box(half_extents * scale);

		struct Pose {
			const char *name;
			Basis orientation;
			Vector3 scale;
		};

		const Pose poses[] = {
			{ "uniform scale", Basis(), Vector3(scale, scale, scale) },
			{ "rotated and scaled", rotation, Vector3(scale, scale, scale) },
			{ "mirrored", rotation, Vector3(-scale, scale, scale) },
		};

		for (const Target &target : targets) {
			for (const Pose &pose : poses) {
				const Basis scaled_basis = pose.orientation * Basis::from_scale(pose.scale);

				INFO("target: ", String(target.name), ", pose: ", String(pose.name), ", scale: ", scale);

				const real_t sphere_expected = world.cast_motion(sphere_resized, Transform3D(pose.orientation, target.start), target.motion);
				const real_t sphere_actual = world.cast_motion(sphere, Transform3D(scaled_basis, target.start), target.motion);

				// Every target is in range, so the reference cast has to hit.
				CHECK(sphere_expected < MOTION_LENGTH - 0.1);

				const real_t box_expected = world.cast_motion(box_resized, Transform3D(pose.orientation, target.start), target.motion);
				const real_t box_actual = world.cast_motion(box, Transform3D(scaled_basis, target.start), target.motion);

				CHECK(box_expected < MOTION_LENGTH - 0.1);
				CHECK(Math::abs(box_actual - box_expected) < TOLERANCE);
				CHECK(Math::abs(sphere_actual - sphere_expected) < TOLERANCE);
			}
		}
	}
}

TEST_CASE("[JoltPhysics] cast_motion with a scaled sphere stops at the wall") {
	JoltTestWorld world;
	const LocalVector<Target> targets = add_targets(world);

	const real_t radius = 0.2;
	const real_t scale = 1.0 / 6.0;
	const RID sphere = world.sphere(radius);

	// The facing box, the facing mesh and the height field are all 1 m away from the start point.
	for (const Target &target : targets) {
		if (String(target.name).contains("45")) {
			continue;
		}

		INFO("target: ", String(target.name));

		const real_t travel = world.cast_motion(sphere, Transform3D(Basis::from_scale(Vector3(scale, scale, scale)), target.start), target.motion);
		CHECK(travel < 1.0 - radius * scale + 0.001);
		CHECK(travel > 1.0 - radius * scale - TOLERANCE);
	}
}

TEST_CASE("[JoltPhysics] body_test_motion with a scaled body or shape matches the equivalent unscaled body") {
	JoltTestWorld world;
	const LocalVector<Target> targets = add_targets(world);

	// Short enough that even at scale 2 the capsule starts clear of the height field 1 m below.
	const real_t radius = 0.3;
	const real_t height = 0.9;

	for (const real_t scale : { real_t(1.0 / 6.0), real_t(0.5), real_t(1.0), real_t(2.0) }) {
		const RID capsule = world.capsule(radius, height);
		const RID capsule_resized = world.capsule(radius * scale, height * scale);

		for (const Target &target : targets) {
			// A mirrored capsule has the same geometry, so it uses the same reference.
			for (const bool mirrored : { false, true }) {
				const Basis scale_basis = Basis::from_scale(Vector3(mirrored ? -scale : scale, scale, scale));

				INFO("target: ", String(target.name), ", scale: ", scale, ", mirrored: ", mirrored);

				// Park the bodies far away from the targets, the query only uses `from`.
				const Transform3D parked = Transform3D(Basis(), Vector3(0, 100, 0));

				const RID reference = world.add_body(PhysicsServer3D::BODY_MODE_KINEMATIC, capsule_resized, parked);
				const RID scaled_body = world.add_body(PhysicsServer3D::BODY_MODE_KINEMATIC, capsule, Transform3D(scale_basis, parked.origin));
				const RID scaled_shape = world.add_body(PhysicsServer3D::BODY_MODE_KINEMATIC, capsule, parked, Transform3D(scale_basis, Vector3()));

				bool reference_collided = false;
				const real_t expected = world.test_motion(reference, Transform3D(Basis(), target.start), target.motion, reference_collided);
				CHECK(reference_collided);
				CHECK(expected < MOTION_LENGTH - 0.1);

				bool scaled_body_collided = false;
				const real_t scaled_body_travel = world.test_motion(scaled_body, Transform3D(scale_basis, target.start), target.motion, scaled_body_collided);
				CHECK(scaled_body_collided);
				CHECK(Math::abs(scaled_body_travel - expected) < TOLERANCE);

				bool scaled_shape_collided = false;
				const real_t scaled_shape_travel = world.test_motion(scaled_shape, Transform3D(Basis(), target.start), target.motion, scaled_shape_collided);
				CHECK(scaled_shape_collided);
				CHECK(Math::abs(scaled_shape_travel - expected) < TOLERANCE);
			}
		}
	}
}

} // namespace TestJoltMotionQueries
