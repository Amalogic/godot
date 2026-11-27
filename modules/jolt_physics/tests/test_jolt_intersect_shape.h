/**************************************************************************/
/*  test_jolt_intersect_shape.h                                           */
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
#include "../jolt_project_settings.h"

#include "tests/test_macros.h"

namespace TestJoltIntersectShape {

// One row of an `intersect_shape` result.
struct Row {
	RID rid;
	int shape = 0;

	bool operator==(const Row &p_other) const { return rid == p_other.rid && shape == p_other.shape; }
};

// The test runner only creates a physics server for `[SceneTree]` and `[Editor]` tests, so these tests
// own a Jolt server of their own for their whole duration.
class JoltTestWorld {
	LocalVector<RID> objects;
	LocalVector<RID> shapes;
	bool edge_removal_before = false;

public:
	JoltPhysicsServer3D *server = nullptr;
	RID space;

	JoltTestWorld() {
		REQUIRE_MESSAGE(PhysicsServer3D::get_singleton() == nullptr, "These tests need to own the physics server.");
		edge_removal_before = JoltProjectSettings::use_enhanced_internal_edge_removal_for_queries;
		server = memnew(JoltPhysicsServer3D(false));
		server->init();
		space = server->space_create();
		server->space_set_active(space, true);
	}

	~JoltTestWorld() {
		for (const RID &object : objects) {
			server->free_rid(object);
		}
		for (const RID &shape : shapes) {
			server->free_rid(shape);
		}
		server->free_rid(space);
		server->finish();
		memdelete(server);
		JoltProjectSettings::use_enhanced_internal_edge_removal_for_queries = edge_removal_before;
	}

	RID box(const Vector3 &p_half_extents) {
		const RID shape = server->box_shape_create();
		server->shape_set_data(shape, p_half_extents);
		shapes.push_back(shape);
		return shape;
	}

	RID sphere(real_t p_radius) {
		const RID shape = server->sphere_shape_create();
		server->shape_set_data(shape, p_radius);
		shapes.push_back(shape);
		return shape;
	}

	RID mesh(const PackedVector3Array &p_faces, bool p_backface_collision) {
		Dictionary data;
		data["faces"] = p_faces;
		data["backface_collision"] = p_backface_collision;
		const RID shape = server->concave_polygon_shape_create();
		server->shape_set_data(shape, data);
		shapes.push_back(shape);
		return shape;
	}

	// A triangulated sphere facing outwards, like the one `SphereMesh.create_trimesh_shape()` gives.
	RID sphere_mesh(real_t p_radius, int p_rings = 16, int p_segments = 32) {
		PackedVector3Array faces;

		const auto point = [&](int p_ring, int p_segment) {
			const real_t polar = Math::PI * p_ring / p_rings;
			const real_t azimuth = Math::TAU * p_segment / p_segments;
			return Vector3(Math::sin(polar) * Math::cos(azimuth), Math::cos(polar), Math::sin(polar) * Math::sin(azimuth)) * p_radius;
		};

		const auto add_face = [&](const Vector3 &p_a, Vector3 p_b, Vector3 p_c) {
			// Front faces are clockwise.
			if ((p_a - p_c).cross(p_a - p_b).dot(p_a + p_b + p_c) < 0.0) {
				SWAP(p_b, p_c);
			}
			faces.push_back(p_a);
			faces.push_back(p_b);
			faces.push_back(p_c);
		};

		for (int ring = 0; ring < p_rings; ring++) {
			for (int segment = 0; segment < p_segments; segment++) {
				// The first and the last ring are triangle fans around the poles.
				if (ring > 0) {
					add_face(point(ring, segment), point(ring, segment + 1), point(ring + 1, segment + 1));
				}
				if (ring < p_rings - 1) {
					add_face(point(ring, segment), point(ring + 1, segment + 1), point(ring + 1, segment));
				}
			}
		}

		return mesh(faces, false);
	}

	// A quad made of the two triangles `a b c` and `a c d`, colliding on both sides.
	RID quad(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c, const Vector3 &p_d) {
		return mesh(PackedVector3Array({ p_a, p_b, p_c, p_a, p_c, p_d }), true);
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

	RID add_body(const Vector3 &p_position, uint32_t p_collision_layer = 1) {
		const RID body = server->body_create();
		server->body_set_mode(body, PhysicsServer3D::BODY_MODE_STATIC);
		server->body_set_collision_layer(body, p_collision_layer);
		server->body_set_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM, Transform3D(Basis(), p_position));
		server->body_set_space(body, space);
		objects.push_back(body);
		return body;
	}

	RID add_body(const Vector3 &p_position, RID p_shape, uint32_t p_collision_layer = 1) {
		const RID body = add_body(p_position, p_collision_layer);
		add_shape(body, p_shape);
		return body;
	}

	void add_shape(RID p_body, RID p_shape, const Vector3 &p_offset = Vector3(), bool p_disabled = false) {
		server->body_add_shape(p_body, p_shape, Transform3D(Basis(), p_offset), p_disabled);
	}

	RID add_area(const Vector3 &p_position) {
		const RID area = server->area_create();
		server->area_set_transform(area, Transform3D(Basis(), p_position));
		server->area_set_space(area, space);
		objects.push_back(area);
		return area;
	}

	RID add_area(const Vector3 &p_position, RID p_shape) {
		const RID area = add_area(p_position);
		server->area_add_shape(area, p_shape, Transform3D(), false);
		return area;
	}

	void set_edge_removal(bool p_enabled) {
		JoltProjectSettings::use_enhanced_internal_edge_removal_for_queries = p_enabled;
	}

	LocalVector<Row> intersect_shape(const PhysicsDirectSpaceState3D::ShapeParameters &p_parameters, int p_max_results = 4096) {
		PhysicsDirectSpaceState3D *state = server->space_get_direct_state(space);
		REQUIRE(state != nullptr);

		LocalVector<PhysicsDirectSpaceState3D::ShapeResult> results;
		results.resize(p_max_results);
		const int count = state->intersect_shape(p_parameters, results.ptr(), p_max_results);
		REQUIRE(count >= 0);
		REQUIRE(count <= p_max_results);

		LocalVector<Row> rows;
		for (int i = 0; i < count; i++) {
			rows.push_back({ results[i].rid, results[i].shape });
		}
		return rows;
	}

	// The number of contact pairs `collide_shape` returns.
	int collide_shape(const PhysicsDirectSpaceState3D::ShapeParameters &p_parameters, int p_max_results = 4096) {
		PhysicsDirectSpaceState3D *state = server->space_get_direct_state(space);
		REQUIRE(state != nullptr);

		LocalVector<Vector3> points;
		points.resize(p_max_results * 2);
		int count = 0;
		state->collide_shape(p_parameters, points.ptr(), p_max_results, count);
		return count;
	}
};

int count_row(const LocalVector<Row> &p_rows, const Row &p_row) {
	int count = 0;
	for (const Row &row : p_rows) {
		if (row == p_row) {
			count++;
		}
	}
	return count;
}

// Checks that the result holds the expected rows, each of them once, in any order.
void check_rows(const LocalVector<Row> &p_rows, const LocalVector<Row> &p_expected) {
	CHECK(p_rows.size() == p_expected.size());
	for (uint32_t i = 0; i < p_expected.size(); i++) {
		INFO("expected row ", i, ", shape index ", p_expected[i].shape);
		CHECK(count_row(p_rows, p_expected[i]) == 1);
	}
}

// Checks that the result holds `p_count` different rows, all of them expected ones.
void check_distinct_rows(const LocalVector<Row> &p_rows, const LocalVector<Row> &p_expected, int p_count) {
	CHECK((int)p_rows.size() == p_count);
	for (uint32_t i = 0; i < p_rows.size(); i++) {
		INFO("returned row ", i, ", shape index ", p_rows[i].shape);
		CHECK(count_row(p_rows, p_rows[i]) == 1);
		CHECK(count_row(p_expected, p_rows[i]) == 1);
	}
}

struct Scene {
	PhysicsDirectSpaceState3D::ShapeParameters query;
	LocalVector<Row> expected;
};

// A 2 m query box at the origin, overlapping convex shapes, triangle meshes and a height field, on
// bodies with one or several shapes.
Scene add_scene(JoltTestWorld &p_world) {
	Scene scene;
	scene.query.shape_rid = p_world.box(Vector3(1, 1, 1));

	const RID small_box = p_world.box(Vector3(0.25, 0.25, 0.25));

	const RID concave_a = p_world.add_body(Vector3(1.6, 0, 0), p_world.sphere_mesh(1.0));
	scene.expected.push_back({ concave_a, 0 });

	const RID concave_b = p_world.add_body(Vector3(0, 0, -1.6), p_world.sphere_mesh(1.0));
	scene.expected.push_back({ concave_b, 0 });

	const RID two_boxes = p_world.add_body(Vector3(-1.1, 0, 0));
	p_world.add_shape(two_boxes, small_box, Vector3(0, 0.5, 0));
	p_world.add_shape(two_boxes, small_box, Vector3(0, -0.5, 0));
	scene.expected.push_back({ two_boxes, 0 });
	scene.expected.push_back({ two_boxes, 1 });

	const RID crate = p_world.add_body(Vector3(0, 1.1, 0), small_box);
	scene.expected.push_back({ crate, 0 });

	const RID floor = p_world.add_body(Vector3(0, -0.9, 0), p_world.height_map());
	scene.expected.push_back({ floor, 0 });

	// A disabled mesh, a box and a mesh. The disabled shape keeps its index.
	const RID mixed = p_world.add_body(Vector3(0, 0, 1.6));
	p_world.add_shape(mixed, p_world.sphere_mesh(1.0), Vector3(), true);
	p_world.add_shape(mixed, small_box, Vector3(0, 0, -0.5));
	p_world.add_shape(mixed, p_world.sphere_mesh(1.0));
	scene.expected.push_back({ mixed, 1 });
	scene.expected.push_back({ mixed, 2 });

	// Out of reach.
	p_world.add_body(Vector3(5, 0, 0), small_box);
	p_world.add_body(Vector3(0, 5, 0), p_world.sphere_mesh(1.0));

	return scene;
}

TEST_CASE("[JoltPhysics] intersect_shape returns each intersected shape once") {
	for (const bool edge_removal : { false, true }) {
		JoltTestWorld world;
		world.set_edge_removal(edge_removal);
		const Scene scene = add_scene(world);

		INFO("enhanced internal edge removal: ", edge_removal);

		check_rows(world.intersect_shape(scene.query), scene.expected);
	}
}

TEST_CASE("[JoltPhysics] intersect_shape counts each intersected shape once towards max_results") {
	for (const bool edge_removal : { false, true }) {
		JoltTestWorld world;
		world.set_edge_removal(edge_removal);
		const Scene scene = add_scene(world);
		const int expected_count = scene.expected.size();

		for (int max_results = 1; max_results <= expected_count + 2; max_results++) {
			INFO("enhanced internal edge removal: ", edge_removal, ", max_results: ", max_results);

			check_distinct_rows(world.intersect_shape(scene.query, max_results), scene.expected, MIN(max_results, expected_count));
		}
	}
}

TEST_CASE("[JoltPhysics] intersect_shape returns every body when many triangle meshes are intersected") {
	JoltTestWorld world;

	// More bodies than the default `max_results` of 32, all of them inside the query box.
	LocalVector<Row> expected;
	const RID sphere_mesh = world.sphere_mesh(0.5);
	for (int x = 0; x < 8; x++) {
		for (int z = 0; z < 5; z++) {
			expected.push_back({ world.add_body(Vector3(x * 2 - 7, 0, z * 2 - 4), sphere_mesh), 0 });
		}
	}

	PhysicsDirectSpaceState3D::ShapeParameters query;
	query.shape_rid = world.box(Vector3(10, 2, 10));

	check_rows(world.intersect_shape(query), expected);
	check_distinct_rows(world.intersect_shape(query, 32), expected, 32);
	check_distinct_rows(world.intersect_shape(query, 40), expected, 40);
}

TEST_CASE("[JoltPhysics] intersect_shape returns every shape of a body or area with several shapes") {
	JoltTestWorld world;

	// A mesh, a box and a mesh in a row along the X axis, all of them reaching into the query box.
	const RID sphere_mesh = world.sphere_mesh(0.5);
	const RID small_box = world.box(Vector3(0.25, 0.25, 0.25));
	const Transform3D shape_transforms[3] = {
		Transform3D(Basis(), Vector3(-1.5, 0, 0)),
		Transform3D(Basis(), Vector3()),
		Transform3D(Basis(), Vector3(1.5, 0, 0)),
	};

	PhysicsDirectSpaceState3D::ShapeParameters query;
	query.shape_rid = world.box(Vector3(3, 1, 1));

	// The scale and the center of mass each change how the shapes of a body are put together.
	Transform3D transform;
	bool custom_center_of_mass = false;
	bool area = false;

	SUBCASE("Body") {
	}

	SUBCASE("Scaled body") {
		transform.basis.scale(Vector3(2, 2, 2));
	}

	SUBCASE("Body with a custom center of mass") {
		custom_center_of_mass = true;
	}

	SUBCASE("Scaled body with a custom center of mass") {
		transform.basis.scale(Vector3(2, 2, 2));
		custom_center_of_mass = true;
	}

	SUBCASE("Area") {
		area = true;
	}

	SUBCASE("Scaled area") {
		transform.basis.scale(Vector3(2, 2, 2));
		area = true;
	}

	RID object;

	if (area) {
		object = world.add_area(Vector3());
		world.server->area_add_shape(object, sphere_mesh, shape_transforms[0], false);
		world.server->area_add_shape(object, small_box, shape_transforms[1], false);
		world.server->area_add_shape(object, sphere_mesh, shape_transforms[2], false);
		world.server->area_set_transform(object, transform);

		query.collide_with_areas = true;
		query.collide_with_bodies = false;
	} else {
		object = world.add_body(Vector3());
		world.server->body_add_shape(object, sphere_mesh, shape_transforms[0], false);
		world.server->body_add_shape(object, small_box, shape_transforms[1], false);
		world.server->body_add_shape(object, sphere_mesh, shape_transforms[2], false);
		world.server->body_set_state(object, PhysicsServer3D::BODY_STATE_TRANSFORM, transform);

		if (custom_center_of_mass) {
			world.server->body_set_param(object, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS, Vector3(0.3, 0.5, 0.2));
		}
	}

	const LocalVector<Row> expected = { { object, 0 }, { object, 1 }, { object, 2 } };

	check_rows(world.intersect_shape(query), expected);
	check_distinct_rows(world.intersect_shape(query, 2), expected, 2);
}

TEST_CASE("[JoltPhysics] intersect_shape returns the same shapes with and without enhanced internal edge removal") {
	for (const bool edge_removal : { false, true }) {
		JoltTestWorld world;
		world.set_edge_removal(edge_removal);

		// Two meshes on one body, sharing the edge along the Z axis: a floor on the -X side, and a slope
		// going down on the +X side.
		const RID floor = world.quad(Vector3(-4, 0, -4), Vector3(0, 0, -4), Vector3(0, 0, 4), Vector3(-4, 0, 4));
		const RID slope = world.quad(Vector3(0, 0, -4), Vector3(4, -4, -4), Vector3(4, -4, 4), Vector3(0, 0, 4));
		const RID body = world.add_body(Vector3());
		world.add_shape(body, floor);
		world.add_shape(body, slope);

		// A sphere resting on the floor, and reaching over the edge into the slope.
		PhysicsDirectSpaceState3D::ShapeParameters query;
		query.shape_rid = world.sphere(0.5);
		query.transform.origin = Vector3(-0.2, 0.3, 0);

		INFO("enhanced internal edge removal: ", edge_removal);

		check_rows(world.intersect_shape(query), { { body, 0 }, { body, 1 } });
	}
}

TEST_CASE("[JoltPhysics] intersect_shape applies the margin, the collision mask and the exclusions") {
	JoltTestWorld world;

	const RID small_box = world.box(Vector3(0.25, 0.25, 0.25));

	// On layer 1, on layer 2, and an area on layer 1.
	const RID mesh_body = world.add_body(Vector3(1.6, 0, 0), world.sphere_mesh(1.0), 1);
	const RID masked_body = world.add_body(Vector3(-1.6, 0, 0), world.sphere_mesh(1.0), 2);
	const RID area = world.add_area(Vector3(0, 0, -1.6), world.sphere_mesh(1.0));

	// 0.15 m and 0.2 m away from the query box.
	const RID near_box = world.add_body(Vector3(0, 0, 1.4), small_box);
	const RID near_wall = world.add_body(Vector3(0, 1.2, 0), world.quad(Vector3(-4, 0, -4), Vector3(4, 0, -4), Vector3(4, 0, 4), Vector3(-4, 0, 4)));

	PhysicsDirectSpaceState3D::ShapeParameters query;
	query.shape_rid = world.box(Vector3(1, 1, 1));

	SUBCASE("Collision mask") {
		query.collision_mask = 1;
		check_rows(world.intersect_shape(query), { { mesh_body, 0 } });

		query.collision_mask = 2;
		check_rows(world.intersect_shape(query), { { masked_body, 0 } });

		query.collision_mask = 3;
		check_rows(world.intersect_shape(query), { { mesh_body, 0 }, { masked_body, 0 } });
	}

	SUBCASE("Areas and bodies") {
		query.collide_with_areas = true;
		check_rows(world.intersect_shape(query), { { mesh_body, 0 }, { masked_body, 0 }, { area, 0 } });

		query.collide_with_bodies = false;
		check_rows(world.intersect_shape(query), { { area, 0 } });
	}

	SUBCASE("Exclusions") {
		query.exclude.insert(mesh_body);
		check_rows(world.intersect_shape(query), { { masked_body, 0 } });

		// An excluded body does not use up `max_results`.
		check_rows(world.intersect_shape(query, 1), { { masked_body, 0 } });
	}

	SUBCASE("Margin") {
		query.margin = 0.1;
		check_rows(world.intersect_shape(query), { { mesh_body, 0 }, { masked_body, 0 } });

		query.margin = 0.5;
		check_rows(world.intersect_shape(query), { { mesh_body, 0 }, { masked_body, 0 }, { near_box, 0 }, { near_wall, 0 } });
	}
}

TEST_CASE("[JoltPhysics] collide_shape still returns the contacts of every triangle") {
	JoltTestWorld world;

	world.add_body(Vector3(1.6, 0, 0), world.sphere_mesh(1.0));

	PhysicsDirectSpaceState3D::ShapeParameters query;
	query.shape_rid = world.box(Vector3(1, 1, 1));

	CHECK(world.collide_shape(query) > 32);
	CHECK(world.collide_shape(query, 8) == 8);
}

} // namespace TestJoltIntersectShape
