#pragma once
#include <VHACD.h>

//
//	Structure containing objects created during composition
struct DecompResults {
	btCompoundShape* CompoundShape = nullptr;
	btAlignedObjectArray<btConvexShape*> m_convexShapes = {};
	btAlignedObjectArray<btTriangleMesh*> m_trimeshes = {};
};

//
//	FBXObject contains vectors of Vertices and Indices
DecompResults* Decomp(FBXObject* FBX) {
	//
	//	Setup Indices
	const uint32_t nTriangles = FBX->Indices.size();
	std::vector<uint32_t> Triangles;
	for (uint32_t i = 0; i < nTriangles; i++) {
		Triangles.push_back(FBX->Indices[i]);
	}
	//
	//	Setup Points (3 Points is 1 Vertex)
	const uint32_t nPoints = FBX->Vertices.size();
	std::vector<float> Points;
	for (uint32_t i = 0; i < nPoints; i++) {
		Points.push_back(FBX->Vertices[i].pos.x);
		Points.push_back(FBX->Vertices[i].pos.y);
		Points.push_back(FBX->Vertices[i].pos.z);
	}
	//
	//	Setup VHACD Parameters and create its interface
	VHACD::IVHACD::Parameters params;
	VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
	VHACD::IVHACD::ConvexHull Hull;
	//
	//	Compute approximate convex decomposition
	//printf("Compute V-HACD: Points %i Triangles %i\n", Points.size(), Triangles.size());
	bool res = interfaceVHACD->Compute(Points.data(), (uint32_t)(Points.size() / 3),
		Triangles.data(), (uint32_t)(Triangles.size() / 3), params);
	//
	//	Get the number of convex hulls
	unsigned int nConvexHulls = interfaceVHACD->GetNConvexHulls();
	//printf("V-HACD Done: Hull Count %i\n", nConvexHulls);
	//
	//	Create a new DecompResults structure
	DecompResults* Results = new DecompResults;
	//
	//	Create a new Compound Shape for this decomposition
	Results->CompoundShape = new btCompoundShape();
	//
	//	Iterate through each convex hull and fill results
	for (unsigned int h = 0; h < nConvexHulls; ++h)
	{
		//printf("\tHull: %i\n", h);
		//printf("\t\tPoints: %i\n", Hull.m_points);
		//printf("\t\tTriangles: %i\n", Hull.m_triangles);
		//printf("\t\tVertices: %i\n", vertices.size());
		//
		//	Fill 'Hull' for each individual convex hull
		interfaceVHACD->GetConvexHull(h, Hull);
		//
		//	Create a new Triangle Mesh for this hull
		btTriangleMesh* trimesh = new btTriangleMesh();
		Results->m_trimeshes.push_back(trimesh);
		//
		//	Grab the hulls center position
		const btVector3 centroid(Hull.m_center[0], Hull.m_center[1], Hull.m_center[2]);
		printf("Hull Center %f %f %f\n", Hull.m_center[0], Hull.m_center[1], Hull.m_center[2]);
		//
		//	Iterate through this hulls triangles
		for (unsigned int i = 0; i < Hull.m_nTriangles; i++) {
			//
			//	Calculate indices
			const unsigned int index0 = Hull.m_triangles[i * 3];
			const unsigned int index1 = Hull.m_triangles[i * 3 + 1];
			const unsigned int index2 = Hull.m_triangles[i * 3 + 2];
			//
			//	Calculate vertices
			const btVector3 vertex0(Hull.m_points[index0 * 3], Hull.m_points[index0 * 3 + 1], Hull.m_points[index0 * 3 + 2]);
			const btVector3 vertex1(Hull.m_points[index1 * 3], Hull.m_points[index1 * 3 + 1], Hull.m_points[index1 * 3 + 2]);
			const btVector3 vertex2(Hull.m_points[index2 * 3], Hull.m_points[index2 * 3 + 1], Hull.m_points[index2 * 3 + 2]);
			//
			//	Add this triangle into our Triangle Mesh
			trimesh->addTriangle(vertex0 - centroid, vertex1 - centroid, vertex2 - centroid);
		}
		//
		//	Create a new ConvexShape from this hulls Triangle Mesh
		btConvexShape* convexShape = new btConvexTriangleMeshShape(trimesh);
		Results->m_convexShapes.push_back(convexShape);
		//
		//	Create a transform using centroid as origin
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(centroid);
		//
		//	Add this ConvexShape to our CompoundShape
		Results->CompoundShape->addChildShape(trans, convexShape);
	}
	//
	// release memory
	interfaceVHACD->Clean();
	interfaceVHACD->Release();
	//
	//	Return our DecompResults containing the CompoundShape full of Convexically Decomposed Convex Shapes
	return Results;
}
