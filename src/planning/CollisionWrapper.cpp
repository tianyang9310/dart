#include <fcl/simple_setup.h>
#include "CollisionWrapper.h"

void CollisionWrapper::getModel_3DSToFCL(Model3DS _model3DS,
		std::vector<fcl::Triangle> *_triangles,
		std::vector<fcl::Vec3f> *_vertices) {
	int numVertices;
	double* vertices;
	int numIndices;
	short int* indicesTriangles;

	_model3DS.Get(&numVertices, vertices, &numIndices, indicesTriangles);

	for (int i = 0; i < numVertices; i += 3)
		_vertices->push_back(fcl::Vec3f(vertices[i], vertices[i + 1], vertices[i + 2]));

	for (int i = 0; i < numIndices; i += 3)
		_triangles->push_back(fcl::Triangle(indicesTriangles[i], indicesTriangles[i + 1],
						indicesTriangles[i + 2]));

}

fcl::Vec3f CollisionWrapper::getTrans_EigenToFCL(Eigen::Transform<double, 3, Eigen::Affine> _tf) {
	return fcl::Vec3f(_tf.translation()(0), _tf.translation()(1), _tf.translation()(2));
}

fcl::Vec3f CollisionWrapper::getRotation_EigenToFCL(Eigen::Transform<double, 3, Eigen::Affine> _tf, int col) {
	return fcl::Vec3f(_tf.rotation()(col, 0), _tf.rotation()(col, 1), _tf.rotation()(col, 2));
}