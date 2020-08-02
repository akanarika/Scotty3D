#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 {
namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, vector<size_t>& v) : mesh(mesh), v(v) {}
Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3)
    : mesh(mesh), v1(v1), v2(v2), v3(v3) {}

BBox Triangle::get_bbox() const {
  // TODO (PathTracer):
  // compute the bounding box of the triangle
	double min_x, max_x, min_y, max_y, min_z, max_z;
    
    min_x = min(mesh->positions[v1].x, min(mesh->positions[v2].x, mesh->positions[v3].x));
    min_y = min(mesh->positions[v1].y, min(mesh->positions[v2].y, mesh->positions[v3].y));
    min_z = min(mesh->positions[v1].z, min(mesh->positions[v2].z, mesh->positions[v3].z));
    max_x = max(mesh->positions[v1].x, max(mesh->positions[v2].x, mesh->positions[v3].x));
    max_y = max(mesh->positions[v1].y, max(mesh->positions[v2].y, mesh->positions[v3].y));
    max_z = max(mesh->positions[v1].z, max(mesh->positions[v2].z, mesh->positions[v3].z));
    
    BBox b = BBox(min_x, min_y, min_z, max_x, max_y, max_z);
        
    return b; 

}

bool Triangle::intersect(const Ray& r) const {
    Vector3D p0 = mesh->positions[v1];
    Vector3D p1 = mesh->positions[v2];
    Vector3D p2 = mesh->positions[v3];

    Vector3D e1 = p1 - p0;
    Vector3D e2 = p2 - p0;
    Vector3D s = r.o - p0;
    if (dot(r.d, cross(e1, e2)) == 0) return false;

    double fac = 1.0 / dot(cross(e1, r.d), e2);

    double u = dot(cross(e2, s), r.d) * fac;
    double v = dot(cross(e1, r.d), s) * fac;
    double t = dot(cross(e2, s), e1) * fac;
    if (t < r.min_t || t > r.max_t) return false;
    if (u < 0 || u > 1) return false;
    if (v < 0 || v > 1 || u + v > 1) return false;

    return true;
}

bool Triangle::intersect(const Ray& r, Intersection* isect) const {
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
    Vector3D p0 = mesh->positions[v1];
    Vector3D p1 = mesh->positions[v2];
    Vector3D p2 = mesh->positions[v3];

    Vector3D e1 = p1 - p0;
    Vector3D e2 = p2 - p0;
    Vector3D s = r.o - p0;
    if (dot(r.d, cross(e1, e2)) == 0) return false;

    double fac = 1.0 / dot(cross(e1, r.d), e2);

    double u = dot(cross(e2, s), r.d) * fac;
    double v = dot(cross(e1, r.d), s) * fac;
    double t = dot(cross(e2, s), e1) * fac;
    if (t < r.min_t || t > r.max_t) return false;
    if (u < 0 || u > 1) return false;
    if (v < 0 || v > 1 || u + v > 1) return false;

    r.max_t = t;

    isect->t = t;
    isect->n = (1 - u - v) * mesh->normals[v1] + u * mesh->normals[v2] + v * mesh->normals[v3];
    if (dot(isect->n, r.d) >= 0) isect->n = -isect->n;
    isect->primitive = this;
    isect->bsdf = mesh->get_bsdf();

    return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

}  // namespace StaticScene
}  // namespace CMU462
