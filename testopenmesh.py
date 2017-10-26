from openmesh import *


ar = TriMeshModAspectRatio()


mesh = TriMesh()


read_mesh(mesh, r"image data\lowerJawMesh.obj")

write_mesh(mesh,  r"image data\lowerJawMesh_test.obj")
mesh = TriMesh()
mesh.request_halfedge_normals()
mesh.request_vertex_normals()

options = Options()
options += Options.VertexNormal

result = read_mesh(mesh, r"image data\lowerJawMesh_test.obj", options)

if result:
       print "everything worked"
else:
       print "something went wrong"