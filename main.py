import renderer
import reorient
import vtk

# my_renderer = renderer.Renderer(file_name=r".\image data\lowerJawMesh.obj", wire_frame=False)

# my_renderer.render()


reader = vtk.vtkOBJReader()

reader.SetFileName("lowerJawMesh.obj")

reader.Update()

polydata = reader.GetOutput()

my_renderer = renderer.Renderer(poly_data=polydata, wire_frame=False)
my_renderer.render()

reorientation = reorient.ReOrient(poly_data=polydata)
reorientation.applyPCA()
reorientedPolyData = reorientation.rotateX()
reorientation = reorient.ReOrient(poly_data=reorientedPolyData)
reorientation.applyPCA()
reorientedPolyData = reorientation.rotateY()


my_renderer2 = renderer.Renderer(poly_data=reorientedPolyData, wire_frame=False)
my_renderer2.render()
