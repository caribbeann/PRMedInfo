import renderer
import meshOperations
import vtk

# my_renderer = renderer.Renderer(file_name=r".\image data\lowerJawMesh.obj", wire_frame=False)

# my_renderer.render()


reader = vtk.vtkOBJReader()

reader.SetFileName(r".\image data\lowerJawMesh.obj")

reader.Update()

polydata = reader.GetOutput()

print polydata.GetBounds()[0]
print polydata.GetBounds()[1]
print polydata.GetBounds()[2]
print polydata.GetBounds()[3]
print polydata.GetBounds()[4]
print polydata.GetBounds()[5]

my_renderer = renderer.Renderer(poly_data=polydata, wire_frame=False)
my_renderer.render()

meshOp = meshOperations.MeshOperations(poly_data=polydata)
PCADict = meshOp.computePCA()
reorientedPolyData = meshOp.rotate([1.0,0.0,0.0],PCADict['eigenvectors'][0])
meshOp.changePolyData(reorientedPolyData)
PCADict2 = meshOp.computePCA()
reorientedPolyData = meshOp.rotate([0.0,1.0,0.0],PCADict2['eigenvectors'][1])


my_renderer2 = renderer.Renderer(poly_data=reorientedPolyData, wire_frame=False)
my_renderer2.render()
