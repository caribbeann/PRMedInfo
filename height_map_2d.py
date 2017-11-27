import vtk
import renderer
import meshOperations as meshop
from vtk.util import numpy_support

meshop = meshop.MeshOperations()



input_poly_data = meshop.read(r"./image_data/12_FI_01_idealcutline/lowerJawMesh.obj")
ev_dict = meshop.compute_pca(input_poly_data)
input_poly_data, _ = meshop.rotate(input_poly_data, [0,0,1], ev_dict["eigenvectors"][2])

bounds = input_poly_data.GetBounds()

s_elev = vtk.vtkElevationFilter()
s_elev.SetInputData(input_poly_data)
s_elev.SetHighPoint(0,0,bounds[5])
s_elev.SetLowPoint(0,0,bounds[4])
#s_elev.SetScalarRange(bounds[4],bounds[5])
s_elev.Update()

output = s_elev.GetOutput()


elevation = vtk.vtkFloatArray.SafeDownCast(output.GetPointData().GetArray("Elevation"))

numpy_data = numpy_support.vtk_to_numpy(elevation)


# WHAT IS THE SHAPE OF THE NEW ARRAY?

print(numpy_data)