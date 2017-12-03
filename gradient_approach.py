import vtk
import renderer
import meshOperations as meshop
from vtk.util import numpy_support

meshop = meshop.MeshOperations()



input_poly_data = meshop.read(r"./image_data/06_Gips06/lowerJawMesh.obj")
ev_dict = meshop.compute_pca(input_poly_data)
input_poly_data, _ = meshop.rotate(input_poly_data, [0,0,1], ev_dict["eigenvectors"][2])


bounds = input_poly_data.GetBounds()

s_elev = vtk.vtkElevationFilter()
s_elev.SetInputData(input_poly_data)
s_elev.SetHighPoint(0,0,bounds[5])
s_elev.SetLowPoint(0,0,bounds[4])
#s_elev.SetScalarRange(bounds[4],bounds[5])
s_elev.Update()



edges = vtk.vtkExtractEdges()
edges.SetInputData(input_poly_data)


tubes = vtk.vtkTubeFilter()
tubes.SetInputConnection(edges.GetOutputPort())
tubes.SetRadius(0.0625)
tubes.SetVaryRadiusToVaryRadiusOff()
tubes.SetNumberOfSides(32)
tubes.Update()


gradients = vtk.vtkGradientFilter()
gradients.SetInputData(s_elev.GetOutput())
gradients.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, "Elevation")
gradients.Update()

vectors = vtk.vtkAssignAttribute()
vectors.SetInputData(gradients.GetOutput())
vectors.Assign("Gradients", vtk.vtkDataSetAttributes.VECTORS, vtk.vtkAssignAttribute.POINT_DATA)
vectors.Update()

arrow = vtk.vtkArrowSource()


glyphs = vtk.vtkGlyph3D()
glyphs.SetInputConnection(0, vectors.GetOutputPort())
glyphs.SetInputConnection(1, arrow.GetOutputPort())
glyphs.ScalingOn()
glyphs.SetScaleModeToScaleByVector()
glyphs.SetScaleFactor(0.25)
glyphs.OrientOn()
glyphs.ClampingOff()
glyphs.SetVectorModeToUseVector()
glyphs.SetIndexModeToOff()
glyphs.Update()

rend = renderer.Renderer()
rend.add_actor(input_poly_data)
rend.add_actor(glyphs.GetOutput())
#rend.add_actor(tubes.GetOutput())
rend.render()