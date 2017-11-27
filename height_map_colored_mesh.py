import vtk
import renderer
import meshOperations as meshop

meshop = meshop.MeshOperations()

points  = vtk.vtkPoints()


#pca to put the smallest dimension of the mesh along the Z axis

input_poly_data = meshop.read(r"./image_data/12_FI_01_idealcutline/lowerJawMesh.obj")
ev_dict = meshop.compute_pca(input_poly_data)
input_poly_data, _ = meshop.rotate(input_poly_data, [0,0,1], ev_dict["eigenvectors"][2])


# Triangulate the grid points
delaunay = vtk.vtkDelaunay2D()


delaunay.SetInputData(input_poly_data)

delaunay.Update()
output_poly = delaunay.GetOutput()


bounds = output_poly.GetBounds()

# Find min and max z
minz = bounds[4]
maxz = bounds[5]

# Create the color map
color_lut = vtk.vtkLookupTable()
color_lut.SetTableRange(minz, maxz)
color_lut.Build()

# Generate the colors for each point based on the color map
colors = vtk.vtkUnsignedCharArray()
colors.SetNumberOfComponents(3)
colors.SetName("Colors")


for i in range(output_poly.GetNumberOfPoints()):

    p = output_poly.GetPoint(i)

    dcolor = [0, 0, 0]
    color_lut.GetColor(p[2], dcolor)

    color = [0, 0, 0]
    for j in range(3):

        color[j] = (255.0 * dcolor[j])

    colors.InsertNextTuple(color)

output_poly.GetPointData().SetScalars(colors)

transform = vtk.vtkTransform()
transform.Scale(1.0,1.0,0.0)

s_elev = vtk.vtkElevationFilter()
s_elev.SetInputData(output_poly)
s_elev.SetHighPoint(0,0,bounds[5])
s_elev.SetLowPoint(0,0,bounds[4])
#s_elev.SetScalarRange(bounds[4],bounds[5])
s_elev.Update()

elev = s_elev.GetOutput()




transform_filter = vtk.vtkTransformPolyDataFilter()
transform_filter.SetInputData(output_poly)
transform_filter.SetTransform(transform)
transform_filter.Update()
output_poly = transform_filter.GetOutput()







# Render
rend = renderer.Renderer()
rend.add_actor(elev)
rend.render()
