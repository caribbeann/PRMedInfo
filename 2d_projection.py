import vtk
import meshOperations

meshOp = meshOperations.MeshOperations()

sphereSource = vtk.vtkSphereSource()
sphereSource.SetCenter(0.0, 0.0, 0.0)
sphereSource.SetRadius(5.0)
sphereSource.Update()


original_polydata = meshOp.read(r"C:\Users\oance\Documents\Tu Wien\SEM 3\PR\PRMedInfo\image_data\13_FI_02_idealcutline\lowerJawMesh.obj")
# move mesh to origin in any case

reoriented_polydata, transform = meshOp.translate_to_origin(original_polydata)

# aling along the smallest dimension in any case

reoriented_polydata, transform = meshOp.align_to_z_axis(reoriented_polydata)

# make z min 0 in any case
reoriented_polydata, transform = meshOp.translate_to_xy_plane(reoriented_polydata)

# Visualize
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputData(reoriented_polydata)

actor = vtk.vtkActor()
actor.SetMapper(mapper)

renderer = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()
renderWindow.AddRenderer(renderer)
renderWindow.SetAlphaBitPlanes(1) # enable


renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)

renderer.AddActor(actor)
renderer.SetBackground(1, 1, 1) # Background


renderWindow.Render()

# Screenshot
windowToImageFilter = vtk.vtkWindowToImageFilter()
windowToImageFilter.SetInput(renderWindow)
#windowToImageFilter.SetMagnification(3) # set

windowToImageFilter.SetInputBufferTypeToRGB() # also

windowToImageFilter.ReadFrontBufferOff() # read


windowToImageFilter.Update()

writer = vtk.vtkPNGWriter()
writer.SetFileName("screenshot2.png")
writer.SetInputConnection(windowToImageFilter.GetOutputPort())
writer.Write()

renderWindow.Render()
renderer.ResetCamera()
renderWindow.Render()
renderWindowInteractor.Start()