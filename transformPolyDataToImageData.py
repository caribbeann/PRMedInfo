## turn the poly data to image data

# prepare the binary image's voxel grid
whiteImage = vtk.vtkImageData()
bounds = last_crop_poly.GetBounds()
spacing = [0.5, 0.5, 0.5]
whiteImage.SetSpacing(spacing)

# compute dimensions

dim = np.zeros((3))
for i in range (3):
    dim[i] = int(math.ceil((bounds[i * 2 + 1] - bounds[i * 2]) / spacing[i])) + 1
    if dim[i] < 1:
      dim[i] = 1

dim = dim.astype("uint16")
whiteImage.SetDimensions(dim)
whiteImage.SetExtent(0, dim[0] - 1, 0, dim[1] - 1, 0, dim[2] - 1)
origin = np.zeros((3))

origin[0] = bounds[0]
origin[1] = bounds[2]
origin[2] = bounds[4]
whiteImage.SetOrigin(origin)

whiteImage.AllocateScalars(vtk.VTK_UNSIGNED_CHAR,1)


# fill the image with foreground voxels:
inval = 255
outval = 0
count = whiteImage.GetNumberOfPoints()
for i in range(count):
    whiteImage.GetPointData().GetScalars().SetTuple1(i, inval)

# sweep polygonal data (this is the important thing with contours!)
extruder = vtk.vtkLinearExtrusionFilter()
extruder.SetInputData(last_crop_poly)
extruder.SetScaleFactor(1.)
extruder.SetExtrusionTypeToNormalExtrusion()
extruder.SetVector(0, 0, 1)
extruder.Update()


# polygonal data --> image stencil:
pol2stenc = vtk.vtkPolyDataToImageStencil()
pol2stenc.SetTolerance(0)
pol2stenc.SetInputConnection(extruder.GetOutputPort())
pol2stenc.SetOutputOrigin(origin)
pol2stenc.SetOutputSpacing(spacing)
pol2stenc.SetOutputWholeExtent(whiteImage.GetExtent())
pol2stenc.Update()

# cut the corresponding white image and set the background:
imgstenc = vtk.vtkImageStencil()
imgstenc.SetInputData(whiteImage)
imgstenc.SetStencilConnection(pol2stenc.GetOutputPort())

imgstenc.ReverseStencilOff()
imgstenc.SetBackgroundValue(outval)
imgstenc.Update()


mapper = vtk.vtkDataSetMapper()
mapper.SetInputData(imgstenc.GetOutput())

actor = vtk.vtkActor()
actor.SetMapper(mapper)

renderWindow = vtk.vtkRenderWindow()
renderer = vtk.vtkRenderer()


renderWindow.AddRenderer(renderer)

renderer.AddActor(actor)
renderer.ResetCamera()
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)
renderWindow.Render()
renderWindowInteractor.Start()