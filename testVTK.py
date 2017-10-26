import vtk

import vtk


def render(file):
    # Create a rendering window and renderer

    ren = vtk.vtkRenderer()

    renWin = vtk.vtkRenderWindow()

    renWin.AddRenderer(ren)

    # Create a RenderWindowInteractor to permit manipulating the camera

    iren = vtk.vtkRenderWindowInteractor()

    iren.SetRenderWindow(renWin)

    style = vtk.vtkInteractorStyleTrackballCamera()

    iren.SetInteractorStyle(style)


    polydata = loadOBJ(file)

    polydata_decimated = decimation(polydata, 0.95)

    ren.AddActor(polyDataToActor(polydata_decimated))

    ren.SetBackground(0.1, 0.1, 0.1)

    # enable user interface interactor

    iren.Initialize()

    renWin.Render()

    iren.Start()


def loadOBJ(fname):
    """Load the given STL file, and return a vtkPolyData object for it."""

    reader = vtk.vtkOBJReader()

    reader.SetFileName(fname)

    reader.Update()

    polydata = reader.GetOutput()

    return polydata


def polyDataToActor(polydata):

    """Wrap the provided vtkPolyData object in a mapper and an actor, returning

    the actor."""

    mapper = vtk.vtkPolyDataMapper()

    if vtk.VTK_MAJOR_VERSION <= 5:

        #mapper.SetInput(reader.GetOutput())

        mapper.SetInput(polydata)

    else:

        mapper.SetInputData(polydata)

    actor = vtk.vtkActor()

    actor.SetMapper(mapper)

    #actor.GetProperty().SetRepresentationToWireframe()

    actor.GetProperty().SetColor(0.5, 0.5, 1.0)

    return actor


def decimation(pd, level):



    print("Before decimation\n"
          "-----------------\n"
          "There are " + str(pd.GetNumberOfPoints()) + "points.\n"
          "There are " + str(pd.GetNumberOfPolys()) + "polygons.\n")

    decimate = vtk.vtkDecimatePro()
    decimate.SetInputData(pd)
    decimate.SetTargetReduction(level)
    decimate.Update()

    decimatedPoly = vtk.vtkPolyData()
    decimatedPoly.ShallowCopy(decimate.GetOutput())

    print("After decimation \n"
          "-----------------\n"
          "There are " + str(decimatedPoly.GetNumberOfPoints()) + "points.\n"
          "There are " + str(decimatedPoly.GetNumberOfPolys()) + "polygons.\n")

    return decimatedPoly

render(r"C:\Users\oance\Documents\Tu Wien\SEM 3\PR\PRMedInfo\image data\upperJawMesh.obj")