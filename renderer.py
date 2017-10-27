import vtk

class Renderer():
    def __init__(self, *args, **kwargs):
        self.poly_data = kwargs.get("poly_data")
        self.file_name = kwargs.get("file_name")
        self.wire_frame = kwargs.get("wire_frame")

    def render(self):
        if self.poly_data == None:
           self.poly_data = self.__loadOBJ()

        # Create a rendering window and renderer

        ren = vtk.vtkRenderer()

        transform = vtk.vtkTransform()
        transform.Translate(1.0, 0.0, 0.0)
        transform.Scale(50,50,50)
         
        axes = vtk.vtkAxesActor()
        axes.SetUserTransform(transform)
        ren.AddActor(axes)

        ren.ResetCamera()

        renWin = vtk.vtkRenderWindow()

        renWin.AddRenderer(ren)

        # Create a RenderWindowInteractor to permit manipulating the camera

        iren = vtk.vtkRenderWindowInteractor()

        iren.SetRenderWindow(renWin)

        style = vtk.vtkInteractorStyleTrackballCamera()

        iren.SetInteractorStyle(style)


        ren.AddActor(self.__polyDataToActor())

        ren.SetBackground(0.1, 0.1, 0.1)

        # enable user interface interactor

        iren.Initialize()

        renWin.Render()

        iren.Start()

    def __loadOBJ(self):

        reader = vtk.vtkOBJReader()

        reader.SetFileName(self.file_name)

        reader.Update()

        polydata = reader.GetOutput()

        return polydata

    def __polyDataToActor(self):

        """Wrap the provided vtkPolyData object in a mapper and an actor, returning

        the actor."""

        mapper = vtk.vtkPolyDataMapper()

        if vtk.VTK_MAJOR_VERSION <= 5:

            # mapper.SetInput(reader.GetOutput())

            mapper.SetInput(self.poly_data)

        else:

            mapper.SetInputData(self.poly_data)

        actor = vtk.vtkActor()

        actor.SetMapper(mapper)

        if self.wire_frame:
            actor.GetProperty().SetRepresentationToWireframe()

        actor.GetProperty().SetColor(0.5, 0.5, 1.0)

        return actor
