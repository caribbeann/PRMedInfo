import vtk


class Renderer:

    def __init__(self):
        self.ren = vtk.vtkRenderer()

    def render(self):
        """
        Creates vtkAxesActor, a render window, an interactor and starts the rendering
        :return:
        """

        # Create a rendering window and renderer
        transform = vtk.vtkTransform()
        transform.Translate(1.0, 0.0, 0.0)
        transform.Scale(50, 50, 50)

        # add axes
        axes = vtk.vtkAxesActor()
        axes.SetUserTransform(transform)
        self.ren.AddActor(axes)

        self.ren.ResetCamera()

        ren_win = vtk.vtkRenderWindow()

        ren_win.AddRenderer(self.ren)

        # Create a RenderWindowInteractor to permit manipulating the camera

        iren = vtk.vtkRenderWindowInteractor()

        iren.SetRenderWindow(ren_win)

        self.ren.SetBackground(0.1, 0.1, 0.1)

        # enable user interface interactor

        iren.Initialize()

        ren_win.Render()

        iren.Start()

    def add_actor(self, input_poly_data, **kwargs):
        """
        Transforms a vtkPolyData to an actor and adds it to the renderer
        :param input_poly_data: vtkPolyData to be transformed
        :param kwargs: color, wireframe boolean, linewidth, etc
        :return: vtkActor
        """

        mapper = vtk.vtkPolyDataMapper()

        if vtk.VTK_MAJOR_VERSION <= 5:

            # mapper.SetInput(reader.GetOutput())

            mapper.SetInput(input_poly_data)

        else:

            mapper.SetInputData(input_poly_data)

        actor = vtk.vtkActor()

        actor.SetMapper(mapper)

        if "wireframe" in kwargs:
            if kwargs.get("wireframe"):
                actor.GetProperty().SetRepresentationToWireframe()

        if "color" in kwargs:
            actor.GetProperty().SetColor(kwargs.get("color"))

        if "linewidth" in kwargs:
            actor.GetProperty().SetLineWidth(kwargs.get("linewidth"))

        self.ren.AddActor(actor)
        return actor