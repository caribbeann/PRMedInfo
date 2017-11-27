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

        self.ren_win = ren_win

        #self.ren.SetBackground(0.1, 0.1, 0.1)

        # enable user interface interactor

        iren.Initialize()

        ren_win.Render()

        iren.Start()



    def add_camera(self, position = [0, 0, 20], focal_point = [0, 0 ,0]):
        """
        Adds a camera to this renderer
        :param position: position coordinates xyz of the camera
        :param focal_point: point coordinates xyz that the camrea looks at
        :return: camera created using the given parameters
        """
        camera = vtk.vtkCamera()
        camera.SetPosition(position)
        camera.SetFocalPoint(focal_point)

        self.ren.SetActiveCamera(camera)

        return camera


    def get_ren_win(self):
        """
        Getter function to obtain the rendering window
        :return:
        """
        return self.ren_win

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

    def set_background(self, r, g, b):
        """
        Sets the color of the rendering background to the given color
        :param color: [r, g, b] float
        :return:
        """
        self.ren.SetBackground(r, g ,b)