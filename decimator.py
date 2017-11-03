import vtk
class Decimator():
    def __init__(self, *args, **kwargs):
        self.level = kwargs.get("level")



    def decimate(self, poly_data_input):
        dec = vtk.vtkDecimatePro()
        dec.SetInputData(poly_data_input)
        dec.SetTargetReduction(self.level)
        dec.Update()

        decimatedPoly = vtk.vtkPolyData()
        decimatedPoly.ShallowCopy(dec.GetOutput())

        return decimatedPoly

