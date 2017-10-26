import vtk
class Decimator():
    def __init__(self, *args, **kwargs):
        self.level = kwargs.get("level")
        self.poly_data = kwargs.get("poly_data")


    def decimate(self):
        dec = vtk.vtkDecimatePro()
        dec.SetInputData(self.poly_data)
        dec.SetTargetReduction(self.level)
        dec.Update()

        decimatedPoly = vtk.vtkPolyData()
        decimatedPoly.ShallowCopy(dec.GetOutput())
        self.decimated_poly = decimatedPoly


    def get_decimated_poly(self):
        return self.decimated_poly