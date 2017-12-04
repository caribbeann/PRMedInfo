import renderer
import meshOperations
import decimator
import os
import vtk

base_path = r'./image_data//'
keep_percentage_after_crop = 0.6
crop_step_size = 0.025

for i, case in enumerate(os.listdir(base_path)):
    print case
    if i+1 >= 1:
        for side in ["upper", "lower"]:


            ########################
            #### 0. READ MESHES ####
            ########################
            meshOp = meshOperations.MeshOperations()

            original_polydata = meshOp.read(base_path + case+ "//{}JawMesh.obj".format(side))
            suggest_line = meshOp.read(base_path + case + "//suggest_alveolarRidgeLine_{}_coarse.obj".format(side))

            ########################
            ##### 1. ALIGN MESH ####
            ########################

            # move mesh to origin in any case

            reoriented_polydata, transform, gingiva = meshOp.align_to_axes(original_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)

            rend = renderer.Renderer()
            rend.add_actor(reoriented_polydata, color=[1, 1, 1], wireframe=False)
            rend.add_actor(suggest_line, color=[1, 0, 1], wireframe=False)
            rend.add_actor(gingiva, color=[1, 0, 1], wireframe=False)
            rend.render()