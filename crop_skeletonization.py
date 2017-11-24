import renderer
import meshOperations
import decimator
import os

base_path = r'.\image_data\\'

for i, case in enumerate(os.listdir(base_path)):
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

            reoriented_polydata, transform = meshOp.translate_to_origin(original_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)


            reoriented_polydata, transform = meshOp.align_to_axes(reoriented_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)


            reoriented_polydata, transform = meshOp.translate_to_xy_plane(reoriented_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)


            #####################################
            #### 2. SIMPLIFY (decimate) MESH ####
            #####################################

            dec = decimator.Decimator(level=0.0)
            dec_poly_data = dec.decimate(reoriented_polydata)

            ######################
            #### 3. CROP MESH ####
            ######################


            cropped_poly = meshOp.crop_mesh_maximum(dec_poly_data, dec_poly_data.GetBounds()[0],
                                            dec_poly_data.GetBounds()[1],
                                            dec_poly_data.GetBounds()[2], dec_poly_data.GetBounds()[3],
                                            dec_poly_data.GetBounds()[5], dec_poly_data.GetBounds()[5], 0.15, 0.90, 0.0125)



            #############################################
            #### 4. SIMPLIFY (decimate) MESH (again) ####
            #############################################

            dec = decimator.Decimator(level=0.0)
            dec_poly_data = dec.decimate(cropped_poly)


            #############################
            #### 5. SKELETONIZE/THIN ####
            #############################

            # get outer edges

            edge_poly = meshOp.get_outer_edges(dec_poly_data)
            g_edge = meshOp.compute_center_of_mass(edge_poly)
            edge_poly,trans = meshOp.translate_to_xy_y_centered(edge_poly)
            original_aligned_poly, transform = meshOp.translate_tuple(reoriented_polydata, trans)
            suggest_line, _ = meshOp.transform(suggest_line, transform)

            skeleton, skeleton_points = meshOp.get_edge_skeleton(edge_poly)

            # find intersection of vertical lines with the original aligned mesh

            skeleton_final = meshOp.place_skeleton_on_original_mesh(original_aligned_poly, skeleton_points)


            ###############################
            ### 6. visualize everything ###
            ###############################

            rend = renderer.Renderer()
            rend.add_actor(original_aligned_poly, color=[1,1,1], wireframe= False)
            rend.add_actor(suggest_line, color=[1,0,0], wireframe=False, linewidth=4)
            rend.add_actor(skeleton_final, color = [0,0,1], wireframe=False, linewidth=6)

            print("Case: " + case + "\n" + "Side: " + side)
            rend.render()



