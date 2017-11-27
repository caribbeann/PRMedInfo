import renderer
import meshOperations
import decimator
import vtk
import os

base_path = r'.\image_data\\'
keep_percentage_after_crop = 0.6
crop_step_size = 0.025

for i, case in enumerate(os.listdir(base_path)):
    if i+1 >= 18:
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

            reoriented_polydata, transform = meshOp.translate_to_origin(original_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)

            # aling along the smallest dimension in any case

            reoriented_polydata, transform = meshOp.align_to_z_axis(reoriented_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)


            # make z min 0 in any case
            reoriented_polydata, transform = meshOp.translate_to_xy_plane(reoriented_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)

            #####################################
            #### 2. SIMPLIFY (decimate) MESH ####
            #####################################

            dec = decimator.Decimator(level=0.0)
            reoriented_polydata = dec.decimate(reoriented_polydata)

            ######################
            #### 3. CROP MESH ####
            ######################
            cropped_polies = []
            if "Gips" in case or "gips" in case:
                if meshOp.is_mesh_with_basis_on_head(reoriented_polydata):
                    reoriented_polydata, t = meshOp.rotate_angle(reoriented_polydata, [1, 0, 0], 180)
                    suggest_line, _ = meshOp.transform(suggest_line, t)
                    reoriented_polydata, t = meshOp.translate_to_xy_plane(reoriented_polydata)
                    suggest_line, _ = meshOp.transform(suggest_line, t)

                cropped_poly = meshOp.crop_mesh_maximum(reoriented_polydata, reoriented_polydata.GetBounds()[0],
                                                        reoriented_polydata.GetBounds()[1],
                                                        reoriented_polydata.GetBounds()[2],
                                                        reoriented_polydata.GetBounds()[3],
                                                        reoriented_polydata.GetBounds()[5],
                                                        reoriented_polydata.GetBounds()[5],
                                                        0.10, 0.85, crop_step_size, keep_percentage_after_crop)
                cropped_polies.append(cropped_poly)
            else: # if it is not gips

                cropped_poly1 = meshOp.crop_mesh_maximum(reoriented_polydata, reoriented_polydata.GetBounds()[0],
                                                         reoriented_polydata.GetBounds()[1],
                                                         reoriented_polydata.GetBounds()[2],
                                                         reoriented_polydata.GetBounds()[3],
                                                         reoriented_polydata.GetBounds()[5],
                                                         reoriented_polydata.GetBounds()[5],
                                                         0.10, 0.85, crop_step_size, keep_percentage_after_crop)

                # turn the mesh around maybe it is on its head and check if we can cut more in this position
                # keep track of transforms because if the rotation was a good idea, the suggestion line must also be rotated
                tmp, t1 = meshOp.rotate_angle(reoriented_polydata, [1, 0, 0], 180)
                tmp_transform = vtk.vtkTransform()
                tmp_transform.Concatenate(t1)
                tmp, t2 = meshOp.translate_to_xy_plane(tmp)
                tmp_transform.Concatenate(t2)
                cropped_poly2 = meshOp.crop_mesh_maximum(tmp, tmp.GetBounds()[0],
                                                         tmp.GetBounds()[1],
                                                         tmp.GetBounds()[2], tmp.GetBounds()[3],
                                                         tmp.GetBounds()[5], tmp.GetBounds()[5],
                                                         0.10, 0.85, crop_step_size, keep_percentage_after_crop)


                if abs((cropped_poly1.GetBounds()[5] - cropped_poly1.GetBounds()[4]) >
                               abs(cropped_poly2.GetBounds()[5] - cropped_poly2.GetBounds()[4])):
                    cropped_poly = cropped_poly2
                    suggest_line, _ = meshOp.transform(suggest_line, tmp_transform)
                    reoriented_polydata, _ = meshOp.transform(reoriented_polydata,
                                                              tmp_transform)  # also rotate the reoriented thing
                    reoriented_polydata, transform = meshOp.translate_to_xy_plane(reoriented_polydata)
                    suggest_line, _ = meshOp.transform(suggest_line, transform)

                    cropped_polies.append(cropped_poly1)
                    cropped_polies.append(cropped_poly2)
                else: # if 180 rotation was NOT good idea
                    cropped_poly = cropped_poly1
                    cropped_polies.append(cropped_poly)


            #############################################
            #### 4. SIMPLIFY (decimate) MESH (again) ####
            #############################################

            dec = decimator.Decimator(level=0.0)
            dec_poly_data = dec.decimate(cropped_poly)


            #############################
            #### 5. SKELETONIZE/THIN ####
            #############################

            # get outer edges
            edge_polies = []
            skeletons = []
            for c in cropped_polies:
                edge_poly = meshOp.get_outer_edges(c)
                edge_poly, _ = meshOp.flatten(edge_poly) # make it flat
                edge_polies.append(edge_poly)

            #edge_poly,trans = meshOp.translate_to_xy_x_centered(edge_poly)
            #original_aligned_poly, transform = meshOp.translate_tuple(reoriented_polydata, trans)
            #suggest_line, _ = meshOp.transform(suggest_line, transform)

                skeleton, skeleton_points = meshOp.get_edge_skeleton(edge_poly)

            # find intersection of vertical lines with the original aligned mesh

                skeleton_final = meshOp.place_skeleton_on_original_mesh(reoriented_polydata, skeleton_points)
                skeletons.append(skeleton_final)

            ###############################
            ### 6. visualize everything ###
            ###############################

            rend = renderer.Renderer()
            rend.add_actor(reoriented_polydata, color=[1,1,1], wireframe= False)
            rend.add_actor(suggest_line, color=[1,0,0], wireframe=False, linewidth=4)
            for s,c in zip(skeletons, [[0,0,1], [0,0.5,0]]):
                rend.add_actor(s, color = c, wireframe=False, linewidth=6)

            print("Case: " + case + "\n" + "Side: " + side)
            rend.render()



