import renderer
import meshOperations
import decimator
import os
import smoothing as sm
import vtk

base_path = r'.\image_data\\'

# parameters
keep_percentage_after_crop = 0.6
crop_step_size = 0.025
inwards_radius_percent = 0.25
outwards_radius_percent = 0.25
granularity_of_profiles = 100
magnitude_threshold = 0.02
minimum_z_percent_to_crop = 0.1
maximum_z_percent_to_crop = 0.7

for i, case in enumerate(os.listdir(base_path)):
    if i+1 >= 1 :
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


            # align along the smallest dimension in any case and with the Z showing the teeth growth direction

            reoriented_polydata, transform, _ = meshOp.align_to_axes(reoriented_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)


            # make z min 0 in any case
            reoriented_polydata, transform = meshOp.translate_to_xy_plane(reoriented_polydata)
            suggest_line, _ = meshOp.transform(suggest_line, transform)

            # keep an unremoved-tongue poly data for later viewing
            original = reoriented_polydata

            # remove tongue, try different thresholds
            reoriented_polydata = meshOp.remove_tongue(reoriented_polydata, threshold=magnitude_threshold)


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
                                            dec_poly_data.GetBounds()[5], dec_poly_data.GetBounds()[5], minimum_z_percent_to_crop, maximum_z_percent_to_crop,
                                                    crop_step_size, keep_percentage_after_crop)



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
            edge_poly, _ = meshOp.flatten(edge_poly)  # make it flat


            skeleton, skeleton_points = meshOp.get_edge_skeleton(edge_poly)

            # find intersection of vertical lines with the original aligned mesh

            skeleton_final = meshOp.place_skeleton_on_original_mesh(reoriented_polydata, skeleton_points, granularity_of_profiles, inwards_radius_percent, outwards_radius_percent )

            smoother = sm.Smoothing()
            #skeleton_final = smoother.polyDataSmoothed(skeleton_final, 5, 5)


            ###############################
            ### 6. visualize everything ###
            ###############################

            rend = renderer.Renderer()
            rend.add_actor(original, color=[1,1,1], wireframe= False)
            rend.add_actor(suggest_line, color=[1,0,0], wireframe=False, linewidth=4)
            rend.add_actor(skeleton_final, color = [0,0,1], wireframe=False, linewidth=6)

            print("Case: " + case + "\n" + "Side: " + side)
            rend.render()



