import utils
# import utils.get_positions
# import plot3d_coordinates
results = utils.get_positions.position_from_image(input_image='images.jpg')

utils.plot3d_coordinates.plot_pose(results= results)
