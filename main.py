import utils

results = utils.get_positions.position_from_image('test_image1.jpg')

results = utils.add_spine.AddSpine(results = results)
utils.plot3d_coordinates.plot_pose(results=results)