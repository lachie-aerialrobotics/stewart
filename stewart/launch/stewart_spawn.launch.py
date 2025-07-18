from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)

sl.declare_arg('sliders',default_value=True)
sl.declare_arg('robot', default_value='stewart_platform_fixed')


def launch_setup():

    robot = sl.arg('robot')
    sl.spawn_gz_model(name = robot, model_file = sl.find('gz_attach_links', robot + '.sdf'),
                      spawn_args=[])


    return sl.launch_description()

generate_launch_description = sl.launch_description(launch_setup)
