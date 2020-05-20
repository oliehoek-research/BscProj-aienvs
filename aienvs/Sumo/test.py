from aienvs.Sumo.SumoGymAdapter import SumoGymAdapter
import matplotlib.pyplot as plt

params = {
    'gui': False,
    # 'scene': 'grid_2x2',
    'scene': 'manhattan2',
    # 'tlphasesfile': 'grid_2x2.tll.xml',
    'tlphasesfile': 'osm.net.xml',
    'box_bottom_corner': (0, 0),
    'box_top_corner': (100, 80),
    'resolutionInPixelsPerMeterX': 1,
    'resolutionInPixelsPerMeterY': 1,
    'y_t': 6,
    'trips_generate': True,
    'trips_generate_options': ['--validate'],
    'car_pr': 0.1,
    'car_tm': 1000,
    'route_starts': ['gneE0'],
    'route_segments': ['-gneE4', 'gneE1 gneE2 gneE3'],
    'route_min_segments': 1,
    'route_max_segments': 1,
    'route_ends': ['-gneE5 -gneE7'],
    'generate_conf': True,
    'libsumo': False,
    'waiting_penalty': 1,
    'new_reward': False,
    'lightPositions': {},
    'scaling_factor': 1.0,
    'maxConnectRetries': 50,
    'seed': None,
    'shape': (2, 2),
    'lane_length': 100}
# env = GridSumoEnv()
env = SumoGymAdapter(params)
env.reset()
s, r, done, _ = env.step({})
plt.imshow(s)
plt.show()

env._parameters['gui'] = True
env.reset()
while True:
    # a1 = agents[0]
    # print(a1.state)
#         print(a1.id)
#         print(a1.getActionSpace())
#         print(a1.getQ(a1.state, {'l_1_1': 0}))


    # a1.step(observation=obs, reward=reward, done=done)
    # print(a1.state.shape)
    # a1.getQ(a1.state, 0)


    # action = coordinator.step(observation=obs, reward=reward, done=done)
    obs, reward, done, _ = env.step({})
    # Return += reward
    if done:
        print('hoi 2')
        env.close()
        # send the final transition to the agent
        # coordinator.step(observation=obs, reward=reward, done=done)
        break