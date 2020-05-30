from aienvs.Sumo import state_representation


# Generic object factory, to create a state_factory instance with
# from https://realpython.com/factory-method-python/
class ObjectFactory:
    def __init__(self):
        self._builders = {}

    def register_builder(self, key, builder):
        self._builders[key] = builder

    def create(self, key, **kwargs):
        builder = self._builders.get(key)
        if not builder:
            raise ValueError(key)
        return builder(**kwargs)


def LdmMatrixState_builder(ldm, box_bottom_corner, box_top_corner, **_):
    return state_representation.LdmMatrixState(ldm=ldm,
                                               data=[box_bottom_corner, box_top_corner],
                                               type="byCorners")

def SimpleState_builder(ldm, controlled_agent_tl_id, **_):
    return state_representation.SimpleState(ldm=ldm, controlled_agent_tl_id=controlled_agent_tl_id)

def LinearFeatureState_builder(ldm, controlled_agent_tl_id, extra="default", **_):
    return state_representation.LinearFeatureState(ldm=ldm, controlled_agent_tl_id=controlled_agent_tl_id, extra=extra)

def LinearFeatureState_thesis_for_dqn(ldm, controlled_agent_tl_id, extra="thesis_for_dqn", **_):
    return state_representation.LinearFeatureState(ldm=ldm, controlled_agent_tl_id=controlled_agent_tl_id, extra=extra)

def LinearFeatureState_queue_size_phase_builder(ldm, controlled_agent_tl_id, **_):
    return state_representation.LinearFeatureState(ldm=ldm, controlled_agent_tl_id=controlled_agent_tl_id, extra="queue_size_phase")

def TimingState_builder(ldm, controlled_agent_tl_id, **_):
    return state_representation.TimingState(ldm=ldm, controlled_agent_tl_id=controlled_agent_tl_id)

state_factory = ObjectFactory()
state_factory.register_builder("LdmMatrixState", LdmMatrixState_builder)
state_factory.register_builder("SimpleState", SimpleState_builder)
state_factory.register_builder("LinearFeatureState", LinearFeatureState_builder)
state_factory.register_builder("LinearFeatureState_queue_size_phase", LinearFeatureState_queue_size_phase_builder)
state_factory.register_builder("TimingState", TimingState_builder)

# State space with same features as Elise's thesis but without multiplying actions,
# as by feeding it into a neural network, non linearity in the state isn't necessary.
state_factory.register_builder("LinearFeatureState_thesis_for_dqn", LinearFeatureState_thesis_for_dqn)