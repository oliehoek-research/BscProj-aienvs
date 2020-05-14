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


def LinearFeatureState_builder(ldm, **_):
    return state_representation.LinearFeatureState(ldm=ldm)

def SimpleState_builder(ldm, controlled_agent_tl_id, **_):
    return state_representation.SimpleState(ldm=ldm, controlled_agent_tl_id=controlled_agent_tl_id)


state_factory = ObjectFactory()
state_factory.register_builder("LdmMatrixState", LdmMatrixState_builder)
state_factory.register_builder("LinearFeatureState", LinearFeatureState_builder)
state_factory.register_builder("SimpleState", SimpleState_builder)
