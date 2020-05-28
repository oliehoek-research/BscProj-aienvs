from xml.etree import ElementTree


class NetFileUtil:

    def __init__(self, filename):
        self._tree = ElementTree.parse(filename)
        self._root = self._tree.getroot()

        assert self._tree is not None

        print(self._tree)
        print(self._root)

    # Looksup the tl phase id associated with a junction
    def find_tlid_associated_with_junction(self, junction: dict) -> str:
        int_lanes: str = junction['intLanes']

        an_int_lane = int_lanes.split(" ")[0]

        assert isinstance(an_int_lane, str) and an_int_lane != "", f"Failed to get an internal lane from junction: {junction}"

        return self._find_tlid_of_int_lane(an_int_lane)

    def find_junction(self, junction_id: str) -> dict:
        for junction in self._root.iter('junction'):
            print(junction.attrib)
            if junction.attrib['id'] == junction_id:
                return junction.attrib

        raise Exception(f"Did not find junction with id: {junction_id}")

    def _find_tlid_of_int_lane(self, intLane: str) -> str:
        return self._find_connection_with_via(intLane)['tl']

    def _find_connection_with_via(self, via: str) -> dict:
        for connection in self._root.iter('connection'):
            if 'via' in connection.attrib.keys() and connection.attrib['via'] == via:
                return connection.attrib

        raise Exception(f"Did not find connection via {via}")
