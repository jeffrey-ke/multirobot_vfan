import time
from src.SimulatedField import SimulatedField

def test_MapCreation():
    sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))
    assert sf._field.shape == (100, 100)
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [80, 80]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [80, 80]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [80, 80]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [80, 80]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [80, 80]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [80, 80]])
    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    time.sleep(10)
