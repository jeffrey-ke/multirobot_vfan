from src.SimulatedField.SimulatedField import SimulatedField
import time

def test_SpawnFeature():
    sf = SimulatedField(width=100, height=100)

    sf.spawnFeature(id="maximum", location=(50, 50))
    time.sleep(2)
    sf.clear()
    time.sleep(2)
    sf.spawnFeature(id="maximum", location=(50,50))
    time.sleep(2)
    sf.clear()

def test_SpawnRobot():
    sf = SimulatedField(width=100, height=100)
    sf.spawnRobot(id="0", location=(50,50))
    sf.spawnRobot(id="A", location=(80, 80))
    sf.spawnRobot(id="1", location=(20, 20))
    time.sleep(5)
