from laser import Reading

def test_reading():
    """unit test of the reading object to make sure it continues to work"""
    r1 = laser.Reading(90, 2540, 0x2000)
    assert r1.distance == 2540
    assert not r1.error
    assert int(round(r1.distance_in_inches())) == 100
    assert str(r1) == "90,100.00"
    r2 = laser.Reading(240, 0x8123, 100)
    assert r2.error
    assert r2.distance_in_inches() == 777
    r3 = laser.Reading(240, 0x4000, 200)
    assert r3.warning
                        
