class Analyzer:
    """
    Static methods grouped together.   All will probably work
    on either polar or cartesian data.
    """
    start = -10
    stop = 10
    @staticmethod
    def range_at_heading(polar_data, sweep):
        """
        Find the closest hit at a heading in the sweep.
        Return the range and the heading as a tuple.
        (heading, range)
        """
        min_reading = (0, 0)
        sweep_data = [x for x in polar_data if x[0] in range(*sweep)]
        
        if sweep_data:
            min_reading = min(sweep_data, key=lambda r:r[1])
            
        return min_reading

