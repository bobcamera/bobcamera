from time import perf_counter

###################################################################################################################
# A class to provide some sort of execution time reporting so that we can determine application bottle necks etc. #
###################################################################################################################
class Stopwatch:

    def __init__(self, mask='Execution time: {s:0.4f} seconds', enable=False):
        self.mask = mask
        self.enable = enable

    def __enter__(self):
        self.time = perf_counter()
        return self

    def __exit__(self, type, value, traceback):
        self.time = perf_counter() - self.time
        self.readout = self.mask.format(s=self.time)
        if self.enable:
            print(self.readout)