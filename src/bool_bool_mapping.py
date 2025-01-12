class BoolBoolMapping(object):

    def __init__(self, signal, slot):
        self._signal = signal
        self._slot = slot

    def update(self):
        self._slot(self._signal)
