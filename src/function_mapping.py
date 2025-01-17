# class FunctionMapping(object):  # TODO - one signal can fire multiple slots
#     """A mapping between parameter-less functions."""
#
#     def __init__(self, signal, slot):
#         self._signal = signal
#         self._slot = slot
#         self._is_connected = True
#
#     def update(self):
#         if self._is_connected & self._signal():
#             self._slot()
#
#     def disconnect(self):
#         self._is_connected = False
#
#     def connect(self):
#         self._is_connected = True