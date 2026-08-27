"""Dashboard unit mode에서 import 경계만 제공하는 fail-closed canonical fixture."""
import sys
import types


JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
          'wrist_flex', 'wrist_roll']
PAN0 = (0.0, 0.0, 0.0)


def _unexpected(module_name, attribute):
    raise AssertionError(
        f'unit mode가 canonical {module_name}.{attribute}를 호출했습니다; '
        '이 계약은 integration mode에서 검증해야 합니다')


def _strict_getattr(module_name, attribute):
    if attribute.startswith('__'):
        raise AttributeError(attribute)
    return _unexpected(module_name, attribute)


def install():
    arm_lib = types.ModuleType('arm_lib')
    arm_lib.JOINTS = list(JOINTS)
    arm_lib.PAN0 = PAN0
    arm_lib.load_mapping = lambda: {
        'signs': {joint: 1.0 for joint in JOINTS},
        'offsets': {joint: 0.0 for joint in JOINTS},
    }
    arm_lib.__getattr__ = lambda name: _strict_getattr('arm_lib', name)

    ds_record = types.ModuleType('ds_record')
    ds_record.DEFAULT_ROOT = '/unit-mode-canonical-data-unavailable'
    ds_record.__getattr__ = lambda name: _strict_getattr('ds_record', name)

    arm_gui = types.ModuleType('arm_gui')

    class Worker:
        def __init__(self, *_args, **_kwargs):
            _unexpected('arm_gui.Worker', '__init__')

        def submit(self, *_args, **_kwargs):
            _unexpected('arm_gui.Worker', 'submit')

        def command_status(self, *_args, **_kwargs):
            _unexpected('arm_gui.Worker', 'command_status')
    arm_gui.Worker = Worker
    arm_gui.__getattr__ = lambda name: _strict_getattr('arm_gui', name)

    ros_monitor = types.ModuleType('ros_base_monitor')

    class BaseMonitor:
        def __init__(self, *_args, **_kwargs):
            _unexpected('ros_base_monitor.BaseMonitor', '__init__')
    ros_monitor.BaseMonitor = BaseMonitor
    ros_monitor.__getattr__ = lambda name: _strict_getattr('ros_base_monitor', name)

    for name, module in (
            ('arm_lib', arm_lib), ('ds_record', ds_record),
            ('arm_gui', arm_gui), ('ros_base_monitor', ros_monitor)):
        sys.modules[name] = module
