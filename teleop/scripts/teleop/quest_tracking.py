"""Validate complete XR frame snapshots before updating vendor shared poses."""
import math


def _valid_matrix(matrix):
    if not isinstance(matrix, list) or len(matrix) != 16:
        return False
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in matrix):
        return False
    if any(abs(matrix[i]-expected) > 1e-5 for i, expected in ((3,0),(7,0),(11,0),(15,1))):
        return False
    a, b, c = columns = [matrix[i:i+3] for i in (0,4,8)]
    for i in range(3):
        for j in range(3):
            if abs(sum(x*y for x,y in zip(columns[i],columns[j]))-(i == j)) > 1e-3:
                return False
    determinant = a[0]*(b[1]*c[2]-b[2]*c[1])-b[0]*(a[1]*c[2]-a[2]*c[1])+c[0]*(a[1]*b[2]-a[2]*b[1])
    return abs(determinant-1) <= 1e-3


def valid_snapshot(value):
    if not isinstance(value, dict) or value.get('valid') is not True:
        return False
    if not isinstance(value.get('camera'), dict):
        return False
    mask = value.get('arm_tracking', {'left': True, 'right': True})
    if not isinstance(mask, dict) or any(type(mask.get(side)) is not bool for side in ('left', 'right')) or not mask['right']:
        return False
    matrices = [value['camera'].get('matrix'), value.get('right')]
    if mask['left']:
        matrices.append(value.get('left'))
    elif value.get('left') is not None:
        return False
    for matrix in matrices:
        if not _valid_matrix(matrix):
            return False
    for side in ('left', 'right'):
        state = value.get(side + 'State')
        if not isinstance(state, dict) or type(state.get('trigger')) is not bool:
            return False
        if not mask[side] and (state.get('trigger') or state.get('triggerValue') != 0):
            return False
        analog = state.get('triggerValue')
        if type(analog) not in (int, float) or not math.isfinite(analog) or not 0 <= analog <= 1:
            return False
        for name in ('squeezeValue',):
            analog = state.get(name, 0)
            if type(analog) not in (int, float) or not math.isfinite(analog) or not 0 <= analog <= 1:
                return False
        axes = state.get('thumbstickValue', [0, 0])
        if not isinstance(axes, list) or len(axes) != 2:
            return False
        if any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > 1 for x in axes):
            return False
    return True


class TriggerHysteresis:
    """Stable valid trigger input; zero release always stops immediately."""
    def __init__(self, press=0.5, release=0.15):
        self.press, self.release = press, release
        self.held = False

    def update(self, state):
        value = state['triggerValue']
        if value <= self.release:
            self.held = False
        elif state['trigger'] or value >= self.press:
            self.held = True
        return self.held


class TrackingGate:
    # Motion stays STOP throughout this interval. This is not a longer pose or
    # network lease: only a live same-controller trigger can permit recovery.
    POSITION_PAUSE_S = 2.0
    RECOVERY_STABLE_S = 0.1
    def __init__(self):
        self.last_sequence = -1
        self.received = None
        self.release_required = True
        self.triggers = {side: TriggerHysteresis() for side in ('left', 'right')}
        self.reason = 'waiting_for_xr_frame'
        self.left_latched_active = False
        self.right_was_held = False
        self.right_source_id = None
        self.position_pause_started = None
        self.recovery_started = None

    def _same_live_trigger(self, value):
        state = value.get('rightState', {})
        if not isinstance(state, dict):
            return False
        analog = state.get('triggerValue')
        source_id = value.get('rightInputId')
        return (value.get('rightInputConnected') is True and
                type(source_id) is int and source_id > 0 and source_id == self.right_source_id and
                state.get('trigger') is True and type(analog) in (int, float) and
                math.isfinite(analog) and .5 <= analog <= 1)

    def update(self, value, now):
        import copy
        sequence = value.get('sequence') if isinstance(value, dict) else None
        advancing = type(sequence) is int and sequence > self.last_sequence
        valid = advancing and valid_snapshot(value)
        if type(sequence) is int:
            self.last_sequence = max(sequence, self.last_sequence)
        if self.received is not None and (now - self.received > 0.5 or now < self.received):
            self.release_required = True
        self.received = now
        if not valid:
            # A position estimate is never forwarded to IK. Permit a bounded
            # STOP pause only while button/source/viewer freshness survives.
            proof = value.get('positionPauseProof') if isinstance(value, dict) else None
            if (advancing and value.get('valid') is False and
                    value.get('reason') == 'right_pose_missing_or_emulated' and
                    isinstance(proof, dict) and proof.get('pose_present') is True and
                    proof.get('position_emulated') is True and proof.get('viewer_tracked') is True and
                    proof.get('visibility') == 'visible' and _valid_matrix(proof.get('viewer_matrix')) and
                    not self.release_required and self.right_was_held and self._same_live_trigger(value)):
                if self.position_pause_started is None:
                    self.position_pause_started = now
                self.recovery_started = None
                if now-self.position_pause_started <= self.POSITION_PAUSE_S:
                    self.reason = 'right_position_paused'
                    return None
                self.reason = 'right_position_pause_timeout'
            else:
                self.reason = str(value.get('reason', 'invalid_or_replayed_snapshot'))[:160] if isinstance(value, dict) else 'invalid_snapshot'
            self.position_pause_started = self.recovery_started = None
            self.release_required = True
            for trigger in self.triggers.values():
                trigger.held = False
            return None
        if self.position_pause_started is not None:
            if (self.release_required or now-self.position_pause_started > self.POSITION_PAUSE_S or
                    not self._same_live_trigger(value)):
                self.release_required = True
                self.position_pause_started = self.recovery_started = None
            else:
                if self.recovery_started is None:
                    self.recovery_started = now
                if now-self.recovery_started < self.RECOVERY_STABLE_S:
                    self.reason = 'right_position_recovering'
                    return None
                self.position_pause_started = self.recovery_started = None
        result = copy.deepcopy(value)
        mask = dict(value.get('arm_tracking', {'left': True, 'right': True}))
        right_held = self.triggers['right'].update(value['rightState'])
        if not right_held or not self.right_was_held:
            self.left_latched_active = mask['left']
        elif not mask['left']:
            self.left_latched_active = False
        self.right_was_held = right_held
        self.right_source_id = value.get('rightInputId')
        mask['left'] = mask['left'] and self.left_latched_active
        result['arm_tracking'] = mask
        if not mask['left']:
            result['left'] = None
            result['leftState'] = {'trigger': False, 'triggerValue': 0}

        if all(not value[side + 'State']['trigger'] and value[side + 'State']['triggerValue'] <= 0.05
               for side in ('left', 'right')):
            self.release_required = False
        for side in ('left', 'right'):
            state = result[side + 'State']
            pressed = self.triggers[side].update(state) and not self.release_required
            state['trigger'] = pressed
            # Vendor analog OR digital must not bypass the same gate.
            state['triggerValue'] = 1.0 if pressed else 0.0
        self.reason = 'release_required' if self.release_required else 'valid'
        return result
