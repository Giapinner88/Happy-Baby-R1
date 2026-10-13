/* HB XR frame transport v3: observe the renderer's actual WebXR frame. */
(() => {
  const root = globalThis;
  if (root.__hbTrackingV3) return;
  root.__hbTrackingV3 = true;
  const version = 'xr-frame-v3';
  const client = `quest-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  const contexts = new WeakMap();
  // Preserve native poses, button mapping and all STOP conditions.
  // Session feature filtering is enabled only by an explicit diagnostic URL.
  // A bounded event history survives the server's one-second diagnostic sampling.
  const auditRevision = 'input-audit-4', events = [], sourceIds = new WeakMap();
  const controllerOnly = () => /[?&]hb_controller_only=1(?:&|$)/.test(String(location.href).split('#')[0]);
  const extraTrackingFeatures = new Set(['hand-tracking', 'body-tracking', 'plane-detection',
    'mesh-detection', 'hit-test', 'anchors']);
  let eventId = 0, sourceId = 0, contextId = 0, auditTimer = null, currentContext = null;
  let frameDetail = {}, mediaDetail = {};
  let installed = false, requestAuditInstalled = false, lastReport = '', lastReportAt = -Infinity;
  const now = () => performance.now();
  const browserMemory = () => {
    try {
      const value = performance.memory;
      if (!value) return {available: false};
      return {available: true, used_js_heap_bytes: value.usedJSHeapSize,
        total_js_heap_bytes: value.totalJSHeapSize, js_heap_limit_bytes: value.jsHeapSizeLimit};
    } catch (_) { return {available: false}; }
  };
  const button = value => value ? {v: Number.isFinite(value.value) ? value.value : null,
    p: value.pressed === true, t: value.touched === true} : null;
  const describeSource = source => {
    try {
      if (!sourceIds.has(source)) sourceIds.set(source, ++sourceId);
      return {id: sourceIds.get(source), side: source.handedness, ray: source.targetRayMode,
        profiles: Array.from(source.profiles || []).slice(0, 3).map(value => String(value).slice(0, 80)),
        hand: !!source.hand, connected: source.gamepad?.connected, mapping: source.gamepad?.mapping,
        trigger: button(source.gamepad?.buttons?.[0]), grip: button(source.gamepad?.buttons?.[1])};
    } catch (_) { return {audit_error: 'source_inspection'}; }
  };
  const sourceList = session => Array.from(session?.inputSources || []).map(describeSource);
  const audit = (type, detail = {}) => {
    try {
      events.push({id: ++eventId, ms: Math.round(now()), type, session_id: currentContext?.id, ...detail});
      while (events.length > 16 || JSON.stringify(events).length > 1800) events.shift();
      if (currentContext && auditTimer === null) auditTimer = setTimeout(() => {
        auditTimer = null; report('audit');
      }, 250);
    } catch (_) {} // Diagnostics cannot change control validity.
  };
  const report = (stage, reason = '', detail = {}) => {
    const at = now(), key = stage + ':' + reason;
    if (key === lastReport && at - lastReportAt < 5000) return;
    lastReport = key; lastReportAt = at;
    const value = {version, client, stage, reason, url: location.href,
      secure: root.isSecureContext === true, xr: !!navigator.xr, ...detail,
      audit_revision: auditRevision, browser_ms: Math.round(at), events: events.slice(),
      session_profile: controllerOnly() ? 'controller-only' : 'renderer-default',
      browser_memory: browserMemory(),
      user_agent: String(navigator.userAgent || '').slice(0, 220),
      frame: frameDetail, media: mediaDetail};
    // The existing diagnostic endpoint accepts at most 4096 characters.
    while (value.events.length && JSON.stringify(value).length > 3600) value.events.shift();
    if (JSON.stringify(value).length > 3900) { value.frame = {}; value.media = {}; }
    if (typeof fetch === 'function') fetch('/tracking/diagnostic', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(value), keepalive: true
    }).catch(() => {});
    const labels = {boot: 'Đang chuẩn bị tracking', installed: 'Đang chờ phiên VR',
      connected: 'Đang chờ tracking', ready: 'Tracking sẵn sàng',
      rearm: 'Thả cò rồi nhấn lại để điều khiển',
      invalid: 'Mất tracking — thả cò rồi thử lại', paused: 'Thiếu vị trí tay — tạm dừng, chờ tracking phục hồi', ended: 'Phiên VR đã kết thúc',
      socket_closed: 'Mất kết nối tracking', error: 'Tracking chưa sẵn sàng'};
    if (stage !== 'audit' && typeof document !== 'undefined' && document.body) {
      let label = document.getElementById('hb-tracking-status');
      if (!label) {
        label = document.createElement('div'); label.id = 'hb-tracking-status';
        label.style.cssText = 'position:fixed;bottom:8px;left:8px;z-index:10000;padding:6px 10px;background:#111c;color:white;font:14px sans-serif;pointer-events:none';
        document.body.appendChild(label);
      }
      label.textContent = stage === 'ready' && detail.arm_tracking?.left === false ?
        'Tay phải sẵn sàng; tay trái đang giữ' : labels[stage] || 'Đang chờ phiên VR';
      label.style.color = ['invalid','error','socket_closed','rearm'].includes(stage) ? '#ff7373' : '#fff';
    }
  };
  report('boot');
  // Observe lifecycle events before the renderer creates an immersive session.
  const listen = (object, name, callback) => {
    try { object?.addEventListener?.(name, callback); } catch (_) {}
  };
  if (typeof document !== 'undefined') listen(document, 'visibilitychange', () =>
    audit('document_visibility', {state: document.visibilityState}));
  if (typeof document !== 'undefined') {
    for (const name of ['webglcontextlost', 'webglcontextrestored']) {
      try { document.addEventListener(name, () => audit(name), true); } catch (_) {}
    }
  }
  for (const name of ['focus', 'blur', 'pagehide', 'pageshow']) listen(root, name, () => audit(name));
  listen(root, 'error', event => audit('browser_error', {message: String(event.message).slice(0, 180)}));
  listen(root, 'unhandledrejection', event => audit('promise_error', {message: String(event.reason).slice(0, 180)}));
  const installRequestAudit = () => {
    if (requestAuditInstalled) return;
    const proto = root.XRSystem?.prototype;
    if (!proto || typeof proto.requestSession !== 'function') return;
    const nativeRequest = proto.requestSession;
    try { Object.defineProperty(proto, 'requestSession', {configurable: true, writable: true,
      value: function(mode, options) {
        let effective = options;
        if (controllerOnly()) {
          effective = {...options};
          for (const key of ['requiredFeatures', 'optionalFeatures']) {
            if (options?.[key] !== undefined) effective[key] = Array.from(options[key]).filter(value => !extraTrackingFeatures.has(value));
          }
        }
        audit('request_session', {mode, profile: controllerOnly() ? 'controller-only' : 'renderer-default',
          required: Array.from(effective?.requiredFeatures || []).slice(0, 8),
          optional: Array.from(effective?.optionalFeatures || []).slice(0, 8)});
        if (controllerOnly()) return nativeRequest.call(this, mode, effective);
        return nativeRequest.apply(this, arguments);
      }}); requestAuditInstalled = true; } catch (_) {}
  };
  installRequestAudit();
  // Read decoder statistics on the existing camera peer; do not change SDP or media.
  const rtcProto = root.RTCPeerConnection?.prototype;
  if (rtcProto && typeof rtcProto.setRemoteDescription === 'function') {
    const nativeSetRemote = rtcProto.setRemoteDescription;
    const watched = new WeakSet();
    try { Object.defineProperty(rtcProto, 'setRemoteDescription', {configurable: true, writable: true,
      value: function() {
        const result = nativeSetRemote.apply(this, arguments);
        if (!watched.has(this)) {
          watched.add(this);
          const peer = this;
          const sample = async () => {
            if (peer.signalingState === 'closed') { mediaDetail = {state: 'closed'}; return; }
            try {
              const stats = await peer.getStats();
              const detail = {state: peer.connectionState};
              stats.forEach(row => {
                if (row.type === 'inbound-rtp' && (row.kind === 'video' || row.mediaType === 'video')) {
                  for (const key of ['framesReceived','framesDecoded','framesDropped','freezeCount',
                    'totalFreezesDuration','totalDecodeTime','jitterBufferDelay','jitterBufferEmittedCount',
                    'packetsLost','bytesReceived']) if (Number.isFinite(row[key])) detail[key] = row[key];
                }
                if (row.type === 'candidate-pair' && row.state === 'succeeded' && row.nominated)
                  detail.rtt_s = row.currentRoundTripTime;
              });
              mediaDetail = detail;
            } catch (_) {}
            setTimeout(sample, 5000);
          };
          setTimeout(sample, 1000);
        }
        return result;
      }}); } catch (_) {}
  }
  const connect = ctx => {
    if (ctx.ended) return;
    const scheme = location.protocol === 'http:' ? 'ws' : 'wss';
    const socket = new WebSocket(`${scheme}://${location.host}/tracking?client=${encodeURIComponent(client)}&version=${version}`);
    ctx.socket = socket;
    socket.onopen = () => { audit('socket_open', {session_id: ctx.id}); report('connected'); };
    socket.onmessage = event => {
      try {
        const gate = JSON.parse(event.data);
        if (gate.type !== 'tracking_gate') return;
        ctx.releaseRequired = gate.requires_release;
        ctx.armTracking = gate.arm_tracking;
        ctx.positionPaused = !gate.valid && ['right_position_paused','right_position_recovering'].includes(gate.reason);
        audit('gate', {session_id: ctx.id, release: gate.requires_release, reason: gate.reason, arms: gate.arm_tracking});
        if (ctx.releaseRequired) report('rearm', gate.reason);
        else if (ctx.positionPaused) report('paused', gate.reason);
      } catch (_) {}
    };
    socket.onerror = () => report('error', 'tracking_socket_error');
    socket.onclose = event => {
      if (ctx.socket !== socket) return;
      audit('socket_close', {session_id: ctx.id, code: event.code, reason: String(event.reason || '').slice(0, 100), clean: event.wasClean});
      report('socket_closed', String(event.code));
      if (!ctx.ended) setTimeout(() => connect(ctx), 1000);
    };
  };
  const send = (ctx, value) => {
    const socket = ctx.socket;
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    if (socket.bufferedAmount > 16384) {
      report('error', 'control_backpressure'); socket.close(); return;
    }
    socket.send(JSON.stringify({sequence: ++ctx.sequence, ...value}));
  };
  const stop = (ctx, reason, detail = {}) => {
    if (ctx.stopReason !== reason) audit('stop', {session_id: ctx.id, reason});
    ctx.stopReason = reason;
    send(ctx, {valid: false, reason, ...detail});
    report(ctx.positionPaused && reason === 'right_pose_missing_or_emulated' ? 'paused' : 'invalid', reason);
  };
  const observe = (frame, reference, viewer) => {
    const session = frame.session;
    if (!session) return;
    let ctx = contexts.get(session);
    if (!ctx) {
      const sources = Array.from(session.inputSources || []);
      // Do not claim control for a nonimmersive/inline preview session.
      if ((!viewer || !viewer.views || viewer.views.length < 2) &&
          !sources.some(source => source.targetRayMode === 'tracked-pointer')) return;
      ctx = {id: ++contextId, ended: false, sequence: 0, nextAt: -Infinity, socket: null, releaseRequired: true,
        armTracking: {left: false, right: false}, lastFrameAt: null, maxFrameGap: 0};
      contexts.set(session, ctx);
      currentContext = ctx;
      try { audit('session_observed', {visibility: session.visibilityState,
        features: Array.from(session.enabledFeatures || []).slice(0, 10), sources: sourceList(session)}); } catch (_) {}
      listen(session, 'inputsourceschange', event => audit('input_sources_change', {session_id: ctx.id, trusted: event.isTrusted === true,
        added: Array.from(event.added || []).map(describeSource),
        removed: Array.from(event.removed || []).map(describeSource)}));
      for (const name of ['selectstart','selectend','squeezestart','squeezeend']) listen(session, name,
        event => audit(name, {session_id: ctx.id, trusted: event.isTrusted === true,
          source: event.inputSource ? describeSource(event.inputSource) : null}));
      session.addEventListener('visibilitychange', () => {
        audit('session_visibility', {session_id: ctx.id, state: session.visibilityState, sources: sourceList(session)});
        if (session.visibilityState !== 'visible') stop(ctx, 'session_' + session.visibilityState);
      });
      session.addEventListener('end', () => {
        audit('session_end', {session_id: ctx.id});
        stop(ctx, 'session_ended'); ctx.ended = true; ctx.socket?.close(); report('ended');
      });
      connect(ctx);
    }
    const at = now();
    if (ctx.ended) return;
    currentContext = ctx;
    const gap = ctx.lastFrameAt === null ? 0 : at - ctx.lastFrameAt;
    ctx.lastFrameAt = at; ctx.maxFrameGap = Math.max(ctx.maxFrameGap, gap);
    if (gap > 250) audit('frame_gap', {gap_ms: Math.round(gap)});
    if (at < ctx.nextAt) return;
    ctx.nextAt = at + 1000 / 30;
    try { frameDetail = {session_id: ctx.id, sequence: ctx.sequence, visibility: session.visibilityState, viewer: !!viewer,
      viewer_emulated: !!viewer?.emulatedPosition, gap_ms: Math.round(gap),
      max_gap_ms: Math.round(ctx.maxFrameGap), sources: sourceList(session)}; } catch (_) {}
    try {
      const getter = Object.getOwnPropertyDescriptor(root.XRSession?.prototype || {}, 'inputSources')?.get;
      if (getter) frameDetail.native_source_ids = Array.from(getter.call(session)).map(source => describeSource(source).id);
    } catch (_) {}
    if (session.visibilityState !== 'visible') { stop(ctx, 'session_' + session.visibilityState); return; }
    if (!viewer || viewer.emulatedPosition) { stop(ctx, 'viewer_pose_missing_or_emulated'); return; }
    const value = {valid: true, arm_tracking: {left: true, right: true}, camera: {matrix: Array.from(viewer.transform.matrix)}};
    try {
      for (const side of ['left', 'right']) {
        const input = Array.from(session.inputSources || []).find(source =>
          source.handedness === side && source.targetRayMode === 'tracked-pointer');
        const pose = input && frame.getPose(input.targetRaySpace, reference);
        const buttons = input?.gamepad?.buttons;
        try {
          const descriptor = frameDetail.sources?.find(source => source.side === side);
          if (descriptor) {
            descriptor.pose = !!pose; descriptor.emulated = !!pose?.emulatedPosition;
            const gripPose = input.gripSpace && frame.getPose(input.gripSpace, reference);
            descriptor.grip_pose = !!gripPose; descriptor.grip_emulated = !!gripPose?.emulatedPosition;
          }
          if (side === 'right') {
            const classify = b => !b ? 'missing' : b.value <= .15 ? 'released' : b.value < .5 ? 'partial' : 'held';
            const state = [input ? sourceIds.get(input) : null, classify(buttons?.[0]),
              !!buttons?.[0]?.pressed, classify(buttons?.[1]), !!buttons?.[1]?.pressed].join(':');
            if (state !== ctx.buttonState) { ctx.buttonState = state; audit('right_buttons', {source: descriptor || null}); }
          }
        } catch (_) {}
        const reason = !input ? side + '_input_source_missing' :
          !pose || pose.emulatedPosition ? side + '_pose_missing_or_emulated' :
          !buttons?.[0] || !Number.isFinite(buttons[0].value) ? side + '_trigger_missing' : '';
        if (reason) {
          if (side === 'right') {
            const detail = pose?.emulatedPosition && buttons?.[0] ? {
              rightInputId: sourceIds.get(input), rightInputConnected: input.gamepad?.connected === true,
              rightState: {trigger: buttons[0].pressed, triggerValue: buttons[0].value},
              positionPauseProof: {pose_present: true, position_emulated: true, viewer_tracked: true,
                visibility: session.visibilityState, viewer_matrix: value.camera.matrix}
            } : {};
            stop(ctx, reason, detail); return;
          }
          // Missing non-deadman hand is explicitly inactive, never a fresh pose.
          value.arm_tracking.left = false; value.left = null;
          value.leftState = {trigger: false, triggerValue: 0};
          continue;
        }
        value[side] = Array.from(pose.transform.matrix);
        if (side === 'right') {
          value.rightInputId = sourceIds.get(input);
          value.rightInputConnected = input.gamepad?.connected === true;
        }
        value[side + 'State'] = {
          trigger: buttons[0].pressed, triggerValue: buttons[0].value,
          squeeze: !!buttons[1]?.pressed, squeezeValue: buttons[1]?.value || 0,
          thumbstick: !!buttons[3]?.pressed,
          thumbstickValue: [input.gamepad.axes[2] || 0, input.gamepad.axes[3] || 0],
          aButton: !!buttons[4]?.pressed, bButton: !!buttons[5]?.pressed
        };
      }
      ctx.stopReason = '';
      send(ctx, value);
      if (ctx.socket?.readyState === WebSocket.OPEN) report(ctx.releaseRequired ? 'rearm' : ctx.positionPaused ? 'paused' : 'ready', '', {
        right_trigger: value.rightState.triggerValue, arm_tracking: ctx.armTracking,
        input_sources: Array.from(session.inputSources).map(source => source.handedness)});
    } catch (error) { stop(ctx, 'frame_read_error'); report('error', String(error)); }
  };
  const install = () => {
    if (installed) return;
    installRequestAudit();
    const proto = root.XRFrame?.prototype;
    if (!proto || typeof proto.getViewerPose !== 'function') return;
    const nativeGetViewerPose = proto.getViewerPose;
    const wrapped = function(reference) {
      const viewer = nativeGetViewerPose.call(this, reference);
      try { observe(this, reference, viewer); } catch (error) { report('error', String(error)); }
      return viewer;
    };
    try {
      Object.defineProperty(proto, 'getViewerPose', {value: wrapped, writable: true, configurable: true});
      installed = proto.getViewerPose === wrapped;
      report(installed ? 'installed' : 'error', installed ? '' : 'xr_frame_hook_failed');
    } catch (error) { report('error', 'xr_frame_hook_failed: ' + String(error)); }
  };
  install();
  // WebXR globals can become available after page initialization. No early exit.
  const retry = () => { if (!installed) { install(); setTimeout(retry, 250); } };
  if (!installed) setTimeout(retry, 250);
})();
