import React, {
  createContext, useCallback, useContext, useEffect, useRef, useState,
} from 'react';

const WS_PROTO = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const WS_BASE = `${WS_PROTO}//${window.location.host}`;

// Telemetry counts as live while new health messages keep arriving within this window.
const LINK_TIMEOUT_MS = 3000;
// PX4 nav_state values of the external (ROS 2) modes, EXTERNAL1..EXTERNAL8.
const EXTERNAL_MODE_MIN = 23;
const EXTERNAL_MODE_MAX = 30;
// Mode started by the rover session; the readiness sequence activates this one.
const DEFAULT_MODE = 'RoverManual';

const PX4_MODES = [
  { name: 'RoverManual', label: 'RoverManual', desc: 'Open loop: speed sets throttle, turn rate sets steering. Used by Nav2.' },
  { name: 'Rover Speed Steering', label: 'Speed / Steering', desc: 'PX4 holds the speed; steering is direct.' },
  { name: 'Rover Speed Attitude', label: 'Speed / Attitude', desc: 'PX4 holds the speed and the heading.' },
  { name: 'Rover Speed Rate', label: 'Speed / Rate', desc: 'PX4 holds the speed and the turn rate.' },
];

const ALERT_TEXT = {
  PX4_HEARTBEAT_LOST: 'Flight controller heartbeat lost',
  PX4_BATTERY_CRITICAL: 'Battery critical',
  PX4_BATTERY_LOW: 'Battery low',
  JETSON_CPU_HOT: 'Onboard computer is hot',
  HW_THROTTLE_THERMAL: 'Onboard computer throttled: too hot',
  HW_THROTTLE_POWER: 'Onboard computer throttled: low power',
  JETSON_RAM_HIGH: 'Onboard memory almost full',
  JETSON_DISK_LOW: 'Onboard disk almost full',
  SLAM_LATE: 'Map position is late',
  NET_DROP: 'Telemetry messages dropped',
  NAV2_GOAL_ERROR: 'Navigation goal failed',
};
const CAMERA_ALERT_TEXT = {
  DISCONNECTED: 'camera disconnected',
  STUTTER: 'camera frames stuttering',
  DEPTH_QUALITY: 'depth image mostly empty',
};

function describeAlert(code) {
  if (ALERT_TEXT[code]) return ALERT_TEXT[code];
  const match = /^(.*)_(DISCONNECTED|STUTTER|DEPTH_QUALITY)$/.exec(code);
  if (match) return `${match[1].toUpperCase()} ${CAMERA_ALERT_TEXT[match[2]]}`;
  return code;
}

function alertTone(code) {
  return /(DISCONNECTED|HEARTBEAT_LOST|CRITICAL|GOAL_ERROR)/.test(code) ? 'crit' : 'warn';
}

const num = (value, digits = 1, fallback = '–') => (
  typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : fallback
);

// ── Hooks ────────────────────────────────────────────────────────────

function useWebSocket(path, initialUrl = null) {
  const [data, setData] = useState(null);
  const wsRef = useRef(null);

  useEffect(() => {
    let cancelled = false;
    let backoff = 1000;

    const fetchInitial = async () => {
      if (!initialUrl) return;
      try {
        const response = await fetch(initialUrl);
        if (!response.ok) return;
        const payload = await response.json();
        if (!cancelled && payload) {
          setData((current) => current ?? (payload.health ?? payload));
        }
      } catch {
        // Best-effort bootstrap only; live updates come from the WebSocket.
      }
    };

    const connect = () => {
      const ws = new WebSocket(`${WS_BASE}${path}`);
      wsRef.current = ws;
      ws.onopen = () => { backoff = 1000; };
      ws.onmessage = (e) => setData(JSON.parse(e.data));
      ws.onclose = () => {
        if (!cancelled) {
          setTimeout(connect, backoff);
          backoff = Math.min(backoff * 2, 30000);
        }
      };
      ws.onerror = (err) => {
        console.error('WebSocket error:', path, err);
        ws.close();
      };
    };

    fetchInitial();
    connect();
    return () => {
      cancelled = true;
      wsRef.current?.close();
    };
  }, [initialUrl, path]);

  return data;
}

// The server replays its cached health on connect, so a message only proves the
// link is alive when its seq differs from the previous one.
function useLinkState(health) {
  const lastSeqRef = useRef(null);
  const [freshAt, setFreshAt] = useState(0);
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    if (!health) return;
    const seq = health.seq ?? 0;
    if (lastSeqRef.current !== null && seq !== lastSeqRef.current) setFreshAt(Date.now());
    lastSeqRef.current = seq;
  }, [health]);

  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 500);
    return () => clearInterval(timer);
  }, []);

  const ageMs = freshAt ? now - freshAt : null;
  return {
    live: ageMs !== null && ageMs < LINK_TIMEOUT_MS,
    everLive: freshAt > 0,
    ageS: ageMs === null ? null : Math.max(0, Math.round(ageMs / 1000)),
  };
}

// Theme: ?theme=dark|light in the URL wins (kiosk displays), then the saved
// choice, then the operating system setting ("auto").
function useTheme() {
  const [theme, setTheme] = useState(() => {
    const fromUrl = new URLSearchParams(window.location.search).get('theme');
    if (fromUrl === 'dark' || fromUrl === 'light') return fromUrl;
    try { return localStorage.getItem('cc-theme') || 'auto'; } catch { return 'auto'; }
  });
  useEffect(() => {
    const root = document.documentElement;
    if (theme === 'auto') root.removeAttribute('data-theme');
    else root.setAttribute('data-theme', theme);
    try { localStorage.setItem('cc-theme', theme); } catch { /* storage unavailable */ }
  }, [theme]);
  const cycle = () => setTheme((t) => (t === 'auto' ? 'light' : t === 'light' ? 'dark' : 'auto'));
  return [theme, cycle];
}

// ── Commands and toasts ──────────────────────────────────────────────

const CommandContext = createContext({ send: async () => false, notify: () => {} });
const useCommands = () => useContext(CommandContext);

function useToasts() {
  const [toasts, setToasts] = useState([]);
  const dismiss = useCallback((id) => setToasts((list) => list.filter((t) => t.id !== id)), []);
  const notify = useCallback((text, tone = 'crit') => {
    const id = `${Date.now()}-${Math.random().toString(16).slice(2)}`;
    setToasts((list) => [...list.slice(-3), { id, text, tone }]);
    setTimeout(() => dismiss(id), 7000);
  }, [dismiss]);
  return { toasts, notify, dismiss };
}

function Toasts({ toasts, dismiss }) {
  return (
    <div className="toasts" role="status" aria-live="polite">
      {toasts.map((t) => (
        <div key={t.id} className={`toast tone-${t.tone}`}>
          <span>{t.text}</span>
          <button type="button" aria-label="Dismiss" onClick={() => dismiss(t.id)}>×</button>
        </div>
      ))}
    </div>
  );
}

// ── Small building blocks ────────────────────────────────────────────

// stale: the card shows telemetry, and telemetry is not arriving. Its values are
// then the last ones received, so mark them instead of presenting them as current.
function Card({ title, tag, tagTone, wide, stale, className = '', children }) {
  const shownTag = stale ? 'LAST KNOWN' : tag;
  const shownTone = stale ? 'warn' : tagTone;
  return (
    <section className={`card ${wide ? 'wide' : ''} ${stale ? 'is-stale' : ''} ${className}`}>
      <div className="card-head">
        <h2 className="card-title">{title}</h2>
        {shownTag && <span className={`tag ${shownTone ? `tone-${shownTone}` : ''}`}>{shownTag}</span>}
      </div>
      {children}
    </section>
  );
}

const hwTag = (mode) => (mode === 'hw' ? 'HW' : mode === 'mock' ? 'MOCK' : null);

function Row({ label, value, unit, tone = 'idle' }) {
  return (
    <div className={`kv-row tone-${tone}`}>
      <span>{label}</span>
      <span className="kv-value">{value}{unit && <span className="unit">{unit}</span>}</span>
    </div>
  );
}

function Meter({ value, max = 100, tone = 'accent' }) {
  const pct = Math.max(0, Math.min(100, ((value ?? 0) / max) * 100));
  return <div className={`meter tone-${tone}`}><i style={{ width: `${pct}%` }} /></div>;
}

const yesNo = (flag) => (flag ? 'Yes' : 'No');

// Two clicks within three seconds: a deliberate action without a modal dialog.
function ConfirmButton({ label, confirmLabel, onConfirm, disabled, className = 'btn btn-primary' }) {
  const [pending, setPending] = useState(false);
  const timerRef = useRef(null);
  useEffect(() => () => clearTimeout(timerRef.current), []);
  const click = () => {
    if (pending) {
      clearTimeout(timerRef.current);
      setPending(false);
      onConfirm();
    } else {
      setPending(true);
      timerRef.current = setTimeout(() => setPending(false), 3000);
    }
  };
  return (
    <button type="button" className={pending ? 'btn btn-danger' : className} disabled={disabled} onClick={click}>
      {pending ? confirmLabel : label}
    </button>
  );
}

// ── Status bar ───────────────────────────────────────────────────────

function batteryTone(pct) {
  if (pct < 20) return 'crit';
  if (pct < 40) return 'warn';
  return 'ok';
}

function Vital({ label, value, tone = 'idle' }) {
  return (
    <div className={`vital tone-${tone}`}>
      <span className="label">{label}</span>
      <span className="vital-value">{value}</span>
    </div>
  );
}

function TopBar({ link, health, px4, extMode, theme, cycleTheme }) {
  const { send, notify } = useCommands();
  const healthTone = { OK: 'ok', WARN: 'warn', ERROR: 'crit' }[health?.overallHealth] || 'idle';
  const batteryPct = px4.batteryRemainingPct ?? 0;
  const linkValue = link.live ? 'LIVE' : link.everLive ? `LOST ${link.ageS} s` : 'WAITING';
  return (
    <header className="topbar">
      <div className="topbar-inner">
        <div className="brand">
          <span className="brand-name">Rover Control Center</span>
          <span className="brand-sub">seq {health?.seq ?? '–'}</span>
        </div>
        <div className="vitals">
          <Vital label="Link" value={linkValue} tone={link.live ? 'ok' : 'crit'} />
          <Vital label="Health" value={link.live ? (health?.overallHealth || '–') : '–'} tone={link.live ? healthTone : 'idle'} />
          <Vital label="Mode" value={link.live ? (px4.navStateLabel || '–') : '–'} tone={link.live && extMode ? 'accent' : 'idle'} />
          {/* Armed is the one state worth showing from stale data: the motor may still be live. */}
          <Vital
            label="State"
            value={px4.armed ? (link.live ? 'ARMED' : 'ARMED ?') : (link.live ? 'DISARMED' : '–')}
            tone={px4.armed ? 'warn' : 'idle'}
          />
          <Vital
            label="Battery"
            value={link.live ? `${num(px4.batteryVoltageV ?? 0, 1)} V · ${num(batteryPct, 0)}%` : '–'}
            tone={link.live ? batteryTone(batteryPct) : 'idle'}
          />
        </div>
        <div className="topbar-actions">
          <button type="button" className="btn btn-ghost btn-sm" onClick={cycleTheme} title="Switch between automatic, light and dark">
            Theme: {theme}
          </button>
          {/* Disarm is always reachable: it must not depend on the readiness steps or on telemetry. */}
          <button
            type="button"
            className="btn btn-warn btn-sm"
            onClick={async () => { if (await send('disarm')) notify('Disarm sent to the rover'); }}
            title="Disarm the motor: always allowed"
          >
            Disarm
          </button>
          <button
            type="button"
            className="estop"
            onClick={async () => { if (await send('estop')) notify('E-STOP sent: zero speed and forced disarm'); }}
            title="Emergency stop: always allowed"
          >
            E-STOP
          </button>
        </div>
      </div>
    </header>
  );
}

// ── Drive readiness ──────────────────────────────────────────────────

function buildSteps({ link, px4, extMode }) {
  const linkText = link.live
    ? 'Rover telemetry is arriving.'
    : link.everLive
      ? `No telemetry for ${link.ageS} s. Check the rover session and the network.`
      : 'Waiting for the rover. Start the rover session with telemetry.';
  // text: the step's current state. hint: what the step is, shown while it is
  // still waiting on an earlier step (its state is then not worth reading).
  return [
    { key: 'link', name: 'Telemetry link', ok: link.live, text: linkText, hint: 'Telemetry from the rover.' },
    {
      key: 'px4',
      name: 'Flight controller',
      ok: !!px4.connected,
      text: px4.connected
        ? `PX4 connected, heartbeat ${px4.heartbeatAgeMs ?? 0} ms old.`
        : 'PX4 not connected. Check the XRCE agent pane and the serial cable.',
      hint: 'PX4 connected to the onboard computer.',
    },
    {
      key: 'mode',
      name: 'Drive mode',
      ok: extMode,
      text: extMode
        ? `${px4.navStateLabel} is active.`
        : `PX4 is in ${px4.navStateLabel || 'an unknown mode'}. Activate a rover mode.`,
      hint: 'A rover drive mode is active.',
    },
    {
      key: 'checks',
      name: 'Pre-arm checks',
      ok: !!px4.armable || !!px4.armed,
      text: (px4.armable || px4.armed)
        ? 'PX4 reports ready to arm.'
        : (px4.errorMsg || 'PX4 pre-arm checks are failing.'),
      hint: 'PX4 reports ready to arm.',
    },
    {
      key: 'arm',
      name: 'Arm',
      ok: !!px4.armed,
      text: px4.armed ? 'Armed. The motor is live.' : 'Arm only while someone is watching the rover.',
      hint: 'Arm only while someone is watching the rover.',
    },
  ];
}

function ReadinessCard({ link, px4, extMode }) {
  const { send } = useCommands();
  const steps = buildSteps({ link, px4, extMode });
  const currentIndex = steps.findIndex((s) => !s.ok);
  const ready = currentIndex === -1;
  const current = ready ? null : steps[currentIndex];

  const actionFor = (step) => {
    if (step.key === 'mode') {
      return (
        <button type="button" className="btn btn-primary" onClick={() => send('set_mode', { mode_name: DEFAULT_MODE })}>
          Activate {DEFAULT_MODE}
        </button>
      );
    }
    if (step.key === 'arm') {
      return <ConfirmButton label="Arm" confirmLabel="Click again to arm" onConfirm={() => send('arm')} />;
    }
    return null;
  };

  // Losing the link while armed is the one case the operator must not miss.
  const lostWhileArmed = !link.live && !!px4.armed;
  const bannerTone = lostWhileArmed ? 'crit' : ready ? 'warn' : 'idle';
  const bannerState = lostWhileArmed ? 'Link lost · last state armed' : ready ? 'Armed · ready to drive' : 'Not ready';
  const bannerText = lostWhileArmed
    ? 'The rover may still be armed. E-STOP and Disarm are still sent if the network allows.'
    : ready
      ? 'Send a goal or enable manual drive. Disarm when you are done.'
      : `Next: ${current.name.toLowerCase()}.`;

  return (
    <Card title="Drive readiness" className="ready" wide>
      <div className="ready-banner">
        <span className={`ready-state tone-${bannerTone}`}>
          <span className={`dot tone-${bannerTone}`} />
          {bannerState}
        </span>
        <span className="muted">{bannerText}</span>
        {lostWhileArmed && (
          <button type="button" className="btn btn-warn btn-sm" onClick={() => send('disarm')}>Disarm</button>
        )}
      </div>
      <ol className="steps">
        {steps.map((step, index) => {
          const done = ready || index < currentIndex;
          const isCurrent = index === currentIndex;
          const tone = done ? 'ok' : isCurrent ? 'accent' : 'idle';
          return (
            <li key={step.key} className={`step tone-${tone} ${isCurrent ? 'is-current' : ''} ${!done && !isCurrent ? 'is-pending' : ''}`}>
              <div className="step-head">
                <span className="step-num">{String(index + 1).padStart(2, '0')}</span>
                <span className="step-name">{step.name}</span>
                <span className="step-mark">{done ? 'OK' : isCurrent ? 'NEXT' : '···'}</span>
              </div>
              <p className="step-text">{done || isCurrent ? step.text : step.hint}</p>
              <div className="step-action">
                {isCurrent && actionFor(step)}
                {step.key === 'arm' && done && (
                  <button type="button" className="btn btn-warn" onClick={() => send('disarm')}>Disarm</button>
                )}
              </div>
            </li>
          );
        })}
      </ol>
    </Card>
  );
}

// ── Telemetry cards ──────────────────────────────────────────────────

function AlertsCard({ alerts, stale }) {
  const list = alerts || [];
  return (
    <Card title="Alerts" tag={list.length ? `${list.length} ACTIVE` : null} tagTone={list.length ? 'crit' : null} stale={stale}>
      {list.length > 0 ? (
        <ul className="alerts">
          {list.map((code) => (
            <li key={code} className={`alert tone-${alertTone(code)}`}>
              <span>{describeAlert(code)}</span>
              <code>{code}</code>
            </li>
          ))}
        </ul>
      ) : (
        <p className="all-clear"><span className="dot tone-ok" />No active alerts</p>
      )}
    </Card>
  );
}

function VehicleCard({ px4, hwMode, stale }) {
  const batteryPct = px4.batteryRemainingPct ?? 0;
  return (
    <Card title="Vehicle · PX4" tag={hwTag(hwMode)} stale={stale}>
      <div className="kv">
        <Row label="Connected" value={yesNo(px4.connected)} tone={px4.connected ? 'ok' : 'crit'} />
        <Row label="Mode" value={px4.navStateLabel || '–'} />
        <Row label="Armed" value={yesNo(px4.armed)} tone={px4.armed ? 'warn' : 'idle'} />
        <Row label="Pre-arm checks" value={px4.armable ? 'Pass' : 'Fail'} tone={px4.armable ? 'ok' : 'warn'} />
        <Row label="Heartbeat age" value={px4.heartbeatAgeMs ?? 0} unit="ms" />
      </div>
      <div className="divider" />
      <div className="kv">
        <Row label="Battery" value={`${num(px4.batteryVoltageV ?? 0, 2)} V`} unit={`${num(batteryPct, 0)}%`} tone={batteryTone(batteryPct)} />
        <Meter value={batteryPct} tone={batteryTone(batteryPct)} />
        <Row label="Current" value={num(px4.batteryCurrentA ?? 0, 2)} unit="A" />
        {px4.cpuLoadPct != null && <Row label="Controller CPU" value={num(px4.cpuLoadPct)} unit="%" />}
        {px4.ramUsagePct != null && <Row label="Controller RAM" value={num(px4.ramUsagePct)} unit="%" />}
      </div>
      {px4.errorMsg && <p className="note tone-warn">{px4.errorMsg}</p>}
    </Card>
  );
}

function CamerasCard({ cameras, hwMode, stale }) {
  const list = cameras || [];
  return (
    <Card title="Cameras" tag={hwTag(hwMode)} stale={stale}>
      {list.length === 0 && <p className="note">No camera data.</p>}
      <div className="cams">
        {list.map((cam, index) => {
          const tracking = cam.cameraId === 't265';
          return (
            <div key={cam.cameraId || index}>
              <div className="cam-head">
                <span className={`dot tone-${cam.connected ? 'ok' : 'crit'}`} />
                <span className="cam-name">{cam.cameraId || 'camera'}</span>
                <span className="muted">{tracking ? 'tracking' : 'depth'}</span>
              </div>
              <div className="kv">
                {tracking ? (
                  <>
                    <Row label="Odometry" value={cam.odomActive ? 'Active' : 'Lost'} tone={cam.odomActive ? 'ok' : 'crit'} />
                    <Row label="Fisheye stream" value={cam.streamAvailable ? num(cam.streamFps) : 'Off'} unit={cam.streamAvailable ? 'fps' : null} />
                  </>
                ) : (
                  <>
                    <Row label="Color" value={num(cam.streamFps ?? 0)} unit="fps" />
                    <Row label="Depth" value={num(cam.depthFps ?? 0)} unit="fps" />
                    <Row
                      label="Depth fill"
                      value={num((cam.depthQualitySampled ?? 0) * 100, 0)}
                      unit="%"
                      tone={(cam.depthQualitySampled ?? 0) < 0.5 ? 'warn' : 'idle'}
                    />
                    <Row label="Frame gap" value={num(cam.frameDeltaMs ?? 0, 0)} unit="ms" />
                  </>
                )}
                <Row label="IMU" value={cam.imuActive ? 'Active' : 'Off'} />
              </div>
              {cam.errorMsg && <p className="note tone-warn">{cam.errorMsg}</p>}
            </div>
          );
        })}
      </div>
    </Card>
  );
}

function ComputerCard({ jetson, hwMode, stale }) {
  if (!jetson) {
    return <Card title="Onboard computer" tag={hwTag(hwMode)}><p className="note">No data.</p></Card>;
  }
  const cores = jetson.cpuUsagePct || [];
  const cpuAvg = cores.length ? cores.reduce((sum, v) => sum + v, 0) / cores.length : null;
  const ramPct = jetson.ramTotalMb ? ((jetson.ramUsedMb ?? 0) / jetson.ramTotalMb) * 100 : 0;
  const hours = Math.floor((jetson.uptimeS ?? 0) / 3600);
  return (
    <Card title="Onboard computer" tag={hwTag(hwMode)} stale={stale}>
      <div className="kv">
        <Row label="CPU" value={num(cpuAvg)} unit="%" />
        <div className="cores" aria-hidden="true">
          {cores.map((v, i) => <i key={i} style={{ height: `${Math.max(8, Math.min(100, v))}%` }} />)}
        </div>
        <Row label="GPU" value={num(jetson.gpuUsagePct ?? 0)} unit="%" />
        <Row label="RAM" value={`${jetson.ramUsedMb ?? 0} / ${jetson.ramTotalMb ?? 0}`} unit="MB" tone={ramPct > 90 ? 'warn' : 'idle'} />
        <Meter value={ramPct} tone={ramPct > 90 ? 'warn' : 'accent'} />
        <Row label="CPU temperature" value={num(jetson.tempCpuC)} unit="°C" />
        <Row label="Disk free" value={num(jetson.diskFreeGb)} unit="GB" tone={(jetson.diskFreeGb ?? 0) < 5 ? 'warn' : 'idle'} />
        {jetson.wifiSignalDbm != null && <Row label="Wi-Fi" value={num(jetson.wifiSignalDbm, 0)} unit="dBm" />}
        <Row label="Uptime" value={hours} unit="h" />
        <Row
          label="Throttled"
          value={jetson.isThermalThrottled ? 'Thermal' : jetson.isPowerThrottled ? 'Power' : 'No'}
          tone={(jetson.isThermalThrottled || jetson.isPowerThrottled) ? 'warn' : 'idle'}
        />
      </div>
    </Card>
  );
}

// ── Operation cards ──────────────────────────────────────────────────

function ModesCard({ px4, extMode }) {
  const { send } = useCommands();
  const [manage, setManage] = useState(false);
  return (
    <Card title="Drive modes" tag={px4.navStateLabel || null} tagTone={extMode ? 'accent' : null}>
      <ul className="modes">
        {PX4_MODES.map((mode) => (
          <li key={mode.name} className="mode">
            <span className="mode-name">{mode.label}</span>
            <button type="button" className="btn btn-sm" onClick={() => send('set_mode', { mode_name: mode.name })}>
              Activate
            </button>
            <span className="mode-desc">{mode.desc}</span>
            {manage && (
              <span className="mode-manage">
                <button type="button" className="btn btn-sm btn-ghost" onClick={() => send('register_mode', { mode_name: mode.name })}>
                  Register
                </button>
                <button type="button" className="btn btn-sm btn-ghost" onClick={() => send('unregister_mode', { mode_name: mode.name })}>
                  Unregister
                </button>
              </span>
            )}
          </li>
        ))}
      </ul>
      <div className="btn-row">
        <button type="button" className="btn btn-sm btn-ghost" aria-expanded={manage} onClick={() => setManage((v) => !v)}>
          {manage ? 'Hide register / unregister' : 'Register or unregister a mode'}
        </button>
      </div>
      <p className="note">
        The rover session starts {DEFAULT_MODE}. Register the other modes before activating them.
      </p>
    </Card>
  );
}

function DriveCard({ canDrive }) {
  const [enabled, setEnabled] = useState(false);
  const [speed, setSpeed] = useState(0);
  const [steering, setSteering] = useState(0);
  const intervalRef = useRef(null);
  const speedRef = useRef(0);
  const steeringRef = useRef(0);
  const wasEnabledRef = useRef(false);
  const canDriveRef = useRef(canDrive);

  useEffect(() => { speedRef.current = speed; }, [speed]);
  useEffect(() => { steeringRef.current = steering; }, [steering]);
  useEffect(() => { canDriveRef.current = canDrive; }, [canDrive]);

  // Drive commands are high-rate and fire-and-forget: no toasts, no log entries.
  const sendDrive = async (s, st) => {
    try {
      await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          cmd_type: 'drive',
          params: { speed_ms: s, steering: st },
          issued_by: 'dashboard',
        }),
      });
    } catch { /* fire-and-forget */ }
  };

  // The safety gate rejects drive commands unless the rover is armed, so stop
  // sending as soon as that is no longer true.
  useEffect(() => {
    if (!canDrive && enabled) setEnabled(false);
  }, [canDrive, enabled]);

  // While enabled: publish at 5 Hz, so a slider change or a stop reaches the rover
  // within 0.2 s (RoverManual stops by itself after 2 s without commands).
  // When disabled: stop publishing.
  useEffect(() => {
    if (enabled) {
      wasEnabledRef.current = true;
      intervalRef.current = setInterval(() => {
        sendDrive(speedRef.current, steeringRef.current);
      }, 200);
    } else {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
      // Reset the sliders and send one final zero after an active drive session.
      setSpeed(0);
      setSteering(0);
      if (wasEnabledRef.current) {
        if (canDriveRef.current) sendDrive(0, 0);
        wasEnabledRef.current = false;
      }
    }
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [enabled]);

  return (
    <Card title="Manual drive" tag={enabled ? 'SENDING 5 HZ' : null} tagTone={enabled ? 'warn' : null}>
      <label className={`switch ${canDrive ? '' : 'is-disabled'}`}>
        <input
          id="drive-enable"
          type="checkbox"
          checked={enabled}
          disabled={!canDrive}
          onChange={(e) => setEnabled(e.target.checked)}
        />
        <span>Enable manual drive</span>
      </label>
      {!canDrive && <p className="note">Available once the rover is armed and telemetry is live.</p>}
      <div className={enabled ? '' : 'is-off'}>
        <div className="slider">
          <div className="slider-head">
            <label className="label" htmlFor="drive-speed">Speed</label>
            <span className="mono">{speed.toFixed(1)}<span className="unit">m/s</span></span>
          </div>
          <input
            id="drive-speed"
            type="range"
            min={0}
            max={2}
            step={0.1}
            value={speed}
            onChange={(e) => setSpeed(parseFloat(e.target.value))}
          />
        </div>
        <div className="slider" style={{ marginTop: 10 }}>
          <div className="slider-head">
            <label className="label" htmlFor="drive-steering">Steering</label>
            <span className="mono">{steering.toFixed(2)}</span>
          </div>
          <input
            id="drive-steering"
            type="range"
            min={-1}
            max={1}
            step={0.05}
            value={steering}
            onChange={(e) => setSteering(parseFloat(e.target.value))}
          />
        </div>
        <div className="btn-row" style={{ marginTop: 12 }}>
          <button
            type="button"
            className="btn btn-sm"
            onClick={() => {
              setSpeed(0);
              setSteering(0);
              if (enabled) sendDrive(0, 0);
            }}
          >
            Zero speed and steering
          </button>
        </div>
      </div>
      <p className="note">
        Forward only. Open loop: speed sets a throttle level, and the wheels start
        turning at about 0.5 m/s.
      </p>
    </Card>
  );
}

function NavigationCard({ nav2, slamLatencyMs, armed, stale }) {
  const { send, notify } = useCommands();
  const [mapX, setMapX] = useState('');
  const [mapY, setMapY] = useState('');
  const [yawDeg, setYawDeg] = useState('0');
  const nav = nav2 || {};

  const sendGoal = () => {
    const x = Number(mapX);
    const y = Number(mapY);
    const yaw = Number(yawDeg || 0);
    if (mapX === '' || mapY === '' || !Number.isFinite(x) || !Number.isFinite(y)) {
      notify('Enter Map X and Map Y before sending a goal.', 'warn');
      return;
    }
    // The command schema names the map coordinates latitude and longitude.
    send('nav_goal', {
      latitude: x,
      longitude: y,
      altitude: 0,
      yaw_deg: Number.isFinite(yaw) ? yaw : 0,
    });
  };

  const tfAge = slamLatencyMs == null || slamLatencyMs < 0 ? '–' : num(slamLatencyMs, 0);
  return (
    <Card title="Navigation · Nav2" tag={nav.available ? 'READY' : 'UNAVAILABLE'} tagTone={nav.available ? 'ok' : 'crit'} stale={stale}>
      <div className="kv">
        <Row label="Goal" value={nav.goalStatusLabel || '–'} tone={nav.navigating ? 'accent' : 'idle'} />
        <Row label="Distance left" value={nav.distanceRemainingM != null ? num(nav.distanceRemainingM, 2) : '–'} unit="m" />
        <Row label="ETA" value={nav.etaSeconds != null ? num(nav.etaSeconds, 0) : '–'} unit="s" />
        <Row label="Recoveries" value={nav.numberOfRecoveries ?? 0} />
        <Row label="Map position age" value={tfAge} unit="ms" />
      </div>
      <div className="divider" />
      <div className="fields">
        <div className="field">
          <label className="label" htmlFor="nav-x">Map X (m)</label>
          <input id="nav-x" type="number" step="any" value={mapX} onChange={(e) => setMapX(e.target.value)} />
        </div>
        <div className="field">
          <label className="label" htmlFor="nav-y">Map Y (m)</label>
          <input id="nav-y" type="number" step="any" value={mapY} onChange={(e) => setMapY(e.target.value)} />
        </div>
        <div className="field">
          <label className="label" htmlFor="nav-yaw">Yaw (deg)</label>
          <input id="nav-yaw" type="number" step="any" value={yawDeg} onChange={(e) => setYawDeg(e.target.value)} />
        </div>
      </div>
      <div className="btn-row">
        <button type="button" className="btn btn-primary" disabled={!armed || stale} onClick={sendGoal}>Send goal</button>
        <button type="button" className="btn" onClick={() => send('cancel_goal')}>Cancel goal</button>
      </div>
      {(!armed || stale) && <p className="note">Goals are accepted once the rover is armed and telemetry is live.</p>}
    </Card>
  );
}

function RecordingCard({ diskFreeGb }) {
  const { send } = useCommands();
  return (
    <Card title="Recording">
      <div className="btn-row">
        <button type="button" className="btn btn-danger" onClick={() => send('record', { action: 'start' })}>
          ● Start segment
        </button>
        <button type="button" className="btn" onClick={() => send('record', { action: 'stop' })}>
          ■ Stop
        </button>
      </div>
      <div className="kv">
        <Row label="Rover disk free" value={num(diskFreeGb)} unit="GB" tone={(diskFreeGb ?? 0) < 5 ? 'warn' : 'idle'} />
      </div>
      <p className="note">
        Each start opens a new bag segment on the rover; stop finalizes it. Needs the
        recorder window of the rover session. Camera bags grow by roughly 1.4 GB per minute.
      </p>
    </Card>
  );
}

// ── Command log ──────────────────────────────────────────────────────

function normalizeCommandLogEntry(entry) {
  if (!entry) return null;
  return {
    cmdId: entry.cmdId ?? entry.cmd_id ?? 'unknown',
    cmdType: entry.cmdType ?? entry.cmd_type ?? 'unknown',
    ackStatus: entry.ackStatus ?? entry.ack_status ?? 'UNKNOWN',
    message: entry.message ?? '',
    roundTripMs: entry.roundTripMs ?? entry.round_trip_ms ?? 0,
  };
}

function ackTone(status) {
  if (/REJECTED|FAILED/.test(status)) return 'crit';
  if (/ACCEPTED|COMPLETED/.test(status)) return 'ok';
  return 'idle';
}

function CommandLogCard() {
  const [log, setLog] = useState([]);
  const ackData = useWebSocket('/ws/cmd_ack');

  useEffect(() => {
    fetch('/api/command/history')
      .then((r) => r.json())
      .then((entries) => (Array.isArray(entries) ? entries.map(normalizeCommandLogEntry).filter(Boolean) : []))
      .then(setLog)
      .catch(() => {});
  }, []);

  useEffect(() => {
    if (!ackData) return;
    const nextEntry = normalizeCommandLogEntry(ackData);
    if (!nextEntry) return;
    setLog((prev) => {
      const filtered = prev.filter((entry) => entry.cmdId !== nextEntry.cmdId);
      return [nextEntry, ...filtered].slice(0, 50);
    });
  }, [ackData]);

  return (
    <Card title="Command log" tag={log.length ? `${log.length}` : null} wide>
      {log.length === 0 ? (
        <p className="note">No commands sent yet. Each command and the rover's answer appear here.</p>
      ) : (
        <div className="log-wrap">
          <table className="log">
            <thead>
              <tr><th>Status</th><th>Command</th><th>Answer</th><th>Round trip</th><th>ID</th></tr>
            </thead>
            <tbody>
              {log.map((entry) => (
                <tr key={entry.cmdId}>
                  <td><span className={`status tone-${ackTone(entry.ackStatus)}`}>{entry.ackStatus.replace(/^ACK_/, '')}</span></td>
                  <td className="mono">{entry.cmdType}</td>
                  <td>{entry.message}</td>
                  <td className="mono">{entry.roundTripMs} ms</td>
                  <td className="mono muted">{String(entry.cmdId).slice(0, 8)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </Card>
  );
}

// ── App ──────────────────────────────────────────────────────────────

export default function App() {
  const health = useWebSocket('/ws/health', '/api/health/history');
  const link = useLinkState(health);
  const [theme, cycleTheme] = useTheme();
  const { toasts, notify, dismiss } = useToasts();
  const [hwMode, setHwMode] = useState({});

  useEffect(() => {
    const fetchHwMode = () => {
      fetch('/api/hw_mode')
        .then((r) => r.json())
        .then(setHwMode)
        .catch(() => {});
    };
    fetchHwMode();
    const interval = setInterval(fetchHwMode, 10000);
    return () => clearInterval(interval);
  }, []);

  const send = useCallback(async (cmdType, params = {}) => {
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ cmd_type: cmdType, params, issued_by: 'dashboard' }),
      });
      const data = await res.json();
      if (data.error) {
        notify(`${cmdType} rejected: ${String(data.error).replace(/^Safety gate rejected: /, '')}`);
        return false;
      }
      return true;
    } catch (e) {
      notify(`${cmdType} not sent: ${e.message}`);
      return false;
    }
  }, [notify]);

  const px4 = health?.px4 || {};
  const navState = px4.navState ?? 0;
  const extMode = navState >= EXTERNAL_MODE_MIN && navState <= EXTERNAL_MODE_MAX;
  const armed = !!px4.armed;
  // Telemetry cards show the last values received when the link is down.
  const stale = !link.live && health != null;

  return (
    <CommandContext.Provider value={{ send, notify }}>
      <TopBar link={link} health={health} px4={px4} extMode={extMode} theme={theme} cycleTheme={cycleTheme} />
      <main className="page">
        <ReadinessCard link={link} px4={px4} extMode={extMode} />
        <div className="grid">
          <AlertsCard alerts={health?.activeAlerts} stale={stale} />
          <VehicleCard px4={px4} hwMode={hwMode.px4} stale={stale} />
          <CamerasCard cameras={health?.cameras} hwMode={hwMode.camera} stale={stale} />
          <ComputerCard jetson={health?.jetson} hwMode={hwMode.jetson} stale={stale} />
          <ModesCard px4={px4} extMode={extMode} />
          <DriveCard canDrive={armed && link.live} />
          <NavigationCard nav2={health?.nav2} slamLatencyMs={health?.slamLatencyMs} armed={armed} stale={stale} />
          <RecordingCard diskFreeGb={health?.jetson?.diskFreeGb} />
          <CommandLogCard />
        </div>
      </main>
      <Toasts toasts={toasts} dismiss={dismiss} />
    </CommandContext.Provider>
  );
}
