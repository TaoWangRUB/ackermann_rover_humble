import React, { useState, useEffect, useRef } from 'react';

const WS_BASE = `ws://${window.location.host}`;

function useWebSocket(path, initialUrl = null) {
  const [data, setData] = useState(null);
  const wsRef = useRef(null);

  useEffect(() => {
    let cancelled = false;

    const fetchInitial = async () => {
      if (!initialUrl) return;
      try {
        const response = await fetch(initialUrl);
        if (!response.ok) return;
        const payload = await response.json();
        if (!cancelled && payload) {
          setData(payload.health ?? payload);
        }
      } catch {
        // Best-effort bootstrap only; live updates come from WebSocket.
      }
    };

    const connect = () => {
      const ws = new WebSocket(`${WS_BASE}${path}`);
      wsRef.current = ws;
      ws.onmessage = (e) => setData(JSON.parse(e.data));
      ws.onclose = () => {
        if (!cancelled) setTimeout(connect, 2000);
      };
      ws.onerror = () => ws.close();
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

function StatusBadge({ health }) {
  const colors = { OK: '#22c55e', WARN: '#f59e0b', ERROR: '#ef4444' };
  return (
    <span style={{
      background: colors[health] || '#6b7280',
      color: 'white', padding: '2px 8px', borderRadius: 4, fontSize: 12
    }}>
      {health || 'UNKNOWN'}
    </span>
  );
}

function formatMapOdomTfAge(slamLatencyMs) {
  if (slamLatencyMs == null || slamLatencyMs < 0) return 'N/A';
  return `${slamLatencyMs.toFixed(0)}ms`;
}

function HwModeBadge({ mode }) {
  if (!mode) return null;
  const isHw = mode === 'hw';
  return (
    <span style={{
      background: isHw ? '#3b82f6' : '#9333ea',
      color: 'white', padding: '1px 6px', borderRadius: 4, fontSize: 10,
      marginLeft: 6, fontWeight: 600, letterSpacing: 0.5,
    }}>
      {isHw ? 'HW' : 'MOCK'}
    </span>
  );
}

function Panel({ title, hwMode, children }) {
  return (
    <div style={{ border: '1px solid #e5e7eb', borderRadius: 8, padding: 16, margin: 8 }}>
      <h3 style={{ margin: '0 0 8px', fontSize: 14, color: '#374151' }}>
        {title}
        <HwModeBadge mode={hwMode} />
      </h3>
      {children}
    </div>
  );
}

function CameraPanel({ cam, hwMode }) {
  if (!cam) return <Panel title="Camera" hwMode={hwMode}><p>No data</p></Panel>;
  const label = cam.cameraId ? `Camera (${cam.cameraId})` : 'Camera';
  const isTrackingCamera = cam.cameraId === 't265';
  const streamFpsText = cam.streamAvailable ? cam.streamFps?.toFixed(1) : 'N/A';
  return (
    <Panel title={label} hwMode={hwMode}>
      <p>Connected: {cam.connected ? 'Yes' : 'No'}</p>
      {isTrackingCamera ? (
        <>
          <p>Fisheye FPS: {streamFpsText}</p>
          <p>Odom Active: {cam.odomActive ? 'Yes' : 'No'}</p>
        </>
      ) : (
        <>
          <p>Frame Delta: {cam.frameDeltaMs?.toFixed(1)} ms</p>
          <p>Depth FPS: {cam.depthFps?.toFixed(1)}</p>
        </>
      )}
      <p>IMU Active: {cam.imuActive ? 'Yes' : 'No'}</p>
    </Panel>
  );
}

function Px4Panel({ px4, hwMode }) {
  if (!px4) return <Panel title="PX4" hwMode={hwMode}><p>No data</p></Panel>;
  return (
    <Panel title="PX4" hwMode={hwMode}>
      <p>Connected: {px4.connected ? 'Yes' : 'No'}</p>
      <p>Armed: {px4.armed ? 'Yes' : 'No'}</p>
      <p>Armable: {px4.armable ? 'Yes' : 'No'}</p>
      <p>Nav State: {px4.navStateLabel}</p>
      <p>Battery: {px4.batteryRemainingPct?.toFixed(0)}% ({px4.batteryVoltageV?.toFixed(1)}V)</p>
      <p>Heartbeat Age: {px4.heartbeatAgeMs} ms</p>
    </Panel>
  );
}

function JetsonPanel({ jetson, hwMode }) {
  if (!jetson) return <Panel title="Jetson" hwMode={hwMode}><p>No data</p></Panel>;
  const avgCpuPct = jetson.cpuUsagePct?.length
    ? jetson.cpuUsagePct.reduce((sum, value) => sum + value, 0) / jetson.cpuUsagePct.length
    : null;
  return (
    <Panel title="Jetson" hwMode={hwMode}>
      <p>CPU: {avgCpuPct?.toFixed(1) ?? 'N/A'}%</p>
      <p>GPU: {jetson.gpuUsagePct?.toFixed(1)}%</p>
      <p>RAM: {jetson.ramUsedMb}/{jetson.ramTotalMb} MB</p>
      <p>Temp CPU: {jetson.tempCpuC?.toFixed(1)}C</p>
      <p>Disk Free: {jetson.diskFreeGb?.toFixed(1)} GB</p>
      <p>Thermal Throttle: {jetson.isThermalThrottled ? 'YES' : 'No'}</p>
    </Panel>
  );
}

function AlertsPanel({ alerts }) {
  return (
    <Panel title="Active Alerts">
      {alerts && alerts.length > 0 ? (
        <ul style={{ margin: 0, paddingLeft: 16 }}>
          {alerts.map((a, i) => <li key={i} style={{ color: '#ef4444' }}>{a}</li>)}
        </ul>
      ) : (
        <p style={{ color: '#22c55e' }}>No active alerts</p>
      )}
    </Panel>
  );
}

function DrivePanel() {
  const [enabled, setEnabled] = useState(false);
  const [speed, setSpeed] = useState(0);
  const [steering, setSteering] = useState(0);
  const intervalRef = useRef(null);
  const speedRef = useRef(0);
  const steeringRef = useRef(0);
  const wasEnabledRef = useRef(false);

  useEffect(() => { speedRef.current = speed; }, [speed]);
  useEffect(() => { steeringRef.current = steering; }, [steering]);

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

  // When enabled: publish at 2 Hz. When disabled: stop publishing.
  useEffect(() => {
    if (enabled) {
      wasEnabledRef.current = true;
      intervalRef.current = setInterval(() => {
        sendDrive(speedRef.current, steeringRef.current);
      }, 500);
    } else {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
      // Reset sliders and send a final zero only after an active drive session.
      setSpeed(0);
      setSteering(0);
      if (wasEnabledRef.current) {
        sendDrive(0, 0);
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

  const disabledStyle = { opacity: enabled ? 1 : 0.4, pointerEvents: enabled ? 'auto' : 'none' };
  const sliderStyle = { width: '100%', margin: '4px 0' };
  const valStyle = { fontSize: 12, color: '#6b7280', textAlign: 'right', minWidth: 48, display: 'inline-block' };

  return (
    <Panel title="Drive Control">
      <label style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 12, cursor: 'pointer' }}>
        <input type="checkbox" checked={enabled} onChange={e => setEnabled(e.target.checked)} />
        <span style={{ fontSize: 13, fontWeight: 600 }}>Enable</span>
      </label>
      <div style={disabledStyle}>
        <div>
          <label style={{ fontSize: 12 }}>
            Speed (m/s)
            <span style={valStyle}>{speed.toFixed(1)}</span>
          </label>
          <input type="range" min={-2} max={2} step={0.1} value={speed}
            onChange={e => setSpeed(parseFloat(e.target.value))}
            style={sliderStyle} />
        </div>
        <div style={{ marginTop: 8 }}>
          <label style={{ fontSize: 12 }}>
            Steering
            <span style={valStyle}>{steering.toFixed(2)}</span>
          </label>
          <input type="range" min={-1} max={1} step={0.05} value={steering}
            onChange={e => setSteering(parseFloat(e.target.value))}
            style={sliderStyle} />
        </div>
      </div>
    </Panel>
  );
}

async function sendDashboardCommand(cmdType, params = {}) {
  try {
    const res = await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd_type: cmdType, params, issued_by: 'dashboard' }),
    });
    const data = await res.json();
    if (data.error) alert(`Rejected: ${data.error}`);
  } catch (e) {
    alert(`Failed: ${e.message}`);
  }
}

const PX4_MODES = [
  { label: 'Speed/Steering', name: 'Rover Speed Steering' },
  { label: 'Speed/Attitude', name: 'Rover Speed Attitude' },
  { label: 'RoverManual', name: 'RoverManual' },
];

function Px4CommandsPanel() {
  const modeButtonStyle = { background: '#1f2937', color: 'white', fontSize: 12 };
  return (
    <Panel title="PX4 Commands">
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
        <button onClick={() => sendDashboardCommand('arm')}>Arm</button>
        <button onClick={() => sendDashboardCommand('disarm')}>Disarm</button>
        <button onClick={() => sendDashboardCommand('estop')} style={{ background: '#ef4444', color: 'white' }}>
          E-STOP
        </button>
      </div>
      <div style={{ fontSize: 11, color: '#6b7280', margin: '10px 0 4px' }}>Modes</div>
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
        {PX4_MODES.map(({ label, name }) => (
          <button
            key={name}
            onClick={() => sendDashboardCommand('set_mode', { mode_name: name })}
            style={modeButtonStyle}
            title={name}
          >
            {label}
          </button>
        ))}
      </div>
    </Panel>
  );
}

function RecordPanel() {
  // The recorder publishes no liveness back to CC yet, so the UI is
  // fire-and-forget: each click sends start/stop on rover/cmd/record
  // and the rover-side bridge forwards it to /record/cmd.
  return (
    <Panel title="Recording">
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
        <button
          onClick={() => sendDashboardCommand('record', { action: 'start' })}
          style={{ background: '#dc2626', color: 'white' }}
        >
          ● Start
        </button>
        <button onClick={() => sendDashboardCommand('record', { action: 'stop' })}>
          ■ Stop
        </button>
      </div>
      <p style={{ fontSize: 11, color: '#6b7280', margin: '8px 0 0' }}>
        Each Start opens a new bag segment (run_…/_segN). Stop finalizes
        the current segment.
      </p>
    </Panel>
  );
}

function Nav2ControlPanel({ nav2 }) {
  const [latitude, setLatitude] = useState('');
  const [longitude, setLongitude] = useState('');
  const [altitude, setAltitude] = useState('0');
  const [yawDeg, setYawDeg] = useState('0');

  const nav2Data = nav2 || {};
  const availableText = nav2Data.available ? 'Ready' : 'Unavailable';
  const navigatingText = nav2Data.navigating ? 'Yes' : 'No';
  const localizationText = nav2Data.localizationActive ? 'Active' : 'Inactive';
  const goalStatusText = nav2Data.goalStatusLabel || 'UNKNOWN';
  const feedbackStatusText = nav2Data.feedbackStatus || 'unknown';
  const distanceText = nav2Data.distanceRemainingM != null
    ? `${nav2Data.distanceRemainingM.toFixed(2)} m`
    : 'N/A';
  const etaText = nav2Data.etaSeconds != null
    ? `${nav2Data.etaSeconds.toFixed(1)} s`
    : 'N/A';
  const navigationTimeText = nav2Data.navigationTimeS != null
    ? `${nav2Data.navigationTimeS.toFixed(1)} s`
    : 'N/A';
  const recoveriesText = nav2Data.numberOfRecoveries ?? 'N/A';

  const inputStyle = {
    width: '100%',
    padding: '6px 8px',
    border: '1px solid #d1d5db',
    borderRadius: 6,
    fontSize: 12,
    boxSizing: 'border-box',
  };

  const labelStyle = {
    display: 'block',
    fontSize: 12,
    color: '#374151',
    marginBottom: 4,
  };

  const sendNavGoal = () => {
    const parsedLatitude = Number(latitude);
    const parsedLongitude = Number(longitude);
    const parsedAltitude = Number(altitude || 0);
    const parsedYawDeg = Number(yawDeg || 0);

    if (!Number.isFinite(parsedLatitude) || !Number.isFinite(parsedLongitude)) {
      alert('Latitude and longitude are required.');
      return;
    }

    sendDashboardCommand('nav_goal', {
      latitude: parsedLatitude,
      longitude: parsedLongitude,
      altitude: Number.isFinite(parsedAltitude) ? parsedAltitude : 0,
      yaw_deg: Number.isFinite(parsedYawDeg) ? parsedYawDeg : 0,
    });
  };

  return (
    <Panel title="Nav2 Control">
      <div style={{ marginBottom: 12, fontSize: 12, color: '#374151' }}>
        <p style={{ margin: '0 0 4px' }}>Server: {availableText}</p>
        <p style={{ margin: '0 0 4px' }}>Goal: {goalStatusText}</p>
        <p style={{ margin: '0 0 4px' }}>Navigating: {navigatingText}</p>
        <p style={{ margin: '0 0 4px' }}>Localization: {localizationText}</p>
        <p style={{ margin: '0 0 4px' }}>Feedback: {feedbackStatusText}</p>
        <p style={{ margin: '0 0 4px' }}>Distance Remaining: {distanceText}</p>
        <p style={{ margin: '0 0 4px' }}>ETA: {etaText}</p>
        <p style={{ margin: '0 0 4px' }}>Time Taken: {navigationTimeText}</p>
        <p style={{ margin: 0 }}>Recoveries: {recoveriesText}</p>
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, minmax(0, 1fr))', gap: 8, marginBottom: 12 }}>
        <div>
          <label style={labelStyle}>Map X</label>
          <input
            type="number"
            step="any"
            value={latitude}
            onChange={(e) => setLatitude(e.target.value)}
            style={inputStyle}
          />
        </div>
        <div>
          <label style={labelStyle}>Map Y</label>
          <input
            type="number"
            step="any"
            value={longitude}
            onChange={(e) => setLongitude(e.target.value)}
            style={inputStyle}
          />
        </div>
        <div>
          <label style={labelStyle}>Altitude</label>
          <input
            type="number"
            step="any"
            value={altitude}
            onChange={(e) => setAltitude(e.target.value)}
            style={inputStyle}
          />
        </div>
        <div>
          <label style={labelStyle}>Yaw (deg)</label>
          <input
            type="number"
            step="any"
            value={yawDeg}
            onChange={(e) => setYawDeg(e.target.value)}
            style={inputStyle}
          />
        </div>
      </div>
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
        <button onClick={sendNavGoal}>Send Goal</button>
        <button onClick={() => sendDashboardCommand('cancel_goal')}>Cancel Goal</button>
      </div>
    </Panel>
  );
}

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

function CommandLogPanel() {
  const [log, setLog] = useState([]);
  const ackData = useWebSocket('/ws/cmd_ack');

  useEffect(() => {
    fetch('/api/command/history')
      .then(r => r.json())
      .then(entries => Array.isArray(entries) ? entries.map(normalizeCommandLogEntry).filter(Boolean) : [])
      .then(setLog)
      .catch(() => {});
  }, []);

  useEffect(() => {
    if (ackData) {
      const nextEntry = normalizeCommandLogEntry(ackData);
      if (!nextEntry) return;
      setLog(prev => {
        const filtered = prev.filter(entry => entry.cmdId !== nextEntry.cmdId);
        return [nextEntry, ...filtered].slice(0, 50);
      });
    }
  }, [ackData]);

  return (
    <Panel title="Command Log">
      <div style={{ maxHeight: 200, overflow: 'auto', fontSize: 11 }}>
        {log.map((entry, i) => (
          <div key={i} style={{ borderBottom: '1px solid #f3f4f6', padding: '2px 0' }}>
            {entry.cmdId}: {entry.ackStatus} ({entry.roundTripMs}ms) - {entry.message}
          </div>
        ))}
      </div>
    </Panel>
  );
}

export default function App() {
  const health = useWebSocket('/ws/health', '/api/health/history');
  const [hwMode, setHwMode] = useState({});

  useEffect(() => {
    const fetchHwMode = () => {
      fetch('/api/hw_mode')
        .then(r => r.json())
        .then(setHwMode)
        .catch(() => {});
    };
    fetchHwMode();
    const interval = setInterval(fetchHwMode, 10000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div style={{ fontFamily: 'system-ui', maxWidth: 1000, margin: '0 auto', padding: 16 }}>
      <div style={{ marginBottom: 16 }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
          <h1 style={{ fontSize: 20, margin: 0 }}>Rover Control Center</h1>
          <StatusBadge health={health?.overallHealth} />
        </div>
        {health && (
          <div style={{ display: 'flex', alignItems: 'center', gap: 12, marginTop: 4, fontSize: 12, color: '#6b7280' }}>
            <span>SLAM: {formatMapOdomTfAge(health.slamLatencyMs)}</span>
            <span>seq: {health.seq}</span>
            {health.cameras?.length > 0 && <span>📷 {health.cameras.map(c => c.cameraId || '?').join(', ')}</span>}
          </div>
        )}
      </div>

      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))', gap: 0 }}>
        {(health?.cameras || []).map((cam, i) =>
          <CameraPanel key={cam.cameraId || i} cam={cam} hwMode={hwMode.camera} />
        )}
        {(!health?.cameras || health.cameras.length === 0) &&
          <CameraPanel cam={null} hwMode={hwMode.camera} />
        }
        <Px4Panel px4={health?.px4} hwMode={hwMode.px4} />
        <JetsonPanel jetson={health?.jetson} hwMode={hwMode.jetson} />
        <AlertsPanel alerts={health?.activeAlerts} />
        <DrivePanel />
        <Px4CommandsPanel />
        <RecordPanel />
        <Nav2ControlPanel nav2={health?.nav2} />
        <CommandLogPanel />
      </div>
    </div>
  );
}
